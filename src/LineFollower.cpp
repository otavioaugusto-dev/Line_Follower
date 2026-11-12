#include "LineFollower.h"

LineFollower::LineFollower()
    : _state(RobotState::FOLLOWING_LINE),
      _kp(DEFAULT_KP),
      _kd(DEFAULT_KD),
      _ki(DEFAULT_KI),
      _lastError(0.0f),
      _filteredDerivative(0.0f),
      _integral(0.0f),
      _lastLoopTimeMicros(0),
      _startTimeMs(0),
      _stateTimerMs(0),
      _lastDiagTimeMs(0),
      _intersectionConfirmCount(0),
      _intersectionExitCount(0),
      _diagnosticEnabled(CONFIG_MODE_DIAGNOSTIC_DEFAULT != 0),
      _ledState(false) {}

void LineFollower::begin() {
    if (_diagnosticEnabled) {
        Serial.begin(DIAGNOSTIC_BAUD_RATE);
    }

    _sensors.begin();
    _motors.begin();

    _state = RobotState::FOLLOWING_LINE;
    _startTimeMs = millis();
    _stateTimerMs = millis();
    _lastLoopTimeMicros = micros();
}

void LineFollower::setDiagnosticEnabled(bool enabled) {
    _diagnosticEnabled = enabled;
    if (_diagnosticEnabled && !Serial) {
        Serial.begin(DIAGNOSTIC_BAUD_RATE);
    }
}

void LineFollower::setPIDGains(float kp, float kd, float ki) {
    _kp = kp;
    _kd = kd;
    _ki = ki;
}

RobotState LineFollower::getState() const {
    return _state;
}

void LineFollower::update() {
    uint32_t nowMicros = micros();
    uint32_t elapsedMicros = nowMicros - _lastLoopTimeMicros;

    // Temporização não bloqueante do ciclo principal de controle
    if (elapsedMicros < LOOP_TARGET_MICROS) {
        return;
    }
    _lastLoopTimeMicros = nowMicros;

    // Leitura atualizada dos sensores de linha
    _sensors.update();

    // Verificação contínua do tempo limite da prova (60s regulamento)
    checkSafetyTimeout();

    // Execução da Máquina de Estados Finitos (FSM)
    switch (_state) {
        case RobotState::FOLLOWING_LINE:
            processFollowingLine(elapsedMicros);
            break;

        case RobotState::INTERSECTION_CROSSING:
            processIntersectionCrossing();
            break;

        case RobotState::GAP_CROSSING:
            processGapCrossing();
            break;

        case RobotState::SEARCHING_LINE:
            processSearchingLine();
            break;

        case RobotState::STOPPED:
            processStopped();
            break;

        default:
            _state = RobotState::FOLLOWING_LINE;
            break;
    }

    // Emissão de telemetria serial se ativada
    if (_diagnosticEnabled && (millis() - _lastDiagTimeMs >= DIAGNOSTIC_INTERVAL_MS)) {
        _lastDiagTimeMs = millis();
        printDiagnostic();
    }
}

void LineFollower::processFollowingLine(uint32_t elapsedMicros) {
    // 1. Verificação de Cruzamento de 90° (Vários sensores na linha)
    if (_sensors.isIntersection(INTERSECTION_SENSOR_THRESHOLD)) {
        _intersectionConfirmCount++;
        if (_intersectionConfirmCount >= INTERSECTION_CONFIRM_SAMPLES) {
            _state = RobotState::INTERSECTION_CROSSING;
            _stateTimerMs = millis();
            _intersectionConfirmCount = 0;
            return;
        }
    } else {
        _intersectionConfirmCount = 0;
    }

    // 2. Verificação de Perda de Linha / GAP de 10 cm
    if (!_sensors.isLineDetected()) {
        // Reseta o integral e o derivativo ao sair do seguimento: evita "chute" de windup
        // ao reencontrar a linha após um gap ou após a busca inteligente.
        _integral = 0.0f;
        _filteredDerivative = 0.0f;

        if (abs((long)_lastError) < STRAIGHT_ERROR_THRESHOLD) {
            // Estava centralizado em reta: classifica como possível GAP de 10 cm
            _state = RobotState::GAP_CROSSING;
            _stateTimerMs = millis();
            return;
        } else {
            // Estava em curva ou com erro elevado: inicia busca inteligente
            _state = RobotState::SEARCHING_LINE;
            _stateTimerMs = millis();
            return;
        }
    }

    // 3. Cálculo do Controlador PD/PID
    float error = (float)_sensors.getPosition();
    float dt = (float)elapsedMicros / 1000000.0f;

    // Blindagem contra micros() rollover (a cada ~71 min) e divisão por zero
    if (dt <= 0.0001f || dt > 0.1f) {
        dt = 0.002f;
    }

    // Termo Derivativo com filtro passa-baixas
    float rawDerivative = (error - _lastError) / dt;
    _filteredDerivative = (DERIVATIVE_ALPHA * _filteredDerivative) + ((1.0f - DERIVATIVE_ALPHA) * rawDerivative);

    // Termo Integral opcional com anti-windup
    _integral += error * dt;
    _integral = constrain(_integral, -INTEGRAL_WINDUP_LIMIT, INTEGRAL_WINDUP_LIMIT);

    // Cálculo da sinal de correção
    float correction = (_kp * error) + (_kd * _filteredDerivative) + (_ki * _integral);
    correction = constrain(correction, (float)-MAX_CORRECTION, (float)MAX_CORRECTION);
    _lastError = error;

    // 4. Modulação Adaptativa de Velocidade
    float absErr = abs(error);
    int16_t currentBaseSpeed = BASE_SPEED;

    if (absErr < 600.0f) {
        // Trecho Reto: Acelera progressivamente até MAX_SPEED
        float straightFactor = 1.0f - (absErr / 600.0f);
        currentBaseSpeed = BASE_SPEED + (int16_t)((MAX_SPEED - BASE_SPEED) * straightFactor);
    } else {
        // Curva: Reduz velocidade proporcionalmente à severidade do erro
        float curveSeverity = (absErr - 600.0f) / 2900.0f;
        currentBaseSpeed = BASE_SPEED - (int16_t)((BASE_SPEED - MIN_SPEED) * curveSeverity * CURVE_SLOWDOWN_FACTOR);
    }
    currentBaseSpeed = constrain(currentBaseSpeed, MIN_SPEED, MAX_SPEED);

    // 5. Aplicação do Controle Diferencial nos Motores
    int16_t leftSpeed  = currentBaseSpeed + (int16_t)correction;
    int16_t rightSpeed = currentBaseSpeed - (int16_t)correction;

    leftSpeed  = constrain(leftSpeed,  MIN_SPEED, MAX_SPEED);
    rightSpeed = constrain(rightSpeed, MIN_SPEED, MAX_SPEED);

    _motors.setSpeeds(leftSpeed, rightSpeed);
}

void LineFollower::processIntersectionCrossing() {
    // Mantém o robô em linha reta ignorando correções laterais
    _motors.setSpeeds(BASE_SPEED, BASE_SPEED);

    // Condição de saída 1: Tempo máximo de cruzamento atingido
    if (millis() - _stateTimerMs > INTERSECTION_MAX_TIME_MS) {
        _state = RobotState::FOLLOWING_LINE;
        return;
    }

    // Condição de saída 2: Sensores retornam ao padrão normal de seguimento
    if (!_sensors.isIntersection(INTERSECTION_SENSOR_THRESHOLD)) {
        _intersectionExitCount++;
        if (_intersectionExitCount >= INTERSECTION_EXIT_SAMPLES) {
            _state = RobotState::FOLLOWING_LINE;
            _intersectionExitCount = 0;
        }
    } else {
        _intersectionExitCount = 0;
    }
}

void LineFollower::processGapCrossing() {
    // Mantém a velocidade e trajetória constante no GAP de 10 cm
    _motors.setSpeeds(GAP_SPEED, GAP_SPEED);

    // Se qualquer sensor reencontrar a linha preta, retoma o seguimento
    if (_sensors.isLineDetected()) {
        _state = RobotState::FOLLOWING_LINE;
        return;
    }

    // Se exceder o tempo estimado para 10 cm, muda para busca de linha
    if (millis() - _stateTimerMs > GAP_MAX_TIME_MS) {
        _state = RobotState::SEARCHING_LINE;
        _stateTimerMs = millis();
    }
}

void LineFollower::processSearchingLine() {
    // Varredura alternada: se a busca anterior não encontrou a linha no lado
    // inicial, inverte a direção de varredura a cada SEARCH_SWEEP_TIME_MS.
    uint32_t elapsed = millis() - _stateTimerMs;
    bool sweepLeft = ((elapsed / SEARCH_SWEEP_TIME_MS) % 2) == 0;

    if (sweepLeft) {
        // Varredura para a esquerda
        _motors.setSpeeds(-SEARCH_SPEED, SEARCH_SPEED);
    } else {
        // Varredura para a direita
        _motors.setSpeeds(SEARCH_SPEED, -SEARCH_SPEED);
    }

    // Se reencontrar a linha, retoma o seguimento normal
    if (_sensors.isLineDetected()) {
        _state = RobotState::FOLLOWING_LINE;
        return;
    }

    // Se esgotar o tempo limite de busca, para por segurança
    if (elapsed > SEARCH_MAX_TIME_MS) {
        _state = RobotState::STOPPED;
    }
}

void LineFollower::processStopped() {
    _motors.stop();
}

void LineFollower::checkSafetyTimeout() {
    if (_state == RobotState::FOLLOWING_LINE ||
        _state == RobotState::INTERSECTION_CROSSING ||
        _state == RobotState::GAP_CROSSING ||
        _state == RobotState::SEARCHING_LINE) {

        if (millis() - _startTimeMs >= MAX_EXECUTION_TIME_MS) {
            _state = RobotState::STOPPED;
            _motors.brake();
        }
    }
}

void LineFollower::printDiagnostic() {
    if (!Serial) return;

    // Formatação conforme especificação:
    // SENSORS: 00011000
    // ACTIVE: 2
    // POSITION: 0
    // ERROR: 0
    // LEFT_PWM: 135
    // RIGHT_PWM: 135
    // STATE: FOLLOWING_LINE

    uint8_t bitmask = _sensors.getRawBitmask();
    char bitStr[9];
    for (int i = 7; i >= 0; i--) {
        bitStr[7 - i] = (bitmask & (1 << i)) ? '1' : '0';
    }
    bitStr[8] = '\0';

    Serial.print(F("SENSORS: "));
    Serial.println(bitStr);

    Serial.print(F("ACTIVE: "));
    Serial.println(_sensors.getActiveCount());

    Serial.print(F("POSITION: "));
    Serial.println(_sensors.getPosition());

    Serial.print(F("ERROR: "));
    Serial.println((int16_t)_lastError);

    Serial.print(F("LEFT_PWM: "));
    Serial.println(_motors.getLeftPWM());

    Serial.print(F("RIGHT_PWM: "));
    Serial.println(_motors.getRightPWM());

    Serial.print(F("STATE: "));
    Serial.println(getRobotStateName(_state));

    Serial.print(F("LINE: "));
    Serial.println(_sensors.isLineDetected() ? F("YES") : F("NO"));

    Serial.println(F("--------------------"));
}