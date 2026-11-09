#include "LineSensors.h"

LineSensors::LineSensors()
    : _rawBitmask(0), _activeCount(0), _position(0), _lastValidPosition(0), _lineDetected(false) {
    for (uint8_t i = 0; i < NUM_SENSORS; i++) {
        _states[i] = 0;
    }
}

void LineSensors::begin() {
    // Configura os 8 pinos dos sensores como entrada digital
    for (uint8_t i = 0; i < NUM_SENSORS; i++) {
        pinMode(_pins[i], INPUT);
    }

    // Habilita os emissores IR do módulo HY-S301
    pinMode(PIN_HY301_IR_EN, OUTPUT);
    digitalWrite(PIN_HY301_IR_EN, HIGH);
}

void LineSensors::update() {
    _activeCount = 0;
    int32_t weightedSum = 0;
    _rawBitmask = 0;

    for (uint8_t i = 0; i < NUM_SENSORS; i++) {
        // Leitura digital direta do pino
        uint8_t rawReading = digitalRead(_pins[i]);

        // Mapeia de acordo com a inversão física se configurada
        uint8_t idx = SENSOR_INVERT_ORDER ? (NUM_SENSORS - 1 - i) : i;

        // Identifica se o sensor está sobre a linha preta
        if (rawReading == BLACK_LINE_STATE) {
            _states[idx] = 1;
            _activeCount++;
            weightedSum += SENSOR_WEIGHTS[idx];
        } else {
            _states[idx] = 0;
        }
    }

    // Constrói a máscara binária (Bit 7 = S1, Bit 0 = S8)
    for (uint8_t i = 0; i < NUM_SENSORS; i++) {
        if (_states[i] == 1) {
            _rawBitmask |= (1 << (7 - i));
        }
    }

    // Cálculo da posição ponderada do centro da linha preta
    if (_activeCount > 0) {
        _position = (int16_t)(weightedSum / _activeCount);
        _lastValidPosition = _position;
        _lineDetected = true;
    } else {
        // NENHUM SENSOR NA LINHA: Mantém a última posição conhecida para decisões de GAP / SEARCH
        _position = _lastValidPosition;
        _lineDetected = false;
    }
}

uint8_t LineSensors::getRawBitmask() const {
    return _rawBitmask;
}

uint8_t LineSensors::getActiveCount() const {
    return _activeCount;
}

int16_t LineSensors::getPosition() const {
    return _position;
}

int16_t LineSensors::getLastValidPosition() const {
    return _lastValidPosition;
}

bool LineSensors::isLineDetected() const {
    return _lineDetected;
}

bool LineSensors::isIntersection(uint8_t threshold) const {
    return (_activeCount >= threshold);
}

bool LineSensors::isAllWhite() const {
    return (_activeCount == 0);
}

bool LineSensors::isAllBlack() const {
    return (_activeCount == NUM_SENSORS);
}

uint8_t LineSensors::getSensorState(uint8_t index) const {
    if (index < NUM_SENSORS) {
        return _states[index];
    }
    return 0;
}
