#ifndef LINE_FOLLOWER_H
#define LINE_FOLLOWER_H

#include <Arduino.h>
#include "Config.h"
#include "RobotState.h"
#include "LineSensors.h"
#include "MotorDriver.h"

/**
 * @class LineFollower
 * @brief Orquestrador principal do robô seguidor de linha (FSM, PID, diagnósticos, timers).
 */
class LineFollower {
public:
    LineFollower();

    /**
     * @brief Inicializa o robô, motores, sensores e pinos de controle.
     */
    void begin();

    /**
     * @brief Função de atualização executada repetidamente no loop() principal.
     */
    void update();

    /**
     * @brief Retorna o estado atual da máquina de estados do robô.
     */
    RobotState getState() const;

    /**
     * @brief Habilita ou desabilita a transmissão de dados de diagnóstico pela Serial.
     */
    void setDiagnosticEnabled(bool enabled);

    /**
     * @brief Permite ajustar os ganhos PID em tempo de execução.
     */
    void setPIDGains(float kp, float kd, float ki);

    /**
     * @brief Exibe as métricas de diagnóstico na Serial se habilitado.
     */
    void printDiagnostic();

private:
    LineSensors _sensors;
    MotorDriver _motors;
    RobotState  _state;

    float _kp;
    float _kd;
    float _ki;

    float _lastError;
    float _filteredDerivative;
    float _integral;

    uint32_t _lastLoopTimeMicros;
    uint32_t _startTimeMs;
    uint32_t _stateTimerMs;
    uint32_t _lastDiagTimeMs;
    uint32_t _countdownLedTimerMs;

    uint8_t _intersectionConfirmCount;
    uint8_t _intersectionExitCount;
    bool    _diagnosticEnabled;
    bool    _ledState;

    void processWaitingStart();
    void processStartCountdown();
    void processFollowingLine(uint32_t elapsedMicros);
    void processIntersectionCrossing();
    void processGapCrossing();
    void processSearchingLine();
    void processStopped();
    void checkSafetyTimeout();
};

#endif // LINE_FOLLOWER_H
