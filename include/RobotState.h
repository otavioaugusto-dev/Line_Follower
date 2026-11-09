#ifndef ROBOT_STATE_H
#define ROBOT_STATE_H

/**
 * @file RobotState.h
 * @brief Definição dos estados da Máquina de Estados Finitos (FSM) do robô seguidor de linha.
 */

enum class RobotState {
    WAITING_START,          ///< Aguardando sinal de início (botão ou tempo)
    START_COUNTDOWN,        ///< Contagem regressiva com indicação por LED
    FOLLOWING_LINE,         ///< Controle principal PD/PID de seguimento de linha
    INTERSECTION_CROSSING,  ///< Travessia reta de cruzamento de 90 graus
    GAP_CROSSING,           ///< Travessia de interrupção da linha (gap de até 10 cm)
    SEARCHING_LINE,         ///< Busca inteligente de linha após perda de trajetória
    STOPPED                 ///< Parado (tempo limite atingido, fim da prova ou emergência)
};

/**
 * @brief Retorna o nome textual do estado para exibições e diagnósticos via Serial.
 * @param state Estado atual do robô.
 * @return String constante com o nome do estado.
 */
inline const char* getRobotStateName(RobotState state) {
    switch (state) {
        case RobotState::WAITING_START:         return "WAITING_START";
        case RobotState::START_COUNTDOWN:       return "START_COUNTDOWN";
        case RobotState::FOLLOWING_LINE:        return "FOLLOWING_LINE";
        case RobotState::INTERSECTION_CROSSING: return "INTERSECTION_CROSSING";
        case RobotState::GAP_CROSSING:          return "GAP_CROSSING";
        case RobotState::SEARCHING_LINE:        return "SEARCHING_LINE";
        case RobotState::STOPPED:               return "STOPPED";
        default:                                return "UNKNOWN";
    }
}

#endif // ROBOT_STATE_H
