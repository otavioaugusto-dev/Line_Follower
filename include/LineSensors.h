#ifndef LINE_SENSORS_H
#define LINE_SENSORS_H

#include <Arduino.h>
#include "Config.h"

/**
 * @class LineSensors
 * @brief Gerencia a leitura, filtragem e cálculo de posição do módulo HY-S301 de 8 sensores digitais IR.
 */
class LineSensors {
public:
    LineSensors();

    /**
     * @brief Inicializa os pinos dos sensores e habilita o emissor IR.
     */
    void begin();

    /**
     * @brief Executa a leitura simultânea dos 8 canais e calcula a posição ponderada da linha.
     */
    void update();

    /**
     * @brief Retorna a máscara binária dos 8 sensores (Bit 7 = S1 / Extremo Esquerdo, Bit 0 = S8 / Extremo Direito).
     */
    uint8_t getRawBitmask() const;

    /**
     * @brief Retorna a quantidade de sensores que identificam a linha preta no ciclo atual.
     */
    uint8_t getActiveCount() const;

    /**
     * @brief Retorna o deslocamento lateral calculado da linha em relação ao centro do robô (-3500 a +3500).
     */
    int16_t getPosition() const;

    /**
     * @brief Retorna a última posição válida calculada quando a linha estava visível.
     */
    int16_t getLastValidPosition() const;

    /**
     * @brief Indica se ao menos um sensor está detectando a linha preta.
     */
    bool isLineDetected() const;

    /**
     * @brief Verifica se a contagem de sensores ativos atinge o limiar de cruzamento.
     * @param threshold Limiar mínimo de sensores ativos (ex: 5).
     */
    bool isIntersection(uint8_t threshold = INTERSECTION_SENSOR_THRESHOLD) const;

    /**
     * @brief Verifica se todos os sensores estão identificando fundo branco (fora da linha).
     */
    bool isAllWhite() const;

    /**
     * @brief Verifica se todos os 8 sensores estão identificando linha preta simultaneamente.
     */
    bool isAllBlack() const;

    /**
     * @brief Retorna o estado lógico individual de um sensor (0 = branco, 1 = linha preta).
     * @param index Índice do sensor (0 a 7).
     */
    uint8_t getSensorState(uint8_t index) const;

private:
    const uint8_t _pins[NUM_SENSORS] = {
        PIN_SENSOR_1, PIN_SENSOR_2, PIN_SENSOR_3, PIN_SENSOR_4,
        PIN_SENSOR_5, PIN_SENSOR_6, PIN_SENSOR_7, PIN_SENSOR_8
    };

    uint8_t _states[NUM_SENSORS];     ///< Estado lógico normalizado (1 = sobre a linha preta, 0 = no fundo branco)
    uint8_t _rawBitmask;             ///< Representação binária dos 8 canais
    uint8_t _activeCount;            ///< Quantidade de sensores ativos na linha preta
    int16_t _position;               ///< Posição atual ponderada (-3500 a +3500)
    int16_t _lastValidPosition;      ///< Memória da última posição antes de perder a linha
    bool    _lineDetected;           ///< Flag de linha visível
};

#endif // LINE_SENSORS_H
