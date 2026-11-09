#ifndef MOTOR_DRIVER_H
#define MOTOR_DRIVER_H

#include <Arduino.h>
#include "Config.h"

/**
 * @class MotorDriver
 * @brief Interface de controle para a Ponte H Dupla (ENA/ENB, IN1/IN2/IN3/IN4) dos motores DC.
 */
class MotorDriver {
public:
    MotorDriver();

    /**
     * @brief Configura os pinos dos motores como saídas e inicializa em estado parado.
     */
    void begin();

    /**
     * @brief Define as velocidades dos motores com compensação, inversão e rampa de aceleração.
     * @param leftSpeed Velocidade desejada para o motor esquerdo (-255 a +255).
     * @param rightSpeed Velocidade desejada para o motor direito (-255 a +255).
     */
    void setSpeeds(int16_t leftSpeed, int16_t rightSpeed);

    /**
     * @brief Define a velocidade do motor esquerdo individualmente (-255 a +255).
     */
    void setLeftMotor(int16_t speed);

    /**
     * @brief Define a velocidade do motor direito individualmente (-255 a +255).
     */
    void setRightMotor(int16_t speed);

    /**
     * @brief Parada suave (desativa a alimentação dos motores / inércia livre).
     */
    void stop();

    /**
     * @brief Frenagem ativa dos motores (curto-circuito nos enrolamentos via ponte H).
     */
    void brake();

    /**
     * @brief Retorna a velocidade PWM atual aplicada ao motor esquerdo.
     */
    int16_t getLeftPWM() const;

    /**
     * @brief Retorna a velocidade PWM atual aplicada ao motor direito.
     */
    int16_t getRightPWM() const;

private:
    int16_t _currentLeftPWM;    ///< PWM atual aplicado no motor esquerdo
    int16_t _currentRightPWM;   ///< PWM atual aplicado no motor direito

    /**
     * @brief Aplica diretamente a tensão/sentido nos pinos da ponte H de um motor.
     */
    void driveMotorPins(uint8_t pwmPin, uint8_t in1Pin, uint8_t in2Pin, int16_t speed, bool invert);
};

#endif // MOTOR_DRIVER_H
