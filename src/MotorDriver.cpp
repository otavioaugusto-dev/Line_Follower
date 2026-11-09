#include "MotorDriver.h"

MotorDriver::MotorDriver() : _currentLeftPWM(0), _currentRightPWM(0) {}

void MotorDriver::begin() {
    pinMode(PIN_MOTOR_LEFT_PWM, OUTPUT);
    pinMode(PIN_MOTOR_RIGHT_PWM, OUTPUT);
    pinMode(PIN_MOTOR_LEFT_IN1, OUTPUT);
    pinMode(PIN_MOTOR_LEFT_IN2, OUTPUT);
    pinMode(PIN_MOTOR_RIGHT_IN3, OUTPUT);
    pinMode(PIN_MOTOR_RIGHT_IN4, OUTPUT);

    stop();
}

void MotorDriver::setSpeeds(int16_t leftSpeed, int16_t rightSpeed) {
    // Aplica trim de compensação mecânica individual
    int16_t targetLeft = (int16_t)(leftSpeed * MOTOR_LEFT_TRIM);
    int16_t targetRight = (int16_t)(rightSpeed * MOTOR_RIGHT_TRIM);

    // Rampa de aceleração/desaceleração para o motor esquerdo
    int16_t diffL = targetLeft - _currentLeftPWM;
    if (diffL > ACCEL_RAMP_STEP) {
        _currentLeftPWM += ACCEL_RAMP_STEP;
    } else if (diffL < -ACCEL_RAMP_STEP) {
        _currentLeftPWM -= ACCEL_RAMP_STEP;
    } else {
        _currentLeftPWM = targetLeft;
    }

    // Rampa de aceleração/desaceleração para o motor direito
    int16_t diffR = targetRight - _currentRightPWM;
    if (diffR > ACCEL_RAMP_STEP) {
        _currentRightPWM += ACCEL_RAMP_STEP;
    } else if (diffR < -ACCEL_RAMP_STEP) {
        _currentRightPWM -= ACCEL_RAMP_STEP;
    } else {
        _currentRightPWM = targetRight;
    }

    // Atualiza saídas físicas da Ponte H
    driveMotorPins(PIN_MOTOR_LEFT_PWM, PIN_MOTOR_LEFT_IN1, PIN_MOTOR_LEFT_IN2, _currentLeftPWM, MOTOR_LEFT_INVERT);
    driveMotorPins(PIN_MOTOR_RIGHT_PWM, PIN_MOTOR_RIGHT_IN3, PIN_MOTOR_RIGHT_IN4, _currentRightPWM, MOTOR_RIGHT_INVERT);
}

void MotorDriver::setLeftMotor(int16_t speed) {
    setSpeeds(speed, _currentRightPWM);
}

void MotorDriver::setRightMotor(int16_t speed) {
    setSpeeds(_currentLeftPWM, speed);
}

void MotorDriver::stop() {
    _currentLeftPWM = 0;
    _currentRightPWM = 0;

    digitalWrite(PIN_MOTOR_LEFT_IN1, LOW);
    digitalWrite(PIN_MOTOR_LEFT_IN2, LOW);
    analogWrite(PIN_MOTOR_LEFT_PWM, 0);

    digitalWrite(PIN_MOTOR_RIGHT_IN3, LOW);
    digitalWrite(PIN_MOTOR_RIGHT_IN4, LOW);
    analogWrite(PIN_MOTOR_RIGHT_PWM, 0);
}

void MotorDriver::brake() {
    _currentLeftPWM = 0;
    _currentRightPWM = 0;

    digitalWrite(PIN_MOTOR_LEFT_IN1, HIGH);
    digitalWrite(PIN_MOTOR_LEFT_IN2, HIGH);
    analogWrite(PIN_MOTOR_LEFT_PWM, 255);

    digitalWrite(PIN_MOTOR_RIGHT_IN3, HIGH);
    digitalWrite(PIN_MOTOR_RIGHT_IN4, HIGH);
    analogWrite(PIN_MOTOR_RIGHT_PWM, 255);
}

int16_t MotorDriver::getLeftPWM() const {
    return _currentLeftPWM;
}

int16_t MotorDriver::getRightPWM() const {
    return _currentRightPWM;
}

void MotorDriver::driveMotorPins(uint8_t pwmPin, uint8_t in1Pin, uint8_t in2Pin, int16_t speed, bool invert) {
    if (invert) {
        speed = -speed;
    }

    speed = constrain(speed, -255, 255);

    if (speed > 0) {
        digitalWrite(in1Pin, HIGH);
        digitalWrite(in2Pin, LOW);
        analogWrite(pwmPin, speed);
    } else if (speed < 0) {
        digitalWrite(in1Pin, LOW);
        digitalWrite(in2Pin, HIGH);
        analogWrite(pwmPin, -speed);
    } else {
        digitalWrite(in1Pin, LOW);
        digitalWrite(in2Pin, LOW);
        analogWrite(pwmPin, 0);
    }
}
