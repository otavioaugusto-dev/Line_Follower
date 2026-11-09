#include <Arduino.h>
#include "Config.h"
#include "LineFollower.h"

/**
 * @file main.cpp
 * @brief Ponto de entrada principal do firmware do seguidor de linha competitivo.
 */

// Instância global do controlador do robô
LineFollower robot;

void setup() {
    robot.begin();
}

void loop() {
    robot.update();
}
