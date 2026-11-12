#ifndef CONFIG_H
#define CONFIG_H

#include <Arduino.h>

/**
 * @file Config.h
 * @brief Configurações globais, pinagem e parâmetros do Seguidor de Linha Competitivo.
 *
 * ATENÇÃO: Todos os parâmetros de calibração mecânica, elétrica e de controle
 * estão centralizados neste arquivo para facilitar ajustes em pista.
 */

// ============================================================================
// 1. MAPEAMENTO DE PINOS (ARDUINO NANO ATMEGA328P)
// ============================================================================

// Ponte H Dual - Motores
#define PIN_MOTOR_LEFT_PWM      5   // ENA - PWM Motor Esquerdo (Pino PWM Timer 0)
#define PIN_MOTOR_RIGHT_PWM     6   // ENB - PWM Motor Direito (Pino PWM Timer 0)
#define PIN_MOTOR_LEFT_IN1      7   // IN1 - Sentido Motor Esquerdo
#define PIN_MOTOR_LEFT_IN2      8   // IN2 - Sentido Motor Esquerdo
#define PIN_MOTOR_RIGHT_IN3     9   // IN3 - Sentido Motor Direito
#define PIN_MOTOR_RIGHT_IN4     10  // IN4 - Sentido Motor Direito

// Módulo HY-S301 - Sensores Ópticos Digitais (D1 a D8, da Esquerda para a Direita)
#define PIN_SENSOR_1            2   // Sensor 1 - Extremo Esquerdo
#define PIN_SENSOR_2            3   // Sensor 2
#define PIN_SENSOR_3            4   // Sensor 3
#define PIN_SENSOR_4            A0  // Sensor 4 (Entrada Digital A0)
#define PIN_SENSOR_5            A1  // Sensor 5 (Entrada Digital A1)
#define PIN_SENSOR_6            A2  // Sensor 6 (Entrada Digital A2)
#define PIN_SENSOR_7            A3  // Sensor 7 (Entrada Digital A3)
#define PIN_SENSOR_8            A4  // Sensor 8 - Extremo Direito (Entrada Digital A4)

// Controle de Periféricos
#define PIN_HY301_IR_EN         12  // Habilitação dos leds IR do HY-S301 (HIGH = Ligado)

// Pinagem preservada para diagnósticos via comunicação Serial:
// D0 (RX) e D1 (TX)

// ============================================================================
// 2. CONFIGURAÇÕES DOS SENSORES (HY-S301)
// ============================================================================

#define NUM_SENSORS             8

// Polaridade das leituras digitais:
// Superfície vinílica branca = HIGH (1)
// Linha preta = LOW (0)
#define BLACK_LINE_STATE        LOW
#define WHITE_SURFACE_STATE     HIGH

// Inversão lógica da ordem dos sensores (caso o módulo seja instalado invertido 180°):
// false: S1=Extremo Esquerdo ... S8=Extremo Direito
// true:  S1=Extremo Direito ... S8=Extremo Esquerdo
#define SENSOR_INVERT_ORDER     false

// Pesos posicionais para o cálculo da média ponderada (em micrômetros/unidades relativas):
// Esquerda é negativa (-3500 a -500), Direita é positiva (+500 a +3500)
const int16_t SENSOR_WEIGHTS[NUM_SENSORS] = {
    -3500, -2500, -1500, -500,
      500,  1500,  2500, 3500
};

// ============================================================================
// 3. CONFIGURAÇÕES DOS MOTORES E PONTE H
// ============================================================================

// Inversão de sentido de rotação individual dos motores
#define MOTOR_LEFT_INVERT       false
#define MOTOR_RIGHT_INVERT      true

// Fator de compensação individual para equilibrar diferenças mecânicas entre motores (0.50 a 1.00)
#define MOTOR_LEFT_TRIM         1.00f
#define MOTOR_RIGHT_TRIM        1.00f

// Limites de velocidade (PWM: 0 a 255)
// IMPORTANTE: Alimentação dos motores limitada a 6 V nominal!
#define MIN_SPEED               40    // Velocidade mínima para evitar stall/bloqueio dos motores
#define BASE_SPEED              130   // Velocidade base de cruzeiro
#define MAX_SPEED               210   // Velocidade máxima em retas longas
#define GAP_SPEED               110   // Velocidade constante durante travessia de gap
#define SEARCH_SPEED            90    // Velocidade reduzida durante busca de linha

// Redução de velocidade em curvas (0.0 = sem redução, 1.0 = reduz até MIN_SPEED)
#define CURVE_SLOWDOWN_FACTOR   0.50f

// Limite máximo da sinal de correção PD/PID aplicado aos motores
#define MAX_CORRECTION          160

// Taxa limite de rampa de aceleração por ciclo de controle (evita patinagem e picos de corrente)
#define ACCEL_RAMP_STEP         15

// ============================================================================
// 4. PARÂMETROS DO CONTROLADOR PD/PID
// ============================================================================

// Ganhos padrão do controlador (Valores iniciais conservadores - ajustar em pista)
#define DEFAULT_KP              0.042f    // Ganho Proporcional
#define DEFAULT_KD              0.220f    // Ganho Derivativo
#define DEFAULT_KI              0.000f    // Ganho Integral (zerado por padrão para evitar oscilações)

// Filtro passa-baixas no termo derivativo para atenuação de ruído digital de amostragem
// 0.0 = sem filtragem (usa D puro); 1.0 = mantém valor anterior totalmente
#define DERIVATIVE_ALPHA        0.65f

// Limite de estouro do termo integral para anti-windup (evita "chute" ao sair de gaps/cruzamentos)
#define INTEGRAL_WINDUP_LIMIT   800.0f

// Tempo mínimo válido de ciclo de controle (proteção contra micros() rollover/divisão por zero)
#define MIN_CYCLE_DT_US         100       // 100 us
#define MAX_CYCLE_DT_US         100000    // 100 ms (ciclo inválido se ultrapassar)

// ============================================================================
// 5. DETECÇÃO E TRAVESSIA DE CRUZAMENTOS (90 GRAUS)
// ============================================================================

// Número mínimo de sensores sobre a linha preta para caracterizar cruzamento
#define INTERSECTION_SENSOR_THRESHOLD   5

// Número de amostras consecutivas para confirmar entrada e saída de cruzamento
#define INTERSECTION_CONFIRM_SAMPLES    2
#define INTERSECTION_EXIT_SAMPLES       2

// Tempo máximo em milissegundos que o robô pode permanecer em modo de cruzamento
#define INTERSECTION_MAX_TIME_MS        400

// ============================================================================
// 6. TRAVESSIA DE GAPS (INTERRUPÇÕES DE ATÉ 10 CM)
// ============================================================================

// Limite do erro lateral antes do gap para considerar que o robô estava em reta
#define STRAIGHT_ERROR_THRESHOLD        900

// Estimativa de travessia do gap de 10 cm (100 mm):
// Tempo estimado = (100 mm / VELOCIDADE_ESTIMADA_MM_S) * 1000 + margem de segurança
#define MAX_GAP_DISTANCE_MM             100
#define MEASURED_ROBOT_SPEED_MM_S       500.0f
#define GAP_MAX_TIME_MS                 260  // Milissegundos limite para manter trajetória no gap

// ============================================================================
// 7. RECUPERAÇÃO E BUSCA DE LINHA (SEARCHING_LINE)
// ============================================================================

// Tempo máximo permitido para buscar a linha antes de parar motores com segurança
#define SEARCH_MAX_TIME_MS              1000

// Tempo de varredura para cada lado durante a busca
#define SEARCH_SWEEP_TIME_MS            300

// ============================================================================
// 8. TEMPO DE COMPETIÇÃO E CONTAGEM REGRESSIVA
// ============================================================================

// Duração da contagem regressiva antes da largada (milissegundos)
#define COUNTDOWN_DURATION_MS           3000

// Limite absoluto de tempo de execução da prova (Regulamento: 60s max)
// Margem de segurança de 1s para garantir desligamento dos motores
#define MAX_EXECUTION_TIME_MS           59000UL

// Período alvo do ciclo de controle da FSM/PID (microssegundos)
// 2000 us = 2.0 ms -> Frequência de amostragem de 500 Hz
#define LOOP_TARGET_MICROS              2000

// ============================================================================
// 9. DIAGNÓSTICO SERIAL E TELEMETRIA
// ============================================================================

#ifndef CONFIG_MODE_DIAGNOSTIC_DEFAULT
#define CONFIG_MODE_DIAGNOSTIC_DEFAULT   0  // 0 = Competição (Serial desativada), 1 = Diagnóstico ativo
#endif

#define DIAGNOSTIC_BAUD_RATE            115200
#define DIAGNOSTIC_INTERVAL_MS          100 // Intervalo de envio serial em modo diagnóstico (10 Hz)

#endif // CONFIG_H
