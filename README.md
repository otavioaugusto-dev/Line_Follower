
# Line Follower Robot — Competitive Robotics

<div align="center">

  <strong>High-Performance Autonomous Line Following Robot</strong>

  <em>Embedded Systems · Robotics · Control Systems · PlatformIO</em>

  <br><br>

  <img src="https://img.shields.io/badge/PlatformIO-FF7F00?style=for-the-badge&logo=platformio&logoColor=white" alt="PlatformIO"/>
  <img src="https://img.shields.io/badge/Arduino_Nano-00979D?style=for-the-badge&logo=arduino&logoColor=white" alt="Arduino Nano"/>
  <img src="https://img.shields.io/badge/C%2B%2B-00599C?style=for-the-badge&logo=cplusplus&logoColor=white" alt="C++"/>
  <img src="https://img.shields.io/badge/Embedded_Systems-333333?style=for-the-badge" alt="Embedded Systems"/>

</div>

---

## Overview

This project presents an autonomous differential-drive line follower robot developed for competitive mobile robotics.

Built around the **Arduino Nano (ATmega328P)**, the system combines digital infrared sensing, real-time motor control, adaptive speed management, and intelligent line recovery strategies.

The firmware is designed to prioritize tracking accuracy, stability, and high-speed performance while handling challenging track conditions.

**Main competition objective:** maximize the distance traveled and minimize failures and penalties on a 60-meter track within a 60-second time limit.

## Key Features

- **8-channel infrared sensing:** HY-S301 digital reflective sensor array.
- **PD/PID-based control:** weighted line-position estimation and error correction.
- **Differential drive:** independent PWM control of two DC motors.
- **Adaptive speed control:** acceleration on straights and automatic slowdown on sharp curves.
- **Intersection handling:** dedicated logic for 90-degree crossings.
- **Gap traversal:** inertia-guided navigation across line interruptions of up to 10 cm on straight sections.
- **Intelligent line recovery:** active searching based on recent trajectory history.
- **Non-blocking architecture:** finite state machine using `micros()` and `millis()`.
- **Serial diagnostics:** sensor monitoring and firmware calibration.

## Hardware Architecture

### Main Components

| Component | Specification | Quantity |
|---|---|---:|
| Microcontroller | Arduino Nano — ATmega328P, 16 MHz | 1 |
| Line sensor | HY-S301, 8-channel digital IR | 1 |
| Motor driver | Dual H-Bridge (L298N, TB6612FNG or MX1508) | 1 |
| DC motors | 6 V geared motors | 2 |
| Drive wheels | High-grip wheels | 2 |
| Rear support | Caster wheel | 1 |
| Main power | 2S LiPo / 2-cell Li-Ion, 7.4 V | 1 |
| Motor regulator | DC-DC buck converter, adjusted to 6 V | 1 |
| Logic regulator | Dedicated 5 V supply | 1 |
| Start button | Momentary push button | 1 |

### System Specifications

| Parameter | Value |
|---|---|
| Controller | ATmega328P |
| Clock frequency | 16 MHz |
| Operating logic voltage | 5 V |
| Flash memory | 32 KB |
| SRAM | 2 KB |
| Sensor channels | 8 |
| Motor configuration | Differential drive |
| Nominal motor voltage | 6 V |
| Sensor sampling target | Approximately 500 Hz |
| Maximum track gap target | 10 cm |
| Competition track | 60 m |
| Competition time limit | 60 s |

## Pin Mapping

### Arduino Nano Connections

| Function | Arduino Pin | Type |
|---|---|---|
| Left motor PWM (ENA) | D5 | PWM |
| Right motor PWM (ENB) | D6 | PWM |
| Left motor IN1 | D7 | Digital Output |
| Left motor IN2 | D8 | Digital Output |
| Right motor IN3 | D9 | Digital Output |
| Right motor IN4 | D10 | Digital Output |
| IR Sensor S1 | D2 | Digital Input |
| IR Sensor S2 | D3 | Digital Input |
| IR Sensor S3 | D4 | Digital Input |
| IR Sensor S4 | A0 | Digital Input |
| IR Sensor S5 | A1 | Digital Input |
| IR Sensor S6 | A2 | Digital Input |
| IR Sensor S7 | A3 | Digital Input |
| IR Sensor S8 | A4 | Digital Input |
| Start Button | D11 | Digital Input |
| IR Enable | D12 | Digital Output |
| Status LED | D13 | Digital Output |
| Serial Communication | D0 / D1 | UART |

**Important:** All system components must share a common ground (GND), including the battery, voltage regulators, motor driver, Arduino, and sensor array.

## Firmware Architecture

The control system is organized around a non-blocking finite state machine (FSM).

Its main responsibilities include:

1. Reading and processing the eight infrared sensors.
2. Estimating the line's lateral position.
3. Calculating tracking error.
4. Applying PD/PID correction.
5. Adjusting left and right motor speeds.
6. Managing acceleration and deceleration.
7. Detecting intersections and track gaps.
8. Recovering the line when it is temporarily lost.

### Control Strategy

The robot uses weighted sensor-position estimation to calculate the tracking error.

The controller then adjusts the motor outputs according to the proportional and derivative terms:

- **Proportional gain (Kp):** responds to the current tracking error.
- **Derivative gain (Kd):** reduces oscillations by responding to error variation.
- **Derivative filtering:** smooths high-frequency noise in sensor readings.

### Adaptive Speed Management

Speed is dynamically adjusted according to the estimated line position.

| Track condition | Behavior |
|---|---|
| Straight sections | Accelerates toward maximum PWM |
| Mild curves | Maintains base speed |
| Sharp curves | Reduces speed according to tracking error |
| Sudden changes | Uses acceleration ramp to avoid abrupt motor commands |

Reference configuration values from the original firmware documentation:

| Parameter | Reference value |
|---|---:|
| Maximum PWM | 210 |
| Base PWM | 130 |
| Minimum PWM | 40 |
| Straight-section error threshold | 600 |
| Sharp-curve error threshold | 1500 |
| Derivative filter alpha | 0.65 |
| Target control interval | 2 ms |

Actual performance depends on mechanical construction, sensor calibration, motor characteristics, and track conditions.

## Mechanical Design

Recommended construction parameters:

| Parameter | Dimension |
|---|---:|
| Chassis length | 240 mm |
| Chassis width | 160 mm |
| Wheel track width | 105 mm |
| Drive axle to rear caster | 160 mm |
| Sensor-to-drive-axle distance | 120–150 mm |
| Sensor height above track | 3–6 mm |

The sensor array should be mounted at the front of the robot, perpendicular to its direction of travel.

Battery placement should remain close to the drive axle to improve traction and weight distribution.

## Power Management

The robot uses separate regulated power paths for the motors and control electronics.

```text
                 7.4V Battery
                      |
                 Main Switch
                      |
             +--------+--------+
             |                 |
          Buck 6V            Buck 5V
             |                 |
         Motor Driver      Arduino Nano
             |                 |
          DC Motors        HY-S301 Sensor
```

**Electrical safety considerations:**

- Never power the motors directly from the Arduino 5V or VIN pins.
- Adjust the motor regulator to a maximum of 6.0 V for the specified motors.
- Use a motor driver capable of handling the motors' peak and stall current.
- Add decoupling capacitors to reduce electrical noise.
- Ensure a common ground across all modules.

## Development Environment

| Tool | Purpose |
|---|---|
| PlatformIO | Firmware build and upload |
| Visual Studio Code | Development environment |
| Arduino Framework | Microcontroller programming |
| C++ | Embedded firmware |
| Serial Monitor | Diagnostics and calibration |

### Project Configuration

The project uses PlatformIO with an Arduino Nano target based on the ATmega328P.

The relevant environment configuration should be defined in `platformio.ini`.

## Getting Started

### Requirements

- Visual Studio Code
- PlatformIO IDE extension
- Arduino Nano (ATmega328P)
- USB cable compatible with the board
- Correct USB-to-serial drivers

### Build Firmware

Open the project directory in Visual Studio Code with PlatformIO installed.

Alternatively, use the PlatformIO CLI:

```bash
pio run
```

### Upload Firmware

For Arduino Nano boards with the new bootloader:

```bash
pio run -e nanoatmega328new -t upload
```

For boards using the old bootloader:

```bash
pio run -e nanoatmega328old -t upload
```

### Serial Diagnostics

Open the PlatformIO Serial Monitor at:

```text
115200 baud
```

For initial diagnostics, enable the diagnostic mode in `Config.h`:

```cpp
#define CONFIG_MODE_DIAGNOSTIC_DEFAULT 1
```

After calibration, disable diagnostic mode for normal operation:

```cpp
#define CONFIG_MODE_DIAGNOSTIC_DEFAULT 0
```

## Calibration Procedure

1. **Power verification:** check the regulated 6 V motor supply and 5 V logic supply.
2. **Motor direction:** verify the forward direction of both motors.
3. **Sensor calibration:** adjust the HY-S301 potentiometers over the white and black surfaces.
4. **Sensor polarity:** verify that `BLACK_LINE_STATE` matches the actual sensor output.
5. **Serial diagnostics:** inspect the eight-channel sensor mask. A centered robot should produce the expected central sensor pattern, such as `00011000`.
6. **Motor compensation:** adjust `MOTOR_LEFT_TRIM` and `MOTOR_RIGHT_TRIM` if the robot drifts on straight sections.
7. **Gap calibration:** adjust `GAP_MAX_TIME_MS` based on the measured robot speed.
8. **Competition configuration:** disable diagnostic mode and rebuild the firmware.

## PD Controller Tuning

A recommended tuning sequence:

**1. Proportional gain (Kp)**

- Start with `Kd = 0`.
- Increase Kp gradually until the robot follows the sharpest curves.
- Excessive oscillation indicates that Kp may be too high.

**2. Derivative gain (Kd)**

- Increase Kd gradually to damp oscillations.
- Excessive derivative gain can amplify sensor noise and produce high-frequency vibration.

**3. Derivative filtering**

The reference configuration uses:

```cpp
DERIVATIVE_ALPHA = 0.65f
```

This applies low-pass filtering to the derivative term.

## Troubleshooting

| Problem | Possible cause | Recommended action |
|---|---|---|
| Lateral oscillation | Excessive Kp or insufficient Kd | Reduce Kp and tune Kd |
| Robot turns in the wrong direction | Motor inversion or sensor order | Check motor and sensor configuration |
| Robot loses the line on sharp curves | Excessive speed or insufficient correction | Reduce base speed and tune Kp |
| Incorrect intersection behavior | Detection threshold | Adjust intersection detection parameters |
| Robot fails to cross gaps | Incorrect gap timeout | Recalibrate `GAP_MAX_TIME_MS` |
| Arduino resets during acceleration | Power supply voltage drop | Improve power distribution and filtering |
| Robot drifts on straight sections | Different motor speeds | Adjust motor trim values |

## Pre-Competition Checklist

- [ ] Battery fully charged.
- [ ] Motor regulator checked at 6.0 V.
- [ ] Logic supply checked at 5.0 V.
- [ ] All sensors tested on black and white surfaces.
- [ ] Sensor array properly aligned.
- [ ] Wheels clean and properly attached.
- [ ] Rear caster moving freely.
- [ ] Chassis dimensions within competition limits.
- [ ] Motor directions verified.
- [ ] Start button tested.
- [ ] Diagnostic mode disabled.
- [ ] Final firmware compiled and uploaded.

## Project Goals

This project explores practical applications of:

- Embedded systems programming.
- Autonomous mobile robotics.
- Real-time control systems.
- Digital sensor processing.
- Motor control and PWM.
- Finite state machines.
- Hardware integration and power management.
- Firmware optimization and performance tuning.

## Author

**Otávio Augusto**

Computer Engineering | Robotics | Embedded Systems | IoT | Software Development

- GitHub: [@otavioaugusto-dev](https://github.com/otavioaugusto-dev)
- LinkedIn: [Otávio Augusto](https://www.linkedin.com/in/otavio-augusto-854696360/)

---

<p align="center">
  <sub>Designed and developed for competitive autonomous robotics.</sub>
</p>