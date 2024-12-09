# Self-Balancing Robot

## Overview
This project demonstrates the development of a self-balancing robot using a microcontroller, sensors, and motor control mechanisms. The robot adjusts its balance dynamically to stay upright, leveraging a feedback control system implemented with PID (Proportional, Integral, Derivative) control.

The repository is structured as a [PlatformIO](https://platformio.org/) project, facilitating streamlined development and deployment.

## Features
- Dynamic self-balancing using a feedback loop.
- Integration with an IMU (Inertial Measurement Unit) for precise orientation sensing.
- Motor control for dynamic adjustments.
- Easy customization for additional features, such as remote control or obstacle detection.

## Getting Started

### Prerequisites
1. **Hardware Components:**
   - Microcontroller (e.g., ESP32-S3 or ESP32 family).
   - IMU sensor (e.g., MPU6050).
   - Motor driver module (e.g., L298N).
   - DC motors.
   - Battery and power management components.

2. **Software Tools:**
   - [PlatformIO IDE](https://platformio.org/).
   - Required libraries: [Wire](https://www.arduino.cc/reference/en/libraries/wire/), MPU6050 library.

### Setting Up the Project
1. Clone the repository:
   ```bash
   git clone https://github.com/Afnan-Yusuf/Balancing-robot.git

   ### Circuit Connections
| **Component**       | **Microcontroller Pin** | **Details**               |
|----------------------|--------------------------|---------------------------|
| MPU6050 SDA         | GPIO21                  | Connect via pull-up resistor |
| MPU6050 SCL         | GPIO22                  | Connect via pull-up resistor |
| Motor Driver Input1 | GPIO25                  | Control signal for motor  |
| Motor Driver Input2 | GPIO26                  | Control signal for motor  |
| Motor Driver Enable | GPIO27                  | PWM signal for speed control |
| Motor Driver Output | Motor Terminals         | Connected to motors       |

Ensure proper power connections between the motor driver, microcontroller, and motors. Use a regulated power source compatible with both the motors and the microcontroller.

## Running the Code
1. Upload the code to the microcontroller using PlatformIO.
2. Power the robot and observe its balancing behavior.
3. Use the serial monitor to view debugging information or tune parameters.

## Code Explanation
- **PID Control:** Implements the feedback loop to maintain balance.
- **Sensor Integration:** Reads accelerometer and gyroscope data from the MPU6050.
- **Motor Control:** Adjusts motor speed and direction based on PID output.

## Customization
- **PID Tuning:**
  Adjust `Kp`, `Ki`, and `Kd` values in the code to match your hardware setup.
  ```cpp
  float Kp = 1.0; // Proportional gain
  float Ki = 0.0; // Integral gain
  float Kd = 0.5; // Derivative gain

