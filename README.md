# Obstacle Avoiding and Automation Bot using Computer Vision

This project demonstrates how to build an obstacle-avoiding robot using an Arduino, ultrasonic sensor (HC-SR04), motor driver (L298N), and DC motors. The robot detects obstacles in its path using the ultrasonic sensor and changes direction to avoid collisions.

## Features
- **Obstacle Avoidance:** Uses an ultrasonic sensor to detect obstacles and adjust direction.
- **Randomized Turn:** Chooses between turning left or right when an obstacle is encountered.
- **Automation Bot:** Designed for basic navigation without human intervention.

## Components
1. **Arduino Uno** (or any Arduino-compatible board)
2. **HC-SR04 Ultrasonic Sensor** (for distance measurement)
3. **L298N Motor Driver**
4. **Two DC Motors**
5. **Power Supply** (Battery for the motors)
6. **Robot Chassis**
7. **Wiring & connectors**

## Circuit Diagram
- **HC-SR04 Ultrasonic Sensor:**
  - VCC → 5V
  - GND → GND
  - Trig → Arduino Pin 9
  - Echo → Arduino Pin 10
- **L298N Motor Driver:**
  - **Motor A:**
    - ENA (Enable A) → Arduino Pin 5 (PWM)
    - IN1 → Arduino Pin 2
    - IN2 → Arduino Pin 3
  - **Motor B:**
    - ENB (Enable B) → Arduino Pin 6 (PWM)
    - IN3 → Arduino Pin 4
    - IN4 → Arduino Pin 7
  - Power → External Battery
  - GND → Arduino GND

## How It Works
1. **Obstacle Detection:**
   - The HC-SR04 ultrasonic sensor continuously checks the distance to obstacles.
   - If the distance is greater than the defined safe threshold, the robot moves forward.
   - If the distance is less than the threshold, the robot stops and turns either left or right randomly.

2. **Motor Control:**
   - The L298N motor driver controls two DC motors, enabling forward, backward, left, and right movement.
   - The Arduino generates PWM signals to control the speed of the motors via the ENA and ENB pins.

## Code Explanation
- The **Arduino code** is designed to:
  - Continuously measure the distance using the ultrasonic sensor.
  - Control the motors using the L298N motor driver based on the obstacle distance.
  - Randomly choose to turn left or right when an obstacle is detected.

```cpp
// Core Arduino code
long getDistance();           // Measures the distance using the HC-SR04 sensor
void moveForward();           // Moves the robot forward
void stopMoving();            // Stops the robot's motion
void turnLeft();              // Turns the robot to the left
void turnRight();             // Turns the robot to the right
void loop();                  // Main loop checks distance and adjusts movement
