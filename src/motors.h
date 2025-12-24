#pragma once

#include <Arduino.h>

// Motor configuration constants
extern const int MOTOR_MAX_SPEED;
extern const int MOTOR_MIN_SPEED;

extern const int ENABLE_RIGHT_MOTOR_PIN;
extern const int RIGHT_MOTOR_PIN1;
extern const int RIGHT_MOTOR_PIN2;

extern const int ENABLE_LEFT_MOTOR_PIN;
extern const int LEFT_MOTOR_PIN1;
extern const int LEFT_MOTOR_PIN2;

extern const int MOTOR_PWM_FREQ;
extern const int MOTOR_PWM_RESOLUTION;
extern const int RIGHT_MOTOR_PWM_CHANNEL;
extern const int LEFT_MOTOR_PWM_CHANNEL;

class Motors {
public:
    void rotateMotor(int rightMotorSpeed, int leftMotorSpeed);
    void setUpPinModes();

private:
    void setMotorDirections(int motorPin1, int motorPin2, int speed);
};

class MotorChaosMonkey {
public:
    explicit MotorChaosMonkey(Motors *motors);

    void Start();
    bool isActive();
    void Update();

private:
    unsigned long startMillis = 0;
    volatile bool active = false;
    bool phaseOneExecuted = false;
    bool phaseTwoExecuted = false;
    unsigned long chaosDurationMillis = 800;
    unsigned long coolDownMillis = 3000;
    Motors *motors;
};
