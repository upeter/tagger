#include "motors.h"

// Define motor configuration constants (matching previous globals)
const int MOTOR_MAX_SPEED = 255;
const int MOTOR_MIN_SPEED = -MOTOR_MAX_SPEED;

// Right motor
const int ENABLE_RIGHT_MOTOR_PIN = 22;
const int RIGHT_MOTOR_PIN1 = 16;
const int RIGHT_MOTOR_PIN2 = 17;

// Left motor
const int ENABLE_LEFT_MOTOR_PIN = 23;
const int LEFT_MOTOR_PIN1 = 18;
const int LEFT_MOTOR_PIN2 = 19;

// PWM
const int MOTOR_PWM_FREQ = 1000; /* 1 KHz */
const int MOTOR_PWM_RESOLUTION = 8;
const int RIGHT_MOTOR_PWM_CHANNEL = 4;
const int LEFT_MOTOR_PWM_CHANNEL = 5;

void Motors::setMotorDirections(int motorPin1, int motorPin2, int speed)
{
    int pin1State = (speed > 0) ? HIGH : LOW;
    int pin2State = (speed < 0) ? HIGH : LOW;

    digitalWrite(motorPin1, pin1State);
    digitalWrite(motorPin2, pin2State);
}

void Motors::rotateMotor(int rightMotorSpeed, int leftMotorSpeed)
{
    setMotorDirections(RIGHT_MOTOR_PIN1, RIGHT_MOTOR_PIN2, rightMotorSpeed);
    setMotorDirections(LEFT_MOTOR_PIN1, LEFT_MOTOR_PIN2, leftMotorSpeed);
    ledcWrite(RIGHT_MOTOR_PWM_CHANNEL, abs(rightMotorSpeed));
    ledcWrite(LEFT_MOTOR_PWM_CHANNEL, abs(leftMotorSpeed));
}

void Motors::setUpPinModes()
{
    pinMode(ENABLE_RIGHT_MOTOR_PIN, OUTPUT);
    pinMode(RIGHT_MOTOR_PIN1, OUTPUT);
    pinMode(RIGHT_MOTOR_PIN2, OUTPUT);

    pinMode(ENABLE_LEFT_MOTOR_PIN, OUTPUT);
    pinMode(LEFT_MOTOR_PIN1, OUTPUT);
    pinMode(LEFT_MOTOR_PIN2, OUTPUT);

    // Set up PWM for motor speed
    ledcSetup(RIGHT_MOTOR_PWM_CHANNEL, MOTOR_PWM_FREQ, MOTOR_PWM_RESOLUTION);
    ledcSetup(LEFT_MOTOR_PWM_CHANNEL, MOTOR_PWM_FREQ, MOTOR_PWM_RESOLUTION);
    ledcAttachPin(ENABLE_RIGHT_MOTOR_PIN, RIGHT_MOTOR_PWM_CHANNEL);
    ledcAttachPin(ENABLE_LEFT_MOTOR_PIN, LEFT_MOTOR_PWM_CHANNEL);

    rotateMotor(0, 0);
}

MotorChaosMonkey::MotorChaosMonkey(Motors *motors_)
{
    motors = motors_;
}

void MotorChaosMonkey::Start()
{
    unsigned long currentMillis = millis();
    if (!active && (currentMillis - startMillis) >= coolDownMillis)
    {
        active = true;
        startMillis = millis();
        Serial.println((String)"chaos started");
    }
    else
    {
        Serial.println((String)"chaos won't start because it is already active");
    }
}

bool MotorChaosMonkey::isActive()
{
    return active;
}

void MotorChaosMonkey::Update()
{
    unsigned long currentMillis = millis();
    if (active)
    {
        if ((currentMillis - startMillis) >= chaosDurationMillis)
        {
            active = false;
            startMillis = 0;
            phaseOneExecuted = false;
            phaseTwoExecuted = false;
            Serial.println((String)"chaos terminated after: " + chaosDurationMillis);
        }
        else
        {
            if (!phaseOneExecuted)
            {
                motors->rotateMotor(MOTOR_MAX_SPEED, -MOTOR_MAX_SPEED);
                phaseOneExecuted = true;
            }
            if (phaseOneExecuted && !phaseTwoExecuted && currentMillis - startMillis >= (chaosDurationMillis / 2))
            {
                motors->rotateMotor(-MOTOR_MAX_SPEED, MOTOR_MAX_SPEED);
                phaseTwoExecuted = true;
            }
        }
    }
}
