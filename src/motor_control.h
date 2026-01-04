#pragma once

#include <Arduino.h>

struct MotorControlInput
{
	int lStickX; // -127..127
	int lStickY; // -127..127
	int rStickX; // -127..127
	int rStickY; // -127..127
	uint8_t joystickMode; // 1 = one-stick, 2 = two-stick
};

struct MotorControlOutput
{
	int rightMotorSpeed;
	int leftMotorSpeed;
	int rightMotorSpeedRaw;
	int leftMotorSpeedRaw;
};

struct MotorControlConfig
{
	int minSpeed;
	int maxSpeed;
	int trimX;
	int maxX;
	float expo;
	float nullFactor;
	float turnGain;
	int deadzone;
	int standstillThreshold;
	float pivotScale;
	int highThrottleThreshold;
	float highThrottleTurnBoost;
};

MotorControlConfig motorControlDefaultConfig(int minSpeed, int maxSpeed);
MotorControlOutput motorControlCompute(const MotorControlInput &input, const MotorControlConfig &config);
