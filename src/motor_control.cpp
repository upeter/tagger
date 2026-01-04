#include "motor_control.h"

static float withExpo(int x, float expo, float nullFactor)
{
	if (x == 0) {
		return 1.0f;
	}
	x = abs(x);
	float n = (float(x)) / nullFactor;
	float y = expo * powf(n, 3) + ((1.0f - expo) * n);
	return y;
}

MotorControlConfig motorControlDefaultConfig(int minSpeed, int maxSpeed)
{
	MotorControlConfig cfg;
	cfg.minSpeed = minSpeed;
	cfg.maxSpeed = maxSpeed;
	cfg.trimX = 0;
	cfg.maxX = 35;
	cfg.expo = 0.5f;
	cfg.nullFactor = 157.0f;
	cfg.turnGain = 5.0f;
	cfg.deadzone = 10;
	cfg.standstillThreshold = 10;
	cfg.pivotScale = 1.5f;
	cfg.highThrottleThreshold = 80;
	cfg.highThrottleTurnBoost = 2.5f;
	return cfg;
}

MotorControlOutput motorControlCompute(const MotorControlInput &input, const MotorControlConfig &config)
{
	MotorControlOutput out;
	out.rightMotorSpeed = 0;
	out.leftMotorSpeed = 0;
	out.rightMotorSpeedRaw = 0;
	out.leftMotorSpeedRaw = 0;

	int stickY = 0;
	int stickX = 0;

	if (input.joystickMode == 2) {
		// Two-stick: left Y for forward/back, right X for turning
		// Use right X for in-place spins when no forward/back input
		int rawY = input.lStickY;
		int rawTurn = input.rStickX;
		int rawSpin = input.rStickY;

		if (abs(rawY) < config.deadzone && abs(rawSpin) < config.deadzone) {
			// Standing still: steering stick should rotate in place
			stickY = 0;
			stickX = rawTurn;
		} else {
			// Moving: left Y + right X mixed driving
			stickY = rawY;
			stickX = rawTurn;
		}
	} else {
		// One-stick: right stick controls both
		stickY = input.rStickY;
		stickX = input.rStickX;
	}

	float stickXExpo = withExpo(stickX, config.expo, config.nullFactor);
	int stickXWithExpo = constrain(
		(int)(((float(stickX) * stickXExpo) * config.turnGain + config.trimX) * -1),
		-config.maxX,
		config.maxX);

	if (input.joystickMode == 2 && abs(stickY) < config.standstillThreshold) {
		// Pivot in place
		int rawTurn = input.rStickX; // -127..127
		int pivot = constrain((int)(rawTurn * config.pivotScale), -127, 127);

		out.rightMotorSpeedRaw = pivot;
		out.leftMotorSpeedRaw = -pivot;
	} else if (input.joystickMode == 2 && abs(stickY) > config.highThrottleThreshold) {
		// At high throttle in 2-stick mode, boost turning for tighter curves
		int boostedTurn = constrain((int)(stickXWithExpo * config.highThrottleTurnBoost), -config.maxX, config.maxX);
		out.rightMotorSpeedRaw = constrain(stickY - boostedTurn, -127, 127);
		out.leftMotorSpeedRaw = constrain(stickY + boostedTurn, -127, 127);
	} else {
		out.rightMotorSpeedRaw = constrain(stickY - stickXWithExpo, -127, 127);
		out.leftMotorSpeedRaw = constrain(stickY + stickXWithExpo, -127, 127);
	}

	out.rightMotorSpeed = (int)map(out.rightMotorSpeedRaw, -127, 127, config.minSpeed, config.maxSpeed);
	out.leftMotorSpeed = (int)map(out.leftMotorSpeedRaw, -127, 127, config.minSpeed, config.maxSpeed);

	out.rightMotorSpeed = constrain(out.rightMotorSpeed, config.minSpeed, config.maxSpeed);
	out.leftMotorSpeed = constrain(out.leftMotorSpeed, config.minSpeed, config.maxSpeed);

	return out;
}
