#pragma once

#include <Arduino.h>
#include <EEPROM.h>
#include <NeoPixelBus.h>

// EEPROM layout
static const int EEPROM_SIZE_PREFERENCES      = 64;
static const uint8_t EEPROM_MAGIC_PREFERENCES = 0x42;
static const int EEPROM_ADDR_MAGIC_PREFERENCES = 0;
static const int EEPROM_ADDR_TEAM_R           = 1;
static const int EEPROM_ADDR_TEAM_G           = 2;
static const int EEPROM_ADDR_TEAM_B           = 3;
static const int EEPROM_ADDR_DIRECTION        = 4;
static const int EEPROM_ADDR_JOYSTICK_MODE    = 5;

void prefsInit();

struct Prefs {
	RgbColor color;    // team color
	int direction;     // +1 or -1
	uint8_t joystickMode; // 1 = one-stick, 2 = two-stick
	bool hasColor;
	bool hasDirection;
    bool hasJoystickMode;
};

Prefs prefsLoad();
Prefs prefsWithTeamColor(const Prefs &base, const RgbColor &color);
Prefs prefsWithDirection(const Prefs &base, int direction);
Prefs prefsWithJoystickMode(const Prefs &base, uint8_t joystickMode);
