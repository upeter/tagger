#pragma once

#include "gamepad.h"

// Initializes all firmware subsystems that used to live in main.cpp::setup(),
// except transport-specific controller startup.
//
// The caller owns the Gamepad implementation (PS4 adapter, USB HID, etc.).
void main_control_setup(Gamepad &gamepad);

// Optional connection lifecycle hooks (the concrete controller may support them).
void main_control_onConnect();
void main_control_onDisconnect();
