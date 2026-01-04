#pragma once

#include <Arduino.h>

// Return true if firing is allowed right now (game rules / cooldown).
using IrFireCanFireCallback = bool (*)();

// Called after a shot is successfully sent.
using IrFireDidFireCallback = void (*)(uint32_t code);

// Starts the IR transmitter task.
// canFire/didFire may be null.
void ir_fire_begin(IrFireCanFireCallback canFire, IrFireDidFireCallback didFire);

// Request an IR shot to be sent. This is non-blocking.
void ir_fire_request(uint32_t code);
