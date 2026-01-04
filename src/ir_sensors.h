#pragma once

#include <Arduino.h>

// High-level event emitted by the IR sensor subsystem.
struct IREvent {
	uint32_t result;
	const char *rcvGroup;
	uint8_t sensor; // 0 = front, 1 = back
	unsigned long timestamp;
};

// Called when an IR event arrives and passes IR-level debounce.
// Return true if the event was accepted/consumed (used to advance debounce time).
using IrSensorsOnHitCallback = bool (*)(const IREvent *event);

// Start IR sensor subsystem (creates tasks + internal queue).
// onHit may be null.
void ir_sensors_begin(IrSensorsOnHitCallback onHit);
