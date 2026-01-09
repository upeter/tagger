/**
 * Solution to 03 - LED Challenge
 * 
 * One task flashes an LED at a rate specified by a value set in another task.
 * 
 * Date: December 4, 2020
 * Author: Shawn Hymel
 * License: 0BSD
 */

#include <Arduino.h>

#include "PS4_gamepad.h"
#include "main_control.h"

static PS4Gamepad gamepad;

void setup()
{
	main_control_setup(gamepad);

	// Delete "setup and loop" task
	vTaskDelete(NULL);
}

void loop()
{
	// Execution should never get here
	vTaskDelay(portMAX_DELAY);
}