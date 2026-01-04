/**
 * Solution to 03 - LED Challenge
 * 
 * One task flashes an LED at a rate specified by a value set in another task.
 * 
 * Date: December 4, 2020
 * Author: Shawn Hymel
 * License: 0BSD
 */

// Needed for atoi()
// #include "esp_bt_main.h"
// #include "esp_bt_device.h"
#include <stdlib.h>
#include <Arduino.h>
#include "misc.h"
// #include <FastLED.h>
#include <EEPROM.h>
#include <PS4Controller.h>
#include <NeoPixelBus.h>
#include <PS4Controller.h>
#include "prefs.h"
#include "colors.h"
#include "motors.h"
#include "activity_lights.h"
#include "laser.h"
#include "flasher.h"

#include "motor_control.h"

#include "ir_sensors.h"
#include "ir_fire.h"


#define LED_BUILTIN 2

// Use only core 1 for demo purposes
#if CONFIG_FREERTOS_UNICORE
  static const BaseType_t app_cpu = 0;
#else
  static const BaseType_t app_cpu = 1;
#endif

#define LED_DATA_PIN 13
#define NUM_LEDS 67

// Settings
static const uint8_t buf_len = 20;

// Pins
static const int led_pin = LED_BUILTIN;
static const int trigger_pin = 25;
static const int trigger_servo_pin = 18;
static const int laser_pin = 15;

// Motor
int maxSpeed = MOTOR_MAX_SPEED;
int minSpeed = MOTOR_MIN_SPEED;
volatile int trimX = 0;

static MotorControlConfig motorControlConfig;

// Globals
int led_delay = 50;   // ms
static bool flash_led = false;
const uint32_t ir_messages[] =
	{
		0xFF30CF,
		0xFF18E7,
		0xFF7A85,
		0xFF10EF,
		0xFF38C7,
		0xFF5AA5,
		0xFF42BD,
		0xFF4AB5,
};

int direction = 1;
unsigned long lastDirectionToggleMillis = 0;
const unsigned long DIRECTION_DEBOUNCE_MS = 500; // tweak as needed
Prefs currentPrefs;

// Joystick mode: 1 = one-stick (current behavior), 2 = two-stick
uint8_t joystickMode = 1;
unsigned long lastJoystickModeToggleMillis = 0;
const unsigned long JOYSTICK_MODE_DEBOUNCE_MS = 500;

// Debounce for in-game prefs reset (L2+R2)
unsigned long lastPrefsResetMillis = 0;
const unsigned long PREFS_RESET_DEBOUNCE_MS = 2000;

unsigned long lastTimeStamp = 0;


TaskHandle_t xHandle_handleIR,
	xHandle_toggleOnboardLED,
    xHandle_readSerialInput,
	xHandle_triggerButton, 
	xHandle_handleLED;
// Color definitions moved to colors.{h,cpp}

//Classes

// Motors and MotorChaosMonkey are now declared in motors.h / motors.cpp

// Objects

NeoPixelBus<NeoGrbFeature, NeoEsp32RmtMethodBase<NeoEsp32RmtSpeed800Kbps, NeoEsp32RmtChannel3>> strip(NUM_LEDS, LED_DATA_PIN);
Motors *motors;
MotorChaosMonkey *chaosMonkey;
Flasher flasher(led_pin, 1000);
Laser laser(laser_pin);
ActivityLights lights(strip);

static bool onIrHit(const IREvent *event)
{
	(void)event;
	if (lights.canBeHit()) {
		vTaskResume(xHandle_toggleOnboardLED);
		lights.onHit();
		if (chaosMonkey != nullptr) {
			chaosMonkey->Start();
		}
		return true;
	}
	Serial.println(">>> Hit cooldown");
	return false;
}

static bool irCanFire()
{
	return lights.canFire();
}

static void irDidFire(uint32_t code)
{
	(void)code;
	lights.onFire();
}

void notify()
{
	char messageString[200];

	try
	{
		if (millis() - lastTimeStamp > 100)
		{

			int rightMotorSpeed, leftMotorSpeed;
			// Only needed to print the message properly on serial monitor. Else we dont need it.
			// sprintf(messageString, "%4d,%4d,%4d,%4d,%3d,%3d,%3d,%3d,%3d,%3d,%3d,%3d,%3d,%3d,%3d,%3d,%3d,%3d,%3d,%3d,%3d,%3d,%3d,%3d",
			// 		PS4.LStickX(),
			// 		PS4.LStickY(),
			// 		PS4.RStickX(),
			// 		PS4.RStickY(),
			// 		PS4.Left(),
			// 		PS4.Down(),
			// 		PS4.Right(),
			// 		PS4.Up(),
			// 		PS4.Square(),
			// 		PS4.Cross(),
			// 		PS4.Circle(),
			// 		PS4.Triangle(),
			// 		PS4.L1(),
			// 		PS4.R1(),
			// 		PS4.L2(),
			// 		PS4.R2(),
			// 		PS4.Share(),
			// 		PS4.Options(),
			// 		PS4.PSButton(),
			// 		PS4.Touchpad(),
			// 		PS4.Charging(),
			// 		PS4.Audio(),
			// 		PS4.Mic(),
			// 		PS4.Battery());
			// Serial.println(messageString);
			//shoot
			if(lights.isGameOver()) {
				motors->rotateMotor(0, 0);
			} else {
				if(chaosMonkey->isActive()) {
					chaosMonkey->Update();
				} else {

					if (PS4.R1() || PS4.L1()) {
						laser.activate();
						if(PS4.R1()) {
							ir_fire_request(ir_messages[0]);
						}
					} else {
						laser.deactivate();
					}

					if(PS4.Square()) {
						currentPrefs = prefsWithTeamColor(currentPrefs, COLOR_PINK);
						lights.setTeamColor(currentPrefs.color);
					} else if(PS4.Cross()) {
						currentPrefs = prefsWithTeamColor(currentPrefs, COLOR_BLUE);
						lights.setTeamColor(currentPrefs.color);
					} else if (PS4.Circle()){
						currentPrefs = prefsWithTeamColor(currentPrefs, COLOR_ORANGE);
						lights.setTeamColor(currentPrefs.color);
					} else if (PS4.Triangle()) {
						currentPrefs = prefsWithTeamColor(currentPrefs, COLOR_GREEN);
						lights.setTeamColor(currentPrefs.color);
					} else if (PS4.L2() && PS4.R2()) {
						// Reset all user preferences when L2 and R2 are pressed together, with debounce
						unsigned long now = millis();
						if (now - lastPrefsResetMillis > PREFS_RESET_DEBOUNCE_MS) {
							Serial.println("Resetting user preferences (L2+R2)");
							prefsClearAll();
							currentPrefs = prefsLoad();
							// Re-apply any restored prefs to runtime state
							if (currentPrefs.hasColor) {
								lights.setTeamColor(currentPrefs.color);
							}
							if (currentPrefs.hasDirection) {
								direction = currentPrefs.direction;
							}
							if (currentPrefs.hasJoystickMode) {
								joystickMode = currentPrefs.joystickMode;
							}
							// force immediate visual refresh after prefs reset
							lights.updateFrame();
							lights.renderFrame();
							lastPrefsResetMillis = now;
						}
					} else if(PS4.Up()) {
						unsigned long now = millis();
						if (now - lastDirectionToggleMillis > DIRECTION_DEBOUNCE_MS) {
							direction = direction * -1;
							currentPrefs = prefsWithDirection(currentPrefs, direction);
							lastDirectionToggleMillis = now;
						}
					} else if (PS4.Left()) {
						unsigned long now = millis();
						if (now - lastJoystickModeToggleMillis > JOYSTICK_MODE_DEBOUNCE_MS) {
							joystickMode = 2; // two-stick mode
							currentPrefs = prefsWithJoystickMode(currentPrefs, joystickMode);
							Serial.println("Joystick mode set to TWO-STICK (PS.Left)");
							lastJoystickModeToggleMillis = now;
						}
					} else if (PS4.Right()) {
						unsigned long now = millis();
						if (now - lastJoystickModeToggleMillis > JOYSTICK_MODE_DEBOUNCE_MS) {
							joystickMode = 1; // one-stick mode
							currentPrefs = prefsWithJoystickMode(currentPrefs, joystickMode);
							Serial.println("Joystick mode set to ONE-STICK (PS.Right)");
							lastJoystickModeToggleMillis = now;
						}
					}

					MotorControlInput controlInput;
					controlInput.lStickX = PS4.LStickX();
					controlInput.lStickY = PS4.LStickY() * direction;
					controlInput.rStickX = PS4.RStickX();
					controlInput.rStickY = PS4.RStickY() * direction;
					controlInput.joystickMode = joystickMode;

					motorControlConfig.trimX = trimX;
					MotorControlOutput cmd = motorControlCompute(controlInput, motorControlConfig);
					rightMotorSpeed = cmd.rightMotorSpeed;
					leftMotorSpeed = cmd.leftMotorSpeed;
					motors->rotateMotor(rightMotorSpeed, leftMotorSpeed);
				}
			}



			lastTimeStamp = millis();
		}
	}
		catch (const std::exception &e)
		{
			Serial.print("Exception caught: ");
			Serial.println(e.what());
		}
	
}

void onConnect()
{
	Serial.println("Connected!.");
}

void onDisConnect()
{
	motors->rotateMotor(0, 0);
	Serial.println("Disconnected!.");
}


//*****************************************************************************
// Tasks



// Task: Blink LED at rate set by global variable
void toggleOnboardLED(void *parameter) {
  while(true) {
	vTaskSuspend(NULL);
  	flasher.flash();
  }
}



void handleLED(void *parameter){
	const TickType_t frameDelay = pdMS_TO_TICKS(20); // Small tick just to advance time-based animations
	while (1) {
		// Advance internal animation state
		lights.updateFrame();
		// Only renders when an animation or state change requested it
		lights.renderFrame();
		vTaskDelay(frameDelay);
	}
}

//*****************************************************************************
// Main



void setup() {

  // Configure pin
  pinMode(led_pin, OUTPUT);

  // Configure serial and wait a second
  Serial.begin(115200);
  vTaskDelay(1000 / portTICK_PERIOD_MS);
  	Serial.println("Initializing tank");
	Serial.println("Initializing onboard LEDs...");


	// Start blink task
	xTaskCreatePinnedToCore(					// Use xTaskCreate() in vanilla FreeRTOS
		toggleOnboardLED,			// Function to be called
		"Toggle LED",				// Name of task
		2048,						// Stack size (bytes in ESP32, words in FreeRTOS)
		NULL,						// Parameter to pass
		1,							// Task priority
		&xHandle_toggleOnboardLED,
		1); // Task handle


	vTaskDelay(1000);
	strip.Begin();
 	//strip.SetBrightness(128);
    strip.Show();

	Serial.println("Initialize motors...");

	motors = new Motors();
	chaosMonkey = new MotorChaosMonkey(motors);
	motors->setUpPinModes();
	Serial.println("Motors initialized");
	motorControlConfig = motorControlDefaultConfig(minSpeed, maxSpeed);
	// Load user preferences (team color + direction)
	currentPrefs = prefsLoad();
	if (currentPrefs.hasColor) {
		lights.setTeamColor(currentPrefs.color);
	}
	if (currentPrefs.hasDirection) {
		direction = currentPrefs.direction;
	}
	if (currentPrefs.hasJoystickMode) {
		joystickMode = currentPrefs.joystickMode;
	}
	// Ensure LEDs reflect initial state
	lights.updateFrame();
	lights.renderFrame();

	// Start IR subsystems once lights + motors are ready
	ir_sensors_begin(onIrHit);
	ir_fire_begin(irCanFire, irDidFire);

	// Start serial read task
	xTaskCreatePinnedToCore(				   // Use xTaskCreate() in vanilla FreeRTOS
		handleLED,		   // Function to be called
		"handle LED",			   // Name of task
		4096,					   // Stack size (bytes in ESP32, words in FreeRTOS)
		NULL,					   // Parameter to pass
		2,						   // Task priority (must be same to prevent lockup)
		&xHandle_handleLED,
		1); // Task handle


 	Serial.println("Initialize PS4...");
	PS4.attach(notify);
	PS4.attachOnConnect(onConnect);
	PS4.attachOnDisconnect(onDisConnect);
	PS4.begin();
	Serial.println("PS4 initialized");

	// Delete "setup and loop" task
	vTaskDelete(NULL);



}

void loop() {
  // Execution should never get here
   vTaskDelay(portMAX_DELAY); /*wait as much as possible ... */
}