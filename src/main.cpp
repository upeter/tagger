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
#include "IR32/src/IRSend.h"
#include "IR32/src/IRRecv.h"
#include "soc/rtc_wdt.h" 
#include <NeoPixelBus.h>
#include <PS4Controller.h>
#include "prefs.h"
#include "colors.h"
#include "motors.h"
#include "activity_lights.h"


#define LED_BUILTIN 2

// Use only core 1 for demo purposes
#if CONFIG_FREERTOS_UNICORE
  static const BaseType_t app_cpu = 0;
#else
  static const BaseType_t app_cpu = 1;
#endif

#define IR_RMT_RX_FRONT_CHANNEL RMT_CHANNEL_0
#define IR_RMT_RX_BACK_CHANNEL RMT_CHANNEL_1
#define IR_RMT_TX_CHANNEL RMT_CHANNEL_2 /*!< RMT channel for transmitter */
#define IR_PROTOCOL "NEC"              //choose from timing groups lib/IR32/src/IR32.h
#define IR_RMT_TX_GPIO_NUM GPIO_NUM_2 /*!< GPIO number for transmitter signal */
#define IR_RECV_FRONT_PIN 27
#define IR_RECV_BACK_1_PIN 26
#define DEBOUNCE_TIME_MS 50
#define IR_DEBOUNCE_TIME_MS 1000  // Configurable IR debounce time


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
//expo
const float expo = 0.5;
const float nullFactor = 157.0;
const int maxX = 35;
volatile int trimX = 0;

// Globals
static int led_delay = 50;   // ms
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
uint32_t lastTriggerDebounceTime = 0;

// Joystick mode: 1 = one-stick (current behavior), 2 = two-stick
uint8_t joystickMode = 1;
unsigned long lastJoystickModeToggleMillis = 0;
const unsigned long JOYSTICK_MODE_DEBOUNCE_MS = 500;

// Debounce for in-game prefs reset (L2+R2)
unsigned long lastPrefsResetMillis = 0;
const unsigned long PREFS_RESET_DEBOUNCE_MS = 2000;

unsigned long lastTimeStamp = 0;


TaskHandle_t xHandle_handleIR,
	xHandle_handleIRFront,
	xHandle_handleIRBack,
	xHandle_processIRQueue,
	xHandle_toggleOnboardLED,
    xHandle_readSerialInput,
	xHandle_triggerButton, 
	xHandle_handleLED,
	xHandle_fire;

// IR Event Queue Structure
struct IREvent {
    uint32_t result;
    char* rcvGroup;
    uint8_t sensor; // 0 = front, 1 = back
    unsigned long timestamp;
};

QueueHandle_t irEventQueue;
SemaphoreHandle_t irProcessingSemaphore;
volatile unsigned long lastIRProcessTime = 0;
// Color definitions moved to colors.{h,cpp}

//Classes
class Flasher {
public:
  Flasher(int ledPin, long flashDuration) : ledPin(ledPin), flashDuration(flashDuration) {
    pinMode(ledPin, OUTPUT);
  }

  void flash() {
		long startTime = millis();
		while (millis() - startTime < flashDuration) {
			digitalWrite(led_pin, HIGH);
			vTaskDelay(led_delay / portTICK_PERIOD_MS);
			digitalWrite(led_pin, LOW);
			vTaskDelay(led_delay / portTICK_PERIOD_MS);
		}
  }


private:
  int ledPin;
  long flashDuration;
};

class Laser {
public:
  Laser(int laserPin) : laserPin(laserPin), active(false) {
   pinMode(laserPin, OUTPUT);
  }

  void activate() {
		if(!active) {
			active = true;
			digitalWrite(laserPin, HIGH);
		}
  }

  void deactivate() {
		if(active) {
			active = false;
			digitalWrite(laserPin, LOW);
		}
  }



private:
  int laserPin;
  boolean active;
};

// Motors and MotorChaosMonkey are now declared in motors.h / motors.cpp

float withExpo(int x)
{
	if(x == 0) {
		return 1.0;
	} else {
		x = abs(x);
		float n = (float(x)) / nullFactor;
		float y = expo * pow(n, 3) + ((1.0 - expo) * n);
		return y;
	}
}


// Objects

NeoPixelBus<NeoGrbFeature, NeoEsp32RmtMethodBase<NeoEsp32RmtSpeed800Kbps, NeoEsp32RmtChannel3>> strip(NUM_LEDS, LED_DATA_PIN);
Motors *motors;
MotorChaosMonkey *chaosMonkey;
IRRecv ir_rec_front(IR_RMT_RX_FRONT_CHANNEL);
IRRecv ir_rec_back(IR_RMT_RX_BACK_CHANNEL);
IRSend ir_send(IR_RMT_TX_CHANNEL);
Button triggerButton(trigger_pin);
Flasher flasher(led_pin, 1000);
Laser laser(laser_pin);
ActivityLights lights(strip);
void notify()
{
	char messageString[200];

	try
	{
		if (millis() - lastTimeStamp > 100)
		{

			int rightMotorSpeed, leftMotorSpeed;
			int stickY, stickX, stickXWithExpo, stickXWithExpoFloat;
			int rightMotorSpeedRaw, leftMotorSpeedRaw;
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
							Serial.println(">>> Shot!");
							vTaskResume(xHandle_fire);
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

					// Joystick mapping depending on mode
					if (joystickMode == 2) {
						// Two-stick: left Y for forward/back, right X for turning
						// Use right X for in-place spins when no forward/back input
						int rawY = PS4.LStickY() * direction;
						int rawTurn = PS4.RStickX();
						int rawSpin = PS4.RStickY();
						// deadzone for deciding "standing still"
						const int deadzone = 10;
						if (abs(rawY) < deadzone && abs(rawSpin) < deadzone) {
							// Standing still: steering stick should rotate in place
							stickY = 0;
							stickX = rawTurn;
						} else {
							// Moving: left Y + right X mixed driving
							stickY = rawY;
							stickX = rawTurn;
						}
					} else {
						// One-stick: right stick controls both as before
						stickY = PS4.RStickY() * direction;
						stickX = PS4.RStickX();
					}
					float stickXExpo = withExpo(stickX);
					// Increase rotational authority so sharp turns are possible even at high speed
					const float turnGain = 5.0; // adjust if needed after testing
					//stickX = (toggle ? (int)((float)stickX * stickXExpo) : stickX / 3) * -1;
					stickXWithExpo = constrain(
						(int)(((float(stickX) * stickXExpo) * turnGain + trimX) * -1),
						maxX * -1,
						maxX
					);

					// If we are effectively standing still, boost pivot authority
					const int standstillThreshold = 10; // same units as stickY
					if (joystickMode == 2 && abs(stickY) < standstillThreshold) {
						// Use raw right stick X directly for pivot
						int rawTurn = PS4.RStickX();          // -127..127
						const int pivotScale = 1.5;             // increase if still too weak
						int pivot = constrain(rawTurn * pivotScale, -127, 127);

						rightMotorSpeedRaw =  pivot;
						leftMotorSpeedRaw  =  -pivot;
					} else if (joystickMode == 2 && abs(stickY) > 80) {
						// At high throttle in 2-stick mode, boost turning for tighter curves
						int boostedTurn = constrain(stickXWithExpo * 2.5, maxX * -1, maxX);
						rightMotorSpeedRaw = constrain(stickY - boostedTurn, -127, 127);
						leftMotorSpeedRaw  = constrain(stickY + boostedTurn, -127, 127);
					} else {
						rightMotorSpeedRaw = constrain(stickY - stickXWithExpo, -127, 127);
						leftMotorSpeedRaw  = constrain(stickY + stickXWithExpo, -127, 127);
					}
					rightMotorSpeed = map(rightMotorSpeedRaw , -127, 127, minSpeed, maxSpeed); // Left stick  - y axis - forward/backward left motor movement
					leftMotorSpeed = map(leftMotorSpeedRaw, -127, 127, minSpeed, maxSpeed);   // Right stick - y axis - forward/backward right motor movement

					// Only needed to print the message properly on serial monitor. Else we dont need it.
					// Serial.printf("StickY: %4d, StickX: %4d, right-raw: %4d, left-raw: %4d, right: %4d, left: %4d \n", stickY, stickX, rightMotorSpeedRaw, leftMotorSpeedRaw, rightMotorSpeed, leftMotorSpeed);

					// original: working
					//   rightMotorSpeed = map( PS4.RStickY(), -127, 127, -220, 220); //Left stick  - y axis - forward/backward left motor movement
					//   leftMotorSpeed = map( PS4.LStickY(), -127, 127, -220, 220);  //Right stick - y axis - forward/backward right motor movement

					rightMotorSpeed = constrain(rightMotorSpeed, minSpeed, maxSpeed);

					leftMotorSpeed = constrain(leftMotorSpeed, minSpeed, maxSpeed);

					//   Serial.printf("StickY: %4d, StickX: %4d, StickXExpo: %f, StickXWithExpo: %d, trimX: %d ", stickY, stickX, stickXExpo, stickXWithExpo, trimX);
					//   Serial.printf("right-raw: %d,  left-raw: %d, right: %d, left: %d \n", rightMotorSpeedRaw, leftMotorSpeedRaw, rightMotorSpeed, leftMotorSpeed);

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
//   while (true) {
//     digitalWrite(led_pin, HIGH);
//     vTaskDelay(led_delay / portTICK_PERIOD_MS);
//     digitalWrite(led_pin, LOW);
//     vTaskDelay(led_delay / portTICK_PERIOD_MS);
//   }
}

// Task: Read from serial terminal
// Feel free to use Serial.readString() or Serial.parseInt(). I'm going to show
// it with atoi() in case you're doing this in a non-Arduino environment. You'd
// also need to replace Serial with your own UART code for non-Arduino.
void readSerialInput(void *parameters) {
 String inputBuffer = ""; 
 while (true){
	if (Serial.available() > 0) {
		char c = Serial.read();
		if (c == '\n') {
			int ledDelay = inputBuffer.toInt(); // Convert the input to an integer
			if (ledDelay != 0) {
				Serial.print("Updated LED delay to: ");
				Serial.println(ledDelay);
				led_delay = ledDelay;
				inputBuffer = ""; // Clear the buffer
			} else {
				 Serial.println("Invalid input. Please provide an integer value.");
				 inputBuffer = "";
			}
		} else {
			Serial.println("Entered: " + (String)c);
			inputBuffer += c;
			Serial.println(" Buffer: " + inputBuffer);
		}
  	}
 }
}

/**
 * @brief Central IR processing task
 * This function processes IR events from the queue with debouncing
 * @param parameter just a dummy parameter
 */
void processIRQueue(void *parameter)
{
    Serial.println("IR processing task started");
    IREvent event;
    
    while (true) {
        // Wait for an IR event in the queue
        if (xQueueReceive(irEventQueue, &event, portMAX_DELAY)) {
            // Take semaphore to ensure only one IR signal is processed at a time
            if (xSemaphoreTake(irProcessingSemaphore, pdMS_TO_TICKS(100))) {
                // Check debounce timing
                unsigned long currentTime = millis();
                if (currentTime - lastIRProcessTime >= IR_DEBOUNCE_TIME_MS) {
                    Serial.printf("Processing IR from sensor %d: %s/0x%x\n", 
                                event.sensor, event.rcvGroup, event.result);
                    
                    if (lights.canBeHit()) {
                        vTaskResume(xHandle_toggleOnboardLED);
						lights.onHit();
                        chaosMonkey->Start();
                        lastIRProcessTime = currentTime;
                    } else {
                        Serial.println(">>> Hit cooldown");
                    }
                } else {
                    Serial.printf(">>> IR signal dropped (debounce): sensor %d, time since last: %lu ms\n", 
                                event.sensor, currentTime - lastIRProcessTime);
                }
                
                xSemaphoreGive(irProcessingSemaphore);
            }
        }
    }
}

// Structure to pass IR sensor configuration to generic handler
struct IRSensorConfig {
    IRRecv* receiver;
    uint8_t pin;
    uint8_t sensorId;
    const char* sensorName;
};

/**
 * @brief Generic IR sensor handler
 * This function reads incoming infrared data and queues it for processing
 * @param parameter IRSensorConfig* containing sensor configuration
 */
void handleIRSensor(void *parameter)
{
    IRSensorConfig* config = (IRSensorConfig*)parameter;
    
    Serial.printf("handle IR %s task started\n", config->sensorName);
    Serial.printf("init ir rec %s\n", config->sensorName);
    config->receiver->setPreferred(IR_PROTOCOL);
    config->receiver->start(config->pin);
    Serial.printf("init ir rec %s complete\n", config->sensorName);
    
    uint32_t counter = 0;
    uint32_t last = millis();
    
    while (true) {
        try {
            counter++;
            if ((millis() - last) > 1000) {
                rtc_wdt_feed();
                vTaskDelay(50);
                last = millis();
            }
            
            if (config->receiver->available()) {
                Serial.printf("IR %s available\n", config->sensorName);
                char *rcvGroup = nullptr;
                uint32_t result = config->receiver->read(rcvGroup);
                
                if (result) {
                    IREvent event;
                    event.result = result;
                    event.rcvGroup = rcvGroup;
                    event.sensor = config->sensorId;
                    event.timestamp = millis();
                    
                    // Try to queue the event (non-blocking)
                    if (xQueueSend(irEventQueue, &event, 0) != pdTRUE) {
                        Serial.printf(">>> IR queue full, dropping %s sensor event\n", config->sensorName);
                    }
                }
                vTaskDelay(200);
            }
        } catch (const std::invalid_argument& e) {
            Serial.printf("Error occurred in %s IR handler\n", config->sensorName);
        }
    }
}

/**
 * @brief Wrapper for front IR sensor
 */
void handleIRFront(void *parameter)
{
    static IRSensorConfig frontConfig = {
        &ir_rec_front,
        IR_RECV_FRONT_PIN,
        0,
        "Front"
    };
    handleIRSensor(&frontConfig);
}

/**
 * @brief Wrapper for back IR sensor
 */
void handleIRBack(void *parameter)
{
    static IRSensorConfig backConfig = {
        &ir_rec_back,
        IR_RECV_BACK_1_PIN,
        1,
        "Back"
    };
    handleIRSensor(&backConfig);
}


void handleTrigger(void *parameter){
	ir_send.start(IR_RMT_TX_GPIO_NUM, IR_PROTOCOL);
    while (true){
        //wait until the last bounce is longer ago than DEBOUNCETIME
        while (millis() - lastTriggerDebounceTime < DEBOUNCE_TIME_MS) {
            vTaskDelay(10);
		}
        //refresh trigger.pressed
        triggerButton.read_pin();
		if(triggerButton.pressed) {
			Serial.println(">>> Shot!");

			ir_send.send(ir_messages[0]);
			vTaskDelay(200);
		}
		lastTriggerDebounceTime = xTaskGetTickCount();
	}
}

void handleFire(void *parameter){
	ir_send.start(IR_RMT_TX_GPIO_NUM, IR_PROTOCOL);
    while (true){
		vTaskSuspend(NULL);
			try {
				if(lights.canFire()) {
					Serial.println(">>> Shot!");
					ir_send.send(ir_messages[0]);
					lights.onFire();
					vTaskDelay(500);
				} else {
					Serial.println(">>> Shot cooldown");
				}
			} catch (const std::invalid_argument& e) {
				Serial.print("Error occurred: ");
                Serial.println(e.what()); // Log the exception text
				ir_send.stop();
				ir_send.start(IR_RMT_TX_GPIO_NUM, IR_PROTOCOL);
			}
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

 //disable watchdog - seems not to work properly...
 	//  rtc_wdt_protect_off();    // Turns off the automatic wdt service
	// rtc_wdt_enable();         // Turn it on manually
	// rtc_wdt_set_time(RTC_WDT_STAGE0, 20000);  // Define how long you desire to let dog wait.

	//servo init
	// Allow allocation of all timers
	// ESP32PWM::allocateTimer(0);
	// ESP32PWM::allocateTimer(1);
	// ESP32PWM::allocateTimer(2);
	// ESP32PWM::allocateTimer(3);
	
  
  Serial.println("Multi-task LED Demo");
  Serial.println("Enter a number in milliseconds to change the LED delay.");
	//digitalWrite(trigger_servo_pin, LOW);

	// Create IR event queue and semaphore
	irEventQueue = xQueueCreate(10, sizeof(IREvent)); // Queue for 10 IR events
	irProcessingSemaphore = xSemaphoreCreateMutex();
	
	if (irEventQueue == NULL || irProcessingSemaphore == NULL) {
		Serial.println("Failed to create IR queue or semaphore!");
		return;
	}

	// Create IR processing task
	xTaskCreatePinnedToCore(
		processIRQueue,    /* Task function. */
		"processIRQueue",  /* name of task. */
		4096,              /* Stack size of task */
		NULL,              /* parameter of the task */
		3,                 /* priority of the task */
		&xHandle_processIRQueue,
		0 /* Task handle to keep track of created task */
	);

	// Create front IR sensor task
	xTaskCreatePinnedToCore(
		handleIRFront,     /* Task function. */
		"handleIRFront",   /* name of task. */
		4096,              /* Stack size of task */
		NULL,              /* parameter of the task */
		2,                 /* priority of the task */
		&xHandle_handleIRFront,
		0 /* Task handle to keep track of created task */
	);

	// Create back IR sensor task
	xTaskCreatePinnedToCore(
		handleIRBack,      /* Task function. */
		"handleIRBack",    /* name of task. */
		4096,              /* Stack size of task */
		NULL,              /* parameter of the task */
		2,                 /* priority of the task */
		&xHandle_handleIRBack,
		1 /* Task handle to keep track of created task */
	);


	// xTaskCreatePinnedToCore(
	// 	handleTrigger,		   /* Task function. */
	// 	"handleTrigger",	   /* name of task. */
	// 	1024,				   /* Stack size of task */
	// 	NULL,				   /* parameter of the task */
	// 	2,					   /* priority of the task */
	// 	&xHandle_triggerButton,
	// 	1 /* Task handle to keep track of created task */
	// );

	xTaskCreatePinnedToCore(				   // Use xTaskCreate() in vanilla FreeRTOS
		handleFire,		   // Function to be called
		"handle fire",			   // Name of task
		1024,					   // Stack size (bytes in ESP32, words in FreeRTOS)
		NULL,					   // Parameter to pass
		3,						   // Task priority (must be same to prevent lockup)
		&xHandle_fire,
		1); // Task handle

	// Start blink task
	xTaskCreatePinnedToCore(					// Use xTaskCreate() in vanilla FreeRTOS
		toggleOnboardLED,			// Function to be called
		"Toggle LED",				// Name of task
		2048,						// Stack size (bytes in ESP32, words in FreeRTOS)
		NULL,						// Parameter to pass
		1,							// Task priority
		&xHandle_toggleOnboardLED,
		1); // Task handle

	// Start serial read task
	// xTaskCreatePinnedToCore(				   // Use xTaskCreate() in vanilla FreeRTOS
	// 	readSerialInput,		   // Function to be called
	// 	"Read Serial",			   // Name of task
	// 	1024,					   // Stack size (bytes in ESP32, words in FreeRTOS)
	// 	NULL,					   // Parameter to pass
	// 	0,						   // Task priority (must be same to prevent lockup)
	// 	&xHandle_readSerialInput,
	// 	1); // Task handle



	vTaskDelay(1000);
	strip.Begin();
 	//strip.SetBrightness(128);
    strip.Show();
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

	// Start serial read task
	xTaskCreatePinnedToCore(				   // Use xTaskCreate() in vanilla FreeRTOS
		handleLED,		   // Function to be called
		"handle LED",			   // Name of task
		4096,					   // Stack size (bytes in ESP32, words in FreeRTOS)
		NULL,					   // Parameter to pass
		2,						   // Task priority (must be same to prevent lockup)
		&xHandle_handleLED,
		1); // Task handle


    Serial.println("Initialize motors...");

    motors = new Motors();
   	chaosMonkey = new MotorChaosMonkey(motors);
    motors->setUpPinModes();
    Serial.println("Motors initialized");

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