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
#include <ESP32Servo.h>
// #include <tagger.h>
#include <NeoPixelBus.h>
#include <PS4Controller.h>
#include "prefs.h"
#include "colors.h"


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

//Motor
int maxSpeed = 255;
int minSpeed = maxSpeed * -1;

// Right motor
int enableRightMotor = 22;
int rightMotorPin1 = 16;
int rightMotorPin2 = 17;
// Left motor
int enableLeftMotor = 23;
int leftMotorPin1 = 18;
int leftMotorPin2 = 19;
//PWM
const int PWMFreq = 1000; /* 1 KHz */
const int PWMResolution = 8;
const int rightMotorPWMSpeedChannel = 4;
const int leftMotorPWMSpeedChannel = 5;
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

class ActivityLights {

private:
    static const int TEAM_COLOR_START = 0;
    static const int TEAM_COLOR_END = 47;
    static const int HEALTH_START = 48;
    static const int HEALTH_END = 54;
    static const int AMMUNITION_START = 59;
    static const int AMMUNITION_END = 65;
    static const int FIRE_START = HEALTH_START;
    static const int FIRE_END = AMMUNITION_END;
	static const int SHOTS_PER_LED = 5;
	static const unsigned long INVULNERABILITY_DURATION_MS = 3000;
	static const unsigned long FIRE_DURATION_MS = 200;
	static const int HIT_BLINK_COUNT = 5;
	static const unsigned long HIT_BLINK_INTERVAL_MS = 100;

    NeoPixelBus<NeoGrbFeature, NeoEsp32RmtMethodBase<NeoEsp32RmtSpeed800Kbps, NeoEsp32RmtChannel3>>& strip;
    int health;
    int ammunition;
	int totalHealth;
	int totalAmunication;
	RgbColor teamColor;
	
	
	volatile long lastHitMillis = millis();
	
	bool invulnActive = false;
	unsigned long invulnEndMillis = 0;
	
	bool fireActive = false;
	unsigned long fireEndMillis = 0;

	bool hitBlinkActive = false;
	unsigned long hitBlinkNextToggleMillis = 0;
	int hitBlinkRemainingToggles = 0;
	bool hitBlinkOn = false;

	void refreshAllColors() {
		setTeamColorBase();
		updateHealth(0);
		updateAmmunition(0);
	}

	void setTeamColorBase() {
		for (int i = TEAM_COLOR_START; i <= TEAM_COLOR_END; i++) {
			if(i % 2 == 0) {
				strip.SetPixelColor(i, teamColor);
			} else {
				strip.SetPixelColor(i, COLOR_WHITE);
			}
        }
    }

	void updateHealth(int damage) {
		health -= damage;
		int healthLEDs = HEALTH_END - HEALTH_START + 1;
		int ledsToTurnOn =  healthLEDs * health / totalHealth;
		int ledsToTurnOff = healthLEDs - ledsToTurnOn;
		for (int i = HEALTH_END; i >= HEALTH_START; i--) {
			if (i > HEALTH_END - ledsToTurnOff) {
				strip.SetPixelColor(i, COLOR_WHITE);
			} else {
				strip.SetPixelColor(i, COLOR_GREEN_LOW);
			}
		}
		for (int i = HEALTH_END; i <= AMMUNITION_START; i++) {
			strip.SetPixelColor(i, COLOR_WHITE); //clear pixels between health and ammo
		}
	}

	void updateAmmunition(int shotsFired) {
		ammunition -= shotsFired;
		int ammoLEDs = AMMUNITION_START - AMMUNITION_END + 1;
		int ledsToTurnOn =  ammoLEDs * ammunition / totalAmunication;
		int ledsToTurnOff = ammoLEDs - ledsToTurnOn;
		for (int i = AMMUNITION_START; i <= AMMUNITION_END; i++) {
			if (i < AMMUNITION_START + ledsToTurnOff) {
				strip.SetPixelColor(i, COLOR_WHITE);
			} else {
				strip.SetPixelColor(i, COLOR_BLUE_LOW); // Blue for ammunition
			}
		}
	}

	void renderBase() {
		setTeamColorBase();
		updateHealth(0);
		updateAmmunition(0);
	}

	void renderInvulnerabilityOverlay() {
		static uint8_t offset = 0;
		int length = TEAM_COLOR_END - TEAM_COLOR_START + 1;

		for (int i = TEAM_COLOR_START; i <= TEAM_COLOR_END; i++) {
			int logicalIndex = (i - TEAM_COLOR_START + offset) % length;
			if (logicalIndex % 2 == 0) {
				strip.SetPixelColor(i, teamColor);
			} else {
				strip.SetPixelColor(i, COLOR_WHITE);
			}
		}
		offset = (offset + 1) % length;
	}

	void renderFireOverlay() {
		for (int i = FIRE_START; i <= FIRE_END; i++) {
			strip.SetPixelColor(i, COLOR_YELLOW);
		}
	}

	void renderHit() {
		static bool on = false;
		on = !on;
		for (int i = TEAM_COLOR_START; i <= AMMUNITION_END; i++) {
			strip.SetPixelColor(i, on ? COLOR_RED : COLOR_WHITE);
		}
	}



public:
    ActivityLights(NeoPixelBus<NeoGrbFeature, NeoEsp32RmtMethodBase<NeoEsp32RmtSpeed800Kbps, NeoEsp32RmtChannel3>>& strip)
        : strip(strip), health(10), ammunition(100) {
			totalHealth = health;
			totalAmunication = ammunition;
			teamColor = COLOR_ORANGE;
		}

	void setTeamColor(RgbColor color) {
		teamColor = color;
	}

	void onHit() {
		lastHitMillis = millis();
		// Start hit blink phase (overrides other effects in renderFrame)
		hitBlinkActive = true;
		hitBlinkOn = true;
		hitBlinkRemainingToggles = HIT_BLINK_COUNT * 2; // on+off per blink
		hitBlinkNextToggleMillis = lastHitMillis + HIT_BLINK_INTERVAL_MS;
		// Apply damage
		updateHealth(1);
		// Start invulnerability window; will be visible after hit blink ends
		invulnActive = true;
		invulnEndMillis = lastHitMillis + INVULNERABILITY_DURATION_MS;
	}

	void onFire() {
		fireActive = true;
		fireEndMillis = millis() + FIRE_DURATION_MS;
		updateAmmunition(1);
	}

	boolean canBeHit() {
		return lastHitMillis + INVULNERABILITY_DURATION_MS < millis();
	}

	boolean canFire() {
		return lastHitMillis + 600 < millis();
	}


	boolean isGameOver() {
		return health <= 0;
	}

	void updateFrame() {
		unsigned long now = millis();
		// Advance hit blink state
		if (hitBlinkActive && now >= hitBlinkNextToggleMillis) {
			hitBlinkOn = !hitBlinkOn;
			hitBlinkNextToggleMillis = now + HIT_BLINK_INTERVAL_MS;
			hitBlinkRemainingToggles--;
			if (hitBlinkRemainingToggles <= 0) {
				hitBlinkActive = false;
				hitBlinkOn = false;
			}
		}

		if (invulnActive && now >= invulnEndMillis) {
			invulnActive = false;
		}
		if (fireActive && now >= fireEndMillis) {
			fireActive = false;
		}
	}

	void renderFrame() {
		if (isGameOver()) {
			renderHit();
			strip.Show();
			return;
		}
		// Hit blink phase overrides everything else
		if (hitBlinkActive) {
			renderHit();
		} else {
			// base state
			renderBase();
			// overlays (can combine)
			if (invulnActive) {
				renderInvulnerabilityOverlay();
			}
			if (fireActive) {
				renderFireOverlay();
			}
		}
		strip.Show();
	}
};

class Motors
{

  void setMotorDirections(int motorPin1, int motorPin2, int speed)
  {
    int pin1State = (speed > 0) ? HIGH : LOW;
    int pin2State = (speed < 0) ? HIGH : LOW;

    digitalWrite(motorPin1, pin1State);
    digitalWrite(motorPin2, pin2State);
  }

public:
  void rotateMotor(int rightMotorSpeed, int leftMotorSpeed)
  {
    setMotorDirections(rightMotorPin1, rightMotorPin2, rightMotorSpeed);
    setMotorDirections(leftMotorPin1, leftMotorPin2, leftMotorSpeed);
    ledcWrite(rightMotorPWMSpeedChannel, abs(rightMotorSpeed));
    ledcWrite(leftMotorPWMSpeedChannel, abs(leftMotorSpeed));
  }

  void setUpPinModes()
  {
    pinMode(enableRightMotor, OUTPUT);
    pinMode(rightMotorPin1, OUTPUT);
    pinMode(rightMotorPin2, OUTPUT);

    pinMode(enableLeftMotor, OUTPUT);
    pinMode(leftMotorPin1, OUTPUT);
    pinMode(leftMotorPin2, OUTPUT);

    // Set up PWM for motor speed
    ledcSetup(rightMotorPWMSpeedChannel, PWMFreq, PWMResolution);
    ledcSetup(leftMotorPWMSpeedChannel, PWMFreq, PWMResolution);
    ledcAttachPin(enableRightMotor, rightMotorPWMSpeedChannel);
    ledcAttachPin(enableLeftMotor, leftMotorPWMSpeedChannel);

    rotateMotor(0, 0);
  }
};

class MotorChaosMonkey
{

  unsigned long startMillis; // will store last time LED was updated
  volatile boolean active = false;
  boolean phaseOneExecuted;
  boolean phaseTwoExecuted;
  unsigned long chaosDurationMillis = 800;
  unsigned long coolDownMillis = 3000;
  Motors *motors;

public:
  MotorChaosMonkey(Motors *motors_)
  {
    motors = motors_;
  }

  void Start()
  {
     unsigned long currentMillis = millis();
if (!active && (currentMillis - startMillis) >= coolDownMillis)
    {
      active = true;
      startMillis = millis();
      Serial.println((String) "chaos started");
    }
    else
    {
      Serial.println((String) "chaos won't start because it is already active");
    }
  }

  boolean isActive()
  {
    return active;
  }

void Update()
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
        Serial.println((String) "chaos terminated after: " + chaosDurationMillis);
      }
      else
      {
        if (!phaseOneExecuted)
        {
          motors->rotateMotor(maxSpeed, -maxSpeed);
          phaseOneExecuted = true;
        }
        if (phaseOneExecuted && !phaseTwoExecuted && currentMillis - startMillis >= (chaosDurationMillis / 2))
        {
          motors->rotateMotor(-maxSpeed, maxSpeed);
          phaseTwoExecuted = true;
        }
      }
    }
  }
};

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
						// Two-stick: right Y for forward/back, left X for turning
						// Use left Y for in-place spins when no forward/back input
						int rawY = PS4.RStickY() * direction;
						int rawTurn = PS4.LStickX();
						int rawSpin = PS4.LStickY();
						if (abs(rawY) < 10 && abs(rawTurn) < 10) {
							// If user only uses left Y, treat it as in-place rotation
							stickY = 0;
							stickX = rawSpin;
						} else {
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

					rightMotorSpeedRaw = constrain(stickY - stickXWithExpo, -127, 127);
					leftMotorSpeedRaw = constrain(stickY + stickXWithExpo, -127, 127);

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
	const TickType_t frameDelay = pdMS_TO_TICKS(100); // 
	while (1) {
		lights.updateFrame();
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