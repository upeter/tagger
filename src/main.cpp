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
#include <arduino.h>
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


#define LED_BUILTIN 2

// Use only core 1 for demo purposes
#if CONFIG_FREERTOS_UNICORE
  static const BaseType_t app_cpu = 0;
#else
  static const BaseType_t app_cpu = 1;
#endif

#define IR_RMT_RX_FRONT_CHANNEL RMT_CHANNEL_1
#define IR_RMT_TX_CHANNEL RMT_CHANNEL_0 /*!< RMT channel for transmitter */
#define IR_PROTOCOL "NEC"              //choose from timing groups lib/IR32/src/IR32.h
#define IR_RMT_TX_GPIO_NUM GPIO_NUM_2 /*!< GPIO number for transmitter signal */
#define IR_RECV_FRONT_PIN 27
#define IR_RECV_BACK_1_PIN 26
#define DEBOUNCE_TIME_MS 50


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

TaskHandle_t xHandle_handleIR,
	xHandle_toggleOnboardLED,
    xHandle_readSerialInput,
	xHandle_triggerButton, 
	xHandle_handleLED,
	xHandle_fire;
const float brightness = 0.1;
RgbColor applyBrightness(const RgbColor& color) {
    uint8_t r = color.R * brightness;
    uint8_t g = color.G * brightness;
    uint8_t b = color.B * brightness;
    return RgbColor(r, g, b);
}
const RgbColor WHITE = RgbColor(0, 0, 0);
const RgbColor RED = RgbColor(255 , 0, 0);
const RgbColor RED_LOW = applyBrightness(RED);
const RgbColor ORANGE = RgbColor(255 , 165 , 0);
const RgbColor ORANGE_LOW = applyBrightness(ORANGE);
const RgbColor PINK = RgbColor(255 * 0.5, 192 * 0.5, 203 * 0.5);
const RgbColor PINK_LOW = applyBrightness(PINK);

const RgbColor YELLOW = RgbColor(255, 255 , 0);
const RgbColor YELLOW_LOW = applyBrightness(YELLOW);
const RgbColor BLUE = RgbColor(0, 0, 255 );
const RgbColor BLUE_LOW = applyBrightness(BLUE);
const RgbColor GREEN = RgbColor(0, 255 , 0);
const RgbColor GREEN_LOW = applyBrightness(GREEN);

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

    NeoPixelBus<NeoGrbFeature, NeoEsp32RmtMethodBase<NeoEsp32RmtSpeed800Kbps, NeoEsp32RmtChannel2>>& strip;
    int health;
    int ammunition;
	int totalHealth;
	int totalAmunication;
	RgbColor teamColor;
	
	volatile bool isHitActive = false;
	enum LedCommand {
		NONE,
		ALL,
		SET_TEAM_COLOR_COLOR,
		FIRE,
		HIT
	};
	LedCommand ledCommand = NONE;

	void refreshAllColors() {
			setTeamColor();
			updateHealth(0);
			updateAmmunition(0);
	}

	void setTeamColor() {
        for (int i = TEAM_COLOR_START; i <= TEAM_COLOR_END; i = i+2) {
            strip.SetPixelColor(i, teamColor);
        }
        strip.Show();
    }

    void updateHealth(int damage) {
        health -= damage;
		int healthLEDs = HEALTH_END - HEALTH_START + 1;
		int ledsToTurnOn =  healthLEDs * health / totalHealth;
		int ledsToTurnOff = healthLEDs - ledsToTurnOn;
		 for (int i = HEALTH_END; i >= HEALTH_START; i--) {
            if (i > HEALTH_END - ledsToTurnOff) {
                strip.SetPixelColor(i, WHITE);
            } else {
				strip.SetPixelColor(i, GREEN_LOW);
            }
        }
		strip.Show();
	}


	void fire() {
		for (int j = 0; j < 1; j++) {
			for (int i = FIRE_START; i <= FIRE_END; i++) {
				strip.SetPixelColor(i, YELLOW); 
			}
			strip.Show();
			vTaskDelay(100);
			for (int i = FIRE_START; i <= FIRE_END; i++) {
				strip.SetPixelColor(i, WHITE); // Off
			}
			strip.Show();
		   vTaskDelay(100);
		}
		updateAmmunition(1);
	}

	void updateAmmunition(int shotsFired) {
		ammunition -= shotsFired;
		int ammoLEDs = AMMUNITION_START - AMMUNITION_END + 1;
		int ledsToTurnOn =  ammoLEDs * ammunition / totalAmunication;
		int ledsToTurnOff = ammoLEDs - ledsToTurnOn;
		for (int i = AMMUNITION_START; i <= AMMUNITION_END; i++) {
			if (i < AMMUNITION_START + ledsToTurnOff) {
				strip.SetPixelColor(i, WHITE);
			} else {
				strip.SetPixelColor(i, BLUE_LOW); // Blue for ammunition
			}
		}
		strip.Show();
	}

    void hit() {
        for (int j = 0; j < 5; j++) {
            blinkRed();
        }
		updateHealth(1);
		isHitActive = false;
		if(isGameOver()) {	
		 	gameOver();
		}
    }

	void gameOver() {
		while(true) {
			blinkRed();
		}
	}

	void blinkRed() {
		for (int i = TEAM_COLOR_START; i <= AMMUNITION_END; i++) {
                strip.SetPixelColor(i, RED); // Red
            }
            strip.Show();
            vTaskDelay(100);
            for (int i = TEAM_COLOR_START; i <= AMMUNITION_END; i++) {
                strip.SetPixelColor(i, WHITE); // Off
            }
            strip.Show();
            vTaskDelay(100);
	}



public:
    ActivityLights(NeoPixelBus<NeoGrbFeature, NeoEsp32RmtMethodBase<NeoEsp32RmtSpeed800Kbps, NeoEsp32RmtChannel2>>& strip)
        : strip(strip), health(6), ammunition(100) {
			totalHealth = health;
			totalAmunication = ammunition;
			teamColor = ORANGE;
		}

	void triggerRefreshAllColors() {
			ledCommand = ALL;
			vTaskResume(xHandle_handleLED);
	}

	void triggerSetTeamColor(RgbColor color) {
		teamColor = color;
		ledCommand = SET_TEAM_COLOR_COLOR;
		vTaskResume(xHandle_handleLED);
	}

	void triggerFire() {
		if(!isHitActive) {
			ledCommand = FIRE;
			vTaskResume(xHandle_handleLED);
		}
	}

	void triggerHit() {
		if(!isHitActive) {
			isHitActive = true;
			ledCommand = HIT;
			vTaskResume(xHandle_handleLED);
		}
	}

	boolean isGameOver() {
		return health <= 0;
	}

  

	void exectuteCommand() {
		switch (ledCommand)
		{
		case SET_TEAM_COLOR_COLOR:
			refreshAllColors();
			break;
		case FIRE:
			fire();
			refreshAllColors();
			break;
		case HIT:
			hit();
			refreshAllColors();
			break;
		case ALL:
			refreshAllColors();
			break;
		}
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

NeoPixelBus<NeoGrbFeature, NeoEsp32RmtMethodBase<NeoEsp32RmtSpeed800Kbps, NeoEsp32RmtChannel2>> strip(NUM_LEDS, LED_DATA_PIN);
Motors *motors;
MotorChaosMonkey *chaosMonkey;
IRRecv ir_rec(IR_RMT_RX_FRONT_CHANNEL);
IRSend ir_send(IR_RMT_TX_CHANNEL);
Button triggerButton(trigger_pin);
Flasher flasher(led_pin, 1000);
Laser laser(laser_pin);
ActivityLights lights(strip);
uint32_t lastTriggerDebounceTime = 0;

unsigned long lastTimeStamp = 0;
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
			if(!lights.isGameOver()) {
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
						lights.triggerSetTeamColor(PINK);
					} else if(PS4.Cross()) {
						lights.triggerSetTeamColor(BLUE);
					} else if (PS4.Circle()){
						lights.triggerSetTeamColor(ORANGE);
					} else if (PS4.Triangle()) {
						lights.triggerSetTeamColor(GREEN);
					}

					stickY = PS4.RStickY();// * -1;
					stickX = PS4.RStickX();
					float stickXExpo = withExpo(stickX);
					//stickX = (toggle ? (int)((float)stickX * stickXExpo) : stickX / 3) * -1;
					stickXWithExpo = constrain((int)((float(stickX) * stickXExpo) + trimX) * -1, maxX * -1, maxX);

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
 * @brief handle incoming infrared
 * This function reads the incoming infrared data and sends it via bluetooth.
 * @param parameter just a dummy parameter
 */
void handleIR(void *parameter)
{
    Serial.println("handle IR task started");
    Serial.println("init ir rec");
    ir_rec.setPreferred(IR_PROTOCOL);
    ir_rec.start(IR_RECV_FRONT_PIN);
	Serial.println("init ir rec init");
	uint32_t counter = 0;
	uint32_t last = millis();
    while (true){
		try {
		// taskYIELD();
		counter++;
		if((millis() - last) > 1000) {
			rtc_wdt_feed();		
			//Serial.println("available: " + (String)counter);
			vTaskDelay(50);
			last = millis();
		}
        if (ir_rec.available()){
            Serial.println("IR available");
            char *rcvGroup = nullptr;
            uint32_t result = ir_rec.read(rcvGroup);
            if (result){
				Serial.printf("Received: %s/0x%x\n", rcvGroup, result);
				vTaskResume(xHandle_toggleOnboardLED);
				lights.triggerHit();
				chaosMonkey->Start();
            }
			vTaskDelay(100);
        }
		 } catch (const std::invalid_argument& e) {
    			Serial.println("Error occured ");
  		}
	}
    return;
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
				Serial.println(">>> Shot!");
				ir_send.send(ir_messages[0]);
				lights.triggerFire();
				vTaskDelay(500);
			} catch (const std::invalid_argument& e) {
				Serial.print("Error occurred: ");
                Serial.println(e.what()); // Log the exception text
				ir_send.stop();
				ir_send.start(IR_RMT_TX_GPIO_NUM, IR_PROTOCOL);
			}
	}
}

			


void handleLED(void *parameter){

	while(true) {
		// Suspend the task
		vTaskSuspend(NULL);
		Serial.println(">>> ws2812fxTask!");
		lights.exectuteCommand();
        // Loop to run the LED blinking code 5 times
        // for (int j = 0; j < 5; j++) {
		// 	// Example: Blink LEDs
		// 	for (int i = 0; i < NUM_LEDS; i++) {
		// 		 strip.SetPixelColor(i, RED); // Red color
		// 	}
		// 	strip.Show();
		// 	vTaskDelay(100);
		// 	for (int i = 0; i < NUM_LEDS; i++) {
		// 		 strip.SetPixelColor(i, WHITE); // Turn off
		// 	}
		// 	strip.Show();
		// 	vTaskDelay(100);
		// }
		Serial.println(">>> ws2812fxTask! DONE");
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

 	xTaskCreatePinnedToCore(
        handleIR,         /* Task function. */
        "handleIR",       /* name of task. */
        4096,              /* Stack size of task */
        NULL,              /* parameter of the task */
        3,                 /* priority of the task */
        &xHandle_handleIR,
		0 /* Task handle to keep track of created task */
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
		2,						   // Task priority (must be same to prevent lockup)
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
	xTaskCreatePinnedToCore(				   // Use xTaskCreate() in vanilla FreeRTOS
		readSerialInput,		   // Function to be called
		"Read Serial",			   // Name of task
		1024,					   // Stack size (bytes in ESP32, words in FreeRTOS)
		NULL,					   // Parameter to pass
		0,						   // Task priority (must be same to prevent lockup)
		&xHandle_readSerialInput,
		1); // Task handle

// Start serial read task
	xTaskCreatePinnedToCore(				   // Use xTaskCreate() in vanilla FreeRTOS
		handleLED,		   // Function to be called
		"handle LED",			   // Name of task
		1024,					   // Stack size (bytes in ESP32, words in FreeRTOS)
		NULL,					   // Parameter to pass
		1,						   // Task priority (must be same to prevent lockup)
		&xHandle_handleLED,
		1); // Task handle



	vTaskDelay(1000);
	strip.Begin();
 	//strip.SetBrightness(128);
    strip.Show();
	lights.triggerRefreshAllColors();

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


	// for (int i = 0; i < NUM_LEDS; i++) {
	// 			strip.setPixelColor(i, strip.Color(255, 255, 0)); // Red color
	// 		}
	// 		strip.show();


	// Delete "setup and loop" task
	vTaskDelete(NULL);



}

void loop() {
  // Execution should never get here
   vTaskDelay(portMAX_DELAY); /*wait as much as possible ... */
}