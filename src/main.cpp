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
#define IR_RMT_TX_GPIO_NUM GPIO_NUM_22 /*!< GPIO number for transmitter signal */
#define IR_RECV_FRONT_PIN 27
#define DEBOUNCE_TIME_MS 50


#define LED_DATA_PIN 5
#define NUM_LEDS 8

// Settings
static const uint8_t buf_len = 20;

// Pins
static const int led_pin = LED_BUILTIN;
static const int trigger_pin = 21;
static const int trigger_servo_pin = 18;

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


// Objects
IRRecv ir_rec(IR_RMT_RX_FRONT_CHANNEL);
IRSend ir_send(IR_RMT_TX_CHANNEL);
Button triggerButton(trigger_pin);
Flasher flasher(led_pin, 1000);
uint32_t lastTriggerDebounceTime = 0;
//Adafruit_NeoPixel strip = Adafruit_NeoPixel(NUM_LEDS, LED_DATA_PIN, NEO_GRB + NEO_KHZ800);
NeoPixelBus<NeoGrbFeature, NeoEsp32RmtMethodBase<NeoEsp32RmtSpeed800Kbps, NeoEsp32RmtChannel2>> strip(NUM_LEDS, LED_DATA_PIN);

unsigned long lastTimeStamp = 0;
void notify()
{
	char messageString[200];

	try
	{
		if (millis() - lastTimeStamp > 100)
		{
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
			if (PS4.R1() || PS4.L1())
			{
					Serial.println(">>> Shot!");
					ir_send.send(ir_messages[0]);
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
	Serial.println("Disconnected!.");
}


//*****************************************************************************
// Tasks

TaskHandle_t xHandle_handleIR,
    xHandle_toggleOnboardLED,
    xHandle_readSerialInput,
	xHandle_triggerButton, 
	xHandle_handleLED;


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
				vTaskResume(xHandle_handleLED);
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

void handleLED(void *parameter){

	while(true) {
		// Suspend the task
		vTaskSuspend(NULL);
		Serial.println(">>> ws2812fxTask!");
        // Loop to run the LED blinking code 5 times
        for (int j = 0; j < 5; j++) {
			// Example: Blink LEDs
			for (int i = 0; i < NUM_LEDS; i++) {
				 strip.SetPixelColor(i, RgbColor(255, 0, 0)); // Red color
			}
			strip.Show();
			vTaskDelay(100);
			for (int i = 0; i < NUM_LEDS; i++) {
				 strip.SetPixelColor(i, RgbColor(0, 0, 0)); // Turn off
			}
			strip.Show();
			vTaskDelay(100);
		}
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

	xTaskCreatePinnedToCore(
		handleTrigger,		   /* Task function. */
		"handleTrigger",	   /* name of task. */
		1024,				   /* Stack size of task */
		NULL,				   /* parameter of the task */
		2,					   /* priority of the task */
		&xHandle_triggerButton,
		1 /* Task handle to keep track of created task */
	);

	// Start blink task
	xTaskCreatePinnedToCore(					// Use xTaskCreate() in vanilla FreeRTOS
		toggleOnboardLED,			// Function to be called
		"Toggle LED",				// Name of task
		1024,						// Stack size (bytes in ESP32, words in FreeRTOS)
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
    strip.Show();

	PS4.attach(notify);
	PS4.attachOnConnect(onConnect);
	PS4.attachOnDisconnect(onDisConnect);
	PS4.begin();


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