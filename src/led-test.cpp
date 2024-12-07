/**
 * @file tagger.cpp
 * @author gldhnchn (gldhnchn@posteo.de)
 * @brief global variables
 * @date 2019-01-23
 *  
 */
//#define FASTLED_RMT_BUILTIN_DRIVER 1
//#include <FastLED.h>
#include <Arduino.h>
#include <Adafruit_NeoPixel.h>


// Define the number of LEDs and the data pin
#define NUM_LEDS 8
#define LED_DATA_PIN 5
#define LED_OVERALL_BRIGHTNESS 128
#define LED_INDEX_BT 0
#define COLOR_BT_CONNECTION_OFF CRGB::Red

// Create an array to hold the LED data
//CRGB leds2[NUM_LEDS];
Adafruit_NeoPixel strip2 = Adafruit_NeoPixel(NUM_LEDS, LED_DATA_PIN, NEO_GRB + NEO_KHZ800);

TaskHandle_t xHandle_blinkLEDs,xHandle_rainbowEffect, xHandle_handleLEDs;

// Function to handle the LED blinking
// void blinkLEDsTask(void *pvParameters) {
//     CRGB color = *((CRGB*)pvParameters);
//     while (true) {
//         // Turn the LEDs on with the specified color
//         for (int i = 0; i < NUM_LEDS; i++) {
//             leds2[i] = color;
//         }
//         FastLED.show();
//         vTaskDelay(500 / portTICK_PERIOD_MS); // Delay for 500ms

//         // Turn the LEDs off
//         for (int i = 0; i < NUM_LEDS; i++) {
//             leds2[i] = CRGB::Black;
//         }
//         FastLED.show();
//         vTaskDelay(500 / portTICK_PERIOD_MS); // Delay for 500ms
// 		Serial.println("Tick");
//     }
// }

void handleLEDTask(void *parameter){

	while(true) {
		// Suspend the task
		Serial.println(">>> ws2812fxTask!");
        // Loop to run the LED blinking code 5 times
        for (int j = 0; j < 5; j++) {
			// Example: Blink LEDs
			for (int i = 0; i < NUM_LEDS; i++) {
				strip2.setPixelColor(i, strip2.Color(255, 0, 0)); // Red color
			}
			strip2.show();
			vTaskDelay(100);
			for (int i = 0; i < NUM_LEDS; i++) {
				strip2.setPixelColor(i, strip2.Color(0, 0, 0)); // Turn off
			}
			strip2.show();
			vTaskDelay(100);
		}
		Serial.println(">>> ws2812fxTask! DONE");
	}
}


// void rainbowEffectTask(void *pvParameters) {
//     uint8_t hue = 0;
//     while (true) {
//         // Fill the LEDs with a rainbow pattern
//         for (int i = 0; i < NUM_LEDS; i++) {
//             leds2[i] = CHSV(hue + (i * 256 / NUM_LEDS), 255, 255);
//         }
//         FastLED.show();
//         hue += 1;
//         vTaskDelay(5 / portTICK_PERIOD_MS); // Delay for 20ms
//     }
// }



// // Function to start the rainbow effect task
// void createRainbowEffectTask() {
//     // Create the rainbow effect task
//     xTaskCreate(
//         rainbowEffectTask, // Task function
//         "RainbowEffectTask", // Name of the task
//         1000, // Stack size (in words)
//         NULL, // Task input parameter
//         1, // Priority of the task
//         &xHandle_rainbowEffect // Task handle
//     );
// }

// Function to start the LED blinking task
// void createBlinkTask(CRGB color) {
//     // Create the LED blinking task
//     xTaskCreate(
//         blinkLEDsTask, // Task function
//         "BlinkLEDsTask", // Name of the task
//         1000, // Stack size (in words)
//         new CRGB(color), // Task input parameter
//         1, // Priority of the task
//         &xHandle_blinkLEDs // Task handle
//     );
// }

// Function to start the LED blinking task
void createLEDTask() {
    // Create the LED blinking task
   xTaskCreate(
        handleLEDTask, // Task function
        "handleLEDTask", // Name of the task
        2048, // Stack size (in words)
        NULL, // Task input parameter
        1, // Priority of the task
        &xHandle_handleLEDs // Task handle
    );
}




void setup____() {
    Serial.begin(115200);
    Serial.println("Starting");

    //Initialize the FastLED library
    // FastLED.addLeds<NEOPIXEL, LED_DATA_PIN>(leds2, NUM_LEDS);
    // FastLED.setBrightness(LED_OVERALL_BRIGHTNESS);
	 Serial.println("Done init");

//   createBlinkTask(CRGB::Green);
createLEDTask();
  Serial.println("Created task");
 
}

void loop____() {
    // Your main code here
}


