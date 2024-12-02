/**
 * @file tagger.cpp
 * @author gldhnchn (gldhnchn@posteo.de)
 * @brief global variables
 * @date 2019-01-23
 *  
 */

#include <FastLED.h>
#include <Arduino.h>

// Define the number of LEDs and the data pin
#define NUM_LEDS 100
#define LED_DATA_PIN 5
#define LED_OVERALL_BRIGHTNESS 128
#define LED_INDEX_BT 0
#define COLOR_BT_CONNECTION_OFF CRGB::Red

// Create an array to hold the LED data
CRGB leds2[NUM_LEDS];
TaskHandle_t xHandle_blinkLEDs,xHandle_rainbowEffect;

// Function to handle the LED blinking
void blinkLEDsTask(void *pvParameters) {
    CRGB color = *((CRGB*)pvParameters);
    while (true) {
        // Turn the LEDs on with the specified color
        for (int i = 0; i < NUM_LEDS; i++) {
            leds2[i] = color;
        }
        FastLED.show();
        vTaskDelay(500 / portTICK_PERIOD_MS); // Delay for 500ms

        // Turn the LEDs off
        for (int i = 0; i < NUM_LEDS; i++) {
            leds2[i] = CRGB::Black;
        }
        FastLED.show();
        vTaskDelay(500 / portTICK_PERIOD_MS); // Delay for 500ms
    }
}


void rainbowEffectTask(void *pvParameters) {
    uint8_t hue = 0;
    while (true) {
        // Fill the LEDs with a rainbow pattern
        for (int i = 0; i < NUM_LEDS; i++) {
            leds2[i] = CHSV(hue + (i * 256 / NUM_LEDS), 255, 255);
        }
        FastLED.show();
        hue += 1;
        vTaskDelay(5 / portTICK_PERIOD_MS); // Delay for 20ms
    }
}



// Function to start the rainbow effect task
void createRainbowEffectTask() {
    // Create the rainbow effect task
    xTaskCreate(
        rainbowEffectTask, // Task function
        "RainbowEffectTask", // Name of the task
        1000, // Stack size (in words)
        NULL, // Task input parameter
        1, // Priority of the task
        &xHandle_rainbowEffect // Task handle
    );
}

// Function to start the LED blinking task
void createBlinkTask(CRGB color) {
    // Create the LED blinking task
    xTaskCreate(
        blinkLEDsTask, // Task function
        "BlinkLEDsTask", // Name of the task
        1000, // Stack size (in words)
        new CRGB(color), // Task input parameter
        1, // Priority of the task
        &xHandle_blinkLEDs // Task handle
    );
}



void setup____() {
    Serial.begin(115200);
    Serial.println("Hello, this is Open Laser Tag");

    // Initialize the FastLED library
    FastLED.addLeds<NEOPIXEL, LED_DATA_PIN>(leds2, NUM_LEDS);
    FastLED.setBrightness(LED_OVERALL_BRIGHTNESS);

  //createBlinkTask(CRGB::Green);
  createRainbowEffectTask();
}

void loop____() {
    // Your main code here
}


