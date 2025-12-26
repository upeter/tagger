#include <Arduino.h>
#include "flasher.h"

// led_delay is defined in main.cpp
extern int led_delay;

Flasher::Flasher(int ledPin, long flashDuration)
    : ledPin(ledPin), flashDuration(flashDuration) {
    pinMode(ledPin, OUTPUT);
}

void Flasher::flash() {
    long startTime = millis();
    while (millis() - startTime < flashDuration) {
        digitalWrite(ledPin, HIGH);
        vTaskDelay(led_delay / portTICK_PERIOD_MS);
        digitalWrite(ledPin, LOW);
        vTaskDelay(led_delay / portTICK_PERIOD_MS);
    }
}
