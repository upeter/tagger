#pragma once

#include <Arduino.h>

class Flasher {
public:
    Flasher(int ledPin, long flashDuration);

    void flash();

private:
    int ledPin;
    long flashDuration;
};
