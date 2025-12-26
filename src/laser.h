#pragma once

#include <Arduino.h>

class Laser {
public:
    explicit Laser(int laserPin);

    void activate();
    void deactivate();

private:
    int laserPin;
    bool active;
};
