#include "laser.h"

Laser::Laser(int laserPin) : laserPin(laserPin), active(false) {
    pinMode(laserPin, OUTPUT);
}

void Laser::activate() {
    if (!active) {
        active = true;
        digitalWrite(laserPin, HIGH);
    }
}

void Laser::deactivate() {
    if (active) {
        active = false;
        digitalWrite(laserPin, LOW);
    }
}
