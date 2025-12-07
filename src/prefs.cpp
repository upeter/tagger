#include "prefs.h"
#include <NeoPixelBus.h>

static bool prefsInitialized = false;

void prefsInit() {
    if (prefsInitialized) return;
    if (!EEPROM.begin(EEPROM_SIZE_PREFERENCES)) {
        Serial.println("EEPROM init failed");
        return;
    }
    prefsInitialized = true;
}

static Prefs emptyPrefs() {
    Prefs p;
    p.color = RgbColor(0, 0, 0);
    p.direction = 1;
    p.hasColor = false;
    p.hasDirection = false;
    return p;
}

Prefs prefsLoad() {
    prefsInit();
    if (!prefsInitialized) return emptyPrefs();

    uint8_t magic = EEPROM.read(EEPROM_ADDR_MAGIC_PREFERENCES);
    if (magic != EEPROM_MAGIC_PREFERENCES) {
        Serial.println("No user prefs stored, using defaults");
        return emptyPrefs();
    }

    Prefs p = emptyPrefs();
    uint8_t r = EEPROM.read(EEPROM_ADDR_TEAM_R);
    uint8_t g = EEPROM.read(EEPROM_ADDR_TEAM_G);
    uint8_t b = EEPROM.read(EEPROM_ADDR_TEAM_B);
    p.color = RgbColor(r, g, b);
    p.hasColor = true;

    uint8_t encodedDir = EEPROM.read(EEPROM_ADDR_DIRECTION);
    p.direction = (encodedDir == 0) ? -1 : 1;
    p.hasDirection = true;

    Serial.printf("Restored prefs: R=%u G=%u B=%u, dir=%d\n", p.color.R, p.color.G, p.color.B, p.direction);
    return p;
}

Prefs prefsWithTeamColor(const Prefs &base, const RgbColor &color) {
    prefsInit();
    if (prefsInitialized) {
        EEPROM.write(EEPROM_ADDR_TEAM_R, color.R);
        EEPROM.write(EEPROM_ADDR_TEAM_G, color.G);
        EEPROM.write(EEPROM_ADDR_TEAM_B, color.B);
        EEPROM.write(EEPROM_ADDR_MAGIC_PREFERENCES, EEPROM_MAGIC_PREFERENCES);
        EEPROM.commit();
    }

    Prefs p = base;
    p.color = color;
    p.hasColor = true;
    return p;
}

Prefs prefsWithDirection(const Prefs &base, int direction) {
    prefsInit();
    if (prefsInitialized) {
        uint8_t encoded = (direction >= 0) ? 1 : 0;
        EEPROM.write(EEPROM_ADDR_DIRECTION, encoded);
        EEPROM.write(EEPROM_ADDR_MAGIC_PREFERENCES, EEPROM_MAGIC_PREFERENCES);
        EEPROM.commit();
    }

    Prefs p = base;
    p.direction = direction;
    p.hasDirection = true;
    return p;
}
