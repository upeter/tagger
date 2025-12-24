#pragma once

#include <Arduino.h>
#include <NeoPixelBus.h>
#include "colors.h"

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

	// Invulnerability animation timing (so we only animate during invuln window)
	unsigned long lastInvulnAnimMillis = 0;
	static const unsigned long INVULN_FRAME_INTERVAL_MS = 100;

	// When false, LED strip is left as-is and not updated
	volatile bool needsRender = false;

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
		for (int i = TEAM_COLOR_START; i <= AMMUNITION_END; i++) {
			strip.SetPixelColor(i, hitBlinkOn ? COLOR_RED : COLOR_WHITE);
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
		needsRender = true;
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
		lastInvulnAnimMillis = lastHitMillis;
		needsRender = true;
	}

	void onFire() {
		fireActive = true;
		fireEndMillis = millis() + FIRE_DURATION_MS;
		updateAmmunition(1);
		needsRender = true;
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

    // Track previous state so we know if anything changed this tick
    bool prevHitBlinkActive = hitBlinkActive;
    bool prevInvulnActive   = invulnActive;
    bool prevFireActive     = fireActive;
    bool prevHitBlinkOn     = hitBlinkOn;

    if (isGameOver()) {
        // Game over: infinite hit blink with proper interval
        if (!hitBlinkActive) {
            // Ensure we enter blink mode when game over first happens
            hitBlinkActive = true;
            hitBlinkOn = true;
            hitBlinkNextToggleMillis = now + HIT_BLINK_INTERVAL_MS;
        }

        if (now >= hitBlinkNextToggleMillis) {
            hitBlinkOn = !hitBlinkOn;
            hitBlinkNextToggleMillis = now + HIT_BLINK_INTERVAL_MS;
        }

        // Only thing that matters for rendering in game over is hitBlinkOn
        if (prevHitBlinkOn != hitBlinkOn) {
            needsRender = true;
        }

        return; // Nothing else (invuln/fire) is relevant in game over
    } else {
        // Normal play: finite hit blink, invulnerability and fire windows

        if (hitBlinkActive && now >= hitBlinkNextToggleMillis) {
            hitBlinkOn = !hitBlinkOn;
            hitBlinkNextToggleMillis = now + HIT_BLINK_INTERVAL_MS;

            hitBlinkRemainingToggles--;
            if (hitBlinkRemainingToggles <= 0) {
                hitBlinkActive = false;
                hitBlinkOn = false;
            }
        }

        // End of invulnerability window
        if (invulnActive && now >= invulnEndMillis) {
            invulnActive = false;
        }

        // End of fire window
        if (fireActive && now >= fireEndMillis) {
            fireActive = false;
        }

        // While invulnerable and NOT in hit blink, drive the invulnerability animation
        if (!hitBlinkActive && invulnActive) {
            if (now - lastInvulnAnimMillis >= INVULN_FRAME_INTERVAL_MS) {
                lastInvulnAnimMillis = now;
                needsRender = true;
            }
        }

        // Hit blink animation changes: render
        if (prevHitBlinkActive != hitBlinkActive || prevHitBlinkOn != hitBlinkOn) {
            needsRender = true;
        }

        // Invulnerability started or ended: render once
        if (prevInvulnActive != invulnActive) {
            needsRender = true;
        }

        // Fire started or ended: render once
        if (prevFireActive != fireActive) {
            needsRender = true;
        }

        return;
    }
}

	void renderFrame() {
		if (!needsRender) {
			return;
		}
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
		needsRender = false;
	}
};
