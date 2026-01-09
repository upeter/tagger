#pragma once

#include <Arduino.h>
#include <NeoPixelBus.h>
#include "colors.h"

class ActivityLights {

private:
    static const int TEAM_COLOR_START = 0;
    static const int TEAM_COLOR_END = 43; //43
    static const int HEALTH_START = 44; //44
    static const int HEALTH_END = 50; //50
    static const int AMMUNITION_START = 55; //55
    static const int AMMUNITION_END = 61; //61
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

	void refreshAllColors();
	void setTeamColorBase();
	void updateHealth(int damage);
	void updateAmmunition(int shotsFired);
	void renderBase();
	void renderInvulnerabilityOverlay();
	void renderFireOverlay();
	void renderHit();


public:
	ActivityLights(NeoPixelBus<NeoGrbFeature, NeoEsp32RmtMethodBase<NeoEsp32RmtSpeed800Kbps, NeoEsp32RmtChannel3>>& strip);

	void setTeamColor(RgbColor color);
	void onHit();
	void onFire();
	boolean canBeHit();
	boolean canFire();

	boolean isGameOver();

	void updateFrame();
	void renderFrame();
};
