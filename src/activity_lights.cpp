#include "activity_lights.h"

ActivityLights::ActivityLights(NeoPixelBus<NeoGrbFeature, NeoEsp32RmtMethodBase<NeoEsp32RmtSpeed800Kbps, NeoEsp32RmtChannel3>>& strip)
    : strip(strip), health(10), ammunition(100)
{
    totalHealth = health;
    totalAmunication = ammunition;
    teamColor = COLOR_ORANGE;
}

void ActivityLights::refreshAllColors()
{
    setTeamColorBase();
    updateHealth(0);
    updateAmmunition(0);
}

void ActivityLights::setTeamColorBase()
{
    for (int i = TEAM_COLOR_START; i <= TEAM_COLOR_END; i++)
    {
        if (i % 2 == 0)
        {
            strip.SetPixelColor(i, teamColor);
        }
        else
        {
            strip.SetPixelColor(i, COLOR_WHITE);
        }
    }
}

void ActivityLights::updateHealth(int damage)
{
    health -= damage;
    int healthLEDs = HEALTH_END - HEALTH_START + 1;
    int ledsToTurnOn = healthLEDs * health / totalHealth;
    int ledsToTurnOff = healthLEDs - ledsToTurnOn;
    for (int i = HEALTH_END; i >= HEALTH_START; i--)
    {
        if (i > HEALTH_END - ledsToTurnOff)
        {
            strip.SetPixelColor(i, COLOR_WHITE);
        }
        else
        {
            strip.SetPixelColor(i, COLOR_GREEN_LOW);
        }
    }
    for (int i = HEALTH_END; i <= AMMUNITION_START; i++)
    {
        strip.SetPixelColor(i, COLOR_WHITE); // clear pixels between health and ammo
    }
}

void ActivityLights::updateAmmunition(int shotsFired)
{
    ammunition -= shotsFired;
    int ammoLEDs = AMMUNITION_START - AMMUNITION_END + 1;
    int ledsToTurnOn = ammoLEDs * ammunition / totalAmunication;
    int ledsToTurnOff = ammoLEDs - ledsToTurnOn;
    for (int i = AMMUNITION_START; i <= AMMUNITION_END; i++)
    {
        if (i < AMMUNITION_START + ledsToTurnOff)
        {
            strip.SetPixelColor(i, COLOR_WHITE);
        }
        else
        {
            strip.SetPixelColor(i, COLOR_BLUE_LOW); // Blue for ammunition
        }
    }
}

void ActivityLights::renderBase()
{
    setTeamColorBase();
    updateHealth(0);
    updateAmmunition(0);
}

void ActivityLights::renderInvulnerabilityOverlay()
{
    static uint8_t offset = 0;
    int length = TEAM_COLOR_END - TEAM_COLOR_START + 1;

    for (int i = TEAM_COLOR_START; i <= TEAM_COLOR_END; i++)
    {
        int logicalIndex = (i - TEAM_COLOR_START + offset) % length;
        if (logicalIndex % 2 == 0)
        {
            strip.SetPixelColor(i, teamColor);
        }
        else
        {
            strip.SetPixelColor(i, COLOR_WHITE);
        }
    }
    offset = (offset + 1) % length;
}

void ActivityLights::renderFireOverlay()
{
    for (int i = FIRE_START; i <= FIRE_END; i++)
    {
        strip.SetPixelColor(i, COLOR_YELLOW);
    }
}

void ActivityLights::renderHit()
{
    for (int i = TEAM_COLOR_START; i <= AMMUNITION_END; i++)
    {
        strip.SetPixelColor(i, hitBlinkOn ? COLOR_RED : COLOR_WHITE);
    }
}

void ActivityLights::setTeamColor(RgbColor color)
{
    teamColor = color;
    needsRender = true;
}

void ActivityLights::onHit()
{
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

void ActivityLights::onFire()
{
    fireActive = true;
    fireEndMillis = millis() + FIRE_DURATION_MS;
    updateAmmunition(1);
    needsRender = true;
}

boolean ActivityLights::canBeHit()
{
    return lastHitMillis + INVULNERABILITY_DURATION_MS < millis();
}

boolean ActivityLights::canFire()
{
    return lastHitMillis + 600 < millis();
}

boolean ActivityLights::isGameOver()
{
    return health <= 0;
}

void ActivityLights::updateFrame()
{
    unsigned long now = millis();

    // Track previous state so we know if anything changed this tick
    bool prevHitBlinkActive = hitBlinkActive;
    bool prevInvulnActive = invulnActive;
    bool prevFireActive = fireActive;
    bool prevHitBlinkOn = hitBlinkOn;

    if (isGameOver())
    {
        // Game over: infinite hit blink with proper interval
        if (!hitBlinkActive)
        {
            // Ensure we enter blink mode when game over first happens
            hitBlinkActive = true;
            hitBlinkOn = true;
            hitBlinkNextToggleMillis = now + HIT_BLINK_INTERVAL_MS;
        }

        if (now >= hitBlinkNextToggleMillis)
        {
            hitBlinkOn = !hitBlinkOn;
            hitBlinkNextToggleMillis = now + HIT_BLINK_INTERVAL_MS;
        }

        // Only thing that matters for rendering in game over is hitBlinkOn
        if (prevHitBlinkOn != hitBlinkOn)
        {
            needsRender = true;
        }

        return; // Nothing else (invuln/fire) is relevant in game over
    }
    else
    {
        // Normal play: finite hit blink, invulnerability and fire windows

        if (hitBlinkActive && now >= hitBlinkNextToggleMillis)
        {
            hitBlinkOn = !hitBlinkOn;
            hitBlinkNextToggleMillis = now + HIT_BLINK_INTERVAL_MS;

            hitBlinkRemainingToggles--;
            if (hitBlinkRemainingToggles <= 0)
            {
                hitBlinkActive = false;
                hitBlinkOn = false;
            }
        }

        // End of invulnerability window
        if (invulnActive && now >= invulnEndMillis)
        {
            invulnActive = false;
        }

        // End of fire window
        if (fireActive && now >= fireEndMillis)
        {
            fireActive = false;
        }

        // While invulnerable and NOT in hit blink, drive the invulnerability animation
        if (!hitBlinkActive && invulnActive)
        {
            if (now - lastInvulnAnimMillis >= INVULN_FRAME_INTERVAL_MS)
            {
                lastInvulnAnimMillis = now;
                needsRender = true;
            }
        }

        // Hit blink animation changes: render
        if (prevHitBlinkActive != hitBlinkActive || prevHitBlinkOn != hitBlinkOn)
        {
            needsRender = true;
        }

        // Invulnerability started or ended: render once
        if (prevInvulnActive != invulnActive)
        {
            needsRender = true;
        }

        // Fire started or ended: render once
        if (prevFireActive != fireActive)
        {
            needsRender = true;
        }

        return;
    }
}

void ActivityLights::renderFrame()
{
    if (!needsRender)
    {
        return;
    }
    if (isGameOver())
    {
        renderHit();
        strip.Show();
        return;
    }
    // Hit blink phase overrides everything else
    if (hitBlinkActive)
    {
        renderHit();
    }
    else
    {
        // base state
        renderBase();
        // overlays (can combine)
        if (invulnActive)
        {
            renderInvulnerabilityOverlay();
        }
        if (fireActive)
        {
            renderFireOverlay();
        }
    }
    strip.Show();
    needsRender = false;
}
