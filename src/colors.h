#pragma once

#include <NeoPixelBus.h>

// Global brightness factor for all colors
extern const float COLORS_BRIGHTNESS;

// Apply brightness scaling to a color
RgbColor applyBrightness(const RgbColor &color);

// Commonly used colors (both full and dimmed variants)
extern const RgbColor COLOR_WHITE;
extern const RgbColor COLOR_RED;
extern const RgbColor COLOR_RED_LOW;
extern const RgbColor COLOR_ORANGE;
extern const RgbColor COLOR_ORANGE_LOW;
extern const RgbColor COLOR_PINK;
extern const RgbColor COLOR_PINK_LOW;
extern const RgbColor COLOR_YELLOW;
extern const RgbColor COLOR_YELLOW_LOW;
extern const RgbColor COLOR_BLUE;
extern const RgbColor COLOR_BLUE_LOW;
extern const RgbColor COLOR_GREEN;
extern const RgbColor COLOR_GREEN_LOW;
