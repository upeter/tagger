#include "colors.h"

// Adjust this to change overall LED brightness
const float COLORS_BRIGHTNESS = 0.1f;

RgbColor applyBrightness(const RgbColor &color) {
	uint8_t r = static_cast<uint8_t>(color.R * COLORS_BRIGHTNESS);
	uint8_t g = static_cast<uint8_t>(color.G * COLORS_BRIGHTNESS);
	uint8_t b = static_cast<uint8_t>(color.B * COLORS_BRIGHTNESS);
	return RgbColor(r, g, b);
}

const RgbColor COLOR_WHITE   = RgbColor(0,   0,   0);
const RgbColor COLOR_RED     = RgbColor(255, 0,   0);
const RgbColor COLOR_RED_LOW = applyBrightness(COLOR_RED);
const RgbColor COLOR_ORANGE  = RgbColor(255, 165, 0);
const RgbColor COLOR_ORANGE_LOW = applyBrightness(COLOR_ORANGE);
const RgbColor COLOR_PINK    = RgbColor(255 * 0.5, 192 * 0.5, 203 * 0.5);
const RgbColor COLOR_PINK_LOW   = applyBrightness(COLOR_PINK);
const RgbColor COLOR_YELLOW  = RgbColor(255, 255, 0);
const RgbColor COLOR_YELLOW_LOW = applyBrightness(COLOR_YELLOW);
const RgbColor COLOR_BLUE    = RgbColor(0,   0,   255);
const RgbColor COLOR_BLUE_LOW   = applyBrightness(COLOR_BLUE);
const RgbColor COLOR_GREEN   = RgbColor(0,   255, 0);
const RgbColor COLOR_GREEN_LOW  = applyBrightness(COLOR_GREEN);
