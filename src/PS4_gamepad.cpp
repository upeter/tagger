#include "PS4_gamepad.h"

#include <Arduino.h>
#include <PS4Controller.h>

static PS4Gamepad *g_ps4GamepadInstance = nullptr;

PS4Gamepad::PS4Gamepad()
{
    // Only one instance is expected.
    g_ps4GamepadInstance = this;
}

PS4Gamepad::~PS4Gamepad()
{
    if (g_ps4GamepadInstance == this) {
        g_ps4GamepadInstance = nullptr;
    }
}

void PS4Gamepad::begin()
{
    // PS4Controller invokes a no-args function pointer.
    PS4.attach(PS4Gamepad::ps4NotifyTrampoline);
    PS4.begin();
}

void PS4Gamepad::attachOnConnect(void (*cb)())
{
    PS4.attachOnConnect(cb);
}

void PS4Gamepad::attachOnDisconnect(void (*cb)())
{
    PS4.attachOnDisconnect(cb);
}

void PS4Gamepad::attach(GamepadNotifyCallback cb)
{
    notifyCb_ = cb;
}

void PS4Gamepad::setVerboseReportLogging(bool enabled)
{
    verbose_ = enabled;
}

void PS4Gamepad::handleReport(const uint8_t *data, int length)
{
    // PS4Controller manages transport and report parsing internally.
    // This method exists for other HID implementations.
    (void)data;
    (void)length;
}

void PS4Gamepad::ps4NotifyTrampoline()
{
    if (g_ps4GamepadInstance == nullptr) {
        return;
    }

    g_ps4GamepadInstance->updateFromPS4();

    if (g_ps4GamepadInstance->notifyCb_ != nullptr) {
        g_ps4GamepadInstance->notifyCb_(g_ps4GamepadInstance->state_);
    }
}

void PS4Gamepad::updateFromPS4()
{
    state_.lStickX = (int16_t)PS4.LStickX();
    state_.lStickY = (int16_t)PS4.LStickY();
    state_.rStickX = (int16_t)PS4.RStickX();
    state_.rStickY = (int16_t)PS4.RStickY();

    state_.left = PS4.Left();
    state_.down = PS4.Down();
    state_.right = PS4.Right();
    state_.up = PS4.Up();

    state_.square_X = PS4.Square();
    state_.cross_A = PS4.Cross();
    state_.circle_B = PS4.Circle();
    state_.triangle_Y = PS4.Triangle();

    state_.l1_LB = PS4.L1();
    state_.r1_RB = PS4.R1();
    state_.l2_LT = PS4.L2();
    state_.r2_RT = PS4.R2();

    state_.share_Back = PS4.Share();
    state_.options_Start = PS4.Options();
    state_.psButton_Mode = PS4.PSButton();
    state_.touchpad = PS4.Touchpad();

    state_.charging = PS4.Charging();
    state_.audio = PS4.Audio();
    state_.mic = PS4.Mic();
    state_.battery = PS4.Battery();

    if (verbose_) {
        char messageString[200];
        snprintf(messageString,
                 sizeof(messageString),
                 "%4d,%4d,%4d,%4d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d",
                 (int)state_.lStickX,
                 (int)state_.lStickY,
                 (int)state_.rStickX,
                 (int)state_.rStickY,
                 state_.left,
                 state_.down,
                 state_.right,
                 state_.up,
                 state_.square_X,
                 state_.cross_A,
                 state_.circle_B,
                 state_.triangle_Y,
                 state_.l1_LB,
                 state_.r1_RB,
                 state_.l2_LT,
                 state_.r2_RT,
                 state_.share_Back,
                 state_.options_Start,
                 state_.psButton_Mode,
                 state_.touchpad,
                 state_.charging,
                 state_.audio,
                 state_.mic,
                 state_.battery);
        Serial.println(messageString);
    }
}

int16_t PS4Gamepad::LStickX_LX() const { return state_.lStickX; }
int16_t PS4Gamepad::LStickY_LY() const { return state_.lStickY; }
int16_t PS4Gamepad::RStickX_RX() const { return state_.rStickX; }
int16_t PS4Gamepad::RStickY_RY() const { return state_.rStickY; }

bool PS4Gamepad::Left() const { return state_.left; }
bool PS4Gamepad::Down() const { return state_.down; }
bool PS4Gamepad::Right() const { return state_.right; }
bool PS4Gamepad::Up() const { return state_.up; }

bool PS4Gamepad::Square_X() const { return state_.square_X; }
bool PS4Gamepad::Cross_A() const { return state_.cross_A; }
bool PS4Gamepad::Circle_B() const { return state_.circle_B; }
bool PS4Gamepad::Triangle_Y() const { return state_.triangle_Y; }

bool PS4Gamepad::L1_LB() const { return state_.l1_LB; }
bool PS4Gamepad::R1_RB() const { return state_.r1_RB; }
bool PS4Gamepad::L2_LT() const { return state_.l2_LT; }
bool PS4Gamepad::R2_RT() const { return state_.r2_RT; }

bool PS4Gamepad::Share_Back() const { return state_.share_Back; }
bool PS4Gamepad::Options_Start() const { return state_.options_Start; }
bool PS4Gamepad::PSButton_Mode() const { return state_.psButton_Mode; }
bool PS4Gamepad::Touchpad() const { return state_.touchpad; }

bool PS4Gamepad::Charging() const { return state_.charging; }
bool PS4Gamepad::Audio() const { return state_.audio; }
bool PS4Gamepad::Mic() const { return state_.mic; }
uint8_t PS4Gamepad::Battery() const { return state_.battery; }

const GamepadState &PS4Gamepad::state() const { return state_; }
