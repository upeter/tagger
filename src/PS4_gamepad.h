#pragma once

#include "gamepad.h"

// PS4-specific adapter that implements the generic Gamepad interface.
// This keeps the rest of the codebase independent of the PS4Controller library.
class PS4Gamepad final : public Gamepad {
public:
    PS4Gamepad();
    ~PS4Gamepad() override;

    // PS4 transport lifecycle
    void begin();

    // PS4Controller provides separate connect/disconnect callbacks; expose them
    // without leaking PS4Controller headers into the rest of the project.
    void attachOnConnect(void (*cb)());
    void attachOnDisconnect(void (*cb)());

    // Gamepad
    void attach(GamepadNotifyCallback cb) override;
    void setVerboseReportLogging(bool enabled) override;
    void handleReport(const uint8_t *data, int length) override;

    int16_t LStickX_LX() const override;
    int16_t LStickY_LY() const override;
    int16_t RStickX_RX() const override;
    int16_t RStickY_RY() const override;

    bool Left() const override;
    bool Down() const override;
    bool Right() const override;
    bool Up() const override;

    bool Square_X() const override;
    bool Cross_A() const override;
    bool Circle_B() const override;
    bool Triangle_Y() const override;

    bool L1_LB() const override;
    bool R1_RB() const override;
    bool L2_LT() const override;
    bool R2_RT() const override;

    bool Share_Back() const override;
    bool Options_Start() const override;
    bool PSButton_Mode() const override;
    bool Touchpad() const override;

    bool Charging() const override;
    bool Audio() const override;
    bool Mic() const override;
    uint8_t Battery() const override;

    const GamepadState &state() const override;

private:
    static void ps4NotifyTrampoline();

    void updateFromPS4();

    GamepadState state_{};
    GamepadNotifyCallback notifyCb_ = nullptr;
    bool verbose_ = false;
};
