#pragma once

#include <stdint.h>

// Logical gamepad state, mapped to a PS4-like API.
// Axes are signed 16-bit values, buttons are booleans.
// Names combine PS4 and XInput terminology where they differ
// (e.g. Triangle_Y, L1_LB, LStickX_LX).
struct GamepadState {
    int16_t lStickX = 0;
    int16_t lStickY = 0;
    int16_t rStickX = 0;
    int16_t rStickY = 0;

    bool left   = false;
    bool down   = false;
    bool right  = false;
    bool up     = false;

    bool square_X   = false;  // PS4 Square  <-> XInput X
    bool cross_A    = false;  // PS4 Cross   <-> XInput A
    bool circle_B   = false;  // PS4 Circle  <-> XInput B
    bool triangle_Y = false;  // PS4 Triangle<-> XInput Y

    bool l1_LB = false;       // PS4 L1 <-> XInput LB
    bool r1_RB = false;       // PS4 R1 <-> XInput RB
    bool l2_LT = false;       // PS4 L2 <-> XInput LT
    bool r2_RT = false;       // PS4 R2 <-> XInput RT

    bool share_Back     = false;  // PS4 Share   <-> XInput Back
    bool options_Start  = false;  // PS4 Options <-> XInput Start
    bool psButton_Mode  = false;  // PS button / Mode button (if available)
    bool touchpad       = false;  // Not present on many gamepads

    bool charging = false;  // Optional, may not be available
    bool audio    = false;  // Optional, may not be available
    bool mic      = false;  // Optional, may not be available
    uint8_t battery = 0;    // Optional, may not be available
};

// Notification callback invoked when a new HID report has been parsed
// and the GamepadState has been updated.
typedef void (*GamepadNotifyCallback)(const GamepadState &state);

class Gamepad {
public:
    virtual ~Gamepad() = default;

    // Optional lifecycle hooks used by some controller stacks (e.g. PS4).
    // Implementations may override these to wire into their underlying library.
    typedef void (*GamepadLifecycleCallback)();

    // Start the controller stack (if required). Default is a no-op.
    virtual void begin() {}

    // Connection lifecycle hooks. Default implementation stores callbacks only.
    // Implementations that can detect connect/disconnect may invoke them.
    virtual void attachOnConnect(GamepadLifecycleCallback cb) { on_connect_cb_ = cb; }
    virtual void attachOnDisconnect(GamepadLifecycleCallback cb) { on_disconnect_cb_ = cb; }

    // Attach a callback similar to PS4.notify(). Pass nullptr to detach.
    virtual void attach(GamepadNotifyCallback cb) = 0;

    // Enable or disable verbose report logging from the implementation.
    virtual void setVerboseReportLogging(bool enabled) = 0;

    // Feed a raw input report into the gamepad implementation.
    // Implementations parse the report and update internal GamepadState.
    virtual void handleReport(const uint8_t *data, int length) = 0;

    // Stick values as signed axes (range depends on implementation; commonly -127..127).
    virtual int16_t LStickX_LX() const = 0;
    virtual int16_t LStickY_LY() const = 0;
    virtual int16_t RStickX_RX() const = 0;
    virtual int16_t RStickY_RY() const = 0;

    // D-pad (PS4-style names)
    virtual bool Left() const = 0;
    virtual bool Down() const = 0;
    virtual bool Right() const = 0;
    virtual bool Up() const = 0;

    // Face buttons
    virtual bool Square_X() const = 0;
    virtual bool Cross_A() const = 0;
    virtual bool Circle_B() const = 0;
    virtual bool Triangle_Y() const = 0;

    // Shoulder/trigger buttons
    virtual bool L1_LB() const = 0;
    virtual bool R1_RB() const = 0;
    virtual bool L2_LT() const = 0;
    virtual bool R2_RT() const = 0;

    // System buttons
    virtual bool Share_Back() const = 0;
    virtual bool Options_Start() const = 0;
    virtual bool PSButton_Mode() const = 0;
    virtual bool Touchpad() const = 0;

    virtual bool Charging() const = 0;
    virtual bool Audio() const = 0;
    virtual bool Mic() const = 0;
    virtual uint8_t Battery() const = 0;

    // Direct access to the current state if needed
    virtual const GamepadState &state() const = 0;

protected:
    void notifyConnected() { if (on_connect_cb_ != nullptr) { on_connect_cb_(); } }
    void notifyDisconnected() { if (on_disconnect_cb_ != nullptr) { on_disconnect_cb_(); } }

    GamepadLifecycleCallback on_connect_cb_ = nullptr;
    GamepadLifecycleCallback on_disconnect_cb_ = nullptr;
};
