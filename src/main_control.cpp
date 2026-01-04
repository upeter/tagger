#include "main_control.h"

#include <Arduino.h>
#include <EEPROM.h>
#include <NeoPixelBus.h>

#include "activity_lights.h"
#include "colors.h"
#include "flasher.h"
#include "ir_fire.h"
#include "ir_sensors.h"
#include "laser.h"
#include "misc.h"
#include "motor_control.h"
#include "motors.h"
#include "prefs.h"

#define LED_BUILTIN 2

// Use only core 1 for demo purposes
#if CONFIG_FREERTOS_UNICORE
static const BaseType_t app_cpu = 0;
#else
static const BaseType_t app_cpu = 1;
#endif

#define LED_DATA_PIN 13
#define NUM_LEDS 67

// Settings
static const uint8_t buf_len = 20;

// Pins
static const int led_pin = LED_BUILTIN;
static const int trigger_pin = 25;
static const int trigger_servo_pin = 18;
static const int laser_pin = 15;

// Motor
static int maxSpeed = MOTOR_MAX_SPEED;
static int minSpeed = MOTOR_MIN_SPEED;
static volatile int trimX = 0;

static MotorControlConfig motorControlConfig;

// Globals
int led_delay = 50;   // ms
static bool flash_led = false;
static const uint32_t ir_messages[] = {
    0xFF30CF,
    0xFF18E7,
    0xFF7A85,
    0xFF10EF,
    0xFF38C7,
    0xFF5AA5,
    0xFF42BD,
    0xFF4AB5,
};

static int direction = 1;
static unsigned long lastDirectionToggleMillis = 0;
static const unsigned long DIRECTION_DEBOUNCE_MS = 500;
static Prefs currentPrefs;

// Joystick mode: 1 = one-stick (current behavior), 2 = two-stick
static uint8_t joystickMode = 1;
static unsigned long lastJoystickModeToggleMillis = 0;
static const unsigned long JOYSTICK_MODE_DEBOUNCE_MS = 500;

// Debounce for in-game prefs reset (L2+R2)
static unsigned long lastPrefsResetMillis = 0;
static const unsigned long PREFS_RESET_DEBOUNCE_MS = 2000;

static unsigned long lastTimeStamp = 0;

static TaskHandle_t xHandle_handleIR = nullptr;
static TaskHandle_t xHandle_toggleOnboardLED = nullptr;
static TaskHandle_t xHandle_triggerButton = nullptr;
static TaskHandle_t xHandle_handleLED = nullptr;

static NeoPixelBus<NeoGrbFeature,
                   NeoEsp32RmtMethodBase<NeoEsp32RmtSpeed800Kbps, NeoEsp32RmtChannel3>>
    strip(NUM_LEDS, LED_DATA_PIN);

static Motors *motors = nullptr;
static MotorChaosMonkey *chaosMonkey = nullptr;
static Flasher flasher(led_pin, 1000);
static Laser laser(laser_pin);
static ActivityLights lights(strip);

static bool onIrHit(const IREvent *event)
{
    (void)event;
    if (lights.canBeHit()) {
        vTaskResume(xHandle_toggleOnboardLED);
        lights.onHit();
        if (chaosMonkey != nullptr) {
            chaosMonkey->Start();
        }
        return true;
    }
    Serial.println(">>> Hit cooldown");
    return false;
}

static bool irCanFire()
{
    return lights.canFire();
}

static void irDidFire(uint32_t code)
{
    (void)code;
    lights.onFire();
}

static void notify(const GamepadState &state)
{
    char messageString[200];

    try {
        if (millis() - lastTimeStamp > 100) {
            int rightMotorSpeed, leftMotorSpeed;

            if (lights.isGameOver()) {
                if (motors != nullptr) {
                    motors->rotateMotor(0, 0);
                }
            } else {
                if (chaosMonkey != nullptr && chaosMonkey->isActive()) {
                    chaosMonkey->Update();
                } else {
                    if (state.r1_RB || state.l1_LB) {
                        laser.activate();
                        if (state.r1_RB) {
                            ir_fire_request(ir_messages[0]);
                        }
                    } else {
                        laser.deactivate();
                    }

                    if (state.square_X) {
                        currentPrefs = prefsWithTeamColor(currentPrefs, COLOR_PINK);
                        lights.setTeamColor(currentPrefs.color);
                    } else if (state.cross_A) {
                        currentPrefs = prefsWithTeamColor(currentPrefs, COLOR_BLUE);
                        lights.setTeamColor(currentPrefs.color);
                    } else if (state.circle_B) {
                        currentPrefs = prefsWithTeamColor(currentPrefs, COLOR_ORANGE);
                        lights.setTeamColor(currentPrefs.color);
                    } else if (state.triangle_Y) {
                        currentPrefs = prefsWithTeamColor(currentPrefs, COLOR_GREEN);
                        lights.setTeamColor(currentPrefs.color);
                    } else if (state.l2_LT && state.r2_RT) {
                        // Reset all user preferences when L2 and R2 are pressed together, with debounce
                        unsigned long now = millis();
                        if (now - lastPrefsResetMillis > PREFS_RESET_DEBOUNCE_MS) {
                            Serial.println("Resetting user preferences (L2+R2)");
                            prefsClearAll();
                            currentPrefs = prefsLoad();
                            // Re-apply any restored prefs to runtime state
                            if (currentPrefs.hasColor) {
                                lights.setTeamColor(currentPrefs.color);
                            }
                            if (currentPrefs.hasDirection) {
                                direction = currentPrefs.direction;
                            }
                            if (currentPrefs.hasJoystickMode) {
                                joystickMode = currentPrefs.joystickMode;
                            }
                            // force immediate visual refresh after prefs reset
                            lights.updateFrame();
                            lights.renderFrame();
                            lastPrefsResetMillis = now;
                        }
                    } else if (state.up) {
                        unsigned long now = millis();
                        if (now - lastDirectionToggleMillis > DIRECTION_DEBOUNCE_MS) {
                            direction = direction * -1;
                            currentPrefs = prefsWithDirection(currentPrefs, direction);
                            lastDirectionToggleMillis = now;
                        }
                    } else if (state.left) {
                        unsigned long now = millis();
                        if (now - lastJoystickModeToggleMillis > JOYSTICK_MODE_DEBOUNCE_MS) {
                            joystickMode = 2; // two-stick mode
                            currentPrefs = prefsWithJoystickMode(currentPrefs, joystickMode);
                            Serial.println("Joystick mode set to TWO-STICK (PS.Left)");
                            lastJoystickModeToggleMillis = now;
                        }
                    } else if (state.right) {
                        unsigned long now = millis();
                        if (now - lastJoystickModeToggleMillis > JOYSTICK_MODE_DEBOUNCE_MS) {
                            joystickMode = 1; // one-stick mode
                            currentPrefs = prefsWithJoystickMode(currentPrefs, joystickMode);
                            Serial.println("Joystick mode set to ONE-STICK (PS.Right)");
                            lastJoystickModeToggleMillis = now;
                        }
                    }

                    MotorControlInput controlInput;
                    controlInput.lStickX = state.lStickX;
                    controlInput.lStickY = state.lStickY * direction;
                    controlInput.rStickX = state.rStickX;
                    controlInput.rStickY = state.rStickY * direction;
                    controlInput.joystickMode = joystickMode;

                    motorControlConfig.trimX = trimX;
                    MotorControlOutput cmd = motorControlCompute(controlInput, motorControlConfig);
                    rightMotorSpeed = cmd.rightMotorSpeed;
                    leftMotorSpeed = cmd.leftMotorSpeed;

                    if (motors != nullptr) {
                        motors->rotateMotor(rightMotorSpeed, leftMotorSpeed);
                    }
                }
            }

            lastTimeStamp = millis();
        }
    } catch (const std::exception &e) {
        Serial.print("Exception caught: ");
        Serial.println(e.what());
    }
}

void main_control_onConnect()
{
    Serial.println("Connected!.");
}

void main_control_onDisconnect()
{
    if (motors != nullptr) {
        motors->rotateMotor(0, 0);
    }
    Serial.println("Disconnected!.");
}

// Task: Blink LED at rate set by global variable
static void toggleOnboardLED(void *parameter)
{
    (void)parameter;
    while (true) {
        vTaskSuspend(NULL);
        flasher.flash();
    }
}

static void handleLED(void *parameter)
{
    (void)parameter;
    const TickType_t frameDelay = pdMS_TO_TICKS(20);
    while (true) {
        lights.updateFrame();
        lights.renderFrame();
        vTaskDelay(frameDelay);
    }
}

void main_control_setup(Gamepad &gamepad)
{
    // Configure pin
    pinMode(led_pin, OUTPUT);

    // Configure serial and wait a second
    Serial.begin(115200);
    vTaskDelay(1000 / portTICK_PERIOD_MS);
    Serial.println("Initializing tank");
    Serial.println("Initializing onboard LEDs...");

    // Start blink task
    xTaskCreatePinnedToCore(toggleOnboardLED,
                            "Toggle LED",
                            2048,
                            NULL,
                            1,
                            &xHandle_toggleOnboardLED,
                            1);

    vTaskDelay(1000);
    strip.Begin();
    strip.Show();

    Serial.println("Initialize motors...");

    motors = new Motors();
    chaosMonkey = new MotorChaosMonkey(motors);
    motors->setUpPinModes();
    Serial.println("Motors initialized");

    motorControlConfig = motorControlDefaultConfig(minSpeed, maxSpeed);

    // Load user preferences (team color + direction)
    currentPrefs = prefsLoad();
    if (currentPrefs.hasColor) {
        lights.setTeamColor(currentPrefs.color);
    }
    if (currentPrefs.hasDirection) {
        direction = currentPrefs.direction;
    }
    if (currentPrefs.hasJoystickMode) {
        joystickMode = currentPrefs.joystickMode;
    }

    // Ensure LEDs reflect initial state
    lights.updateFrame();
    lights.renderFrame();

    // Start IR subsystems once lights + motors are ready
    ir_sensors_begin(onIrHit);
    ir_fire_begin(irCanFire, irDidFire);

    // Start LED task
    xTaskCreatePinnedToCore(handleLED,
                            "handle LED",
                            4096,
                            NULL,
                            2,
                            &xHandle_handleLED,
                            1);

    // Gamepad callbacks (transport-specific begin/connect hooks happen in wrapper)
    gamepad.attach(notify);

    Serial.println("Main control initialized");
}
