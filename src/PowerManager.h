#ifndef POWER_MANAGER_H
#define POWER_MANAGER_H

#include <Arduino.h>
#include <esp_adc_cal.h>
#include <esp_sleep.h>
#include <esp_task_wdt.h>
#include <esp_ota_ops.h>
#include "Config.h"

class LEDController; // Forward declaration

enum class SleepReason : uint8_t
{
    Latch,      // flat-cell latch standing, no proof of charging
    LowVoltage, // measured voltage below BATTERY_FLAT_THRESHOLD_MV
    Button,     // long press while running
    Charger,    // charger detected, get the load off the cell
    AutoSleep,  // no connection for AUTO_SLEEP_TIME
    WakeCheck,  // woken by the button but the hold was not confirmed
    Command,    // sleep command over BLE
};

class PowerManager
{
private:
    LEDController* ledController;
    esp_adc_cal_characteristics_t adcChars;
    TaskHandle_t batteryTaskHandle;
    unsigned long lastDisconnectTime;
    bool isGoingToSleep;
    bool justWokenUp;
    bool isInitialized;
    int lastBatteryMv;
    int chargerPrevMv;
    int chargerBaselineMv;
    uint8_t chargerRiseStreak;

    // Callback function pointer for battery level updates
    void (*batteryCallback)(int percentage);

    void configureWakeButton();

public:
    PowerManager(LEDController* leds);

    // Initialization
    bool begin();
    void setLedController(LEDController* leds) { ledController = leds; }

    // Battery monitoring
    // Reported over BLE. Deliberately unchanged: the released app blocks OTA at batteryLevel <= 40,
    // so a target in the field would refuse to take this very fix if this number moved.
    int getBatteryPercentage();
    int readBatteryMv();

    // Takes the confirming reading only when the LED output is already off, and says so when it
    // could not. It must never switch the output off itself: the game logic owns the LEDs.
    bool tryReadUnloadedBatteryMv(int &outMv);
    int getLastBatteryMv() const { return lastBatteryMv; }
    static int percentFromMv(int batteryMv);
    void setBatteryCallback(void (*callback)(int));
    void startBatteryMonitoringTask();
    void stopBatteryMonitoringTask();

    // Sleep/Wake functionality
    // wakeOnReleaseIfHeld: if the button is still down when the target goes to sleep, arm the
    // wake on its release instead of on the press. Used on the start path of a latched cell only;
    // every other caller keeps the 1.0.8 wake on the press.
    void goToDeepSleep(SleepReason reason, bool skipLedBlink = false,
                       bool wakeOnReleaseIfHeld = false);
    bool validateWakeUp();
    bool isJustWokenUp() const { return justWokenUp; }
    void clearWakeUpFlag() { justWokenUp = false; }

    // Connection state management
    void setConnected(bool connected);
    bool isConnected() const { return lastDisconnectTime == 0; }
    bool shouldAutoSleep();

    // Charging detection
    bool isCharging();
    bool updateChargerEvidence(int batteryMv);

    // Button handling
    bool isButtonPressed();
    bool holdButtonFor(uint32_t holdMs);

    // BatteryGuard::ButtonHoldProbe. Passing this to decideStart() is what makes the button
    // impossible to skip in the latch hysteresis band: there the guard cannot reach a sleep action
    // without calling it. Below the flat threshold it is deliberately not called (variant A). It
    // is the only place in the start path that decides on the button, so it is asked at most once.
    static bool latchOverrideProbe(void *context);

    // Task function (static)
    static void batteryMonitorTaskFunction(void* pvParameters);
};

#endif // POWER_MANAGER_H
