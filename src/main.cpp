#include <Arduino.h>
#include <Wire.h>
#include <esp_task_wdt.h>
#include <esp_sleep.h>

#include "BuildGuards.h"
#include "Config.h"
#include "PiezoSensor.h"
#include "LEDController.h"
#include "PowerManager.h"
#include "OTAManager.h"
#include "BLEManager.h"
#include "GameSettings.h"
#include "CalibrationStore.h"
#include "BatteryGuard.h"
#include "BatteryLatchStore.h"

// Global instances
LEDController ledController;
PowerManager powerManager(&ledController);
OTAManager otaManager;
PiezoSensor sensor(PIEZO_SENSOR_PIN);
BLEManager bleManager(&ledController, &powerManager, &otaManager, &sensor);

// Forward declarations
void onPiezoHit(int piezoValue);

static void logRefusal(const BatteryGuard::StartDecision &decision)
{
    if (decision.overrideRefused)
    {
        Serial.printf("Override refused: cell below %d mV, charger only\n",
                      BATTERY_FLAT_THRESHOLD_MV);
    }
}
void onPiezoPeak(uint16_t peakValue);
void onBatteryUpdate(int percentage);

void setup()
{
    delay(100);
    Serial.begin(115200);
    Serial.println("Start - Modular Version");

    // Check wake-up cause FIRST, before any other initialization
    esp_sleep_wakeup_cause_t wakeup_cause = esp_sleep_get_wakeup_cause();

    // Battery guard runs before the stabilization delays, before the LED rail and before BLE.
    // A reset from the TPS3823-33 supervisor looks exactly like a fresh boot, so on a flat cell
    // every millisecond spent here is paid again on every loop of that reset.
    // PowerManager::begin() only touches GPIO and the ADC - no LED, no boost rail.
    if (!powerManager.begin())
    {
        Serial.println("Failed to initialize power manager");
        return;
    }
    BatteryLatchStore::begin();

    BatteryGuard::Thresholds thresholds = BatteryGuard::defaultThresholds();

    BatteryGuard::StartInputs guardIn;
    guardIn.batteryMv = powerManager.readBatteryMv();
    guardIn.confirmMv = guardIn.batteryMv;
    guardIn.latched = BatteryLatchStore::isFlat();

#ifdef MIGRATION_TEST_NO_BATTERY
    // No-battery test build (env:migration-test, version 1.0.9nobatt-esp32s3): both voltage triggers
    // are off, because a floating divider reads near 0 mV and the target would go straight back
    // to sleep. A latch already in NVS is ignored and none is written, so a debugging session
    // cannot leave a target that "refuses to boot" behind for the next QA scenario. The charger
    // trigger and the BLE sleep command stay live. See include/BuildGuards.h.
    thresholds.flatMv = 0;
    guardIn.latched = false;
    Serial.println("Battery guard: voltage triggers DISABLED (no-battery test build)");
#endif

    // One sample never decides. A fresh boot can be a TPS3823-33 reset with the rail still
    // settling, so a low reading is confirmed by a second one before it costs the user a target.
    // Only where the latch is about to be WRITTEN. With the latch already standing no second
    // reading is taken and confirmMv stays equal to batteryMv, so there a single 64-sample reading
    // decides between refusing the button (below the flat threshold) and offering it: that path
    // runs on every turn of a brownout loop, and a second reading there would be paid on each.
    if (!guardIn.latched && guardIn.batteryMv < thresholds.flatMv)
    {
        delay(BATTERY_FLAT_CONFIRM_DELAY_MS);
        guardIn.confirmMv = powerManager.readBatteryMv();
    }

    // The button is read in exactly one place on the start path: inside this probe, and only when
    // the latch stands and the cell is in the hysteresis band (flat threshold up to the clear
    // threshold). Below the flat threshold no sleep branch is offered the override (owner decision
    // 2026-11-07, variant A).
    const BatteryGuard::StartDecision decision =
        BatteryGuard::decideStart(guardIn, thresholds,
                                  PowerManager::latchOverrideProbe, &powerManager);
    Serial.printf("Battery guard: mv=%d confirm=%d latch=%d override=%d refused=%d -> %s\n",
                  guardIn.batteryMv, guardIn.confirmMv, (int)guardIn.latched,
                  (int)decision.overridden, (int)decision.overrideRefused,
                  BatteryGuard::actionName(decision.action));

    // Everything below asks the guard's own answer. No second "is this cell healthy" threshold
    // test lives in this file: two tests with the same intent, written in two places, drift apart
    // at the next change of a threshold. Every start that gets past the guard is on a cell at or
    // above the flat threshold, so the rest of setup() is the ordinary start with its LEDs.

    if (decision.overridden)
    {
        // Confirmation for the person holding the target: one short red flash, here and not at the
        // end of setup(). After BLE is up it would arrive seconds late and the instruction stops
        // being "hold until it flashes".
        // Deliberately before the latch write below: if this flash does collapse the rail, the
        // supervisor resets a target whose latch is still standing, so it goes back to sleep dark
        // instead of waking up unlatched.
        ledController.begin();
        ledController.blinkColor(255, 0, 0, 1, 300, 0);
    }

    switch (decision.action)
    {
    case BatteryGuard::StartAction::LatchAndSleep:
        // Persist the decision before any LED and before sleep, so a brownout in the middle of
        // this sequence cannot start the whole attempt over.
        BatteryLatchStore::setFlat(guardIn.confirmMv);
        logRefusal(decision);
        powerManager.goToDeepSleep(SleepReason::LowVoltage, true, true);
        return;

    case BatteryGuard::StartAction::SleepLatched:
        logRefusal(decision);
        powerManager.goToDeepSleep(SleepReason::Latch, true, true);
        return;

    case BatteryGuard::StartAction::ClearLatchAndStart:
        BatteryLatchStore::clearFlat(guardIn.batteryMv,
                                     decision.overridden
                                         ? BatteryLatchStore::ClearedBy::Button
                                         : BatteryLatchStore::ClearedBy::Voltage);
        break;

    case BatteryGuard::StartAction::Start:
        break;
    }

    if (wakeup_cause == ESP_SLEEP_WAKEUP_EXT0)
    {
        Serial.println("Woken up from deep sleep");

        // The override hold already proved intent for longer than WAKE_UP_HOLD_TIME
        if (!decision.overridden && !powerManager.validateWakeUp())
        {
            // Button was not held long enough, go back to sleep
            Serial.println("Wake-up validation failed - returning to deep sleep");
            powerManager.goToDeepSleep(SleepReason::WakeCheck, true);
            return; // This line will never be reached, but good practice
        }

        Serial.println("Wake-up validation successful - continuing normal operation");
    }
    else
    {
        delay(2000);
        // For fresh boots, add extra stabilization time
        delay(3000); // Additional delay for power supply to stabilize
    }

    // Initialize LED controller (this also enables the LED boost rail)
    if (!ledController.begin())
    {
        Serial.println("Failed to initialize LED controller");
        return;
    }

    Serial.printf("Battery level: %d%%\n", PowerManager::percentFromMv(guardIn.batteryMv));

    ledController.setBlueChannel(255);

    // Show initialization progress with LEDs
    if (wakeup_cause != ESP_SLEEP_WAKEUP_EXT0)
    {
        delay(1000);
    }

    // Set battery callback
    powerManager.setBatteryCallback(onBatteryUpdate);

    // Wait before initializing power-hungry components
    if (wakeup_cause != ESP_SLEEP_WAKEUP_EXT0)
    {
        ledController.turnOff();
        delay(1000);
    }

    // Load persisted calibration before starting the sensor task
    CalibrationStore::begin();
    sensor.setThreshold(CalibrationStore::loadThreshold());
    sensor.setDebounceMs(CalibrationStore::loadDebounceMs());
    Serial.printf("Piezo calibration loaded: threshold=%u, debounce=%u ms\n",
                  sensor.getThreshold(), sensor.getDebounceMs());

    // Initialize sensor
    sensor.begin();
    sensor.setCallback(onPiezoHit);
    sensor.setPeakCallback(onPiezoPeak);

    // Initialize BLE manager
    if (!bleManager.begin())
    {
        Serial.println("Failed to initialize BLE manager");
        return;
    }

    // Start power manager tasks
    powerManager.startBatteryMonitoringTask();

    // Start LED status task
    bleManager.startLedStatusTask();

    Serial.println("All modules initialized successfully");

    // Check if this is a reset after OTA
    if (otaManager.isResetAfterOTA())
    {
        Serial.println("System restarted after OTA update");
    }
}

void loop()
{
    static unsigned long pressStartTime = 0;
    static bool buttonPressed = false;

    // The long-press power-off arms only once the button has read released for
    // BATTERY_BUTTON_DEBOUNCE_MS without a break. setup() can return with the button still held -
    // after the wake-up hold and after the 10 s latch override - and loop() would start counting
    // TURN_OFF_TIME from scratch and sleep the target under the finger that just forced it on;
    // from the outside that is indistinguishable from a target that refuses to start. The
    // debounce is the other half of the same bug: one contact bounce during a long hold is a
    // single "released" sample, and arming on it sleeps the target two seconds later with the
    // button still down.
    static bool turnOffArmed = false;
    static bool releaseSeen = false;
    static unsigned long releaseStart = 0;

    // Check for pending OTA restart
    otaManager.checkPendingRestart();

    // Button handling for power off
    if (powerManager.isButtonPressed())
    {
        if (!buttonPressed)
        {
            // Button was just pressed
            pressStartTime = millis();
            buttonPressed = true;
            releaseSeen = false;
            Serial.println("Button pressed");

            // Reset justWokenUp flag immediately when button is pressed
            if (powerManager.isJustWokenUp())
            {
                powerManager.clearWakeUpFlag();
                Serial.println("Reset justWokenUp flag");
            }
        }

        // Check for long press while button is still held
        if (turnOffArmed && (millis() - pressStartTime) > TURN_OFF_TIME)
        {
            Serial.println("TURN OFF TIME DETECTED");
            powerManager.goToDeepSleep(SleepReason::Button);
        }
    }
    else
    {
        buttonPressed = false;

        if (!releaseSeen)
        {
            releaseSeen = true;
            releaseStart = millis();
        }
        else if (!turnOffArmed && (millis() - releaseStart) >= BATTERY_BUTTON_DEBOUNCE_MS)
        {
            turnOffArmed = true;
        }
    }
}

// Callback functions
void onPiezoHit(int piezoValue)
{
    if (GameSettings::isAutoLedOffEnabled())
    {
        ledController.turnOff();
    }
    bleManager.sendPiezoValue(piezoValue);
}

void onPiezoPeak(uint16_t peakValue)
{
    bleManager.sendPeakValue(peakValue);
}

void onBatteryUpdate(int percentage)
{
    bleManager.sendBatteryLevel(percentage);
}
