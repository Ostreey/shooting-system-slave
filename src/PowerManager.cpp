#include "PowerManager.h"
#include "LEDController.h"
#include "BatteryGuard.h"
#include "BatteryLatchStore.h"
#include "BatteryScale.h"
#ifdef BOARD_ESP32S3
#include "driver/rtc_io.h"
#endif

namespace
{
    const char *sleepReasonName(SleepReason reason)
    {
        switch (reason)
        {
        case SleepReason::Latch:
            return "latch";
        case SleepReason::LowVoltage:
            return "lowvolt";
        case SleepReason::Button:
            return "button";
        case SleepReason::Charger:
            return "charger";
        case SleepReason::AutoSleep:
            return "autosleep";
        case SleepReason::WakeCheck:
            return "wakecheck";
        case SleepReason::Command:
            return "command";
        }
        return "unknown";
    }
}

PowerManager::PowerManager(LEDController *leds)
    : ledController(leds), batteryTaskHandle(nullptr), lastDisconnectTime(0),
      isGoingToSleep(false), justWokenUp(false), isInitialized(false),
      lastBatteryMv(0), chargerPrevMv(0), chargerBaselineMv(0), chargerRiseStreak(0),
      batteryCallback(nullptr)
{
}

bool PowerManager::begin()
{
    if (isInitialized)
    {
        return true;
    }

    // Initialize GPIO pins
    pinMode(WAKE_UP_BUTTON_PIN, INPUT_PULLUP);
    gpio_set_pull_mode((gpio_num_t)WAKE_UP_BUTTON_PIN, GPIO_PULLUP_ONLY);
    gpio_set_direction((gpio_num_t)WAKE_UP_BUTTON_PIN, GPIO_MODE_INPUT);
    pinMode(BATTERY_PIN, INPUT);
    pinMode(CHARGING_PIN, INPUT_PULLUP);

    // ADC calibration
    esp_adc_cal_characterize(ADC_UNIT_1, ADC_ATTEN, ADC_WIDTH_BIT_12, DEFAULT_VREF, &adcChars);

    // Initialize disconnect timer to start auto-sleep countdown from boot
    lastDisconnectTime = millis();

    isInitialized = true;
    return true;
}

int PowerManager::percentFromMv(int batteryMv)
{
    // Discharge curve lives in BatteryScale so it can be checked on the host; the numbers are the
    // 1.0.8 ones, because the released app gates OTA on this value.
    return BatteryScale::percentFromMv(batteryMv);
}

int PowerManager::getBatteryPercentage()
{
    if (!isInitialized)
        return 100;

    return percentFromMv(readBatteryMv());
}

int PowerManager::readBatteryMv()
{
    if (!isInitialized)
        return 0;

    uint32_t adcReading = 0;
    for (int i = 0; i < ADC_SAMPLES; i++)
        adcReading += analogRead(BATTERY_PIN);
    adcReading /= ADC_SAMPLES;

    uint32_t voltageMv = esp_adc_cal_raw_to_voltage(adcReading, &adcChars);
    int batteryMv = (int)BatteryScale::cellMvFromAdcMv((int32_t)voltageMv);
    Serial.printf("Battery: %d.%02dV\n", batteryMv / 1000, (batteryMv % 1000) / 10);

    lastBatteryMv = batteryMv;
    return batteryMv;
}

bool PowerManager::tryReadUnloadedBatteryMv(int &outMv)
{
    // The LED rail dominates the load, so a reading taken while it is on says nothing about how
    // much charge is left. This used to switch the output off and put the colour back afterwards,
    // and that restore is a hit counted twice: the game logic turns the target dark after a hit,
    // this measurement relights it, the shooter fires at a target that should have been off and
    // the final score ends up quietly too high. So the output is never touched here - if it is
    // on, there is no reading this cycle. A target that stays lit is covered by the loaded floor
    // in BatteryGuard::decideRuntime instead.
    if (ledController == nullptr)
    {
        outMv = readBatteryMv();
        return true;
    }

    if (ledController->isOutputOn())
    {
        return false;
    }

    const uint32_t writesBefore = ledController->getOutputWriteCount();

    // The output can have gone dark a millisecond ago, so the cell still gets its recovery time
    // before the reading that decides whether it is empty.
    delay(BATTERY_UNLOADED_SETTLE_MS);
    int mv = readBatteryMv();

    // Anything written to the channels during the settle delay or the ADC burst (the blue status
    // blink, a colour command from the app) may have loaded the cell for part of this reading.
    if (ledController->isOutputOn() || ledController->getOutputWriteCount() != writesBefore)
    {
        return false;
    }

    outMv = mv;
    return true;
}

void PowerManager::setBatteryCallback(void (*callback)(int))
{
    batteryCallback = callback;
}

bool PowerManager::isCharging()
{
    // Old ESP32 boards have CHRG# wired here. On the ESP32-S3 board IO9 is not connected at all,
    // so with INPUT_PULLUP this reads HIGH forever - an extra signal, never the only one.
    return digitalRead(CHARGING_PIN) == LOW;
}

bool PowerManager::updateChargerEvidence(int batteryMv)
{
    // Samples taken with the LED output on carry the load step, not the charger, so they are
    // discarded and the streak starts over from the next clean reading.
    if (ledController && ledController->isOutputOn())
    {
        chargerBaselineMv = 0;
        chargerPrevMv = 0;
        chargerRiseStreak = 0;
        return isCharging();
    }

    if (chargerPrevMv == 0)
    {
        chargerPrevMv = batteryMv;
        chargerBaselineMv = batteryMv;
        chargerRiseStreak = 0;
        return isCharging();
    }

    if (batteryMv - chargerPrevMv >= BATTERY_CHARGER_RISE_STEP_MV)
    {
        if (chargerRiseStreak < 255)
            chargerRiseStreak++;
    }
    else
    {
        // The baseline follows the streak instead of being a session minimum: a cell relaxing
        // after one load step gives a single rise and then a plateau, and that must not stay
        // credited forever as "voltage is above the lowest I ever saw".
        chargerRiseStreak = 0;
        chargerBaselineMv = batteryMv;
    }

    chargerPrevMv = batteryMv;

    bool rising = chargerRiseStreak >= 2 &&
                  (batteryMv - chargerBaselineMv) >= BATTERY_CHARGER_RISE_MV;

    return rising || isCharging();
}

void PowerManager::goToDeepSleep(SleepReason reason, bool skipLedBlink, bool wakeOnReleaseIfHeld)
{
    isGoingToSleep = true;

    // DISABLE WATCHDOG FIRST - before any delays or LED operations
    esp_task_wdt_deinit();

    Serial.printf("Sleeping: reason=%s mv=%d\n", sleepReasonName(reason), lastBatteryMv);

    // Below the flat threshold the target gives no light at all, not even a goodbye blink: white
    // drives all three channels of the external LED modules on the 12V rail at once, which is the
    // largest load in the whole duty cycle, the board has no current-limiting resistors and the
    // module current is unknown. That blink is what collapsed the 3.3V rail and had the
    // TPS3823-33 supervisor restart the MCU into another start attempt.
    // lastBatteryMv == 0 means "not measured yet" and suppresses the blink as well.
    bool blinkAllowed = !skipLedBlink &&
                        lastBatteryMv >= BATTERY_FLAT_THRESHOLD_MV &&
                        ledController != nullptr;

    if (blinkAllowed && ledController)
    {
        // Use LED controller for white blinking when powering off
        ledController->blinkColor(255, 255, 255, 3, 200, 200);

        // Small delay to show the device is turning off
        delay(500);
    }

    // The run-time loaded-floor path sleeps a target the app left lit, so the channels are dropped
    // here explicitly instead of trusting the pads to fall dark in deep sleep. Sleeping ends the
    // game, so this does not fight the game logic for the LEDs.
    if (ledController != nullptr && ledController->isOutputOn())
    {
        ledController->turnOff();
    }

    // Configure wake-up button GPIO properly before sleep
    configureWakeButton();

    // Add small delay to stabilize GPIO state
    delay(100);

    // ext0 wakes on a level, not an edge: sleeping on "wake when LOW" under a held button wakes
    // the target again at once. On the start path of a flat, latched cell that turns a stuck
    // button (a target pressed against something in a crate) into a target awake nearly all the
    // time, running the cell into the 2.4V DW01A cutoff. So there the wake is armed on the
    // release instead; the release costs one more short start, which sleeps again armed normally.
    int wakeLevel = 0;
    if (wakeOnReleaseIfHeld && isButtonPressed())
    {
        wakeLevel = 1;
        Serial.println("Button held at sleep - wake armed on release");
    }

    esp_sleep_disable_wakeup_source(ESP_SLEEP_WAKEUP_ALL);
    esp_err_t ext0Result = esp_sleep_enable_ext0_wakeup((gpio_num_t)WAKE_UP_BUTTON_PIN, wakeLevel);
    if (ext0Result != ESP_OK)
    {
        Serial.printf("ext0 wake-up failed, trying ext1: %s\n", esp_err_to_name(ext0Result));
        esp_sleep_enable_ext1_wakeup((1ULL << WAKE_UP_BUTTON_PIN), ESP_EXT1_WAKEUP_ANY_HIGH);
    }

    // Final delay to ensure all operations are complete
    delay(50);

    // Printed last on purpose: t= is the awake time the firmware is actually responsible for,
    // including every delay above. Printed earlier it would understate it by the whole sleep
    // sequence, and that number is the pass criterion of the QA timing scenario.
    Serial.printf("SLEEP reason=%s t=%lu mv=%d\n", sleepReasonName(reason), millis(), lastBatteryMv);
    Serial.flush(); // Ensure all serial data is sent before sleep

    esp_deep_sleep_start();
}

void PowerManager::configureWakeButton()
{
    gpio_set_direction((gpio_num_t)WAKE_UP_BUTTON_PIN, GPIO_MODE_INPUT);
    gpio_set_pull_mode((gpio_num_t)WAKE_UP_BUTTON_PIN, GPIO_PULLUP_ONLY);
#ifdef BOARD_ESP32S3
    // ESP32-S3 requires RTC GPIO init - regular GPIO pull-ups don't survive deep sleep
    rtc_gpio_init((gpio_num_t)WAKE_UP_BUTTON_PIN);
    rtc_gpio_set_direction((gpio_num_t)WAKE_UP_BUTTON_PIN, RTC_GPIO_MODE_INPUT_ONLY);
    rtc_gpio_pulldown_dis((gpio_num_t)WAKE_UP_BUTTON_PIN);
    rtc_gpio_pullup_en((gpio_num_t)WAKE_UP_BUTTON_PIN);
#endif
}

bool PowerManager::holdButtonFor(uint32_t holdMs)
{
    configureWakeButton();

    // Settle before the first read: the pull-up has to take effect after RTC GPIO init.
    delay(BATTERY_BUTTON_DEBOUNCE_MS);

    if (!isButtonPressed())
    {
        return false;
    }

    unsigned long startTime = millis();
    while ((millis() - startTime) < holdMs)
    {
        if (!isButtonPressed())
        {
            return false;
        }
        delay(20);
    }

    Serial.printf("Button held %lu ms - latch override\n", (unsigned long)holdMs);
    return true;
}

bool PowerManager::latchOverrideProbe(void *context)
{
    PowerManager *powerManager = static_cast<PowerManager *>(context);
    if (powerManager == nullptr)
    {
        return false;
    }

    Serial.printf("Guard would sleep - waiting %d ms for a button override\n",
                  BATTERY_LATCH_BUTTON_HOLD_MS);
    return powerManager->holdButtonFor(BATTERY_LATCH_BUTTON_HOLD_MS);
}

bool PowerManager::validateWakeUp()
{
    Serial.println("Validating wake-up - button must be held for 2 seconds...");

    // Ensure wake-up button GPIO is properly configured for reading
    configureWakeButton();

    // Small delay to stabilize GPIO state
    delay(100);

    // Check if button is still pressed when we start validation
    if (digitalRead(WAKE_UP_BUTTON_PIN) != LOW)
    {
        Serial.println("Button not pressed during validation - going back to sleep");
        return false;
    }

    unsigned long startTime = millis();

    while ((millis() - startTime) < WAKE_UP_HOLD_TIME)
    {
        // Check if button was released during the hold period
        if (digitalRead(WAKE_UP_BUTTON_PIN) != LOW)
        {
            Serial.println("Button released before 2 seconds - going back to sleep");
            return false;
        }

        delay(50); // Small delay to prevent excessive polling
    }

    // If we reach here, button was held for full 2 seconds
    Serial.println("Wake-up validated - button held for 2 seconds");

    // Set flag to indicate we just woke up successfully
    // This will prevent turn-off detection for the current button press
    justWokenUp = true;

    return true;
}

void PowerManager::setConnected(bool connected)
{
    if (connected)
    {
        lastDisconnectTime = 0; // Reset disconnect timer when connected
    }
    else
    {
        lastDisconnectTime = millis(); // Record disconnection time
    }
}

bool PowerManager::shouldAutoSleep()
{
    return (lastDisconnectTime > 0 &&
            (millis() - lastDisconnectTime) >= AUTO_SLEEP_TIME);
}

bool PowerManager::isButtonPressed()
{
    return digitalRead(WAKE_UP_BUTTON_PIN) == LOW;
}

void PowerManager::startBatteryMonitoringTask()
{
    if (batteryTaskHandle == nullptr)
    {
        xTaskCreatePinnedToCore(
            batteryMonitorTaskFunction,
            "BatteryMonitor",
            4096,
            this, // Pass this instance as parameter
            2,
            &batteryTaskHandle,
            APP_CPU_NUM);
    }
}

void PowerManager::stopBatteryMonitoringTask()
{
    if (batteryTaskHandle != nullptr)
    {
        vTaskDelete(batteryTaskHandle);
        batteryTaskHandle = nullptr;
    }
}

// Static task function
void PowerManager::batteryMonitorTaskFunction(void *pvParameters)
{
    PowerManager *powerManager = static_cast<PowerManager *>(pvParameters);
    BatteryGuard::Thresholds thresholds = BatteryGuard::defaultThresholds();

    // No exemption after a button override (owner decision 2026-11-07 evening): a target kept
    // connected after an override would otherwise run its cell into the 2.4V DW01A cutoff, because
    // the buck-boost holds the rail and the supervisor never resets it.

#ifdef MIGRATION_TEST_NO_BATTERY
    // No-battery test build: a floating divider reads near 0 mV, so the voltage trigger would put
    // the target to sleep on the first pass. Zeroing the threshold here - instead of skipping the
    // decision - keeps the charger trigger and the BLE sleep command live.
    thresholds.flatMv = 0;
    thresholds.loadedFloorMv = 0;
#endif

    int32_t loadedLowStreak = 0;

    for (;;)
    {
        int batteryMv = powerManager->readBatteryMv();
        int percentage = percentFromMv(batteryMv);

        // Send battery level through callback if set. This path keeps the discharge LUT and the
        // ADC offset untouched: the released app blocks OTA below 41%.
        if (powerManager->batteryCallback)
        {
            powerManager->batteryCallback(percentage);
        }

        // Confirm a low reading without the LED load before acting on it. A cell at 3.7V resting
        // dips under the threshold while the white LED is on, and reading that as an empty cell
        // would end a competition in the middle.
        int unloadedMv = batteryMv;
        bool unloadedKnown = true;
        if (batteryMv < thresholds.flatMv)
        {
            unloadedKnown = powerManager->tryReadUnloadedBatteryMv(unloadedMv);
        }

        loadedLowStreak = BatteryGuard::nextLoadedLowStreak(loadedLowStreak, batteryMv,
                                                            unloadedKnown, thresholds);
        if (!unloadedKnown)
        {
            Serial.printf("Battery guard: mv=%d with LED output on - unloaded check deferred, "
                          "loaded-floor streak %d/%d\n",
                          batteryMv, (int)loadedLowStreak, (int)thresholds.loadedFloorSamples);
        }

        // Never sleep "because of a charger" while a phone is connected: nobody charges a target
        // mid-session, and a target falling asleep during a game - or during an OTA transfer -
        // is worse than slow charging.
        bool chargerEvidence = powerManager->updateChargerEvidence(batteryMv) &&
                               !powerManager->isConnected();

        switch (BatteryGuard::decideRuntime(batteryMv, unloadedMv, unloadedKnown,
                                            loadedLowStreak, chargerEvidence, thresholds))
        {
        case BatteryGuard::RuntimeAction::SleepLowVoltage:
            // Deliberately does NOT set the latch. A target that was running can always be
            // brought back with a normal button press; the latch belongs to the start path,
            // where the cell is measured with the LED rail off and BLE down.
            Serial.printf("Battery guard: run-time low voltage mv=%d unloaded=%d (%s), going to sleep\n",
                          batteryMv, unloadedKnown ? unloadedMv : -1,
                          unloadedKnown ? "unloaded" : "loaded floor");
            powerManager->goToDeepSleep(SleepReason::LowVoltage, true);
            break;

        case BatteryGuard::RuntimeAction::SleepCharger:
            Serial.println("Charging detected, going to sleep");
            powerManager->goToDeepSleep(SleepReason::Charger, true);
            break;

        case BatteryGuard::RuntimeAction::Continue:
            break;
        }

#ifndef MIGRATION_TEST_NO_BATTERY
        // Check for auto-sleep after timeout when not connected. Off in the no-battery test build:
        // a migration session needs a target that stays reachable for hours without a phone.
        if (powerManager->shouldAutoSleep())
        {
            Serial.println("No connection for 5 minutes, going to sleep");
            powerManager->goToDeepSleep(SleepReason::AutoSleep);
        }
#endif

        vTaskDelay(30000 / portTICK_PERIOD_MS); // 30 seconds delay
    }
}
