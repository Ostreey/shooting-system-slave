#include <Arduino.h>
#include <Wire.h>
#include <esp_task_wdt.h>
#include <esp_sleep.h>

#include "Config.h"
#include "PiezoSensor.h"
#include "LEDController.h"
#include "PowerManager.h"
#include "OTAManager.h"
#include "BLEManager.h"
#include "GameSettings.h"

// Global instances
LEDController ledController;
PowerManager powerManager(&ledController);
OTAManager otaManager;
BLEManager bleManager(&ledController, &powerManager, &otaManager);
PiezoSensor sensor(PIEZO_SENSOR_PIN, 400);

// Forward declarations
void onPiezoHit(int piezoValue);
void onBatteryUpdate(int percentage);

void setup()
{
    delay(100);
    Serial.begin(115200);
    Serial.println("Start - Modular Version");

    // Check wake-up cause FIRST, before any other initialization
    esp_sleep_wakeup_cause_t wakeup_cause = esp_sleep_get_wakeup_cause();
    if (wakeup_cause == ESP_SLEEP_WAKEUP_EXT0)
    {
        Serial.println("Woken up from deep sleep");

        // Validate wake-up by checking if button is held for 2 seconds
        if (!powerManager.validateWakeUp())
        {
            // Button was not held long enough, go back to sleep
            Serial.println("Wake-up validation failed - returning to deep sleep");
            powerManager.goToDeepSleep(true); // Skip LED blink since nothing is initialized yet
            return;                           // This line will never be reached, but good practice
        }

        Serial.println("Wake-up validation successful - continuing normal operation");
    }
    else
    {
        delay(2000);
        // For fresh boots, add extra stabilization time
        delay(3000); // Additional delay for power supply to stabilize
    }

    // Initialize LED controller first (needed by other modules)
    if (!ledController.begin())
    {
        Serial.println("Failed to initialize LED controller");
        return;
    }
    if (!powerManager.begin())
    {
        Serial.println("Failed to initialize power manager");
        return;
    }

    // CRITICAL: Check battery voltage BEFORE initializing power-hungry components
    // If voltage is too low for BLE operation, go to sleep immediately
    int batteryPercentage = powerManager.getBatteryPercentage();
    Serial.printf("Battery level: %d%%\n", batteryPercentage);

    if (batteryPercentage < BATTERY_LOW_THRESHOLD){
        ledController.setRedChannel(255);
    } else {
        ledController.setBlueChannel(255);
    }


    // Show initialization progress with LEDs
    if (wakeup_cause == !ESP_SLEEP_WAKEUP_EXT0)
    {
        delay(1000);
    }

    if (batteryPercentage < BATTERY_LOW_THRESHOLD)
    {
        Serial.println("Battery too low for BLE operation, going to sleep immediately");
        powerManager.goToDeepSleep(true); // Skip LED blink to save power
        return;                           // This line will never be reached, but good practice
    }   
    // Set battery callback
    powerManager.setBatteryCallback(onBatteryUpdate);

    // Wait before initializing power-hungry components
    if (wakeup_cause != ESP_SLEEP_WAKEUP_EXT0)
    {
        ledController.turnOff();
        delay(1000);
    }

    // Initialize sensor
    sensor.begin();
    sensor.setCallback(onPiezoHit);

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
            Serial.println("Button pressed");

            // Reset justWokenUp flag immediately when button is pressed
            if (powerManager.isJustWokenUp())
            {
                powerManager.clearWakeUpFlag();
                Serial.println("Reset justWokenUp flag");
            }
        }

        // Check for long press while button is still held
        if ((millis() - pressStartTime) > TURN_OFF_TIME)
        {
            Serial.println("TURN OFF TIME DETECTED");
            powerManager.goToDeepSleep();
        }
    }
    else if (buttonPressed)
    {
        buttonPressed = false;
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

void onBatteryUpdate(int percentage)
{
    bleManager.sendBatteryLevel(percentage);
}

