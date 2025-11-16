#include <Arduino.h>
#include <Wire.h>
#include <esp_task_wdt.h>
#include <esp_sleep.h>
#include <esp_system.h>

#include "Config.h"
#include "PiezoSensor.h"
#include "LEDController.h"
#include "PowerManager.h"
#include "OTAManager.h"
#include "BLEManager.h"

LEDController ledController;
PowerManager powerManager(&ledController);
OTAManager otaManager;
BLEManager bleManager(&ledController, &powerManager, &otaManager);
PiezoSensor sensor(PIEZO_SENSOR_PIN, 400);

void onPiezoHit(int piezoValue);
void onBatteryUpdate(int percentage);

void setup()
{
    delay(100);
    Serial.begin(115200);
    Serial.println("Start - Modular Version");
   

    esp_sleep_wakeup_cause_t wakeup_cause = esp_sleep_get_wakeup_cause();
    if (wakeup_cause == ESP_SLEEP_WAKEUP_EXT0)
    {
        Serial.println("Woken up from deep sleep");

        if (!powerManager.validateWakeUp())
        {

            Serial.println("Wake-up validation failed - returning to deep sleep");
            powerManager.goToDeepSleep(true); // Skip LED blink since nothing is initialized yet
            return;                           // This line will never be reached, but good practice
        }

        Serial.println("Wake-up validation successful - continuing normal operation");
    }
    else
    {
        delay(2000);

        delay(3000); // Additional delay for power supply to stabilize
    }

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

    int batteryPercentage = powerManager.getBatteryPercentage();
    Serial.printf("Battery level: %d%%\n", batteryPercentage);

    if (batteryPercentage < BATTERY_LOW_THRESHOLD)
    {
        ledController.setRedChannel(255);
    }
    else
    {
        ledController.setBlueChannel(255);
    }

    if (wakeup_cause != ESP_SLEEP_WAKEUP_EXT0)
    {
        delay(1000);
    }

    if (batteryPercentage < BATTERY_LOW_THRESHOLD)
    {
        Serial.println("Battery too low for BLE operation, going to sleep immediately");
        powerManager.goToDeepSleep(true);
        return;
    }

    powerManager.setBatteryCallback(onBatteryUpdate);

    if (wakeup_cause != ESP_SLEEP_WAKEUP_EXT0)
    {
        ledController.turnOff();
        delay(1000);
    }

    sensor.begin();
    sensor.setCallback(onPiezoHit);

    if (!bleManager.begin())
    {
        Serial.println("Failed to initialize BLE manager");
        return;
    }

    powerManager.startBatteryMonitoringTask();

    bleManager.startLedStatusTask();

    Serial.println("All modules initialized successfully");

    if (otaManager.isResetAfterOTA())
    {
        Serial.println("System restarted after OTA update");
    }
}

void loop()
{
    static unsigned long pressStartTime = 0;
    static bool buttonPressed = false;

    otaManager.checkPendingRestart();

    if (powerManager.isButtonPressed())
    {
        if (!buttonPressed)
        {

            pressStartTime = millis();
            buttonPressed = true;
            Serial.println("Button pressed");

            if (powerManager.isJustWokenUp())
            {
                powerManager.clearWakeUpFlag();
                Serial.println("Reset justWokenUp flag");
            }
        }

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

void onPiezoHit(int piezoValue)
{
    ledController.turnOff();
    bleManager.sendPiezoValue(piezoValue);
}

void onBatteryUpdate(int percentage)
{
    bleManager.sendBatteryLevel(percentage);
}