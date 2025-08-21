#include "HardwareInterface.h"
#include "LedController.h"
#include "BleManager.h"
#include "PowerManager.h"

extern LedController ledController;
extern BleManager bleManager;
extern PowerManager powerManager;

// Global instance
HardwareInterface hardwareInterface;

HardwareInterface::HardwareInterface() : sensor(32, 400),
                                         ledStatusTaskHandle(NULL),
                                         justWokenUp(false),
                                         pressStartTime(0),
                                         buttonPressed(false),
                                         hitTime(0)
{
}

void HardwareInterface::init()
{
    // Initialize sensor
    sensor.begin();
    sensor.setCallback(piezoCallback);

    Serial.println("Hardware Interface initialized");
}

void HardwareInterface::startLedStatusTask()
{
    xTaskCreatePinnedToCore(
        ledStatusTask,
        "LEDStatus",
        2048,
        NULL,
        1,
        &ledStatusTaskHandle,
        APP_CPU_NUM);
}

void HardwareInterface::handleButton()
{
    // Button handling logic
    if (digitalRead(wakeUpButton) == LOW)
    {
        if (!buttonPressed)
        {
            pressStartTime = millis();
            buttonPressed = true;
            Serial.println("Button pressed");
        }

        // Check for long press while button is still held (skip if this is the wake-up press)
        if (!justWokenUp && (millis() - pressStartTime) > PowerManager::TURN_OFF_TIME)
        {
            Serial.println("TURN OFF TIME DETECTED");
            powerManager.goToDeepSleep();
        }
    }
    else if (buttonPressed)
    {
        buttonPressed = false;

        if (justWokenUp)
        {
            justWokenUp = false;
        }
    }
}

void HardwareInterface::onPiezoHit(int piezoValue)
{
    ledController.setLeds(false);
    bleManager.sendPiezoData(piezoValue, hitTime);
}

void HardwareInterface::onStartCommand()
{
    hitTime = millis();
    ledController.setLeds(true);
}

void HardwareInterface::ledStatusTask(void *pvParameters)
{
    for (;;)
    {
        if (!bleManager.isDeviceConnected() && !powerManager.isGoingToSleep())
        {
            ledController.setLeds(false);
            vTaskDelay(1000 / portTICK_PERIOD_MS);
            ledController.setLeds(true);
            vTaskDelay(1000 / portTICK_PERIOD_MS);
            Serial.println("Device not connected");
        }
        else
        {
            vTaskDelay(100 / portTICK_PERIOD_MS);
        }
    }
}

// Static callback function for piezo sensor
void HardwareInterface::piezoCallback(int value)
{
    hardwareInterface.onPiezoHit(value);
}