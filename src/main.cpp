#include "BLEManager.h"
#include "LEDController.h"
#include "OTAManager.h"
#include "PiezoSensor.h"
#include "PowerManager.h"

#include "Config.h"
#include "IdfCompat.h"

#include "esp_log.h"
#include "esp_sleep.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

namespace
{
constexpr char TAG[] = "MainApp";

LEDController ledController;
PowerManager powerManager(&ledController);
OTAManager otaManager;
BLEManager bleManager(&ledController, &powerManager, &otaManager);
PiezoSensor sensor(PIEZO_SENSOR_PIN, 400);
bool initializationComplete = false;

void onPiezoHit(int piezoValue);
void onBatteryUpdate(int percentage);

void initializeApplication()
{
    initializationComplete = false;
    delay_ms(100);
    ESP_LOGI(TAG, "Start - Modular Version");

    esp_sleep_wakeup_cause_t wakeupCause = esp_sleep_get_wakeup_cause();
    if (wakeupCause == ESP_SLEEP_WAKEUP_EXT0)
    {
        ESP_LOGI(TAG, "Woken up from deep sleep");
        if (!powerManager.validateWakeUp())
        {
            ESP_LOGW(TAG, "Wake-up validation failed, returning to deep sleep");
            powerManager.goToDeepSleep(true);
            return;
        }
        ESP_LOGI(TAG, "Wake-up validation successful");
    }
    else
    {
        delay_ms(2000);
        delay_ms(3000);
    }

    if (!ledController.begin())
    {
        ESP_LOGE(TAG, "Failed to initialize LED controller");
        return;
    }
    if (!powerManager.begin())
    {
        ESP_LOGE(TAG, "Failed to initialize power manager");
        return;
    }

    int batteryPercentage = powerManager.getBatteryPercentage();
    ESP_LOGI(TAG, "Battery level: %d%%", batteryPercentage);

    if (batteryPercentage < BATTERY_LOW_THRESHOLD)
    {
        ledController.setRedChannel(255);
    }
    else
    {
        ledController.setBlueChannel(255);
    }

    if (wakeupCause != ESP_SLEEP_WAKEUP_EXT0)
    {
        delay_ms(1000);
    }

    if (batteryPercentage < BATTERY_LOW_THRESHOLD)
    {
        ESP_LOGW(TAG, "Battery too low for BLE operation, entering deep sleep");
        powerManager.goToDeepSleep(true);
        return;
    }

    powerManager.setBatteryCallback(onBatteryUpdate);

    if (wakeupCause != ESP_SLEEP_WAKEUP_EXT0)
    {
        ledController.turnOff();
        delay_ms(1000);
    }

    sensor.begin();
    sensor.setCallback(onPiezoHit);

    if (!bleManager.begin())
    {
        ESP_LOGE(TAG, "Failed to initialize BLE manager");
        return;
    }

    powerManager.startBatteryMonitoringTask();
    bleManager.startLedStatusTask();

    ESP_LOGI(TAG, "All modules initialized successfully");

    if (otaManager.isResetAfterOTA())
    {
        ESP_LOGI(TAG, "System restarted after OTA update");
    }

    initializationComplete = true;
}

void appLoopIteration()
{
    static uint64_t pressStartTimeMs = 0;
    static bool buttonPressed = false;

    otaManager.checkPendingRestart();

    if (powerManager.isButtonPressed())
    {
        if (!buttonPressed)
        {
            pressStartTimeMs = millis64();
            buttonPressed = true;
            ESP_LOGI(TAG, "Button pressed");

            if (powerManager.isJustWokenUp())
            {
                powerManager.clearWakeUpFlag();
                ESP_LOGI(TAG, "Reset justWokenUp flag");
            }
        }

        if ((millis64() - pressStartTimeMs) > TURN_OFF_TIME)
        {
            ESP_LOGI(TAG, "Turn-off time detected");
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
}

extern "C" void app_main(void)
{
    initializeApplication();

    if (!initializationComplete)
    {
        ESP_LOGE(TAG, "Initialization failed, halting main loop");
        vTaskDelay(portMAX_DELAY);
        return;
    }

    while (true)
    {
        appLoopIteration();
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}