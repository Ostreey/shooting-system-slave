#include <esp_log.h>
#include <esp_sleep.h>
#include <esp_system.h>
#include <nvs_flash.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#include "BLEManager.h"
#include "Config.h"
#include "LEDController.h"
#include "OTAManager.h"
#include "PiezoSensor.h"
#include "PowerManager.h"
#include "TimeUtils.h"

static const char *TAG = "Main";

static LEDController ledController;
static PowerManager powerManager(&ledController);
static OTAManager otaManager;
static BLEManager bleManager(&ledController, &powerManager, &otaManager);
static PiezoSensor sensor(PIEZO_SENSOR_PIN, 400);

static void onPiezoHit(int value)
{
    ledController.turnOff();
    bleManager.sendPiezoValue(value);
}

static void setup()
{
    ESP_LOGI(TAG, "Start - Modular Version");
    esp_sleep_wakeup_cause_t wakeup = esp_sleep_get_wakeup_cause();

    if (wakeup == ESP_SLEEP_WAKEUP_EXT0)
    {
        ESP_LOGI(TAG, "Woken up from deep sleep");
        if (!powerManager.validateWakeUp())
        {
            ESP_LOGW(TAG, "Wake-up validation failed");
            powerManager.goToDeepSleep(true);
            return;
        }
        ESP_LOGI(TAG, "Wake-up validation successful");
    }
    else
    {
        delayMs(5000);
    }

    if (!ledController.begin())
    {
        ESP_LOGE(TAG, "LED init failed");
        return;
    }
    if (!powerManager.begin())
    {
        ESP_LOGE(TAG, "Power manager init failed");
        return;
    }

    int battery = powerManager.getBatteryPercentage();
    ESP_LOGI(TAG, "Battery level %d%%", battery);
    if (battery < BATTERY_LOW_THRESHOLD)
    {
        ledController.setRedChannel(255);
    }
    else
    {
        ledController.setBlueChannel(255);
    }

    if (wakeup != ESP_SLEEP_WAKEUP_EXT0)
    {
        ledController.turnOff();
        delayMs(1000);
    }

    sensor.begin();
    sensor.setCallback(onPiezoHit);

    if (!bleManager.begin())
    {
        ESP_LOGE(TAG, "BLE init failed");
        return;
    }

    bleManager.startLedStatusTask();

    ESP_LOGI(TAG, "Modules initialized");

    if (otaManager.isResetAfterOTA())
    {
        ESP_LOGI(TAG, "System restarted after OTA update");
    }
}

static void loop()
{
    otaManager.checkPendingRestart();
    vTaskDelay(pdMS_TO_TICKS(100));
}

extern "C" void app_main(void)
{
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND)
    {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ESP_ERROR_CHECK(nvs_flash_init());
    }

    setup();
    while (true)
    {
        loop();
    }
}