#include "PowerManager.h"
#include "LEDController.h"

#include <algorithm>
#include <esp_log.h>
#include <driver/adc.h>

namespace
{
const char *TAG = "PowerManager";
constexpr gpio_num_t kWakePin = static_cast<gpio_num_t>(WAKE_UP_BUTTON_PIN);
constexpr gpio_num_t kChargingPin = static_cast<gpio_num_t>(CHARGING_PIN);
constexpr adc1_channel_t kBatteryChannel = ADC1_CHANNEL_0;

int mapRange(int x, int inMin, int inMax, int outMin, int outMax)
{
    return (x - inMin) * (outMax - outMin) / (inMax - inMin) + outMin;
}

void configureInput(gpio_num_t pin, gpio_pull_mode_t pullMode)
{
    gpio_config_t cfg{};
    cfg.intr_type = GPIO_INTR_DISABLE;
    cfg.mode = GPIO_MODE_INPUT;
    cfg.pin_bit_mask = 1ULL << pin;
    cfg.pull_down_en = pullMode == GPIO_PULLDOWN_ONLY ? GPIO_PULLDOWN_ENABLE : GPIO_PULLDOWN_DISABLE;
    cfg.pull_up_en = pullMode == GPIO_PULLUP_ONLY ? GPIO_PULLUP_ENABLE : GPIO_PULLUP_DISABLE;
    gpio_config(&cfg);
}
}

PowerManager::PowerManager(LEDController *leds)
    : ledController(leds),
      batteryTaskHandle(nullptr),
      lastDisconnectTime(0),
      isGoingToSleep(false),
      justWokenUp(false),
      isInitialized(false),
      batteryCallback(nullptr)
{
}

bool PowerManager::begin()
{
    if (isInitialized)
    {
        return true;
    }

    configureInput(kWakePin, GPIO_PULLUP_ONLY);
    configureInput(static_cast<gpio_num_t>(BATTERY_PIN), GPIO_FLOATING);
    configureInput(kChargingPin, GPIO_PULLUP_ONLY);
    adc1_config_width(ADC_WIDTH_BIT_12);
    adc1_config_channel_atten(kBatteryChannel, ADC_ATTEN);
    esp_adc_cal_characterize(ADC_UNIT_1, ADC_ATTEN, ADC_WIDTH_BIT_12, DEFAULT_VREF, &adcChars);
    lastDisconnectTime = millis();
    isInitialized = true;
    return true;
}

int PowerManager::getBatteryPercentage()
{
    if (!isInitialized)
    {
        return 100;
    }

    uint32_t reading = 0;
    for (int i = 0; i < ADC_SAMPLES; ++i)
    {
        reading += adc1_get_raw(kBatteryChannel);
    }
    reading /= ADC_SAMPLES;
    uint32_t voltageMv = esp_adc_cal_raw_to_voltage(reading, &adcChars);
    float batteryVoltage = (voltageMv / 1000.0f) * 1.5f;
    int percentage = mapRange(static_cast<int>(batteryVoltage * 100), BATTERY_MIN_VOLTAGE, BATTERY_MAX_VOLTAGE, 0, 100);
    percentage = std::clamp(percentage, 0, 100);
    return percentage;
}

void PowerManager::setBatteryCallback(void (*callback)(int))
{
    batteryCallback = callback;
}

bool PowerManager::isCharging()
{
    return gpio_get_level(kChargingPin) == 0;
}

void PowerManager::goToDeepSleep(bool skipLedBlink)
{
    isGoingToSleep = true;
    esp_task_wdt_deinit();

    if (!skipLedBlink && ledController)
    {
        ledController->blinkColor(255, 255, 255, 3, 200, 200);
        delayMs(500);
    }

    ESP_LOGI(TAG, "Entering deep sleep");
    configureInput(kWakePin, GPIO_PULLUP_ONLY);
    delayMs(100);
    esp_err_t ext0Result = esp_sleep_enable_ext0_wakeup(kWakePin, 0);
    if (ext0Result != ESP_OK)
    {
        ESP_LOGW(TAG, "EXT0 wakeup failed %s", esp_err_to_name(ext0Result));
        esp_sleep_enable_ext1_wakeup(1ULL << kWakePin, ESP_EXT1_WAKEUP_ANY_HIGH);
    }

    delayMs(50);
    esp_deep_sleep_start();
}

bool PowerManager::validateWakeUp()
{
    ESP_LOGI(TAG, "Validating wake button hold");
    configureInput(kWakePin, GPIO_PULLUP_ONLY);
    delayMs(100);

    if (gpio_get_level(kWakePin) != 0)
    {
        ESP_LOGW(TAG, "Wake button not pressed");
        return false;
    }

    uint32_t startTime = millis();

    while ((millis() - startTime) < WAKE_UP_HOLD_TIME)
    {
        if (gpio_get_level(kWakePin) != 0)
        {
            ESP_LOGW(TAG, "Wake button released early");
            return false;
        }
        delayMs(50);
    }

    justWokenUp = true;
    ESP_LOGI(TAG, "Wake validated");
    return true;
}

void PowerManager::setConnected(bool connected)
{
    if (connected)
    {
        lastDisconnectTime = 0;
    }
    else
    {
        lastDisconnectTime = millis();
    }
}

bool PowerManager::shouldAutoSleep()
{
    return lastDisconnectTime > 0 && (millis() - lastDisconnectTime) >= AUTO_SLEEP_TIME;
}

bool PowerManager::isButtonPressed()
{
    return gpio_get_level(kWakePin) == 0;
}

void PowerManager::startBatteryMonitoringTask()
{
    if (batteryTaskHandle == nullptr)
    {
        xTaskCreatePinnedToCore(
            batteryMonitorTaskFunction,
            "BatteryMonitor",
            4096,
            this,
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

void PowerManager::batteryMonitorTaskFunction(void *pvParameters)
{
    PowerManager *powerManager = static_cast<PowerManager *>(pvParameters);

    for (;;)
    {
        int percentage = powerManager->getBatteryPercentage();

        if (powerManager->batteryCallback)
        {
            powerManager->batteryCallback(percentage);
        }

        if (powerManager->isCharging())
        {
            ESP_LOGI(TAG, "Charging detected, sleeping");
            powerManager->goToDeepSleep(true);
        }

        if (percentage < BATTERY_LOW_THRESHOLD)
        {
            ESP_LOGW(TAG, "Battery low, sleeping");
            powerManager->goToDeepSleep();
        }

        if (powerManager->shouldAutoSleep())
        {
            ESP_LOGI(TAG, "Auto sleep timeout");
            powerManager->goToDeepSleep();
        }

        vTaskDelay(pdMS_TO_TICKS(30000));
    }
}
