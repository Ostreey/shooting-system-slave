#include "PowerManager.h"

#include "LEDController.h"
#include "driver/gpio.h"
#include "driver/rtc_io.h"
#include "esp_log.h"
#include "esp_sleep.h"
#include "esp_system.h"
#include "esp_task_wdt.h"

namespace
{
constexpr char TAG[] = "PowerManager";

void configureInputPin(gpio_num_t pin, bool pullup)
{
    gpio_config_t ioConfig = {};
    ioConfig.intr_type = GPIO_INTR_DISABLE;
    ioConfig.mode = GPIO_MODE_INPUT;
    ioConfig.pin_bit_mask = 1ULL << pin;
    ioConfig.pull_down_en = pullup ? GPIO_PULLDOWN_DISABLE : GPIO_PULLDOWN_ENABLE;
    ioConfig.pull_up_en = pullup ? GPIO_PULLUP_ENABLE : GPIO_PULLUP_DISABLE;
    gpio_config(&ioConfig);
}
}

PowerManager::PowerManager(LEDController *leds)
    : ledController(leds),
      batteryTaskHandle(nullptr),
      lastDisconnectTimeMs(0),
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

    configureInputPin(static_cast<gpio_num_t>(WAKE_UP_BUTTON_PIN), true);
    configureInputPin(static_cast<gpio_num_t>(CHARGING_PIN), true);

    adc1_config_width(ADC_WIDTH_BIT_12);
    adc1_config_channel_atten(BATTERY_ADC_CHANNEL, ADC_ATTEN);
    adc1_config_channel_atten(PIEZO_ADC_CHANNEL, ADC_ATTEN);
    esp_adc_cal_characterize(ADC_UNIT_1, ADC_ATTEN, ADC_WIDTH_BIT_12, DEFAULT_VREF, &adcChars);

    lastDisconnectTimeMs = millis64();
    isInitialized = true;

    ESP_LOGI(TAG, "Power manager initialized");
    return true;
}

int PowerManager::getBatteryPercentage()
{
    if (!isInitialized)
    {
        return 100;
    }

    uint32_t adcReading = 0;
    for (int i = 0; i < ADC_SAMPLES; ++i)
    {
        adcReading += adc1_get_raw(BATTERY_ADC_CHANNEL);
    }
    adcReading /= ADC_SAMPLES;

    uint32_t voltageMv = esp_adc_cal_raw_to_voltage(adcReading, &adcChars);
    float voltage = static_cast<float>(voltageMv) / 1000.0f;
    float batteryVoltage = voltage * 1.5f;
    int percentage = map_value(static_cast<int>(batteryVoltage * 100.0f),
                               BATTERY_MIN_VOLTAGE, BATTERY_MAX_VOLTAGE, 0, 100);
    return clamp_value(percentage, 0, 100);
}

void PowerManager::setBatteryCallback(void (*callback)(int))
{
    batteryCallback = callback;
}

bool PowerManager::isCharging()
{
    return gpio_get_level(static_cast<gpio_num_t>(CHARGING_PIN)) == 0;
}

void PowerManager::goToDeepSleep(bool skipLedBlink)
{
    if (isGoingToSleep)
    {
        return;
    }

    isGoingToSleep = true;
    esp_task_wdt_deinit();

    if (!skipLedBlink && ledController)
    {
        ledController->blinkColor(255, 255, 255, 3, 200, 200);
        delay_ms(500);
    }

    ESP_LOGI(TAG, "Preparing for deep sleep");

    configureInputPin(static_cast<gpio_num_t>(WAKE_UP_BUTTON_PIN), true);
    rtc_gpio_init(static_cast<gpio_num_t>(WAKE_UP_BUTTON_PIN));
    rtc_gpio_set_direction(static_cast<gpio_num_t>(WAKE_UP_BUTTON_PIN), RTC_GPIO_MODE_INPUT_ONLY);
    rtc_gpio_pulldown_dis(static_cast<gpio_num_t>(WAKE_UP_BUTTON_PIN));
    rtc_gpio_pullup_en(static_cast<gpio_num_t>(WAKE_UP_BUTTON_PIN));

    delay_ms(100);

    esp_sleep_disable_wakeup_source(ESP_SLEEP_WAKEUP_ALL);
    esp_err_t ext0Result = esp_sleep_enable_ext0_wakeup(static_cast<gpio_num_t>(WAKE_UP_BUTTON_PIN), 0);
    if (ext0Result != ESP_OK)
    {
        ESP_LOGW(TAG, "ext0 wake-up failed (%s), configuring ext1", esp_err_to_name(ext0Result));
        esp_sleep_enable_ext1_wakeup(1ULL << WAKE_UP_BUTTON_PIN, ESP_EXT1_WAKEUP_ANY_HIGH);
    }

    ESP_LOGI(TAG, "Deep sleep configuration complete");
    fflush(stdout);
    delay_ms(50);
    esp_deep_sleep_start();
}

bool PowerManager::validateWakeUp()
{
    ESP_LOGI(TAG, "Validating wake-up: button must stay pressed for %d ms", WAKE_UP_HOLD_TIME);

    configureInputPin(static_cast<gpio_num_t>(WAKE_UP_BUTTON_PIN), true);
    rtc_gpio_init(static_cast<gpio_num_t>(WAKE_UP_BUTTON_PIN));
    rtc_gpio_set_direction(static_cast<gpio_num_t>(WAKE_UP_BUTTON_PIN), RTC_GPIO_MODE_INPUT_ONLY);
    rtc_gpio_pulldown_dis(static_cast<gpio_num_t>(WAKE_UP_BUTTON_PIN));
    rtc_gpio_pullup_en(static_cast<gpio_num_t>(WAKE_UP_BUTTON_PIN));

    delay_ms(100);

    if (gpio_get_level(static_cast<gpio_num_t>(WAKE_UP_BUTTON_PIN)) != 0)
    {
        ESP_LOGW(TAG, "Button released before validation");
        return false;
    }

    uint64_t startTime = millis64();
    while ((millis64() - startTime) < WAKE_UP_HOLD_TIME)
    {
        if (gpio_get_level(static_cast<gpio_num_t>(WAKE_UP_BUTTON_PIN)) != 0)
        {
            ESP_LOGW(TAG, "Button released before hold interval completed");
            return false;
        }
        delay_ms(50);
    }

    justWokenUp = true;
    ESP_LOGI(TAG, "Wake-up validated");
    return true;
}

void PowerManager::setConnected(bool connected)
{
    if (connected)
    {
        lastDisconnectTimeMs = 0;
    }
    else
    {
        lastDisconnectTimeMs = millis64();
    }
}

bool PowerManager::shouldAutoSleep()
{
    if (lastDisconnectTimeMs == 0)
    {
        return false;
    }
    return (millis64() - lastDisconnectTimeMs) >= AUTO_SLEEP_TIME;
}

bool PowerManager::isButtonPressed()
{
    return gpio_get_level(static_cast<gpio_num_t>(WAKE_UP_BUTTON_PIN)) == 0;
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
            app_cpu());
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
    auto *powerManager = static_cast<PowerManager *>(pvParameters);

    for (;;)
    {
        int percentage = powerManager->getBatteryPercentage();

        if (powerManager->batteryCallback)
        {
            powerManager->batteryCallback(percentage);
        }

        if (powerManager->isCharging())
        {
            ESP_LOGI(TAG, "Charging detected, entering deep sleep");
            powerManager->goToDeepSleep(true);
        }

        if (percentage < BATTERY_LOW_THRESHOLD)
        {
            ESP_LOGW(TAG, "Battery low (%d%%), entering deep sleep", percentage);
            powerManager->goToDeepSleep();
        }

        if (powerManager->shouldAutoSleep())
        {
            ESP_LOGI(TAG, "Auto-sleep timeout reached, entering deep sleep");
            powerManager->goToDeepSleep();
        }

        vTaskDelay(pdMS_TO_TICKS(30000));
    }
}
