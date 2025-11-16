#include "LEDController.h"

#include <algorithm>
#include <esp_err.h>
#include <esp_log.h>
#include <driver/gpio.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

namespace
{
const char *TAG = "LEDController";
constexpr uint8_t kMaxChannelValue = 255;

uint32_t toLedcValue(int value, int brightness)
{
    value = std::clamp(value, 0, 255);
    brightness = std::clamp(brightness, 0, 255);
    return static_cast<uint32_t>((value * brightness) / 255);
}

void applyDuty(ledc_channel_t channel, uint32_t duty)
{
    ledc_set_duty(PWM_MODE, channel, duty);
    ledc_update_duty(PWM_MODE, channel);
}
}

LEDController::LEDController() : brightness(255), isInitialized(false)
{
}

bool LEDController::begin()
{
    if (isInitialized)
    {
        return true;
    }

    gpio_config_t io{};
    io.intr_type = GPIO_INTR_DISABLE;
    io.mode = GPIO_MODE_OUTPUT;
    io.pin_bit_mask = (1ULL << LED_POWER_ENABLE_PIN) | (1ULL << LED_RED_PIN) | (1ULL << LED_GREEN_PIN) | (1ULL << LED_BLUE_PIN);
    io.pull_down_en = GPIO_PULLDOWN_DISABLE;
    io.pull_up_en = GPIO_PULLUP_DISABLE;
    gpio_config(&io);

    gpio_set_level(LED_POWER_ENABLE_PIN, 1);
    gpio_set_level(LED_RED_PIN, 0);
    gpio_set_level(LED_GREEN_PIN, 0);
    gpio_set_level(LED_BLUE_PIN, 0);

    ledc_timer_config_t timerCfg{};
    timerCfg.speed_mode = PWM_MODE;
    timerCfg.timer_num = PWM_TIMER;
    timerCfg.duty_resolution = PWM_RESOLUTION;
    timerCfg.freq_hz = PWM_FREQUENCY;
    timerCfg.clk_cfg = LEDC_AUTO_CLK;
    ESP_ERROR_CHECK(ledc_timer_config(&timerCfg));

    ledc_channel_config_t redCfg{};
    redCfg.speed_mode = PWM_MODE;
    redCfg.channel = PWM_CHANNEL_RED;
    redCfg.timer_sel = PWM_TIMER;
    redCfg.intr_type = LEDC_INTR_DISABLE;
    redCfg.gpio_num = LED_RED_PIN;
    redCfg.duty = 0;
    ESP_ERROR_CHECK(ledc_channel_config(&redCfg));

    ledc_channel_config_t greenCfg = redCfg;
    greenCfg.channel = PWM_CHANNEL_GREEN;
    greenCfg.gpio_num = LED_GREEN_PIN;
    ESP_ERROR_CHECK(ledc_channel_config(&greenCfg));

    ledc_channel_config_t blueCfg = redCfg;
    blueCfg.channel = PWM_CHANNEL_BLUE;
    blueCfg.gpio_num = LED_BLUE_PIN;
    ESP_ERROR_CHECK(ledc_channel_config(&blueCfg));

    isInitialized = true;
    return true;
}

void LEDController::enablePower(bool enable)
{
    gpio_set_level(LED_POWER_ENABLE_PIN, enable ? 1 : 0);
}

void LEDController::setLeds(bool on)
{
    if (!isInitialized)
    {
        return;
    }

    uint32_t duty = on ? toLedcValue(kMaxChannelValue, brightness) : 0;
    applyDuty(PWM_CHANNEL_RED, duty);
    applyDuty(PWM_CHANNEL_GREEN, duty);
    applyDuty(PWM_CHANNEL_BLUE, duty);
}

void LEDController::setRgbColor(int red, int green, int blue)
{
    if (!isInitialized)
    {
        return;
    }

    applyDuty(PWM_CHANNEL_RED, toLedcValue(red, brightness));
    applyDuty(PWM_CHANNEL_GREEN, toLedcValue(green, brightness));
    applyDuty(PWM_CHANNEL_BLUE, toLedcValue(blue, brightness));
    ESP_LOGI(TAG, "RGB set to R=%d G=%d B=%d brightness=%d", red, green, blue, brightness);
}

void LEDController::setRedChannel(int value)
{
    if (!isInitialized)
    {
        return;
    }
    applyDuty(PWM_CHANNEL_RED, toLedcValue(value, brightness));
    ESP_LOGI(TAG, "Red channel=%d brightness=%d", value, brightness);
}

void LEDController::setGreenChannel(int value)
{
    if (!isInitialized)
    {
        return;
    }
    applyDuty(PWM_CHANNEL_GREEN, toLedcValue(value, brightness));
    ESP_LOGI(TAG, "Green channel=%d brightness=%d", value, brightness);
}

void LEDController::setBlueChannel(int value)
{
    if (!isInitialized)
    {
        return;
    }
    applyDuty(PWM_CHANNEL_BLUE, toLedcValue(value, brightness));
    ESP_LOGI(TAG, "Blue channel=%d brightness=%d", value, brightness);
}

void LEDController::setBrightness(int value)
{
    brightness = std::clamp(value, 0, 255);
    ESP_LOGI(TAG, "Brightness=%d", brightness);
}

void LEDController::setPredefinedColor(const std::string &colorName)
{
    if (!isInitialized)
    {
        return;
    }

    if (colorName == "red")
    {
        setRgbColor(255, 0, 0);
    }
    else if (colorName == "green")
    {
        setRgbColor(0, 255, 0);
    }
    else if (colorName == "blue")
    {
        setRgbColor(0, 0, 255);
    }
    else if (colorName == "white")
    {
        setRgbColor(255, 255, 255);
    }
    else if (colorName == "yellow")
    {
        setRgbColor(255, 255, 0);
    }
    else if (colorName == "magenta")
    {
        setRgbColor(255, 0, 255);
    }
    else if (colorName == "cyan")
    {
        setRgbColor(0, 255, 255);
    }
    else if (colorName == "off")
    {
        setRgbColor(0, 0, 0);
    }
    else
    {
        ESP_LOGW(TAG, "Unknown color %s", colorName.c_str());
    }
}

void LEDController::blinkLeds(int count, int onTime, int offTime)
{
    if (!isInitialized)
    {
        return;
    }

    for (int i = 0; i < count; ++i)
    {
        setRgbColor(0, 255, 0);
        vTaskDelay(pdMS_TO_TICKS(onTime));
        setRgbColor(0, 0, 0);
        vTaskDelay(pdMS_TO_TICKS(offTime));
    }
}

void LEDController::blinkColor(int red, int green, int blue, int count, int onTime, int offTime)
{
    if (!isInitialized)
    {
        return;
    }

    for (int i = 0; i < count; ++i)
    {
        setRgbColor(red, green, blue);
        vTaskDelay(pdMS_TO_TICKS(onTime));
        setRgbColor(0, 0, 0);
        vTaskDelay(pdMS_TO_TICKS(offTime));
    }
}

void LEDController::blinkGreen()
{
    if (!isInitialized)
    {
        return;
    }

    setRgbColor(0, 255, 0);
    vTaskDelay(pdMS_TO_TICKS(1000));
    setRgbColor(0, 0, 0);
}

void LEDController::turnOff()
{
    if (!isInitialized)
    {
        return;
    }
    setRgbColor(0, 0, 0);
}
