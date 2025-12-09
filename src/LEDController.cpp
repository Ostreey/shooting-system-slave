#include "LEDController.h"

#include <array>
#include <inttypes.h>

#include "driver/gpio.h"
#include "driver/ledc.h"
#include "esp_log.h"

namespace
{
constexpr char TAG[] = "LEDController";

constexpr ledc_mode_t kLedcMode = LEDC_LOW_SPEED_MODE;
constexpr ledc_timer_t kLedcTimer = LEDC_TIMER_0;

inline uint32_t applyBrightness(int value, int brightness)
{
    value = clamp_value(value, 0, 255);
    brightness = clamp_value(brightness, 0, 255);
    return static_cast<uint32_t>((value * brightness) / 255);
}

void configureChannel(int gpio, ledc_channel_t channel)
{
    ledc_channel_config_t channelConfig = {};
    channelConfig.gpio_num = gpio;
    channelConfig.speed_mode = kLedcMode;
    channelConfig.channel = channel;
    channelConfig.intr_type = LEDC_INTR_DISABLE;
    channelConfig.timer_sel = kLedcTimer;
    channelConfig.duty = 0;
    channelConfig.hpoint = 0;
    ledc_channel_config(&channelConfig);
}

void writeChannel(ledc_channel_t channel, uint32_t duty)
{
    ledc_set_duty(kLedcMode, channel, duty);
    ledc_update_duty(kLedcMode, channel);
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

    gpio_config_t ioConfig = {};
    ioConfig.intr_type = GPIO_INTR_DISABLE;
    ioConfig.mode = GPIO_MODE_OUTPUT;
    ioConfig.pin_bit_mask = (1ULL << LED_POWER_ENABLE_PIN) |
                            (1ULL << LED_BLUE_PIN) |
                            (1ULL << LED_GREEN_PIN) |
                            (1ULL << LED_RED_PIN);
    ioConfig.pull_down_en = GPIO_PULLDOWN_DISABLE;
    ioConfig.pull_up_en = GPIO_PULLUP_DISABLE;
    gpio_config(&ioConfig);

    gpio_set_level(static_cast<gpio_num_t>(LED_POWER_ENABLE_PIN), 1);
    gpio_set_level(static_cast<gpio_num_t>(LED_BLUE_PIN), 0);
    gpio_set_level(static_cast<gpio_num_t>(LED_GREEN_PIN), 0);
    gpio_set_level(static_cast<gpio_num_t>(LED_RED_PIN), 0);

    ledc_timer_config_t timerConfig = {};
    timerConfig.speed_mode = kLedcMode;
    timerConfig.timer_num = kLedcTimer;
    timerConfig.freq_hz = PWM_FREQUENCY;
    timerConfig.duty_resolution = static_cast<ledc_timer_bit_t>(PWM_RESOLUTION);
    timerConfig.clk_cfg = LEDC_AUTO_CLK;
    ledc_timer_config(&timerConfig);

    configureChannel(LED_RED_PIN, static_cast<ledc_channel_t>(PWM_CHANNEL_RED));
    configureChannel(LED_GREEN_PIN, static_cast<ledc_channel_t>(PWM_CHANNEL_GREEN));
    configureChannel(LED_BLUE_PIN, static_cast<ledc_channel_t>(PWM_CHANNEL_BLUE));

    isInitialized = true;
    ESP_LOGI(TAG, "LED controller initialized");
    return true;
}

void LEDController::enablePower(bool enable)
{
    gpio_set_level(static_cast<gpio_num_t>(LED_POWER_ENABLE_PIN), enable ? 1 : 0);
}

void LEDController::setLeds(bool on)
{
    if (!isInitialized)
    {
        return;
    }

    uint32_t duty = on ? clamp_value(brightness, 0, 255) : 0;
    writeChannel(static_cast<ledc_channel_t>(PWM_CHANNEL_RED), duty);
    writeChannel(static_cast<ledc_channel_t>(PWM_CHANNEL_GREEN), duty);
    writeChannel(static_cast<ledc_channel_t>(PWM_CHANNEL_BLUE), duty);
}

void LEDController::setRgbColor(int red, int green, int blue)
{
    if (!isInitialized)
    {
        return;
    }

    uint32_t finalRed = applyBrightness(red, brightness);
    uint32_t finalGreen = applyBrightness(green, brightness);
    uint32_t finalBlue = applyBrightness(blue, brightness);

    writeChannel(static_cast<ledc_channel_t>(PWM_CHANNEL_RED), finalRed);
    writeChannel(static_cast<ledc_channel_t>(PWM_CHANNEL_GREEN), finalGreen);
    writeChannel(static_cast<ledc_channel_t>(PWM_CHANNEL_BLUE), finalBlue);

    ESP_LOGI(TAG, "RGB set to R=%" PRIu32 ", G=%" PRIu32 ", B=%" PRIu32 ", brightness=%d",
             finalRed, finalGreen, finalBlue, brightness);
}

void LEDController::setRedChannel(int value)
{
    if (!isInitialized)
    {
        return;
    }
    uint32_t finalValue = applyBrightness(value, brightness);
    writeChannel(static_cast<ledc_channel_t>(PWM_CHANNEL_RED), finalValue);
    ESP_LOGI(TAG, "Red channel=%" PRIu32 ", brightness=%d", finalValue, brightness);
}

void LEDController::setGreenChannel(int value)
{
    if (!isInitialized)
    {
        return;
    }
    uint32_t finalValue = applyBrightness(value, brightness);
    writeChannel(static_cast<ledc_channel_t>(PWM_CHANNEL_GREEN), finalValue);
    ESP_LOGI(TAG, "Green channel=%" PRIu32 ", brightness=%d", finalValue, brightness);
}

void LEDController::setBlueChannel(int value)
{
    if (!isInitialized)
    {
        return;
    }
    uint32_t finalValue = applyBrightness(value, brightness);
    writeChannel(static_cast<ledc_channel_t>(PWM_CHANNEL_BLUE), finalValue);
    ESP_LOGI(TAG, "Blue channel=%" PRIu32 ", brightness=%d", finalValue, brightness);
}

void LEDController::setBrightness(int value)
{
    brightness = clamp_value(value, 0, 255);
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
        ESP_LOGW(TAG, "Unknown color '%s'", colorName.c_str());
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
        delay_ms(onTime);
        setRgbColor(0, 0, 0);
        delay_ms(offTime);
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
        delay_ms(onTime);
        setRgbColor(0, 0, 0);
        delay_ms(offTime);
    }
}

void LEDController::blinkGreen()
{
    if (!isInitialized)
    {
        return;
    }

    setRgbColor(0, 255, 0);
    delay_ms(1000);
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
