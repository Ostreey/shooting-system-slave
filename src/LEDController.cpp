#include "LEDController.h"

LEDController::LEDController() : brightness(255), isInitialized(false)
{
}

bool LEDController::begin()
{
    if (isInitialized) {
        return true;
    }
    
    // Initialize GPIO pins
    pinMode(LED_POWER_ENABLE_PIN, OUTPUT);
    pinMode(LED_BLUE_PIN, OUTPUT);
    pinMode(LED_GREEN_PIN, OUTPUT);
    pinMode(LED_RED_PIN, OUTPUT);
    
    // Enable LED power
    digitalWrite(LED_POWER_ENABLE_PIN, HIGH);
    
    // Initialize LEDs to off
    digitalWrite(LED_BLUE_PIN, LOW);
    digitalWrite(LED_GREEN_PIN, LOW);
    digitalWrite(LED_RED_PIN, LOW);
    
    // Configure PWM for LEDs
    ledcSetup(PWM_CHANNEL_RED, PWM_FREQUENCY, PWM_RESOLUTION);
    ledcSetup(PWM_CHANNEL_GREEN, PWM_FREQUENCY, PWM_RESOLUTION);
    ledcSetup(PWM_CHANNEL_BLUE, PWM_FREQUENCY, PWM_RESOLUTION);
    
    // Attach PWM channels to GPIO pins
    ledcAttachPin(LED_RED_PIN, PWM_CHANNEL_RED);
    ledcAttachPin(LED_GREEN_PIN, PWM_CHANNEL_GREEN);
    ledcAttachPin(LED_BLUE_PIN, PWM_CHANNEL_BLUE);
    
    isInitialized = true;
    return true;
}

void LEDController::enablePower(bool enable)
{
    digitalWrite(LED_POWER_ENABLE_PIN, enable ? HIGH : LOW);
}

void LEDController::setLeds(bool on)
{
    if (!isInitialized) return;
    
    if (on) {
        ledcWrite(PWM_CHANNEL_RED, brightness);
        ledcWrite(PWM_CHANNEL_GREEN, brightness);
        ledcWrite(PWM_CHANNEL_BLUE, brightness);
    } else {
        ledcWrite(PWM_CHANNEL_RED, 0);
        ledcWrite(PWM_CHANNEL_GREEN, 0);
        ledcWrite(PWM_CHANNEL_BLUE, 0);
    }
}

void LEDController::setRgbColor(int red, int green, int blue)
{
    if (!isInitialized) return;
    
    // Apply global brightness to each color channel
    int finalRed = (red * brightness) / 255;
    int finalGreen = (green * brightness) / 255;
    int finalBlue = (blue * brightness) / 255;
    
    ledcWrite(PWM_CHANNEL_RED, finalRed);
    ledcWrite(PWM_CHANNEL_GREEN, finalGreen);
    ledcWrite(PWM_CHANNEL_BLUE, finalBlue);
    
    Serial.printf("RGB set to: R=%d, G=%d, B=%d (brightness=%d)\n", 
                  finalRed, finalGreen, finalBlue, brightness);
}

void LEDController::setRedChannel(int value)
{
    if (!isInitialized) return;
    
    int finalValue = (value * brightness) / 255;
    ledcWrite(PWM_CHANNEL_RED, finalValue);
    Serial.printf("Red channel set to: %d (brightness=%d)\n", finalValue, brightness);
}

void LEDController::setGreenChannel(int value)
{
    if (!isInitialized) return;
    
    int finalValue = (value * brightness) / 255;
    ledcWrite(PWM_CHANNEL_GREEN, finalValue);
    Serial.printf("Green channel set to: %d (brightness=%d)\n", finalValue, brightness);
}

void LEDController::setBlueChannel(int value)
{
    if (!isInitialized) return;
    
    int finalValue = (value * brightness) / 255;
    ledcWrite(PWM_CHANNEL_BLUE, finalValue);
    Serial.printf("Blue channel set to: %d (brightness=%d)\n", finalValue, brightness);
}

void LEDController::setBrightness(int value)
{
    brightness = constrain(value, 0, 255);
    Serial.printf("Brightness set to: %d\n", brightness);
}

void LEDController::setPredefinedColor(const String& colorName)
{
    if (!isInitialized) return;
    
    if (colorName == "red") {
        setRgbColor(255, 0, 0);
    }
    else if (colorName == "green") {
        setRgbColor(0, 255, 0);
    }
    else if (colorName == "blue") {
        setRgbColor(0, 0, 255);
    }
    else if (colorName == "white") {
        setRgbColor(255, 255, 255);
    }
    else if (colorName == "yellow") {
        setRgbColor(255, 255, 0);
    }
    else if (colorName == "magenta") {
        setRgbColor(255, 0, 255);
    }
    else if (colorName == "cyan") {
        setRgbColor(0, 255, 255);
    }
    else if (colorName == "off") {
        setRgbColor(0, 0, 0);
    }
    else {
        Serial.printf("Unknown color: %s\n", colorName.c_str());
    }
}

void LEDController::blinkLeds(int count, int onTime, int offTime)
{
    if (!isInitialized) return;
    
    for (int i = 0; i < count; i++) {
        setRgbColor(0, 255, 0);
        vTaskDelay(onTime / portTICK_PERIOD_MS);
        setRgbColor(0, 0, 0);
        vTaskDelay(offTime / portTICK_PERIOD_MS);
    }
}

void LEDController::blinkGreen()
{
    if (!isInitialized) return;
    
    setRgbColor(0, 255, 0);
    vTaskDelay(1000 / portTICK_PERIOD_MS);
    setRgbColor(0, 0, 0);
}

void LEDController::turnOff()
{
    if (!isInitialized) return;
    
    setRgbColor(0, 0, 0);
}