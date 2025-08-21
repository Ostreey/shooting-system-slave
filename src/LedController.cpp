#include "LedController.h"

// Global instance
LedController ledController;

LedController::LedController() : currentBrightness(255)
{
}

void LedController::init()
{
    // Initialize LED power enable pin
    pinMode(ledPowerEnable, OUTPUT);
    digitalWrite(ledPowerEnable, HIGH);

    // Initialize LED pins
    pinMode(ledRed, OUTPUT);
    pinMode(ledGreen, OUTPUT);
    pinMode(ledBlue, OUTPUT);

    // Set initial state
    digitalWrite(ledRed, HIGH);
    digitalWrite(ledGreen, HIGH);
    digitalWrite(ledBlue, HIGH);

    // Configure PWM for LEDs
    ledcSetup(PWM_CHANNEL_RED, PWM_FREQUENCY, PWM_RESOLUTION);
    ledcSetup(PWM_CHANNEL_GREEN, PWM_FREQUENCY, PWM_RESOLUTION);
    ledcSetup(PWM_CHANNEL_BLUE, PWM_FREQUENCY, PWM_RESOLUTION);

    // Attach PWM channels to GPIO pins
    ledcAttachPin(ledRed, PWM_CHANNEL_RED);
    ledcAttachPin(ledGreen, PWM_CHANNEL_GREEN);
    ledcAttachPin(ledBlue, PWM_CHANNEL_BLUE);

    Serial.println("LED Controller initialized");
}

void LedController::setLeds(bool on)
{
    if (on)
    {
        ledcWrite(PWM_CHANNEL_RED, currentBrightness);
        ledcWrite(PWM_CHANNEL_GREEN, currentBrightness);
        ledcWrite(PWM_CHANNEL_BLUE, currentBrightness);
    }
    else
    {
        ledcWrite(PWM_CHANNEL_RED, 0);
        ledcWrite(PWM_CHANNEL_GREEN, 0);
        ledcWrite(PWM_CHANNEL_BLUE, 0);
    }
}

void LedController::blinkLeds()
{
    for (int i = 0; i < 3; i++)
    {
        setLeds(true);
        vTaskDelay(200 / portTICK_PERIOD_MS);
        setLeds(false);
        vTaskDelay(200 / portTICK_PERIOD_MS);
    }
}

void LedController::setBrightness(int brightness)
{
    currentBrightness = constrain(brightness, 0, 255);
    Serial.print("LED brightness set to: ");
    Serial.println(currentBrightness);
}