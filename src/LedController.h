#pragma once

#include <Arduino.h>

class LedController
{
public:
    LedController();
    void init();
    void setLeds(bool on);
    void blinkLeds();
    void setBrightness(int brightness);
    int getBrightness() const { return currentBrightness; }

private:
    int currentBrightness;

    // GPIO pins
    static const int ledRed = 19;
    static const int ledGreen = 18;
    static const int ledBlue = 17;
    static const int ledPowerEnable = 4;

    // PWM constants
    static const int PWM_FREQUENCY = 5000;
    static const int PWM_RESOLUTION = 8;
    static const int PWM_CHANNEL_RED = 0;
    static const int PWM_CHANNEL_GREEN = 1;
    static const int PWM_CHANNEL_BLUE = 2;
};

extern LedController ledController;