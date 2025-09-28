#ifndef LED_CONTROLLER_H
#define LED_CONTROLLER_H

#include <Arduino.h>
#include "Config.h"

class LEDController
{
private:
    int brightness;
    bool isInitialized;

public:
    LEDController();

    // Initialization
    bool begin();

    // Basic LED control
    void setLeds(bool on);
    void setRgbColor(int red, int green, int blue);

    // Individual channel control
    void setRedChannel(int value);
    void setGreenChannel(int value);
    void setBlueChannel(int value);

    // Predefined colors
    void setPredefinedColor(const String &colorName);

    // Brightness control
    void setBrightness(int value);
    int getBrightness() const { return brightness; }

    // Special effects
    void blinkLeds(int count = 3, int onTime = 200, int offTime = 200);
    void blinkColor(int red, int green, int blue, int count = 3, int onTime = 200, int offTime = 200);
    void blinkGreen(); // For connection status

    // Turn off all LEDs
    void turnOff();

    // Power control
    void enablePower(bool enable);
};

#endif // LED_CONTROLLER_H
