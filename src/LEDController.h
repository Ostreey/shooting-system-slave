#ifndef LED_CONTROLLER_H
#define LED_CONTROLLER_H

#include <cstdint>
#include <string>
#include "Config.h"

class LEDController
{
private:
    int brightness;
    bool isInitialized;

public:
    LEDController();
    bool begin();
    void setLeds(bool on);
    void setRgbColor(int red, int green, int blue);
    void setRedChannel(int value);
    void setGreenChannel(int value);
    void setBlueChannel(int value);
    void setPredefinedColor(const std::string &colorName);
    void setBrightness(int value);
    int getBrightness() const { return brightness; }
    void blinkLeds(int count = 3, int onTime = 200, int offTime = 200);
    void blinkColor(int red, int green, int blue, int count = 3, int onTime = 200, int offTime = 200);
    void blinkGreen();
    void turnOff();
    void enablePower(bool enable);
};

#endif
