#ifndef LED_CONTROLLER_H
#define LED_CONTROLLER_H

#include <Arduino.h>
#include "Config.h"

class LEDController
{
private:
    int brightness;
    bool isInitialized;
    int requestedRed;
    int requestedGreen;
    int requestedBlue;
    volatile uint32_t outputWrites;
    volatile bool redGreenSwapped;

    enum class Led : uint8_t
    {
        Red,
        Green,
        Blue,
    };

    // The only place a logical colour becomes a PWM channel. Every ledcWrite in this class goes
    // through it, so the red/green swap of the newer LED modules covers every colour: the ones
    // sent by the app and the ones the target lights on its own.
    static uint8_t pwmChannel(Led led, bool swapped);

    void writeAll(int finalRed, int finalGreen, int finalBlue);

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

    // Whether anything is lit right now. Read by the battery task to find out whether an unloaded
    // reading is possible; it is never allowed to switch the output off to make one, because the
    // game logic owns these channels.
    bool isOutputOn() const { return (requestedRed | requestedGreen | requestedBlue) > 0; }

    // Bumped on every write to the channels, from any task. The battery task reads it before and
    // after its unloaded measurement: "off" at both ends is not enough, because the LED status
    // blink or a game command can light the target during the settle delay and switch it off
    // again, and a reading taken under that load must not be trusted as unloaded.
    uint32_t getOutputWriteCount() const { return outputWrites; }

    // Special effects
    void blinkLeds(int count = 3, int onTime = 200, int offTime = 200);
    void blinkColor(int red, int green, int blue, int count = 3, int onTime = 200, int offTime = 200);
    void blinkGreen(); // For connection status

    // Turn off all LEDs
    void turnOff();

    // Power control
    void enablePower(bool enable);

    // Red/green swap of the newer external LED modules. begin() loads it from NVS before the
    // first colour is written. Every call re-applies the current colour with the given mapping.
    bool isRedGreenSwapped() const { return redGreenSwapped; }
    void setRedGreenSwapped(bool swapped);

    // Colour test for the app: drives the board's own red (or green) output, ignoring the swap,
    // so what the user reports depends only on the module, never on the setting stored before.
    void setRawTestColor(bool boardRedPin);
};

#endif // LED_CONTROLLER_H
