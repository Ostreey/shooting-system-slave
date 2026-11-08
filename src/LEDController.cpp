#include "LEDController.h"
#include "LedModuleStore.h"

LEDController::LEDController() : brightness(255), isInitialized(false),
                                 requestedRed(0), requestedGreen(0), requestedBlue(0),
                                 outputWrites(0), redGreenSwapped(false)
{
}

uint8_t LEDController::pwmChannel(Led led, bool swapped)
{
    switch (led)
    {
    case Led::Red:
        return swapped ? PWM_CHANNEL_GREEN : PWM_CHANNEL_RED;
    case Led::Green:
        return swapped ? PWM_CHANNEL_RED : PWM_CHANNEL_GREEN;
    case Led::Blue:
    default:
        return PWM_CHANNEL_BLUE;
    }
}

void LEDController::writeAll(int finalRed, int finalGreen, int finalBlue)
{
    // One snapshot per write: the flag can be changed from the BLE task, and reading it twice
    // could put red and green on the same PWM channel.
    const bool swapped = redGreenSwapped;
    ledcWrite(pwmChannel(Led::Red, swapped), finalRed);
    ledcWrite(pwmChannel(Led::Green, swapped), finalGreen);
    ledcWrite(pwmChannel(Led::Blue, swapped), finalBlue);
}

bool LEDController::begin()
{
    if (isInitialized)
    {
        return true;
    }

    // Loaded here and not in setup(): the override red flash on the flat-cell start path calls
    // begin() long before the rest of the start, and it must already come out red on a new module.
    redGreenSwapped = LedModuleStore::loadRedGreenSwapped();
    Serial.printf("LED module: red/green %s\n", redGreenSwapped ? "swapped" : "normal");

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
    if (!isInitialized)
        return;

    outputWrites = outputWrites + 1;
    requestedRed = on ? 255 : 0;
    requestedGreen = requestedRed;
    requestedBlue = requestedRed;

    if (on)
    {
        writeAll(brightness, brightness, brightness);
    }
    else
    {
        writeAll(0, 0, 0);
    }
}

void LEDController::setRgbColor(int red, int green, int blue)
{
    if (!isInitialized)
        return;

    outputWrites = outputWrites + 1;
    requestedRed = red;
    requestedGreen = green;
    requestedBlue = blue;

    // Apply global brightness to each color channel
    int finalRed = (red * brightness) / 255;
    int finalGreen = (green * brightness) / 255;
    int finalBlue = (blue * brightness) / 255;

    writeAll(finalRed, finalGreen, finalBlue);

    Serial.printf("RGB set to: R=%d, G=%d, B=%d (brightness=%d)\n",
                  finalRed, finalGreen, finalBlue, brightness);
}

void LEDController::setRedChannel(int value)
{
    if (!isInitialized)
        return;

    outputWrites = outputWrites + 1;
    requestedRed = value;
    int finalValue = (value * brightness) / 255;
    ledcWrite(pwmChannel(Led::Red, redGreenSwapped), finalValue);
    Serial.printf("Red channel set to: %d (brightness=%d)\n", finalValue, brightness);
}

void LEDController::setGreenChannel(int value)
{
    if (!isInitialized)
        return;

    outputWrites = outputWrites + 1;
    requestedGreen = value;
    int finalValue = (value * brightness) / 255;
    ledcWrite(pwmChannel(Led::Green, redGreenSwapped), finalValue);
    Serial.printf("Green channel set to: %d (brightness=%d)\n", finalValue, brightness);
}

void LEDController::setBlueChannel(int value)
{
    if (!isInitialized)
        return;

    outputWrites = outputWrites + 1;
    requestedBlue = value;
    int finalValue = (value * brightness) / 255;
    ledcWrite(pwmChannel(Led::Blue, redGreenSwapped), finalValue);
    Serial.printf("Blue channel set to: %d (brightness=%d)\n", finalValue, brightness);
}

void LEDController::setRedGreenSwapped(bool swapped)
{
    if (redGreenSwapped != swapped)
    {
        redGreenSwapped = swapped;
        Serial.printf("LED module: red/green %s\n", swapped ? "swapped" : "normal");
    }

    if (!isInitialized)
        return;

    // Rewritten even when the setting did not change: the answer to the colour test arrives while
    // the raw test pin is lit, and the target must then show what is red under the setting, also
    // on "Repeat test" with the same answer. A changed setting would otherwise also leave the lit
    // colour on the other physical channel. Never lights anything: the requested values are written.
    outputWrites = outputWrites + 1;
    writeAll((requestedRed * brightness) / 255,
             (requestedGreen * brightness) / 255,
             (requestedBlue * brightness) / 255);
}

void LEDController::setRawTestColor(bool boardRedPin)
{
    if (!isInitialized)
        return;

    outputWrites = outputWrites + 1;

    // Recorded as the logical colour of the same name, whatever the current setting: the battery
    // task only needs "something is lit", and the swap setting sent in answer to the test then
    // re-lights the target in what is red under the new setting - the user's visible confirmation.
    requestedRed = boardRedPin ? 255 : 0;
    requestedGreen = boardRedPin ? 0 : 255;
    requestedBlue = 0;

    int finalValue = brightness;
    ledcWrite(PWM_CHANNEL_RED, boardRedPin ? finalValue : 0);
    ledcWrite(PWM_CHANNEL_GREEN, boardRedPin ? 0 : finalValue);
    ledcWrite(PWM_CHANNEL_BLUE, 0);
    Serial.printf("LED raw test: board %s pin = %d\n", boardRedPin ? "red" : "green", finalValue);
}

void LEDController::setBrightness(int value)
{
    brightness = constrain(value, 0, 255);
    Serial.printf("Brightness set to: %d\n", brightness);
}

void LEDController::setPredefinedColor(const String &colorName)
{
    if (!isInitialized)
        return;

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
        Serial.printf("Unknown color: %s\n", colorName.c_str());
    }
}

void LEDController::blinkLeds(int count, int onTime, int offTime)
{
    if (!isInitialized)
        return;

    for (int i = 0; i < count; i++)
    {
        setRgbColor(0, 255, 0);
        vTaskDelay(onTime / portTICK_PERIOD_MS);
        setRgbColor(0, 0, 0);
        vTaskDelay(offTime / portTICK_PERIOD_MS);
    }
}

void LEDController::blinkColor(int red, int green, int blue, int count, int onTime, int offTime)
{
    if (!isInitialized)
        return;

    for (int i = 0; i < count; i++)
    {
        setRgbColor(red, green, blue);
        vTaskDelay(onTime / portTICK_PERIOD_MS);
        setRgbColor(0, 0, 0);
        vTaskDelay(offTime / portTICK_PERIOD_MS);
    }
}

void LEDController::blinkGreen()
{
    if (!isInitialized)
        return;

    setRgbColor(0, 255, 0);
    vTaskDelay(1000 / portTICK_PERIOD_MS);
    setRgbColor(0, 0, 0);
}

void LEDController::turnOff()
{
    if (!isInitialized)
        return;

    setRgbColor(0, 0, 0);
}
