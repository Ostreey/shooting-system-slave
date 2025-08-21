#pragma once

#include <Arduino.h>
#include "PiezoSensor.h"

class HardwareInterface
{
public:
    HardwareInterface();
    void init();
    void handleButton();
    void startLedStatusTask();

    bool isJustWokenUp() const { return justWokenUp; }
    void setJustWokenUp(bool value) { justWokenUp = value; }

    void onPiezoHit(int piezoValue);
    void onStartCommand();

private:
    PiezoSensor sensor;
    TaskHandle_t ledStatusTaskHandle;

    // Button state tracking
    bool justWokenUp;
    unsigned long pressStartTime;
    bool buttonPressed;

    // Hit timing
    unsigned long hitTime;

    static const int wakeUpButton = 15;

    static void ledStatusTask(void *pvParameters);
    static void piezoCallback(int value);
};

extern HardwareInterface hardwareInterface;