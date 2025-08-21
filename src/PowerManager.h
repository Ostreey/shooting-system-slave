#pragma once

#include <Arduino.h>
#include <esp_adc_cal.h>

class PowerManager
{
public:
    PowerManager();
    void init();
    void startBatteryMonitorTask();
    int getBatteryPercentage();
    void goToDeepSleep(bool skipLedBlink = false);
    bool validateWakeUp();
    bool isGoingToSleep() const { return goingToSleep; }

    // Sleep/wake timing constants
    static const int TURN_OFF_TIME = 2000;
    static const int WAKE_UP_HOLD_TIME = 2000;
    static const unsigned long AUTO_SLEEP_TIME = 5 * 60 * 1000; // 5 minutes

private:
    // GPIO pins
    static const int wakeUpButton = 15;
    static const int batteryPin = 33;
    static const int chargingPin = 13;

    // ADC constants
    static const int DEFAULT_VREF = 1100;
    static const int ADC_SAMPLES = 64;
    static const adc_atten_t ADC_ATTEN = ADC_ATTEN_DB_11;

    esp_adc_cal_characteristics_t adc_chars;
    TaskHandle_t batteryTaskHandle;
    bool goingToSleep;

    static void batteryMonitorTask(void *pvParameters);
    void setupADC();
};

extern PowerManager powerManager;