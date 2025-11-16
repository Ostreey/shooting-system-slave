#ifndef POWER_MANAGER_H
#define POWER_MANAGER_H

#include <esp_adc_cal.h>
#include <esp_sleep.h>
#include <esp_task_wdt.h>
#include <esp_ota_ops.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <driver/gpio.h>
#include "Config.h"
#include "TimeUtils.h"

class LEDController;

class PowerManager 
{
private:
    LEDController* ledController;
    esp_adc_cal_characteristics_t adcChars;
    TaskHandle_t batteryTaskHandle;
    uint32_t lastDisconnectTime;
    bool isGoingToSleep;
    bool justWokenUp;
    bool isInitialized;
    void (*batteryCallback)(int percentage);
public:
    PowerManager(LEDController* leds);
    bool begin();
    void setLedController(LEDController* leds) { ledController = leds; }
    int getBatteryPercentage();
    void setBatteryCallback(void (*callback)(int));
    void startBatteryMonitoringTask();
    void stopBatteryMonitoringTask();
    void goToDeepSleep(bool skipLedBlink = false);
    bool validateWakeUp();
    bool isJustWokenUp() const { return justWokenUp; }
    void clearWakeUpFlag() { justWokenUp = false; }
    void setConnected(bool connected);
    bool shouldAutoSleep();
    bool isCharging();
    bool isButtonPressed();
    static void batteryMonitorTaskFunction(void* pvParameters);
};

#endif
