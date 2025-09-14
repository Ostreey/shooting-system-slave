#ifndef POWER_MANAGER_H
#define POWER_MANAGER_H

#include <Arduino.h>
#include <esp_adc_cal.h>
#include <esp_sleep.h>
#include <esp_task_wdt.h>
#include <esp_ota_ops.h>
#include "Config.h"

class LEDController; // Forward declaration

class PowerManager 
{
private:
    LEDController* ledController;
    esp_adc_cal_characteristics_t adcChars;
    TaskHandle_t batteryTaskHandle;
    unsigned long lastDisconnectTime;
    bool isGoingToSleep;
    bool justWokenUp;
    bool isInitialized;
    
    // Callback function pointer for battery level updates
    void (*batteryCallback)(int percentage);
    
public:
    PowerManager(LEDController* leds);
    
    // Initialization
    bool begin();
    void setLedController(LEDController* leds) { ledController = leds; }
    
    // Battery monitoring
    int getBatteryPercentage();
    void setBatteryCallback(void (*callback)(int));
    void startBatteryMonitoringTask();
    void stopBatteryMonitoringTask();
    
    // Sleep/Wake functionality
    void goToDeepSleep(bool skipLedBlink = false);
    bool validateWakeUp();
    bool isJustWokenUp() const { return justWokenUp; }
    void clearWakeUpFlag() { justWokenUp = false; }
    
    // Connection state management
    void setConnected(bool connected);
    bool shouldAutoSleep();
    
    // Charging detection
    bool isCharging();
    
    // Button handling
    bool isButtonPressed();
    
    // Task function (static)
    static void batteryMonitorTaskFunction(void* pvParameters);
};

#endif // POWER_MANAGER_H