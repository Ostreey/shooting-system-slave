#include "PowerManager.h"
#include "LedController.h"
#include "BleManager.h"

extern LedController ledController;
extern BleManager bleManager;

// Global instance
PowerManager powerManager;

PowerManager::PowerManager() : batteryTaskHandle(NULL),
                               goingToSleep(false)
{
}

void PowerManager::init()
{
    // Initialize pins
    pinMode(wakeUpButton, INPUT_PULLUP);
    pinMode(batteryPin, INPUT);
    pinMode(chargingPin, INPUT_PULLUP);

    // Setup ADC calibration
    setupADC();

    Serial.println("Power Manager initialized");
}

void PowerManager::setupADC()
{
    esp_adc_cal_characterize(ADC_UNIT_1, ADC_ATTEN, ADC_WIDTH_BIT_12, DEFAULT_VREF, &adc_chars);
}

void PowerManager::startBatteryMonitorTask()
{
    xTaskCreatePinnedToCore(
        batteryMonitorTask,
        "BatteryMonitor",
        2048,
        NULL,
        2,
        &batteryTaskHandle,
        APP_CPU_NUM);
}

int PowerManager::getBatteryPercentage()
{
    // Read multiple samples and average them
    uint32_t adc_reading = 0;
    for (int i = 0; i < ADC_SAMPLES; i++)
    {
        adc_reading += analogRead(batteryPin);
    }
    adc_reading /= ADC_SAMPLES;

    // Convert to voltage using calibration
    uint32_t voltage_mv = esp_adc_cal_raw_to_voltage(adc_reading, &adc_chars);
    float voltage = voltage_mv / 1000.0;

    // Convert to actual battery voltage (multiply by 1.5 due to voltage divider)
    float battery_voltage = voltage * 1.5;

    // Convert to percentage (assuming 4.2V is 100% and 3.6V is 0%)
    int percentage = map(battery_voltage * 100, 340, 420, 0, 100);
    percentage = constrain(percentage, 0, 100);

    return percentage;
}

void PowerManager::goToDeepSleep(bool skipLedBlink)
{
    goingToSleep = true;

    if (!skipLedBlink)
    {
        ledController.blinkLeds();
        delay(3000);
    }

    digitalWrite(4, LOW); // ledPowerEnable
    Serial.println("Going to deep sleep...");
    esp_sleep_enable_ext0_wakeup(GPIO_NUM_15, 0);
    esp_deep_sleep_start();
}

bool PowerManager::validateWakeUp()
{
    Serial.println("Validating wake-up - button must be held for 2 seconds...");

    // Check if button is still pressed when we start validation
    if (digitalRead(wakeUpButton) != LOW)
    {
        Serial.println("Button not pressed during validation - going back to sleep");
        return false;
    }

    unsigned long startTime = millis();

    while ((millis() - startTime) < WAKE_UP_HOLD_TIME)
    {
        if (digitalRead(wakeUpButton) != LOW)
        {
            Serial.println("Button released before 2 seconds - going back to sleep");
            return false;
        }
        delay(50);
    }

    Serial.println("Wake-up validated - button held for 2 seconds");
    return true;
}

void PowerManager::batteryMonitorTask(void *pvParameters)
{
    for (;;)
    {
        int percentage = powerManager.getBatteryPercentage();
        bleManager.sendBatteryLevel(percentage);

        // Check for charging
        if (digitalRead(chargingPin) == LOW)
        {
            Serial.println("Charging detected, going to sleep");
            powerManager.goToDeepSleep(true);
        }

        if (percentage < 5)
        {
            Serial.println("Battery low, going to sleep");
            powerManager.goToDeepSleep();
        }

        // Check for auto-sleep after timeout when not connected
        if (!bleManager.isDeviceConnected() && bleManager.getLastDisconnectTime() > 0)
        {
            unsigned long disconnectedTime = millis() - bleManager.getLastDisconnectTime();
            if (disconnectedTime >= AUTO_SLEEP_TIME)
            {
                Serial.println("No connection for 5 minutes, going to sleep");
                powerManager.goToDeepSleep();
            }
        }

        vTaskDelay(30000 / portTICK_PERIOD_MS); // 30 seconds delay
    }
}