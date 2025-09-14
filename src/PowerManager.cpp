#include "PowerManager.h"
#include "LEDController.h"

PowerManager::PowerManager(LEDController* leds) 
    : ledController(leds), batteryTaskHandle(nullptr), lastDisconnectTime(0), 
      isGoingToSleep(false), justWokenUp(false), isInitialized(false), 
      batteryCallback(nullptr)
{
}

bool PowerManager::begin()
{
    if (isInitialized) {
        return true;
    }
    
    // Initialize GPIO pins
    pinMode(WAKE_UP_BUTTON_PIN, INPUT_PULLUP);
    gpio_set_pull_mode(GPIO_NUM_15, GPIO_PULLUP_ONLY);
    gpio_set_direction(GPIO_NUM_15, GPIO_MODE_INPUT);
    pinMode(BATTERY_PIN, INPUT);
    pinMode(CHARGING_PIN, INPUT_PULLUP);
    
    // ADC calibration
    esp_adc_cal_characterize(ADC_UNIT_1, ADC_ATTEN, ADC_WIDTH_BIT_12, DEFAULT_VREF, &adcChars);
    
    // Initialize disconnect timer to start auto-sleep countdown from boot
    lastDisconnectTime = millis();
    
    isInitialized = true;
    return true;
}

int PowerManager::getBatteryPercentage()
{
    if (!isInitialized) return 100;
    
    // Read multiple samples and average them
    uint32_t adcReading = 0;
    for (int i = 0; i < ADC_SAMPLES; i++) {
        adcReading += analogRead(BATTERY_PIN);
    }
    adcReading /= ADC_SAMPLES;
    
    // Convert to voltage using calibration
    uint32_t voltageMv = esp_adc_cal_raw_to_voltage(adcReading, &adcChars);
    float voltage = voltageMv / 1000.0; // Convert to volts
    
    // Convert to actual battery voltage (multiply by 1.5 due to voltage divider)
    float batteryVoltage = voltage * 1.5;
    
    // Convert to percentage (assuming 4.2V is 100% and 3.6V is 0%)
    int percentage = map(batteryVoltage * 100, BATTERY_MIN_VOLTAGE, BATTERY_MAX_VOLTAGE, 0, 100);
    percentage = constrain(percentage, 0, 100);
    
    return percentage;
}

void PowerManager::setBatteryCallback(void (*callback)(int))
{
    batteryCallback = callback;
}

bool PowerManager::isCharging()
{
    return digitalRead(CHARGING_PIN) == LOW;
}

void PowerManager::goToDeepSleep(bool skipLedBlink)
{
    isGoingToSleep = true;
    
    // DISABLE WATCHDOG FIRST - before any delays or LED operations
    esp_task_wdt_deinit();
    
    if (!skipLedBlink && ledController) {
        // Use LED controller for blinking
        ledController->blinkLeds(3, 200, 200);
        
        // Small delay to show the device is turning off
        delay(500);
    }
    
    Serial.println("Going to deep sleep...");
    
    // Configure GPIO15 properly before sleep
    // Set as input with internal pull-up to prevent floating state
    gpio_set_direction(GPIO_NUM_15, GPIO_MODE_INPUT);
    gpio_set_pull_mode(GPIO_NUM_15, GPIO_PULLUP_ONLY);
    
    // Add small delay to stabilize GPIO state
    delay(100);
    
    esp_err_t ext0Result = esp_sleep_enable_ext0_wakeup(GPIO_NUM_15, 0);
    if (ext0Result != ESP_OK) {
        Serial.printf("ext0 wake-up failed, trying ext1: %s\n", esp_err_to_name(ext0Result));
        esp_sleep_enable_ext1_wakeup(((1ULL << GPIO_NUM_15)), ESP_EXT1_WAKEUP_ANY_HIGH);
    }
    
    Serial.println("Deep sleep configuration complete");
    Serial.flush(); // Ensure all serial data is sent before sleep
    
    // Final delay to ensure all operations are complete
    delay(50);
    
    esp_deep_sleep_start();
}

bool PowerManager::validateWakeUp()
{
    Serial.println("Validating wake-up - button must be held for 2 seconds...");
    
    // Ensure GPIO15 is properly configured for reading
    gpio_set_direction(GPIO_NUM_15, GPIO_MODE_INPUT);
    gpio_set_pull_mode(GPIO_NUM_15, GPIO_PULLUP_ONLY);
    
    // Small delay to stabilize GPIO state
    delay(100);
    
    // Check if button is still pressed when we start validation
    if (digitalRead(WAKE_UP_BUTTON_PIN) != LOW) {
        Serial.println("Button not pressed during validation - going back to sleep");
        return false;
    }
    
    unsigned long startTime = millis();
    
    while ((millis() - startTime) < WAKE_UP_HOLD_TIME) {
        // Check if button was released during the hold period
        if (digitalRead(WAKE_UP_BUTTON_PIN) != LOW) {
            Serial.println("Button released before 2 seconds - going back to sleep");
            return false;
        }
        
        delay(50); // Small delay to prevent excessive polling
    }
    
    // If we reach here, button was held for full 2 seconds
    Serial.println("Wake-up validated - button held for 2 seconds");
    
    // Set flag to indicate we just woke up successfully
    // This will prevent turn-off detection for the current button press
    justWokenUp = true;
    
    return true;
}

void PowerManager::setConnected(bool connected)
{
    if (connected) {
        lastDisconnectTime = 0; // Reset disconnect timer when connected
    } else {
        lastDisconnectTime = millis(); // Record disconnection time
    }
}

bool PowerManager::shouldAutoSleep()
{
    return (lastDisconnectTime > 0 && 
            (millis() - lastDisconnectTime) >= AUTO_SLEEP_TIME);
}

bool PowerManager::isButtonPressed()
{
    return digitalRead(WAKE_UP_BUTTON_PIN) == LOW;
}

void PowerManager::startBatteryMonitoringTask()
{
    if (batteryTaskHandle == nullptr) {
        xTaskCreatePinnedToCore(
            batteryMonitorTaskFunction,
            "BatteryMonitor",
            2048,
            this, // Pass this instance as parameter
            2,
            &batteryTaskHandle,
            APP_CPU_NUM
        );
    }
}

void PowerManager::stopBatteryMonitoringTask()
{
    if (batteryTaskHandle != nullptr) {
        vTaskDelete(batteryTaskHandle);
        batteryTaskHandle = nullptr;
    }
}

// Static task function
void PowerManager::batteryMonitorTaskFunction(void* pvParameters)
{
    PowerManager* powerManager = static_cast<PowerManager*>(pvParameters);
    
    for (;;) {
        int percentage = powerManager->getBatteryPercentage();
        
        // Send battery level through callback if set
        if (powerManager->batteryCallback) {
            powerManager->batteryCallback(percentage);
        }
        
        // Check for charging - if charging pin goes low, charger is working
        if (powerManager->isCharging()) {
            Serial.println("Charging detected, going to sleep");
            powerManager->goToDeepSleep(true);
        }
        
        if (percentage < BATTERY_LOW_THRESHOLD) {
            Serial.println("Battery low, going to sleep");
            powerManager->goToDeepSleep();
        }
        
        // Check for auto-sleep after timeout when not connected
        if (powerManager->shouldAutoSleep()) {
            Serial.println("No connection for 5 minutes, going to sleep");
            powerManager->goToDeepSleep();
        }
        
        vTaskDelay(30000 / portTICK_PERIOD_MS); // 30 seconds delay
    }
}