#include <Arduino.h>
#include "BleManager.h"
#include "OtaManager.h"
#include "PowerManager.h"
#include "LedController.h"
#include "HardwareInterface.h"

// BLE server name
#define bleServerName "SHOOTING TARGET"

// Firmware version - simple const
const char *FIRMWARE_VERSION = "1.0.0";

// Global module instances (defined in their respective .cpp files)
extern BleManager bleManager;
extern OtaManager otaManager;
extern PowerManager powerManager;
extern LedController ledController;
extern HardwareInterface hardwareInterface;

void setup()
{
    delay(100);
    Serial.begin(115200);
    Serial.println("Start");

    // Check wake-up cause FIRST, before any other initialization
    if (esp_sleep_get_wakeup_cause() == ESP_SLEEP_WAKEUP_EXT0)
    {
        Serial.println("Woken up from deep sleep");

        // Validate wake-up by checking if button is held for 2 seconds
        if (!powerManager.validateWakeUp())
        {
            Serial.println("Wake-up validation failed - returning to deep sleep");
            powerManager.goToDeepSleep(true); // Skip LED blink since nothing is initialized yet
            return;
        }

        Serial.println("Wake-up validation successful - continuing normal operation");
        hardwareInterface.setJustWokenUp(true);
    }
    else
    {
        delay(2000);
    }

    // Initialize all modules
    ledController.init();
    powerManager.init();
    hardwareInterface.init();

    // Initialize BLE
    bleManager.init(bleServerName);
    Serial.print("BLE Device Name: ");
    Serial.println(bleServerName);

    // Initialize OTA after BLE server is created
    otaManager.init(bleManager.getServer());

    // Start BLE advertising
    bleManager.startAdvertising();
    Serial.println("Waiting for a client connection to notify...");

    // Start background tasks
    powerManager.startBatteryMonitorTask();
    hardwareInterface.startLedStatusTask();

    Serial.println("System initialization complete");
}

void loop()
{
    // Check for pending OTA restart
    otaManager.executeRestart();

    // Handle button input
    hardwareInterface.handleButton();

    // Small delay to prevent excessive polling
    delay(10);
}