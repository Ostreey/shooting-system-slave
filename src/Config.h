#ifndef CONFIG_H
#define CONFIG_H

// Firmware version
#define FIRMWARE_VERSION "1.0.3"

// BLE Configuration
#define BLE_SERVER_NAME "SHOOTING TARGET"
#define SERVICE_UUID "91bad492-b950-4226-aa2b-4ede9fa42fff"
#define CHARACTERISTIC "cba1d466-344c-4be3-ab3f-189f80dd75ff"
#define BATTERY_CHARACTERISTIC "cba1d466-344c-4be3-ab3f-189f80dd76ff"
#define FIRMWARE_VERSION_CHARACTERISTIC "cba1d466-344c-4be3-ab3f-189f80dd77ff"

// OTA Service UUIDs
#define OTA_SERVICE_UUID "12345678-1234-5678-1234-56789abc0000"
#define OTA_COMMAND_CHAR_UUID "12345678-1234-5678-1234-56789abc0001"
#define OTA_DATA_CHAR_UUID "12345678-1234-5678-1234-56789abc0002"
#define OTA_STATUS_CHAR_UUID "12345678-1234-5678-1234-56789abc0003"

// ADC Configuration
#define DEFAULT_VREF 1100         // Default reference voltage in mV
#define ADC_SAMPLES 64            // Number of samples for averaging
#define ADC_ATTEN ADC_ATTEN_DB_11 // 11dB attenuation for 0-3.3V range

// PWM Configuration
#define PWM_FREQUENCY 5000
#define PWM_RESOLUTION 8 // 8-bit resolution (0-255)
#define PWM_CHANNEL_RED 0
#define PWM_CHANNEL_GREEN 1
#define PWM_CHANNEL_BLUE 2

// GPIO Pin Definitions
#define LED_BLUE_PIN 17
#define LED_GREEN_PIN 18
#define LED_RED_PIN 19
#define WAKE_UP_BUTTON_PIN 15
#define BATTERY_PIN 33
#define LED_POWER_ENABLE_PIN 4
#define CHARGING_PIN 13
#define PIEZO_SENSOR_PIN 32

// Timing Constants
#define TURN_OFF_TIME 2000              // 2 seconds for long press
#define WAKE_UP_HOLD_TIME 2000          // 2 seconds required to confirm wake up
#define AUTO_SLEEP_TIME (5 * 60 * 1000) // 5 minutes in milliseconds

// OTA Configuration
#define OTA_CHUNK_SIZE 256 // Conservative chunk size to prevent watchdog timeout

// Battery thresholds
#define BATTERY_LOW_THRESHOLD 5
#define BATTERY_MIN_VOLTAGE 340 // 3.4V * 100
#define BATTERY_MAX_VOLTAGE 420 // 4.2V * 100

#endif // CONFIG_H
