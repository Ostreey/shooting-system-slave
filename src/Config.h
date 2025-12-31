#ifndef CONFIG_H
#define CONFIG_H

#include "driver/adc.h"

// Firmware version
#define FIRMWARE_VERSION "1.0.5"

// BLE Configuration
#define BLE_SERVER_NAME "SHOOTING TARGET"
#define SERVICE_UUID "91bad492-b950-4226-aa2b-4ede9fa42fff"
#define CHARACTERISTIC "cba1d466-344c-4be3-ab3f-189f80dd75ff"
#define BATTERY_CHARACTERISTIC "cba1d466-344c-4be3-ab3f-189f80dd76ff"
#define FIRMWARE_VERSION_CHARACTERISTIC "cba1d466-344c-4be3-ab3f-189f80dd77ff"
#define PHY_INFO_CHARACTERISTIC "cba1d466-344c-4be3-ab3f-189f80dd78ff"

// BLE Long Range Configuration (ESP32-S3)
// Set to false to disable extended advertising and use standard advertising (better compatibility)
#define BLE_LONG_RANGE_ENABLED true
#define BLE_CODED_PHY_PREFERRED true

// OTA Service UUIDs
#define OTA_SERVICE_UUID "12345678-1234-5678-1234-56789abc0000"
#define OTA_COMMAND_CHAR_UUID "12345678-1234-5678-1234-56789abc0001"
#define OTA_DATA_CHAR_UUID "12345678-1234-5678-1234-56789abc0002"
#define OTA_STATUS_CHAR_UUID "12345678-1234-5678-1234-56789abc0003"

// ADC Configuration (ESP32-S3)
// ESP32-S3 ADC Pins: ADC1 (GPIO1-10), ADC2 (GPIO11-20, shared with WiFi)
// Using ADC1 pins for better compatibility when WiFi is active
#define DEFAULT_VREF 1100 // Default reference voltage in mV
#define ADC_SAMPLES 64
#define ADC_ATTEN ADC_ATTEN_DB_11
#define BATTERY_ADC_CHANNEL ADC1_CHANNEL_2
#define PIEZO_ADC_CHANNEL ADC1_CHANNEL_7

// PWM Configuration
#define PWM_FREQUENCY 5000
#define PWM_RESOLUTION 8 // 8-bit resolution (0-255)
#define PWM_CHANNEL_RED 0
#define PWM_CHANNEL_GREEN 1
#define PWM_CHANNEL_BLUE 2

// GPIO Pin Definitions (ESP32-S3 compatible)
// Note: These pins are compatible with ESP32-S3 and don't conflict with strapping pins
#define LED_BLUE_PIN 14
#define LED_GREEN_PIN 2
#define LED_RED_PIN 1
#define WAKE_UP_BUTTON_PIN 13
#define BATTERY_PIN 3 // ADC1 Channel 0 (ESP32-S3 compatible)
#define LED_POWER_ENABLE_PIN 21
#define CHARGING_PIN 9
#define PIEZO_SENSOR_PIN 8 // ADC1 Channel 1 (ESP32-S3 compatible)

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
