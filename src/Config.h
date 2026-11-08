#ifndef CONFIG_H
#define CONFIG_H

// Firmware version — bump FIRMWARE_VERSION_BASE only; board suffix is appended automatically.
// FIRMWARE_VERSION_BASE must stay a bare X.Y.Z: CI pulls the release tag out of it with
// [0-9]+\.[0-9]+\.[0-9]+, so every suffix belongs on FIRMWARE_VERSION, never on the base.
#define FIRMWARE_VERSION_BASE "1.0.9"

// The no-battery test build must be identifiable over BLE: a build that reports the same version
// as a release build is how this project lost track of what was running on a target once already.
// The marker goes BEFORE the first dash, and it has to. The released app reads the board with
// substringAfter("-") and matches it against a list of exactly "esp32"/"esp32s3"
// (FirmwareRepositoryImpl.extractBoardFromVersion); a marker after the board member would yield
// "esp32s3-nobatt", fall through to the "esp32" default and quietly pull the firmware for the
// wrong board. Before the dash it lands in the version member instead, where the app shows it
// (MainViewModel displays substringBefore("-")) and its numeric comparison is null-safe.
// Empty for every release build, so FIRMWARE_VERSION is byte-for-byte what it was.
#ifdef MIGRATION_TEST_NO_BATTERY
#define FIRMWARE_VERSION_MARKER "nobatt"
#else
#define FIRMWARE_VERSION_MARKER ""
#endif

#ifdef BOARD_ESP32S3
#define FIRMWARE_VERSION FIRMWARE_VERSION_BASE FIRMWARE_VERSION_MARKER "-esp32s3"
#else
#define FIRMWARE_VERSION FIRMWARE_VERSION_BASE FIRMWARE_VERSION_MARKER "-esp32"
#endif

// BLE Configuration
#define BLE_SERVER_NAME "SHOOTING TARGET"
#define SERVICE_UUID "91bad492-b950-4226-aa2b-4ede9fa42fff"
#define CHARACTERISTIC "cba1d466-344c-4be3-ab3f-189f80dd75ff"
#define BATTERY_CHARACTERISTIC "cba1d466-344c-4be3-ab3f-189f80dd76ff"
#define FIRMWARE_VERSION_CHARACTERISTIC "cba1d466-344c-4be3-ab3f-189f80dd77ff"
#define CALIBRATION_CHARACTERISTIC "cba1d466-344c-4be3-ab3f-189f80dd79ff"

#ifdef BOARD_ESP32S3
#define PHY_INFO_CHARACTERISTIC "cba1d466-344c-4be3-ab3f-189f80dd78ff"
#define BLE_LONG_RANGE_ENABLED true
#define BLE_CODED_PHY_PREFERRED true
#else
#define BLE_LONG_RANGE_ENABLED false
#define BLE_CODED_PHY_PREFERRED false
#endif

// OTA Service UUIDs
#define OTA_SERVICE_UUID "12345678-1234-5678-1234-56789abc0000"
#define OTA_COMMAND_CHAR_UUID "12345678-1234-5678-1234-56789abc0001"
#define OTA_DATA_CHAR_UUID "12345678-1234-5678-1234-56789abc0002"
#define OTA_STATUS_CHAR_UUID "12345678-1234-5678-1234-56789abc0003"

// ADC Configuration
#ifdef BOARD_ESP32S3
// ESP32-S3 ADC Pins: ADC1 (GPIO1-10), ADC2 (GPIO11-20, shared with WiFi)
// Using ADC1 pins for better compatibility when WiFi is active
#endif
#define DEFAULT_VREF 1100         // Default reference voltage in mV
#define ADC_SAMPLES 64            // Number of samples for averaging
#define ADC_ATTEN ADC_ATTEN_DB_11 // 11dB attenuation for 0-3.3V range

// ESP32 ADC systematic underread compensation (mV at battery level, after voltage divider ×1.5)
// Tune by: measure actual battery voltage with multimeter, compare to Serial log "Battery: X.XXV"
// Increase if displayed % is too low, decrease if too high. Typical range: 100–200 mV.
#define ADC_BATTERY_OFFSET_MV 100

// PWM Configuration
#define PWM_FREQUENCY 5000
#define PWM_RESOLUTION 8 // 8-bit resolution (0-255)
#define PWM_CHANNEL_RED 0
#define PWM_CHANNEL_GREEN 1
#define PWM_CHANNEL_BLUE 2

// GPIO Pin Definitions
#ifdef BOARD_ESP32S3
#define LED_BLUE_PIN 14
#define LED_GREEN_PIN 2
#define LED_RED_PIN 1
#define WAKE_UP_BUTTON_PIN 13
#define BATTERY_PIN 3 // ADC1 Channel 0 (ESP32-S3 compatible)
#define LED_POWER_ENABLE_PIN 21
#define CHARGING_PIN 9
#define PIEZO_SENSOR_PIN 8 // ADC1 Channel 1 (ESP32-S3 compatible)
#else                      // BOARD_ESP32
#define LED_BLUE_PIN 17
#define LED_GREEN_PIN 18
#define LED_RED_PIN 19
#define WAKE_UP_BUTTON_PIN 15
#define BATTERY_PIN 33
#define LED_POWER_ENABLE_PIN 4
#define CHARGING_PIN 13
#define PIEZO_SENSOR_PIN 32
#endif

// Timing Constants
#define TURN_OFF_TIME 2000              // 2 seconds for long press
#define WAKE_UP_HOLD_TIME 2000          // 2 seconds required to confirm wake up
#define AUTO_SLEEP_TIME (5 * 60 * 1000) // 5 minutes in milliseconds

// OTA Configuration
#define OTA_CHUNK_SIZE 256 // Conservative chunk size to prevent watchdog timeout

// Power supply type: true = buck-boost (new boards), false = HT7833 LDO (old boards)
#define BUCKBOOST true

// Battery decision thresholds -- INTERNAL decision path only (BatteryGuard).
// The level reported over BLE keeps the discharge LUT and ADC_BATTERY_OFFSET_MV untouched:
// the released Android app blocks OTA at batteryLevel <= 40, so a lower reported percentage
// would stop targets in the field from accepting this very fix.
// Both values are compared against the same mV estimate as the reported level
// (PowerManager::readBatteryMv).
//
// PROPOSED VALUES, TO BE CONFIRMED by the cold-cell measurement (scenario S8 of the QA
// protocol). Change them here only; nothing else carries a battery voltage number.
#if !BUCKBOOST
#error "These thresholds were picked for the buck-boost supply. An HT7833 LDO board browns out on its own near its dropout, so pick a threshold for it deliberately and delete this guard - do not inherit the buck-boost number."
#endif
#define BATTERY_FLAT_THRESHOLD_MV 3500

// A voltage a flat cell cannot reach on its own -- the only measurement that lifts the latch.
#define BATTERY_LATCH_CLEAR_MV 3900

// A single sample never decides: a reading below the flat threshold is confirmed by a second
// one. At boot after this pause, at run time with the LED output switched off, because the LED
// rail dominates the load (12V measured, no current-limiting resistors on the board, so the
// module current is unknown) and a loaded dip must not be read as an empty cell.
#define BATTERY_FLAT_CONFIRM_DELAY_MS 100
#define BATTERY_UNLOADED_SETTLE_MS 200

// Run-time floor for readings taken WITH the LED output on. The game logic owns the LEDs, so the
// battery task never switches them off to measure; a target the app leaves lit while the phone
// stays connected (no idle auto-sleep) would otherwise never get an unloaded reading, the
// buck-boost would hold the 3.3V rail down to ~1.8V input, the TPS3823-33 would never reset, and
// the cell would run into the 2.4V DW01A cutoff - the failure this branch fixes.
// Set well below any sag a healthy cell shows under the LED load, so a lit target in a match is
// not put to sleep: a healthy cell at >= 3.6V resting would have to drop by ~400 mV.
// Needs BATTERY_LOADED_FLOOR_SAMPLES consecutive readings (30 s apart) - one sample never decides.
// PROPOSED VALUE, TO BE CONFIRMED by the LED current measurement (S16) and the live discharge test.
#define BATTERY_LOADED_FLOOR_MV 3200
#define BATTERY_LOADED_FLOOR_SAMPLES 2

// Button hold that lifts the latch in the hysteresis band (BATTERY_FLAT_THRESHOLD_MV up to
// BATTERY_LATCH_CLEAR_MV). Below BATTERY_FLAT_THRESHOLD_MV the button is refused and the target
// does not start at all; only the charger brings it back (owner decision 2026-11-07, variant A).
// There is deliberately no separate override floor: it is the flat threshold. Must stay clearly
// above WAKE_UP_HOLD_TIME, otherwise an ordinary wake-up would clear the latch. 10 s, not 5 s
// (owner decision after the live test 2026-11-07): an accidental hold must not clear the latch.
// The hold runs in setup(), before BLE: the Arduino loop task is not subscribed to the task
// watchdog and the 20 ms delays yield, so the length is not bounded by CONFIG_ESP_TASK_WDT_TIMEOUT_S.
#define BATTERY_LATCH_BUTTON_HOLD_MS 10000
#define BATTERY_BUTTON_DEBOUNCE_MS 100

// Voltage-based charger detection. BATT+ is both the cell and the converter input (no power
// path on the board), so a charger shows up only as a rising cell voltage. Samples taken with
// the LED output on are discarded, and the rise has to be sustained over consecutive samples,
// because a LED being switched off releases the largest load in the whole duty cycle.
#define BATTERY_CHARGER_RISE_STEP_MV 30
#define BATTERY_CHARGER_RISE_MV 100

#endif // CONFIG_H
