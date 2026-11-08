#ifndef BLE_MANAGER_H
#define BLE_MANAGER_H

#include <Arduino.h>
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>
#include <BLEAdvertising.h>
#include "Config.h"
#include "BleCommandParser.h"

class LEDController;
class PowerManager;
class OTAManager;
class PiezoSensor;

class BLEManager
{
private:
    LEDController *ledController;
    PowerManager *powerManager;
    OTAManager *otaManager;
    PiezoSensor *piezoSensor;

    BLEServer *pServer;
    BLEService *piezoService;
    BLECharacteristic *piezoCharacteristic;
    BLECharacteristic *batteryCharacteristic;
    BLECharacteristic *firmwareVersionCharacteristic;
    BLECharacteristic *calibrationCharacteristic;

#ifdef BOARD_ESP32S3
    BLEMultiAdvertising *pMultiAdvertising;
#endif

    volatile bool deviceConnected;
    bool isInitialized;
    unsigned long hitTime;

public:
    BLEManager(LEDController *leds, PowerManager *power, OTAManager *ota, PiezoSensor *sensor);

    // Initialization
    bool begin();

    // Connection state
    bool isConnected() const { return deviceConnected; }

    // Data sending
    void sendPiezoValue(int piezoValue);
    void sendBatteryLevel(int percentage);
    void sendFirmwareVersion();
    void sendCalibrationConfig();
    void sendPeakValue(uint16_t peakValue);
    void sendLedModuleState();

    // Hit detection
    void onPiezoHit();

    // Command parsing
    static BLECommandData parseCommand(const std::string &value);

    // Runs one command from the piezo characteristic. Called only from the command task.
    void executeCommand(const std::string &value);

    // Task functions
    static void initialDeviceInfoTask(void *pvParameters);
    static void ledStatusTask(void *pvParameters);
    static void commandTask(void *pvParameters);
    void startLedStatusTask();

#ifdef BOARD_ESP32S3
    // BLE monitoring and diagnostics
    void logBLEConnectionInfo();
    void logBLEPHYInfo();
    void registerBLEEventCallback();
#endif

    // Callback classes
    class WriteCallbacks : public BLECharacteristicCallbacks
    {
    private:
        BLEManager *bleManager;

    public:
        WriteCallbacks(BLEManager *manager) : bleManager(manager) {}
        void onWrite(BLECharacteristic *pCharacteristic) override;
    };

    class ServerCallbacks : public BLEServerCallbacks
    {
    private:
        BLEManager *bleManager;

    public:
        ServerCallbacks(BLEManager *manager) : bleManager(manager) {}
        void onConnect(BLEServer *pServer) override;
        void onDisconnect(BLEServer *pServer) override;
        void onDisconnect(BLEServer *pServer, esp_ble_gatts_cb_param_t *param) override;
    };

private:
    TaskHandle_t ledStatusTaskHandle;

    // Commands written to the piezo characteristic are only copied into this queue by the write
    // callback and run in their own task. The callback runs in the Bluedroid BTC task, whose
    // 3072-byte stack is fixed in the precompiled Arduino SDK: running ledswap:get there overflowed
    // it (2026-11-07, "Stack canary watchpoint triggered (BTC_TASK)", reset mid-session). The BTC
    // task also sends the write response only after the callback returns, so a 1.2 s blinkrgb in
    // the callback made the phone drop the next write. One producer, one consumer: commands run
    // strictly in the order they were written.
    static constexpr size_t COMMAND_MAX_LEN = 64;
    static constexpr UBaseType_t COMMAND_QUEUE_DEPTH = 16;
    // Deepest paths run here now instead of in BTC: NVS writes (calib:set, ledswap) and the whole
    // sleep sequence (sleep command). Twice the BTC stack that overflowed; the high-water mark is
    // logged when it gets close, so a regression shows up in Serial, not as a reset.
    static constexpr uint32_t COMMAND_TASK_STACK = 6144;
    static constexpr UBaseType_t COMMAND_STACK_WARN_BYTES = 1024;
    struct QueuedCommand
    {
        uint8_t length;
        char data[COMMAND_MAX_LEN];
    };
    QueueHandle_t commandQueue;
    TaskHandle_t commandTaskHandle;
    void enqueueCommand(const std::string &value);

    // Helper methods for command processing
    void handleRGBCommand(const std::string &value);
    void handleColorCommand(const std::string &value);
    void handleBlinkColorCommand(const std::string &value);
    void handleStartColorCommand(const std::string &value);
    void handleCalibSetCommand(const std::string &value);
    void handleCalibResetCommand();
    void handlePeakStartCommand();
    void handlePeakStopCommand();
    void handleLedSwapSetCommand(bool swapped);
    void handleLedTestCommand(bool boardRedPin);

    // Common RGB parsing helper
    bool parseRgbValues(const std::string &rgbStr, int &red, int &green, int &blue);
};

#endif // BLE_MANAGER_H
