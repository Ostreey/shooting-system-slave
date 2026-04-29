#ifndef BLE_MANAGER_H
#define BLE_MANAGER_H

#include <Arduino.h>
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>
#include <BLEAdvertising.h>
#include "Config.h"

class LEDController;
class PowerManager;
class OTAManager;
class PiezoSensor;

enum class BLECommand
{
    UNKNOWN,
    START,
    SLEEP,
    GAME1,
    SET_BRIGHTNESS,
    SET_RGB_COLOR,
    SET_COLOR,
    SET_RED,
    SET_GREEN,
    SET_BLUE,
    SET_AUTO_OFF,
    BLINK_COLOR,
    CALIB_GET,
    CALIB_SET,
    CALIB_RESET,
    PEAK_START,
    PEAK_STOP,
#ifdef BOARD_ESP32S3
    BLE_STATUS,
    BLE_PHY_INFO,
#endif
};

struct BLECommandData
{
    BLECommand command;
    int value;
};

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

    bool deviceConnected;
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

    // Hit detection
    void onPiezoHit();

    // Command parsing
    static BLECommandData parseCommand(const std::string &value);

    // Task functions
    static void initialDeviceInfoTask(void *pvParameters);
    static void ledStatusTask(void *pvParameters);
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
    };

private:
    TaskHandle_t ledStatusTaskHandle;

    // Helper methods for command processing
    void handleRGBCommand(const std::string &value);
    void handleColorCommand(const std::string &value);
    void handleBlinkColorCommand(const std::string &value);
    void handleStartColorCommand(const std::string &value);
    void handleCalibSetCommand(const std::string &value);
    void handleCalibResetCommand();
    void handlePeakStartCommand();
    void handlePeakStopCommand();

    // Common RGB parsing helper
    bool parseRgbValues(const std::string &rgbStr, int &red, int &green, int &blue);
};

#endif // BLE_MANAGER_H
