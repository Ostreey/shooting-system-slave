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
    BLINK_COLOR,
    BLE_STATUS,
    BLE_PHY_INFO,
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

    BLEServer *pServer;
    BLEService *piezoService;
    BLECharacteristic *piezoCharacteristic;
    BLECharacteristic *batteryCharacteristic;
    BLECharacteristic *firmwareVersionCharacteristic;

#ifdef CONFIG_BT_BLE_50_FEATURES_SUPPORTED
    BLEMultiAdvertising *pMultiAdvertising;
#endif

    bool deviceConnected;
    bool isInitialized;
    unsigned long hitTime;

public:
    BLEManager(LEDController *leds, PowerManager *power, OTAManager *ota);

    // Initialization
    bool begin();

    // Connection state
    bool isConnected() const { return deviceConnected; }

    // Data sending
    void sendPiezoValue(int piezoValue);
    void sendBatteryLevel(int percentage);
    void sendFirmwareVersion();

    // Hit detection
    void onPiezoHit();

    // Command parsing
    static BLECommandData parseCommand(const std::string &value);

    // Task functions
    static void initialDeviceInfoTask(void *pvParameters);
    static void ledStatusTask(void *pvParameters);
    void startLedStatusTask();

    // BLE monitoring and diagnostics
    void logBLEConnectionInfo();
    void logBLEPHYInfo();
    void registerBLEEventCallback();

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

    // Common RGB parsing helper
    bool parseRgbValues(const std::string &rgbStr, int &red, int &green, int &blue);
};

#endif // BLE_MANAGER_H
