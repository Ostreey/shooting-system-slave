#ifndef BLE_MANAGER_H
#define BLE_MANAGER_H

#include <array>
#include <cstdint>
#include <string>

#include "Config.h"
#include "IdfCompat.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

extern "C"
{
#include "esp_bt.h"
#include "esp_bt_main.h"
#include "esp_gap_ble_api.h"
#include "esp_gatt_common_api.h"
#include "esp_gatts_api.h"
}

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
public:
    BLEManager(LEDController *leds, PowerManager *power, OTAManager *ota);

    bool begin();

    bool isConnected() const { return deviceConnected; }
    void sendPiezoValue(int piezoValue);
    void sendBatteryLevel(int percentage);
    void sendFirmwareVersion();
    void onPiezoHit();

    static BLECommandData parseCommand(const std::string &value);

    static void initialDeviceInfoTask(void *pvParameters);
    static void ledStatusTask(void *pvParameters);
    void startLedStatusTask();

    void logBLEConnectionInfo();
    void logBLEPHYInfo();

private:
    struct MainServiceHandles
    {
        uint16_t serviceHandle = 0;
        uint16_t piezoCharHandle = 0;
        uint16_t piezoCccHandle = 0;
        uint16_t batteryCharHandle = 0;
        uint16_t batteryCccHandle = 0;
        uint16_t firmwareCharHandle = 0;
        uint16_t firmwareCccHandle = 0;
    };

    struct OtaServiceHandles
    {
        uint16_t serviceHandle = 0;
        uint16_t commandCharHandle = 0;
        uint16_t dataCharHandle = 0;
        uint16_t statusCharHandle = 0;
        uint16_t statusCccHandle = 0;
    };

    LEDController *ledController;
    PowerManager *powerManager;
    OTAManager *otaManager;

    bool deviceConnected;
    bool isInitialized;
    uint64_t hitTimeMs;

    esp_gatt_if_t mainGattIf;
    esp_gatt_if_t otaGattIf;
    uint16_t connId;

    MainServiceHandles mainHandles;
    OtaServiceHandles otaHandles;

    bool piezoNotifyEnabled;
    bool batteryNotifyEnabled;
    bool firmwareNotifyEnabled;
    bool otaStatusNotifyEnabled;

    TaskHandle_t ledStatusTaskHandle;

    std::string lastBatteryValue;
    std::string lastFirmwareValue;
    std::string lastOtaStatus;

    static BLEManager *instance;

    static constexpr uint16_t kMainAppId = 0x55;
    static constexpr uint16_t kOtaAppId = 0x56;

    enum class DescriptorOwner
    {
        None,
        Piezo,
        Battery,
        Firmware,
        OtaStatus
    };

    DescriptorOwner pendingDescriptorOwner;

    bool advDataConfigured;
    std::array<uint8_t, 32> serviceUuidBuffer;

    void handleRGBCommand(const std::string &value);
    void handleColorCommand(const std::string &value);
    void handleBlinkColorCommand(const std::string &value);
    void handleStartColorCommand(const std::string &value);
    bool parseRgbValues(const std::string &rgbStr, int &red, int &green, int &blue);

    void setupBluetoothController();
    void configureAdvertising();
    void startAdvertising();

    static void gapEventHandler(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t *param);
    static void gattsEventHandler(esp_gatts_cb_event_t event,
                                  esp_gatt_if_t gatts_if,
                                  esp_ble_gatts_cb_param_t *param);
    void handleGattEvent(esp_gatts_cb_event_t event,
                         esp_gatt_if_t gatts_if,
                         esp_ble_gatts_cb_param_t *param);
    void handleGapEvent(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t *param);

    void onRegistration(esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param);
    void onCreateService(esp_ble_gatts_cb_param_t *param);
    void onAddCharacteristic(esp_ble_gatts_cb_param_t *param);
    void onAddDescriptor(esp_ble_gatts_cb_param_t *param);
    void onConnectEvent(esp_ble_gatts_cb_param_t *param);
    void onDisconnectEvent();
    void onWriteEvent(esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param);
    void onReadEvent(esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param);

    void processCommand(const std::string &value);
    void sendOtaStatus(const std::string &status);
    void updateNotifyState(uint16_t handle, bool enabled);

    void scheduleInitialInfo();
};

#endif // BLE_MANAGER_H
