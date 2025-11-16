#ifndef BLE_MANAGER_H
#define BLE_MANAGER_H

#include <cstdint>
#include <functional>
#include <string>
#include <esp_gap_ble_api.h>
#include <esp_gatts_api.h>
#include <esp_bt.h>
#include <esp_bt_main.h>
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
    bool deviceConnected;
    bool isInitialized;
    uint32_t hitTime;
    esp_gatt_if_t gattsIf;
    uint16_t connId;
    TaskHandle_t ledStatusTaskHandle;
    uint16_t piezoHandles[10];
    uint16_t otaHandles[10];
    uint8_t advConfigDone;
    static BLEManager *instance;

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
    void registerBLEEventCallback();

private:
    void handleRGBCommand(const std::string &value);
    void handleColorCommand(const std::string &value);
    void handleBlinkColorCommand(const std::string &value);
    void handleStartColorCommand(const std::string &value);
    bool parseRgbValues(const std::string &rgbStr, int &red, int &green, int &blue);
    void handleWrite(uint16_t handle, const uint8_t *value, size_t len);
    void sendNotification(uint16_t handle, const uint8_t *value, uint16_t length);
    void startAdvertising();
    void initGap();
    void initGatt();
    void sendOtaStatus(const std::string &status);
    void configurePhy();
    static void gapEventHandler(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t *param);
    static void gattsEventHandler(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param);
};

#endif
