#include "BLEManager.h"

#include <algorithm>
#include <array>
#include <cstring>
#include <esp_gap_ble_api.h>
#include <esp_log.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#include "LEDController.h"
#include "PowerManager.h"
#include "OTAManager.h"
#include "TimeUtils.h"

namespace
{
const char *TAG = "BLEManager";
constexpr uint8_t ADV_CONFIG_FLAG = 1 << 0;

const uint16_t primaryServiceUuid = ESP_GATT_UUID_PRI_SERVICE;
const uint16_t charDeclUuid = ESP_GATT_UUID_CHAR_DECLARE;
const uint16_t clientConfigUuid = ESP_GATT_UUID_CHAR_CLIENT_CONFIG;

const uint8_t piezoServiceUuid[16] = {0x92, 0xd4, 0xba, 0x91, 0x50, 0xb9, 0x26, 0x42, 0xaa, 0x2b, 0x4e, 0xde, 0x9f, 0xa4, 0x2f, 0xff};
const uint8_t piezoCharUuid[16] = {0x66, 0xd4, 0xa1, 0xcb, 0x4c, 0x34, 0xe3, 0x4b, 0xab, 0x3f, 0x18, 0x9f, 0x80, 0xdd, 0x75, 0xff};
const uint8_t batteryCharUuid[16] = {0x66, 0xd4, 0xa1, 0xcb, 0x4c, 0x34, 0xe3, 0x4b, 0xab, 0x3f, 0x18, 0x9f, 0x80, 0xdd, 0x76, 0xff};
const uint8_t firmwareCharUuid[16] = {0x66, 0xd4, 0xa1, 0xcb, 0x4c, 0x34, 0xe3, 0x4b, 0xab, 0x3f, 0x18, 0x9f, 0x80, 0xdd, 0x77, 0xff};
const uint8_t otaServiceUuid[16] = {0x78, 0x56, 0x34, 0x12, 0x34, 0x12, 0x78, 0x56, 0x12, 0x34, 0x56, 0x78, 0x9a, 0xbc, 0x00, 0x00};
const uint8_t otaCommandUuid[16] = {0x78, 0x56, 0x34, 0x12, 0x34, 0x12, 0x78, 0x56, 0x12, 0x34, 0x56, 0x78, 0x9a, 0xbc, 0x00, 0x01};
const uint8_t otaDataUuid[16] = {0x78, 0x56, 0x34, 0x12, 0x34, 0x12, 0x78, 0x56, 0x12, 0x34, 0x56, 0x78, 0x9a, 0xbc, 0x00, 0x02};
const uint8_t otaStatusUuid[16] = {0x78, 0x56, 0x34, 0x12, 0x34, 0x12, 0x78, 0x56, 0x12, 0x34, 0x56, 0x78, 0x9a, 0xbc, 0x00, 0x03};

enum PiezoAttrIndex
{
    PIEZO_IDX_SVC,
    PIEZO_IDX_CHAR_HIT,
    PIEZO_IDX_CHAR_VAL_HIT,
    PIEZO_IDX_CHAR_CFG_HIT,
    PIEZO_IDX_CHAR_BATT,
    PIEZO_IDX_CHAR_VAL_BATT,
    PIEZO_IDX_CHAR_CFG_BATT,
    PIEZO_IDX_CHAR_FW,
    PIEZO_IDX_CHAR_VAL_FW,
    PIEZO_IDX_CHAR_CFG_FW,
    PIEZO_IDX_NB
};

enum OtaAttrIndex
{
    OTA_IDX_SVC,
    OTA_IDX_CHAR_CMD,
    OTA_IDX_CHAR_VAL_CMD,
    OTA_IDX_CHAR_DATA,
    OTA_IDX_CHAR_VAL_DATA,
    OTA_IDX_CHAR_STATUS,
    OTA_IDX_CHAR_VAL_STATUS,
    OTA_IDX_CHAR_CFG_STATUS,
    OTA_IDX_NB
};

uint8_t piezoCmdValue[20];
uint8_t piezoCmdCcc[2] = {0x00, 0x00};
uint8_t batteryValue[8];
uint8_t batteryCcc[2] = {0x00, 0x00};
uint8_t firmwareValue[16];
uint8_t firmwareCcc[2] = {0x00, 0x00};
uint8_t otaCommandValue[16];
uint8_t otaDataValue[OTA_CHUNK_SIZE];
uint8_t otaStatusValue[32];
uint8_t otaStatusCcc[2] = {0x00, 0x00};

uint8_t hitCharProps = ESP_GATT_CHAR_PROP_BIT_NOTIFY | ESP_GATT_CHAR_PROP_BIT_WRITE;
uint8_t batteryCharProps = ESP_GATT_CHAR_PROP_BIT_NOTIFY | ESP_GATT_CHAR_PROP_BIT_READ;
uint8_t firmwareCharProps = ESP_GATT_CHAR_PROP_BIT_NOTIFY | ESP_GATT_CHAR_PROP_BIT_READ;
uint8_t otaCommandProps = ESP_GATT_CHAR_PROP_BIT_WRITE;
uint8_t otaDataProps = ESP_GATT_CHAR_PROP_BIT_WRITE | ESP_GATT_CHAR_PROP_BIT_WRITE_NR;
uint8_t otaStatusProps = ESP_GATT_CHAR_PROP_BIT_NOTIFY | ESP_GATT_CHAR_PROP_BIT_READ;

esp_gatts_attr_db_t piezo_gatt_db[PIEZO_IDX_NB] = {
    [PIEZO_IDX_SVC] = {{ESP_GATT_AUTO_RSP},
                       {ESP_UUID_LEN_16, (uint8_t *)&primaryServiceUuid, ESP_GATT_PERM_READ,
                        sizeof(piezoServiceUuid), sizeof(piezoServiceUuid), (uint8_t *)piezoServiceUuid}},
    [PIEZO_IDX_CHAR_HIT] = {{ESP_GATT_AUTO_RSP},
                            {ESP_UUID_LEN_16, (uint8_t *)&charDeclUuid, ESP_GATT_PERM_READ,
                             sizeof(uint8_t), sizeof(uint8_t), &hitCharProps}},
    [PIEZO_IDX_CHAR_VAL_HIT] = {{ESP_GATT_AUTO_RSP},
                                {ESP_UUID_LEN_128, (uint8_t *)piezoCharUuid, ESP_GATT_PERM_WRITE,
                                 sizeof(piezoCmdValue), sizeof(uint8_t), piezoCmdValue}},
    [PIEZO_IDX_CHAR_CFG_HIT] = {{ESP_GATT_AUTO_RSP},
                                {ESP_UUID_LEN_16, (uint8_t *)&clientConfigUuid, ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE,
                                 sizeof(uint16_t), sizeof(piezoCmdCcc), piezoCmdCcc}},
    [PIEZO_IDX_CHAR_BATT] = {{ESP_GATT_AUTO_RSP},
                             {ESP_UUID_LEN_16, (uint8_t *)&charDeclUuid, ESP_GATT_PERM_READ,
                              sizeof(uint8_t), sizeof(uint8_t), &batteryCharProps}},
    [PIEZO_IDX_CHAR_VAL_BATT] = {{ESP_GATT_AUTO_RSP},
                                 {ESP_UUID_LEN_128, (uint8_t *)batteryCharUuid, ESP_GATT_PERM_READ,
                                  sizeof(batteryValue), sizeof(uint8_t), batteryValue}},
    [PIEZO_IDX_CHAR_CFG_BATT] = {{ESP_GATT_AUTO_RSP},
                                 {ESP_UUID_LEN_16, (uint8_t *)&clientConfigUuid, ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE,
                                  sizeof(uint16_t), sizeof(batteryCcc), batteryCcc}},
    [PIEZO_IDX_CHAR_FW] = {{ESP_GATT_AUTO_RSP},
                           {ESP_UUID_LEN_16, (uint8_t *)&charDeclUuid, ESP_GATT_PERM_READ,
                            sizeof(uint8_t), sizeof(uint8_t), &firmwareCharProps}},
    [PIEZO_IDX_CHAR_VAL_FW] = {{ESP_GATT_AUTO_RSP},
                               {ESP_UUID_LEN_128, (uint8_t *)firmwareCharUuid, ESP_GATT_PERM_READ,
                                sizeof(firmwareValue), sizeof(uint8_t), firmwareValue}},
    [PIEZO_IDX_CHAR_CFG_FW] = {{ESP_GATT_AUTO_RSP},
                               {ESP_UUID_LEN_16, (uint8_t *)&clientConfigUuid, ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE,
                                sizeof(uint16_t), sizeof(firmwareCcc), firmwareCcc}},
};

esp_gatts_attr_db_t ota_gatt_db[OTA_IDX_NB] = {
    [OTA_IDX_SVC] = {{ESP_GATT_AUTO_RSP},
                     {ESP_UUID_LEN_16, (uint8_t *)&primaryServiceUuid, ESP_GATT_PERM_READ,
                      sizeof(otaServiceUuid), sizeof(otaServiceUuid), (uint8_t *)otaServiceUuid}},
    [OTA_IDX_CHAR_CMD] = {{ESP_GATT_AUTO_RSP},
                          {ESP_UUID_LEN_16, (uint8_t *)&charDeclUuid, ESP_GATT_PERM_READ,
                           sizeof(uint8_t), sizeof(uint8_t), &otaCommandProps}},
    [OTA_IDX_CHAR_VAL_CMD] = {{ESP_GATT_AUTO_RSP},
                              {ESP_UUID_LEN_128, (uint8_t *)otaCommandUuid, ESP_GATT_PERM_WRITE,
                               sizeof(otaCommandValue), sizeof(uint8_t), otaCommandValue}},
    [OTA_IDX_CHAR_DATA] = {{ESP_GATT_AUTO_RSP},
                           {ESP_UUID_LEN_16, (uint8_t *)&charDeclUuid, ESP_GATT_PERM_READ,
                            sizeof(uint8_t), sizeof(uint8_t), &otaDataProps}},
    [OTA_IDX_CHAR_VAL_DATA] = {{ESP_GATT_AUTO_RSP},
                               {ESP_UUID_LEN_128, (uint8_t *)otaDataUuid, ESP_GATT_PERM_WRITE,
                                sizeof(otaDataValue), sizeof(uint8_t), otaDataValue}},
    [OTA_IDX_CHAR_STATUS] = {{ESP_GATT_AUTO_RSP},
                             {ESP_UUID_LEN_16, (uint8_t *)&charDeclUuid, ESP_GATT_PERM_READ,
                              sizeof(uint8_t), sizeof(uint8_t), &otaStatusProps}},
    [OTA_IDX_CHAR_VAL_STATUS] = {{ESP_GATT_AUTO_RSP},
                                 {ESP_UUID_LEN_128, (uint8_t *)otaStatusUuid, ESP_GATT_PERM_READ,
                                  sizeof(otaStatusValue), sizeof(uint8_t), otaStatusValue}},
    [OTA_IDX_CHAR_CFG_STATUS] = {{ESP_GATT_AUTO_RSP},
                                 {ESP_UUID_LEN_16, (uint8_t *)&clientConfigUuid, ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE,
                                  sizeof(uint16_t), sizeof(otaStatusCcc), otaStatusCcc}},
};

esp_ble_adv_params_t advParams = {
    .adv_int_min = 0x20,
    .adv_int_max = 0x40,
    .adv_type = ADV_TYPE_IND,
    .own_addr_type = BLE_ADDR_TYPE_PUBLIC,
    .channel_map = ADV_CHNL_ALL,
    .adv_filter_policy = ADV_FILTER_ALLOW_SCAN_ANY_CON_ANY,
};

esp_ble_adv_data_t advData = {
    .set_scan_rsp = false,
    .include_name = true,
    .include_txpower = true,
    .min_interval = 0x20,
    .max_interval = 0x40,
    .appearance = 0,
    .manufacturer_len = 0,
    .p_manufacturer_data = nullptr,
    .service_data_len = 0,
    .p_service_data = nullptr,
    .service_uuid_len = sizeof(piezoServiceUuid),
    .p_service_uuid = piezoServiceUuid,
    .flag = (ESP_BLE_ADV_FLAG_GEN_DISC | ESP_BLE_ADV_FLAG_BREDR_NOT_SPT),
};
}

BLEManager *BLEManager::instance = nullptr;

BLEManager::BLEManager(LEDController *leds, PowerManager *power, OTAManager *ota)
    : ledController(leds),
      powerManager(power),
      otaManager(ota),
      deviceConnected(false),
      isInitialized(false),
      hitTime(0),
      gattsIf(ESP_GATT_IF_NONE),
      connId(0),
      ledStatusTaskHandle(nullptr),
      advConfigDone(0)
{
    std::fill(std::begin(piezoHandles), std::end(piezoHandles), 0);
    std::fill(std::begin(otaHandles), std::end(otaHandles), 0);
}

bool BLEManager::begin()
{
    if (isInitialized)
    {
        return true;
    }

    instance = this;
    advConfigDone = ADV_CONFIG_FLAG;
    hitTime = millis();

    esp_bt_controller_mem_release(ESP_BT_MODE_CLASSIC_BT);
    esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_bt_controller_init(&bt_cfg));
    ESP_ERROR_CHECK(esp_bt_controller_enable(ESP_BT_MODE_BLE));
    ESP_ERROR_CHECK(esp_bluedroid_init());
    ESP_ERROR_CHECK(esp_bluedroid_enable());
    ESP_ERROR_CHECK(esp_ble_gatts_register_callback(gattsEventHandler));
    ESP_ERROR_CHECK(esp_ble_gap_register_callback(gapEventHandler));
    ESP_ERROR_CHECK(esp_ble_gatts_app_register(0));
    ESP_ERROR_CHECK(esp_ble_gatt_set_local_mtu(517));
    registerBLEEventCallback();
    configurePhy();
    if (otaManager)
    {
        otaManager->setStatusCallback([this](const std::string &status)
                                      { sendOtaStatus(status); });
    }
    isInitialized = true;
    return true;
}

void BLEManager::configurePhy()
{
#ifdef CONFIG_BT_BLE_50_FEATURES_SUPPORTED
    if (BLE_CODED_PHY_PREFERRED)
    {
        esp_ble_gap_set_prefered_default_phy(ESP_BLE_GAP_PHY_CODED_PREF_MASK | ESP_BLE_GAP_PHY_1M_PREF_MASK,
                                             ESP_BLE_GAP_PHY_CODED_PREF_MASK | ESP_BLE_GAP_PHY_1M_PREF_MASK);
    }
#endif
}

void BLEManager::initGap()
{
    esp_ble_gap_set_device_name(BLE_SERVER_NAME);
    esp_ble_gap_config_adv_data(&advData);
}

void BLEManager::initGatt()
{
    ESP_ERROR_CHECK(esp_ble_gatts_create_attr_tab(piezo_gatt_db, gattsIf, PIEZO_IDX_NB, 0));
    ESP_ERROR_CHECK(esp_ble_gatts_create_attr_tab(ota_gatt_db, gattsIf, OTA_IDX_NB, 1));
}

void BLEManager::startAdvertising()
{
    ESP_ERROR_CHECK(esp_ble_gap_start_advertising(&advParams));
}

void BLEManager::onPiezoHit()
{
    hitTime = millis();
}

void BLEManager::sendNotification(uint16_t handle, const uint8_t *value, uint16_t length)
{
    if (!deviceConnected || gattsIf == ESP_GATT_IF_NONE || handle == 0)
    {
        return;
    }
    esp_ble_gatts_send_indicate(gattsIf, connId, handle, length, const_cast<uint8_t *>(value), false);
}

void BLEManager::sendPiezoValue(int piezoValue)
{
    if (!deviceConnected)
    {
        return;
    }
    uint8_t data[4];
    uint16_t delta = static_cast<uint16_t>((millis() - hitTime) / 10);
    data[0] = piezoValue & 0xFF;
    data[1] = (piezoValue >> 8) & 0xFF;
    data[2] = delta & 0xFF;
    data[3] = (delta >> 8) & 0xFF;
    sendNotification(piezoHandles[PIEZO_IDX_CHAR_VAL_HIT], data, sizeof(data));
}

void BLEManager::sendBatteryLevel(int percentage)
{
    if (!deviceConnected)
    {
        return;
    }
    std::string payload = std::to_string(std::clamp(percentage, 0, 100));
    sendNotification(piezoHandles[PIEZO_IDX_CHAR_VAL_BATT], reinterpret_cast<const uint8_t *>(payload.c_str()), payload.size());
}

void BLEManager::sendFirmwareVersion()
{
    if (!deviceConnected)
    {
        return;
    }
    const uint8_t *data = reinterpret_cast<const uint8_t *>(FIRMWARE_VERSION);
    sendNotification(piezoHandles[PIEZO_IDX_CHAR_VAL_FW], data, strlen(FIRMWARE_VERSION));
}

void BLEManager::sendOtaStatus(const std::string &status)
{
    sendNotification(otaHandles[OTA_IDX_CHAR_VAL_STATUS], reinterpret_cast<const uint8_t *>(status.c_str()), status.size());
}

BLECommandData BLEManager::parseCommand(const std::string &value)
{
    BLECommandData result{BLECommand::UNKNOWN, 0};
    if (value.rfind("rgb:", 0) == 0)
    {
        result.command = BLECommand::SET_RGB_COLOR;
        return result;
    }
    if (value.rfind("blinkrgb:", 0) == 0)
    {
        result.command = BLECommand::BLINK_COLOR;
        return result;
    }
    if (value.rfind("color:", 0) == 0)
    {
        result.command = BLECommand::SET_COLOR;
        return result;
    }
    if (value.rfind("r:", 0) == 0)
    {
        result.command = BLECommand::SET_RED;
        result.value = std::stoi(value.substr(2));
        return result;
    }
    if (value.rfind("g:", 0) == 0)
    {
        result.command = BLECommand::SET_GREEN;
        result.value = std::stoi(value.substr(2));
        return result;
    }
    if (value.rfind("b:", 0) == 0)
    {
        result.command = BLECommand::SET_BRIGHTNESS;
        result.value = std::stoi(value.substr(2));
        return result;
    }
    if (value == "start" || value.rfind("start:", 0) == 0)
        result.command = BLECommand::START;
    else if (value == "sleep")
        result.command = BLECommand::SLEEP;
    else if (value == "game1")
        result.command = BLECommand::GAME1;
    else if (value == "ble_status")
        result.command = BLECommand::BLE_STATUS;
    else if (value == "ble_phy")
        result.command = BLECommand::BLE_PHY_INFO;
    else if (value == "off")
        result.command = BLECommand::SET_COLOR;
    return result;
}

bool BLEManager::parseRgbValues(const std::string &rgbStr, int &red, int &green, int &blue)
{
    size_t first = rgbStr.find(',');
    size_t second = rgbStr.find(',', first + 1);
    if (first == std::string::npos || second == std::string::npos)
    {
        return false;
    }
    red = std::stoi(rgbStr.substr(0, first));
    green = std::stoi(rgbStr.substr(first + 1, second - first - 1));
    blue = std::stoi(rgbStr.substr(second + 1));
    red = std::clamp(red, 0, 255);
    green = std::clamp(green, 0, 255);
    blue = std::clamp(blue, 0, 255);
    return true;
}

void BLEManager::handleRGBCommand(const std::string &value)
{
    int r, g, b;
    if (parseRgbValues(value.substr(4), r, g, b) && ledController)
    {
        ledController->setRgbColor(r, g, b);
    }
}

void BLEManager::handleColorCommand(const std::string &value)
{
    if (!ledController)
    {
        return;
    }
    if (value == "off")
    {
        ledController->setPredefinedColor("off");
        return;
    }
    if (value.rfind("color:", 0) == 0)
    {
        ledController->setPredefinedColor(value.substr(6));
    }
}

void BLEManager::handleBlinkColorCommand(const std::string &value)
{
    int r, g, b;
    if (parseRgbValues(value.substr(9), r, g, b) && ledController)
    {
        ledController->blinkColor(r, g, b);
    }
}

void BLEManager::handleStartColorCommand(const std::string &value)
{
    int r, g, b;
    if (parseRgbValues(value.substr(6), r, g, b) && ledController)
    {
        ledController->setRgbColor(r, g, b);
    }
}

void BLEManager::handleWrite(uint16_t handle, const uint8_t *value, size_t len)
{
    if (handle == piezoHandles[PIEZO_IDX_CHAR_VAL_HIT])
    {
        std::string command(reinterpret_cast<const char *>(value), len);
        BLECommandData cmdData = parseCommand(command);
        switch (cmdData.command)
        {
        case BLECommand::START:
            onPiezoHit();
            if (command.rfind("start:", 0) == 0)
            {
                handleStartColorCommand(command);
            }
            break;
        case BLECommand::SLEEP:
            if (powerManager)
            {
                powerManager->goToDeepSleep();
            }
            break;
        case BLECommand::SET_BRIGHTNESS:
            if (ledController)
            {
                ledController->setBrightness(std::clamp(cmdData.value, 0, 255));
            }
            break;
        case BLECommand::SET_RGB_COLOR:
            handleRGBCommand(command);
            break;
        case BLECommand::BLINK_COLOR:
            handleBlinkColorCommand(command);
            break;
        case BLECommand::SET_COLOR:
            handleColorCommand(command);
            break;
        case BLECommand::SET_RED:
            if (ledController)
            {
                ledController->setRedChannel(std::clamp(cmdData.value, 0, 255));
            }
            break;
        case BLECommand::SET_GREEN:
            if (ledController)
            {
                ledController->setGreenChannel(std::clamp(cmdData.value, 0, 255));
            }
            break;
        case BLECommand::SET_BLUE:
            if (ledController)
            {
                ledController->setBlueChannel(std::clamp(cmdData.value, 0, 255));
            }
            break;
        case BLECommand::BLE_STATUS:
            logBLEConnectionInfo();
            break;
        case BLECommand::BLE_PHY_INFO:
            logBLEPHYInfo();
            break;
        default:
            break;
        }
        return;
    }

    if (handle == otaHandles[OTA_IDX_CHAR_VAL_CMD] && otaManager)
    {
        std::string cmd(reinterpret_cast<const char *>(value), len);
        otaManager->handleCommand(cmd);
        return;
    }

    if (handle == otaHandles[OTA_IDX_CHAR_VAL_DATA] && otaManager)
    {
        otaManager->handleDataChunk(value, len);
    }
}

void BLEManager::registerBLEEventCallback()
{
    ESP_LOGI(TAG, "BLE callbacks registered");
}

void BLEManager::logBLEConnectionInfo()
{
    ESP_LOGI(TAG, "Device %sconnected", deviceConnected ? "" : "not ");
}

void BLEManager::logBLEPHYInfo()
{
#ifdef CONFIG_BT_BLE_50_FEATURES_SUPPORTED
    ESP_LOGI(TAG, "PHY modes: 1M, 2M, Coded");
#else
    ESP_LOGI(TAG, "PHY modes: 1M");
#endif
}

void BLEManager::startLedStatusTask()
{
    if (ledStatusTaskHandle == nullptr)
    {
        xTaskCreatePinnedToCore(
            ledStatusTask,
            "LEDStatus",
            2048,
            this,
            1,
            &ledStatusTaskHandle,
            APP_CPU_NUM);
    }
}

void BLEManager::initialDeviceInfoTask(void *pvParameters)
{
    auto *manager = static_cast<BLEManager *>(pvParameters);
    vTaskDelay(pdMS_TO_TICKS(2000));
    if (manager->powerManager)
    {
        int percentage = manager->powerManager->getBatteryPercentage();
        manager->sendBatteryLevel(percentage);
    }
    manager->sendFirmwareVersion();
    vTaskDelete(nullptr);
}

void BLEManager::ledStatusTask(void *pvParameters)
{
    auto *manager = static_cast<BLEManager *>(pvParameters);
    for (;;)
    {
        if (!manager->deviceConnected && manager->ledController)
        {
            manager->ledController->setRgbColor(0, 0, 0);
            vTaskDelay(pdMS_TO_TICKS(1000));
            manager->ledController->setRgbColor(0, 0, 255);
            vTaskDelay(pdMS_TO_TICKS(1000));
        }
        else
        {
            vTaskDelay(pdMS_TO_TICKS(100));
        }
    }
}

void BLEManager::gapEventHandler(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t *param)
{
    if (!instance)
    {
        return;
    }
    switch (event)
    {
    case ESP_GAP_BLE_ADV_DATA_SET_COMPLETE_EVT:
        instance->advConfigDone &= ~ADV_CONFIG_FLAG;
        if (instance->advConfigDone == 0)
        {
            instance->startAdvertising();
        }
        break;
    case ESP_GAP_BLE_ADV_START_COMPLETE_EVT:
        if (param->adv_start_cmpl.status != ESP_BT_STATUS_SUCCESS)
        {
            ESP_LOGE(TAG, "Advertising start failed");
        }
        break;
    case ESP_GAP_BLE_UPDATE_CONN_PARAMS_EVT:
        ESP_LOGI(TAG, "Conn params interval=%d latency=%d timeout=%d",
                 param->update_conn_params.int_min,
                 param->update_conn_params.latency,
                 param->update_conn_params.timeout);
        break;
    default:
        break;
    }
}

void BLEManager::gattsEventHandler(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param)
{
    if (!instance)
    {
        return;
    }

    switch (event)
    {
    case ESP_GATTS_REG_EVT:
        instance->gattsIf = gatts_if;
        instance->initGap();
        instance->initGatt();
        break;
    case ESP_GATTS_CREAT_ATTR_TAB_EVT:
        if (param->add_attr_tab.status != ESP_GATT_OK)
        {
            ESP_LOGE(TAG, "Attr table create failed");
            break;
        }
        if (param->add_attr_tab.svc_inst_id == 0 && param->add_attr_tab.num_handle == PIEZO_IDX_NB)
        {
            memcpy(instance->piezoHandles, param->add_attr_tab.handles, sizeof(instance->piezoHandles));
            esp_ble_gatts_start_service(instance->piezoHandles[PIEZO_IDX_SVC]);
        }
        else if (param->add_attr_tab.svc_inst_id == 1 && param->add_attr_tab.num_handle == OTA_IDX_NB)
        {
            memcpy(instance->otaHandles, param->add_attr_tab.handles, sizeof(instance->otaHandles));
            esp_ble_gatts_start_service(instance->otaHandles[OTA_IDX_SVC]);
        }
        break;
    case ESP_GATTS_CONNECT_EVT:
        instance->connId = param->connect.conn_id;
        instance->deviceConnected = true;
        if (instance->powerManager)
        {
            instance->powerManager->setConnected(true);
        }
        if (instance->ledController)
        {
            instance->ledController->blinkColor(255, 255, 255);
        }
        instance->logBLEConnectionInfo();
        esp_ble_gap_update_conn_params(&param->connect.conn_params);
        xTaskCreate(BLEManager::initialDeviceInfoTask, "InitDeviceInfo", 4096, instance, 1, nullptr);
        instance->startLedStatusTask();
        break;
    case ESP_GATTS_DISCONNECT_EVT:
        instance->deviceConnected = false;
        if (instance->powerManager)
        {
            instance->powerManager->setConnected(false);
        }
        instance->startAdvertising();
        break;
    case ESP_GATTS_WRITE_EVT:
        if (param->write.need_rsp)
        {
            esp_gatt_status_t status = ESP_GATT_OK;
            esp_ble_gatts_send_response(gatts_if, param->write.conn_id, param->write.trans_id, status, nullptr);
        }
        instance->handleWrite(param->write.handle, param->write.value, param->write.len);
        break;
    case ESP_GATTS_MTU_EVT:
        ESP_LOGI(TAG, "MTU updated to %d", param->mtu.mtu);
        break;
    default:
        break;
    }
}
