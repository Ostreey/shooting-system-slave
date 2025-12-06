#include "BLEManager.h"

#include <algorithm>
#include <cctype>
#include <cstdlib>
#include <cstring>
#include <climits>
#include <vector>

#include "LEDController.h"
#include "OTAManager.h"
#include "PowerManager.h"
#include "esp_err.h"
#include "esp_log.h"
#include "nvs_flash.h"

namespace
{
    constexpr char TAG[] = "BLEManager";
    constexpr uint16_t kMainServiceAttrCount = 12;
    constexpr uint16_t kOtaServiceAttrCount = 10;
    std::array<uint8_t, 16> uuidFromString(const std::string &uuidStr)
    {
        std::array<uint8_t, 16> result{};
        std::string hex;
        hex.reserve(32);
        for (char c : uuidStr)
        {
            if (c != '-')
            {
                hex.push_back(static_cast<char>(std::tolower(static_cast<unsigned char>(c))));
            }
        }

        if (hex.size() != 32)
        {
            return result;
        }

        for (size_t i = 0; i < 16; ++i)
        {
            size_t index = (15 - i) * 2;
            std::string byteStr = hex.substr(index, 2);
            result[i] = static_cast<uint8_t>(std::strtoul(byteStr.c_str(), nullptr, 16));
        }
        return result;
    }

    void fillUuid(esp_bt_uuid_t &uuid, const std::array<uint8_t, 16> &value)
    {
        uuid.len = ESP_UUID_LEN_128;
        std::memcpy(uuid.uuid.uuid128, value.data(), value.size());
    }

    bool uuidMatches(const esp_bt_uuid_t &uuid, const std::array<uint8_t, 16> &value)
    {
        return uuid.len == ESP_UUID_LEN_128 &&
               std::memcmp(uuid.uuid.uuid128, value.data(), value.size()) == 0;
    }

    const std::array<uint8_t, 16> kServiceUuid = uuidFromString(SERVICE_UUID);
    const std::array<uint8_t, 16> kPiezoCharUuid = uuidFromString(CHARACTERISTIC);
    const std::array<uint8_t, 16> kBatteryCharUuid = uuidFromString(BATTERY_CHARACTERISTIC);
    const std::array<uint8_t, 16> kFirmwareCharUuid = uuidFromString(FIRMWARE_VERSION_CHARACTERISTIC);
    const std::array<uint8_t, 16> kOtaServiceUuid = uuidFromString(OTA_SERVICE_UUID);
    const std::array<uint8_t, 16> kOtaCommandUuid = uuidFromString(OTA_COMMAND_CHAR_UUID);
    const std::array<uint8_t, 16> kOtaDataUuid = uuidFromString(OTA_DATA_CHAR_UUID);
    const std::array<uint8_t, 16> kOtaStatusUuid = uuidFromString(OTA_STATUS_CHAR_UUID);

    esp_attr_value_t descriptorValue()
    {
        static uint8_t ccc[2] = {0x00, 0x00};
        esp_attr_value_t value = {
            .attr_max_len = sizeof(ccc),
            .attr_len = sizeof(ccc),
            .attr_value = ccc};
        return value;
    }
}

BLEManager *BLEManager::instance = nullptr;

BLEManager::BLEManager(LEDController *leds, PowerManager *power, OTAManager *ota)
    : ledController(leds),
      powerManager(power),
      otaManager(ota),
      deviceConnected(false),
      isInitialized(false),
      hitTimeMs(0),
      mainGattIf(ESP_GATT_IF_NONE),
      otaGattIf(ESP_GATT_IF_NONE),
      connId(0),
      ledStatusTaskHandle(nullptr),
      pendingDescriptorOwner(DescriptorOwner::None),
      advDataConfigured(false)
{
    serviceUuidBuffer.fill(0);
}

void BLEManager::setupBluetoothController()
{
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND)
    {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ESP_ERROR_CHECK(nvs_flash_init());
    }

    esp_bt_controller_mem_release(ESP_BT_MODE_CLASSIC_BT);
    esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
    ret = esp_bt_controller_init(&bt_cfg);
    if (ret != ESP_OK && ret != ESP_ERR_INVALID_STATE)
    {
        ESP_ERROR_CHECK(ret);
    }

    ret = esp_bt_controller_enable(ESP_BT_MODE_BLE);
    if (ret != ESP_OK && ret != ESP_ERR_INVALID_STATE)
    {
        ESP_ERROR_CHECK(ret);
    }

    ret = esp_bluedroid_init();
    if (ret != ESP_OK && ret != ESP_ERR_INVALID_STATE)
    {
        ESP_ERROR_CHECK(ret);
    }

    ret = esp_bluedroid_enable();
    if (ret != ESP_OK && ret != ESP_ERR_INVALID_STATE)
    {
        ESP_ERROR_CHECK(ret);
    }
}

void BLEManager::configureAdvertising()
{
    std::copy(kServiceUuid.begin(), kServiceUuid.end(), serviceUuidBuffer.begin());
    std::copy(kOtaServiceUuid.begin(), kOtaServiceUuid.end(), serviceUuidBuffer.begin() + 16);

    esp_ble_adv_data_t adv_data = {};
    adv_data.set_scan_rsp = false;
    adv_data.include_name = true;
    adv_data.include_txpower = true;
    adv_data.min_interval = 0x0006;
    adv_data.max_interval = 0x0010;
    adv_data.appearance = 0;
    adv_data.manufacturer_len = 0;
    adv_data.p_manufacturer_data = nullptr;
    adv_data.service_data_len = 0;
    adv_data.p_service_data = nullptr;
    adv_data.service_uuid_len = 0;
    adv_data.p_service_uuid = nullptr;
    adv_data.flag = ESP_BLE_ADV_FLAG_GEN_DISC | ESP_BLE_ADV_FLAG_BREDR_NOT_SPT;

    ESP_ERROR_CHECK(esp_ble_gap_config_adv_data(&adv_data));
}

bool BLEManager::begin()
{
    if (isInitialized)
    {
        return true;
    }

    instance = this;
    setupBluetoothController();

    ESP_ERROR_CHECK(esp_ble_gatts_register_callback(gattsEventHandler));
    ESP_ERROR_CHECK(esp_ble_gap_register_callback(gapEventHandler));
    ESP_ERROR_CHECK(esp_ble_gatts_app_register(kMainAppId));
    ESP_ERROR_CHECK(esp_ble_gatts_app_register(kOtaAppId));
    ESP_ERROR_CHECK(esp_ble_gatt_set_local_mtu(517));
    ESP_ERROR_CHECK(esp_ble_gap_set_device_name(BLE_SERVER_NAME));

    configureAdvertising();

    if (otaManager)
    {
        otaManager->begin();
        otaManager->setStatusCallback([this](const std::string &status)
                                      { sendOtaStatus(status); });
    }

    lastFirmwareValue = FIRMWARE_VERSION;
    isInitialized = true;

    ESP_LOGI(TAG, "BLE initialization started");
    return true;
}

void BLEManager::startAdvertising()
{
    if (advDataConfigured)
    {
        esp_ble_adv_params_t params = {};
        params.adv_int_min = 0x0100;
        params.adv_int_max = 0x0100;
        params.adv_type = ADV_TYPE_IND;
        params.own_addr_type = BLE_ADDR_TYPE_PUBLIC;
        params.channel_map = ADV_CHNL_ALL;
        params.adv_filter_policy = ADV_FILTER_ALLOW_SCAN_ANY_CON_ANY;

        ESP_LOGI(TAG, "Starting advertising: int_min=0x%04X int_max=0x%04X type=%d own_addr_type=%d peer_addr_type=%d channel_map=0x%02X",
                 params.adv_int_min,
                 params.adv_int_max,
                 params.adv_type,
                 params.own_addr_type,
                 params.peer_addr_type,
                 params.channel_map);

        esp_err_t err = esp_ble_gap_start_advertising(&params);
        if (err != ESP_OK)
        {
            ESP_LOGE(TAG, "Failed to start advertising: %s", esp_err_to_name(err));
        }
    }
}

void BLEManager::gapEventHandler(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t *param)
{
    if (instance)
    {
        instance->handleGapEvent(event, param);
    }
}

void BLEManager::gattsEventHandler(esp_gatts_cb_event_t event,
                                   esp_gatt_if_t gatts_if,
                                   esp_ble_gatts_cb_param_t *param)
{
    if (instance)
    {
        instance->handleGattEvent(event, gatts_if, param);
    }
}

void BLEManager::handleGapEvent(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t *param)
{
    switch (event)
    {
    case ESP_GAP_BLE_ADV_DATA_SET_COMPLETE_EVT:
        advDataConfigured = true;
        startAdvertising();
        break;
    case ESP_GAP_BLE_ADV_START_COMPLETE_EVT:
        if (param->adv_start_cmpl.status == ESP_BT_STATUS_SUCCESS)
        {
            ESP_LOGI(TAG, "Advertising started");
        }
        else
        {
            ESP_LOGE(TAG, "Advertising start failed");
        }
        break;
    case ESP_GAP_BLE_ADV_STOP_COMPLETE_EVT:
        ESP_LOGI(TAG, "Advertising stopped");
        break;
    case ESP_GAP_BLE_PHY_UPDATE_COMPLETE_EVT:
        ESP_LOGI(TAG, "PHY update: tx=%d rx=%d status=%d",
                 param->phy_update.tx_phy,
                 param->phy_update.rx_phy,
                 param->phy_update.status);
        break;
    default:
        break;
    }
}

void BLEManager::handleGattEvent(esp_gatts_cb_event_t event,
                                 esp_gatt_if_t gatts_if,
                                 esp_ble_gatts_cb_param_t *param)
{
    switch (event)
    {
    case ESP_GATTS_REG_EVT:
        onRegistration(gatts_if, param);
        if (param->reg.app_id == kMainAppId)
        {
            mainGattIf = gatts_if;
        }
        else if (param->reg.app_id == kOtaAppId)
        {
            otaGattIf = gatts_if;
        }
        break;
    case ESP_GATTS_CREATE_EVT:
        onCreateService(param);
        break;
    case ESP_GATTS_ADD_CHAR_EVT:
        onAddCharacteristic(param);
        break;
    case ESP_GATTS_ADD_CHAR_DESCR_EVT:
        onAddDescriptor(param);
        break;
    case ESP_GATTS_CONNECT_EVT:
        if (gatts_if == mainGattIf || gatts_if == otaGattIf)
        {
            connId = param->connect.conn_id;
            onConnectEvent(param);
        }
        break;
    case ESP_GATTS_DISCONNECT_EVT:
        onDisconnectEvent();
        startAdvertising();
        break;
    case ESP_GATTS_READ_EVT:
        onReadEvent(gatts_if, param);
        break;
    case ESP_GATTS_WRITE_EVT:
        onWriteEvent(gatts_if, param);
        break;
    default:
        break;
    }
}

void BLEManager::onRegistration(esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param)
{
    if (param->reg.status != ESP_GATT_OK)
    {
        ESP_LOGE(TAG, "GATTS app registration failed: app_id=%d status=%d",
                 param->reg.app_id, param->reg.status);
    }
    else
    {
        ESP_LOGI(TAG, "GATTS app registered: app_id=%d", param->reg.app_id);

        esp_gatt_srvc_id_t service_id = {};
        service_id.is_primary = true;
        service_id.id.inst_id = (param->reg.app_id == kMainAppId) ? 0 : 1;

        if (param->reg.app_id == kMainAppId)
        {
            fillUuid(service_id.id.uuid, kServiceUuid);
            ESP_ERROR_CHECK(esp_ble_gatts_create_service(gatts_if, &service_id, kMainServiceAttrCount));
        }
        else if (param->reg.app_id == kOtaAppId)
        {
            fillUuid(service_id.id.uuid, kOtaServiceUuid);
            ESP_ERROR_CHECK(esp_ble_gatts_create_service(gatts_if, &service_id, kOtaServiceAttrCount));
        }
    }
}

void BLEManager::onCreateService(esp_ble_gatts_cb_param_t *param)
{
    if (param->create.status != ESP_GATT_OK)
    {
        ESP_LOGE(TAG, "Service creation failed");
        return;
    }

    if (uuidMatches(param->create.service_id.id.uuid, kServiceUuid))
    {
        mainHandles.serviceHandle = param->create.service_handle;
        ESP_ERROR_CHECK(esp_ble_gatts_start_service(mainHandles.serviceHandle));

        esp_bt_uuid_t piezoUuid;
        fillUuid(piezoUuid, kPiezoCharUuid);
        uint8_t initialPiezo[4] = {0};
        esp_attr_value_t piezoVal = {
            .attr_max_len = sizeof(initialPiezo),
            .attr_len = sizeof(initialPiezo),
            .attr_value = initialPiezo};
        ESP_ERROR_CHECK(esp_ble_gatts_add_char(
            mainHandles.serviceHandle,
            &piezoUuid,
            ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE,
            ESP_GATT_CHAR_PROP_BIT_NOTIFY | ESP_GATT_CHAR_PROP_BIT_WRITE,
            &piezoVal,
            nullptr));

        esp_bt_uuid_t piezoDescrUuid = {};
        piezoDescrUuid.len = ESP_UUID_LEN_16;
        piezoDescrUuid.uuid.uuid16 = ESP_GATT_UUID_CHAR_CLIENT_CONFIG;
        esp_attr_value_t piezoDescrVal = descriptorValue();
        ESP_ERROR_CHECK(esp_ble_gatts_add_char_descr(
            mainHandles.serviceHandle,
            &piezoDescrUuid,
            ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE,
            &piezoDescrVal,
            nullptr));

        esp_bt_uuid_t batteryUuid;
        fillUuid(batteryUuid, kBatteryCharUuid);
        uint8_t batteryInit[4] = {'0', 0};
        esp_attr_value_t batteryVal = {
            .attr_max_len = 8,
            .attr_len = 1,
            .attr_value = batteryInit};
        ESP_ERROR_CHECK(esp_ble_gatts_add_char(
            mainHandles.serviceHandle,
            &batteryUuid,
            ESP_GATT_PERM_READ,
            ESP_GATT_CHAR_PROP_BIT_READ | ESP_GATT_CHAR_PROP_BIT_NOTIFY,
            &batteryVal,
            nullptr));

        esp_bt_uuid_t batteryDescrUuid = {};
        batteryDescrUuid.len = ESP_UUID_LEN_16;
        batteryDescrUuid.uuid.uuid16 = ESP_GATT_UUID_CHAR_CLIENT_CONFIG;
        esp_attr_value_t batteryDescrVal = descriptorValue();
        ESP_ERROR_CHECK(esp_ble_gatts_add_char_descr(
            mainHandles.serviceHandle,
            &batteryDescrUuid,
            ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE,
            &batteryDescrVal,
            nullptr));

        esp_bt_uuid_t firmwareUuid;
        fillUuid(firmwareUuid, kFirmwareCharUuid);
        esp_attr_value_t firmwareVal = {
            .attr_max_len = sizeof(FIRMWARE_VERSION),
            .attr_len = static_cast<uint16_t>(std::strlen(FIRMWARE_VERSION)),
            .attr_value = (uint8_t *)FIRMWARE_VERSION};
        ESP_ERROR_CHECK(esp_ble_gatts_add_char(
            mainHandles.serviceHandle,
            &firmwareUuid,
            ESP_GATT_PERM_READ,
            ESP_GATT_CHAR_PROP_BIT_READ | ESP_GATT_CHAR_PROP_BIT_NOTIFY,
            &firmwareVal,
            nullptr));

        esp_bt_uuid_t firmwareDescrUuid = {};
        firmwareDescrUuid.len = ESP_UUID_LEN_16;
        firmwareDescrUuid.uuid.uuid16 = ESP_GATT_UUID_CHAR_CLIENT_CONFIG;
        esp_attr_value_t firmwareDescrVal = descriptorValue();
        ESP_ERROR_CHECK(esp_ble_gatts_add_char_descr(
            mainHandles.serviceHandle,
            &firmwareDescrUuid,
            ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE,
            &firmwareDescrVal,
            nullptr));
    }
    else if (uuidMatches(param->create.service_id.id.uuid, kOtaServiceUuid))
    {
        otaHandles.serviceHandle = param->create.service_handle;
        ESP_LOGI(TAG, "OTA service created, handle=%d", otaHandles.serviceHandle);
        ESP_ERROR_CHECK(esp_ble_gatts_start_service(otaHandles.serviceHandle));
        ESP_LOGI(TAG, "OTA service started");

        esp_bt_uuid_t commandUuid;
        fillUuid(commandUuid, kOtaCommandUuid);
        ESP_ERROR_CHECK(esp_ble_gatts_add_char(
            otaHandles.serviceHandle,
            &commandUuid,
            ESP_GATT_PERM_WRITE,
            ESP_GATT_CHAR_PROP_BIT_WRITE,
            nullptr,
            nullptr));

        esp_bt_uuid_t dataUuid;
        fillUuid(dataUuid, kOtaDataUuid);
        ESP_ERROR_CHECK(esp_ble_gatts_add_char(
            otaHandles.serviceHandle,
            &dataUuid,
            ESP_GATT_PERM_WRITE,
            ESP_GATT_CHAR_PROP_BIT_WRITE | ESP_GATT_CHAR_PROP_BIT_WRITE_NR,
            nullptr,
            nullptr));

        esp_bt_uuid_t statusUuid;
        fillUuid(statusUuid, kOtaStatusUuid);
        esp_attr_value_t statusVal = {
            .attr_max_len = 32,
            .attr_len = 0,
            .attr_value = nullptr};
        ESP_ERROR_CHECK(esp_ble_gatts_add_char(
            otaHandles.serviceHandle,
            &statusUuid,
            ESP_GATT_PERM_READ,
            ESP_GATT_CHAR_PROP_BIT_READ | ESP_GATT_CHAR_PROP_BIT_NOTIFY,
            &statusVal,
            nullptr));

        // Add CCCD descriptor for OTA status notifications
        esp_bt_uuid_t otaStatusDescrUuid = {};
        otaStatusDescrUuid.len = ESP_UUID_LEN_16;
        otaStatusDescrUuid.uuid.uuid16 = ESP_GATT_UUID_CHAR_CLIENT_CONFIG;
        esp_attr_value_t otaStatusDescrVal = descriptorValue();
        ESP_ERROR_CHECK(esp_ble_gatts_add_char_descr(
            otaHandles.serviceHandle,
            &otaStatusDescrUuid,
            ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE,
            &otaStatusDescrVal,
            nullptr));
    }
}

void BLEManager::onAddCharacteristic(esp_ble_gatts_cb_param_t *param)
{
    if (uuidMatches(param->add_char.char_uuid, kPiezoCharUuid))
    {
        mainHandles.piezoCharHandle = param->add_char.attr_handle;
    }
    else if (uuidMatches(param->add_char.char_uuid, kBatteryCharUuid))
    {
        mainHandles.batteryCharHandle = param->add_char.attr_handle;
    }
    else if (uuidMatches(param->add_char.char_uuid, kFirmwareCharUuid))
    {
        mainHandles.firmwareCharHandle = param->add_char.attr_handle;
    }
    else if (uuidMatches(param->add_char.char_uuid, kOtaCommandUuid))
    {
        otaHandles.commandCharHandle = param->add_char.attr_handle;
        ESP_LOGI(TAG, "OTA command characteristic handle set: %d", otaHandles.commandCharHandle);
    }
    else if (uuidMatches(param->add_char.char_uuid, kOtaDataUuid))
    {
        otaHandles.dataCharHandle = param->add_char.attr_handle;
        ESP_LOGI(TAG, "OTA data characteristic handle set: %d", otaHandles.dataCharHandle);
    }
    else if (uuidMatches(param->add_char.char_uuid, kOtaStatusUuid))
    {
        otaHandles.statusCharHandle = param->add_char.attr_handle;
        ESP_LOGI(TAG, "OTA status characteristic handle set: %d", otaHandles.statusCharHandle);
    }
}

void BLEManager::onAddDescriptor(esp_ble_gatts_cb_param_t *param)
{
    if (param->add_char_descr.service_handle == mainHandles.serviceHandle)
    {
        if (mainHandles.piezoCccHandle == 0)
        {
            mainHandles.piezoCccHandle = param->add_char_descr.attr_handle;
            ESP_LOGI(TAG, "Piezo CCCD handle set: %d", mainHandles.piezoCccHandle);
        }
        else if (mainHandles.batteryCccHandle == 0)
        {
            mainHandles.batteryCccHandle = param->add_char_descr.attr_handle;
            ESP_LOGI(TAG, "Battery CCCD handle set: %d", mainHandles.batteryCccHandle);
        }
        else if (mainHandles.firmwareCccHandle == 0)
        {
            mainHandles.firmwareCccHandle = param->add_char_descr.attr_handle;
            ESP_LOGI(TAG, "Firmware CCCD handle set: %d", mainHandles.firmwareCccHandle);
        }
    }
    else if (param->add_char_descr.service_handle == otaHandles.serviceHandle)
    {
        if (otaHandles.statusCccHandle == 0)
        {
            otaHandles.statusCccHandle = param->add_char_descr.attr_handle;
            ESP_LOGI(TAG, "OTA status CCCD handle set: %d", otaHandles.statusCccHandle);
        }
    }
}

void BLEManager::onConnectEvent(esp_ble_gatts_cb_param_t *param)
{
    deviceConnected = true;
    if (powerManager)
    {
        powerManager->setConnected(true);
    }
    if (ledController)
    {
        ledController->blinkColor(255, 255, 255);
    }

#if BLE_CODED_PHY_PREFERRED
    esp_err_t phyErr = esp_ble_gap_set_preferred_default_phy(
        ESP_BLE_GAP_PHY_CODED_PREF_MASK,
        ESP_BLE_GAP_PHY_CODED_PREF_MASK);
    if (phyErr == ESP_OK)
    {
        ESP_LOGI(TAG, "Preferred default Coded PHY set");
    }
    else
    {
        ESP_LOGW(TAG, "Failed to set preferred default PHY: %s", esp_err_to_name(phyErr));
    }
#endif

    scheduleInitialInfo();
    logBLEConnectionInfo();
    logBLEPHYInfo();
}

void BLEManager::onDisconnectEvent()
{
    deviceConnected = false;
    if (powerManager)
    {
        powerManager->setConnected(false);
    }
}

void BLEManager::onReadEvent(esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param)
{
    esp_gatt_rsp_t rsp = {};
    rsp.attr_value.handle = param->read.handle;
    rsp.attr_value.len = 0;

    const uint8_t *data = nullptr;
    size_t len = 0;
    std::vector<uint8_t> buffer;

    if (param->read.handle == mainHandles.batteryCharHandle)
    {
        buffer.assign(lastBatteryValue.begin(), lastBatteryValue.end());
        data = buffer.data();
        len = buffer.size();
    }
    else if (param->read.handle == mainHandles.firmwareCharHandle)
    {
        data = reinterpret_cast<const uint8_t *>(lastFirmwareValue.c_str());
        len = lastFirmwareValue.size();
    }
    else if (param->read.handle == otaHandles.statusCharHandle)
    {
        buffer.assign(lastOtaStatus.begin(), lastOtaStatus.end());
        data = buffer.data();
        len = buffer.size();
    }

    if (data && len <= ESP_GATT_MAX_ATTR_LEN)
    {
        rsp.attr_value.len = len;
        std::memcpy(rsp.attr_value.value, data, len);
        esp_ble_gatts_send_response(gatts_if, param->read.conn_id,
                                    param->read.trans_id, ESP_GATT_OK, &rsp);
    }
    else
    {
        esp_ble_gatts_send_response(gatts_if, param->read.conn_id,
                                    param->read.trans_id, ESP_GATT_READ_NOT_PERMIT, nullptr);
    }
}

void BLEManager::updateNotifyState(uint16_t handle, bool enabled)
{
}

void BLEManager::onWriteEvent(esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param)
{
    ESP_LOGD(TAG, "Write event: handle=%d, len=%d, need_rsp=%d",
             param->write.handle, param->write.len, param->write.need_rsp);

    // Handle CCCD descriptor writes (notification enable/disable)
    if (param->write.handle == mainHandles.piezoCccHandle ||
        param->write.handle == mainHandles.batteryCccHandle ||
        param->write.handle == mainHandles.firmwareCccHandle ||
        param->write.handle == otaHandles.statusCccHandle)
    {
        bool enabled = (param->write.len >= 2) &&
                       (param->write.value[0] != 0 || param->write.value[1] != 0);
        ESP_LOGI(TAG, "CCCD write: handle=%d, enabled=%d", param->write.handle, enabled);

        if (param->write.handle == otaHandles.statusCccHandle)
        {
            otaStatusNotifyEnabled = enabled;
            ESP_LOGI(TAG, "OTA status notifications %s", enabled ? "enabled" : "disabled");
        }
        else if (param->write.handle == mainHandles.piezoCccHandle)
        {
            piezoNotifyEnabled = enabled;
        }
        else if (param->write.handle == mainHandles.batteryCccHandle)
        {
            batteryNotifyEnabled = enabled;
        }
        else if (param->write.handle == mainHandles.firmwareCccHandle)
        {
            firmwareNotifyEnabled = enabled;
        }

        if (param->write.need_rsp)
        {
            esp_ble_gatts_send_response(gatts_if, param->write.conn_id,
                                        param->write.trans_id, ESP_GATT_OK, nullptr);
        }
        return;
    }

    if (param->write.need_rsp)
    {
        esp_ble_gatts_send_response(gatts_if, param->write.conn_id,
                                    param->write.trans_id, ESP_GATT_OK, nullptr);
    }

    if (param->write.handle == mainHandles.piezoCharHandle)
    {
        std::string value(reinterpret_cast<const char *>(param->write.value),
                          param->write.len);
        processCommand(value);
    }
    else if (param->write.handle == otaHandles.commandCharHandle)
    {
        if (otaHandles.commandCharHandle == 0)
        {
            ESP_LOGE(TAG, "OTA command handle is 0 - service not initialized!");
            return;
        }
        ESP_LOGI(TAG, "OTA command write received: handle=%d, len=%d",
                 param->write.handle, param->write.len);
        if (otaManager)
        {
            // Create string from raw bytes using length parameter
            std::string command(reinterpret_cast<const char *>(param->write.value),
                                param->write.len);
            ESP_LOGI(TAG, "Forwarding OTA command to manager: '%s' (len=%zu)",
                     command.c_str(), command.length());
            otaManager->handleCommand(command);
        }
        else
        {
            ESP_LOGE(TAG, "OTA manager is null!");
        }
    }
    else if (param->write.handle == otaHandles.dataCharHandle)
    {
        ESP_LOGD(TAG, "OTA data chunk received: len=%d", param->write.len);
        if (otaManager)
        {
            otaManager->handleDataChunk(param->write.value, param->write.len);
        }
        else
        {
            ESP_LOGE(TAG, "OTA manager is null!");
        }
    }
    else
    {
        ESP_LOGW(TAG, "Write to unknown handle: %d (piezo=%d, cmd=%d, data=%d, status=%d)",
                 param->write.handle,
                 mainHandles.piezoCharHandle,
                 otaHandles.commandCharHandle,
                 otaHandles.dataCharHandle,
                 otaHandles.statusCharHandle);
    }
}

void BLEManager::processCommand(const std::string &value)
{
    BLECommandData cmdData = parseCommand(value);
    switch (cmdData.command)
    {
    case BLECommand::START:
        onPiezoHit();
        if (value.rfind("start:", 0) == 0)
        {
            handleStartColorCommand(value);
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
            ledController->setBrightness(clamp_value(cmdData.value, 0, 255));
        }
        break;
    case BLECommand::SET_RGB_COLOR:
        handleRGBCommand(value);
        break;
    case BLECommand::BLINK_COLOR:
        handleBlinkColorCommand(value);
        break;
    case BLECommand::SET_COLOR:
        handleColorCommand(value);
        break;
    case BLECommand::SET_RED:
        if (ledController)
        {
            ledController->setRedChannel(clamp_value(cmdData.value, 0, 255));
        }
        break;
    case BLECommand::SET_GREEN:
        if (ledController)
        {
            ledController->setGreenChannel(clamp_value(cmdData.value, 0, 255));
        }
        break;
    case BLECommand::SET_BLUE:
        if (ledController)
        {
            ledController->setBlueChannel(clamp_value(cmdData.value, 0, 255));
        }
        break;
    case BLECommand::BLE_STATUS:
        logBLEConnectionInfo();
        break;
    case BLECommand::BLE_PHY_INFO:
        logBLEPHYInfo();
        break;
    default:
        ESP_LOGW(TAG, "Unknown BLE command");
        break;
    }
}

void BLEManager::sendPiezoValue(int piezoValue)
{
    if (!deviceConnected || mainHandles.piezoCharHandle == 0)
    {
        return;
    }

    uint8_t data[4];
    uint16_t elapsed = static_cast<uint16_t>((millis64() - hitTimeMs) / 10);
    data[0] = static_cast<uint8_t>(piezoValue & 0xFF);
    data[1] = static_cast<uint8_t>((piezoValue >> 8) & 0xFF);
    data[2] = static_cast<uint8_t>(elapsed & 0xFF);
    data[3] = static_cast<uint8_t>((elapsed >> 8) & 0xFF);

    esp_ble_gatts_send_indicate(mainGattIf, connId, mainHandles.piezoCharHandle,
                                sizeof(data), data, false);
}

void BLEManager::sendBatteryLevel(int percentage)
{
    lastBatteryValue = std::to_string(percentage);
    if (mainHandles.batteryCharHandle)
    {
        esp_ble_gatts_set_attr_value(mainHandles.batteryCharHandle,
                                     lastBatteryValue.size(),
                                     reinterpret_cast<const uint8_t *>(lastBatteryValue.c_str()));
    }

    if (deviceConnected && mainHandles.batteryCharHandle)
    {
        esp_ble_gatts_send_indicate(mainGattIf, connId,
                                    mainHandles.batteryCharHandle,
                                    lastBatteryValue.size(),
                                    const_cast<uint8_t *>(reinterpret_cast<const uint8_t *>(lastBatteryValue.c_str())),
                                    false);
    }
}

void BLEManager::sendFirmwareVersion()
{
    lastFirmwareValue = FIRMWARE_VERSION;
    if (mainHandles.firmwareCharHandle)
    {
        esp_ble_gatts_set_attr_value(mainHandles.firmwareCharHandle,
                                     lastFirmwareValue.size(),
                                     reinterpret_cast<const uint8_t *>(lastFirmwareValue.c_str()));
    }
    if (deviceConnected && mainHandles.firmwareCharHandle)
    {
        esp_ble_gatts_send_indicate(mainGattIf, connId,
                                    mainHandles.firmwareCharHandle,
                                    lastFirmwareValue.size(),
                                    const_cast<uint8_t *>(reinterpret_cast<const uint8_t *>(lastFirmwareValue.c_str())),
                                    false);
    }
}

void BLEManager::sendOtaStatus(const std::string &status)
{
    lastOtaStatus = status;
    ESP_LOGI(TAG, "sendOtaStatus: '%s' (connected=%d, notifyEnabled=%d, handle=%d)",
             status.c_str(), deviceConnected, otaStatusNotifyEnabled, otaHandles.statusCharHandle);

    if (otaHandles.statusCharHandle)
    {
        esp_err_t err = esp_ble_gatts_set_attr_value(otaHandles.statusCharHandle,
                                                     lastOtaStatus.size(),
                                                     reinterpret_cast<const uint8_t *>(lastOtaStatus.c_str()));
        if (err != ESP_OK)
        {
            ESP_LOGE(TAG, "Failed to set OTA status attribute: %s", esp_err_to_name(err));
        }
    }

    if (deviceConnected && otaStatusNotifyEnabled && otaHandles.statusCharHandle)
    {
        esp_err_t err = esp_ble_gatts_send_indicate(otaGattIf, connId,
                                                    otaHandles.statusCharHandle,
                                                    lastOtaStatus.size(),
                                                    const_cast<uint8_t *>(reinterpret_cast<const uint8_t *>(lastOtaStatus.c_str())),
                                                    false);
        if (err != ESP_OK)
        {
            ESP_LOGE(TAG, "Failed to send OTA status notification: %s", esp_err_to_name(err));
        }
        else
        {
            ESP_LOGI(TAG, "OTA status notification sent successfully");
        }
    }
    else
    {
        ESP_LOGW(TAG, "Cannot send OTA status: connected=%d, notifyEnabled=%d, handle=%d",
                 deviceConnected, otaStatusNotifyEnabled, otaHandles.statusCharHandle);
    }
}

void BLEManager::onPiezoHit()
{
    hitTimeMs = millis64();
}

BLECommandData BLEManager::parseCommand(const std::string &value)
{
    BLECommandData result = {BLECommand::UNKNOWN, 0};

    auto parseInt = [](const std::string &text) -> int
    {
        char *end = nullptr;
        long value = std::strtol(text.c_str(), &end, 10);
        if (end == text.c_str())
        {
            return 0;
        }
        if (value < INT_MIN)
            return INT_MIN;
        if (value > INT_MAX)
            return INT_MAX;
        return static_cast<int>(value);
    };

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

    if (value.rfind("color:", 0) == 0 || value == "off")
    {
        result.command = BLECommand::SET_COLOR;
        return result;
    }

    if (value.rfind("r:", 0) == 0)
    {
        result.command = BLECommand::SET_RED;
        result.value = parseInt(value.substr(2));
        return result;
    }
    if (value.rfind("g:", 0) == 0)
    {
        result.command = BLECommand::SET_GREEN;
        result.value = parseInt(value.substr(2));
        return result;
    }
    if (value.rfind("b:", 0) == 0)
    {
        result.command = BLECommand::SET_BRIGHTNESS;
        result.value = parseInt(value.substr(2));
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
    size_t firstComma = rgbStr.find(',');
    size_t secondComma = rgbStr.find(',', firstComma + 1);

    if (firstComma == std::string::npos || secondComma == std::string::npos)
    {
        return false;
    }

    {
        std::string rStr = rgbStr.substr(0, firstComma);
        std::string gStr = rgbStr.substr(firstComma + 1, secondComma - firstComma - 1);
        std::string bStr = rgbStr.substr(secondComma + 1);

        char *end = nullptr;
        long r = std::strtol(rStr.c_str(), &end, 10);
        if (end == rStr.c_str())
            return false;
        end = nullptr;
        long g = std::strtol(gStr.c_str(), &end, 10);
        if (end == gStr.c_str())
            return false;
        end = nullptr;
        long b = std::strtol(bStr.c_str(), &end, 10);
        if (end == bStr.c_str())
            return false;

        red = static_cast<int>(r);
        green = static_cast<int>(g);
        blue = static_cast<int>(b);
    }

    red = clamp_value(red, 0, 255);
    green = clamp_value(green, 0, 255);
    blue = clamp_value(blue, 0, 255);
    return true;
}

void BLEManager::handleRGBCommand(const std::string &value)
{
    std::string rgbStr = value.substr(4);
    int red, green, blue;
    if (parseRgbValues(rgbStr, red, green, blue) && ledController)
    {
        ledController->setRgbColor(red, green, blue);
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
    }
    else if (value.rfind("color:", 0) == 0)
    {
        ledController->setPredefinedColor(value.substr(6));
    }
}

void BLEManager::handleBlinkColorCommand(const std::string &value)
{
    if (!ledController)
    {
        return;
    }

    int red, green, blue;
    if (parseRgbValues(value.substr(9), red, green, blue))
    {
        ledController->blinkColor(red, green, blue);
    }
}

void BLEManager::handleStartColorCommand(const std::string &value)
{
    if (!ledController)
    {
        return;
    }

    int red, green, blue;
    if (parseRgbValues(value.substr(6), red, green, blue))
    {
        ledController->setRgbColor(red, green, blue);
    }
}

void BLEManager::startLedStatusTask()
{
    if (ledStatusTaskHandle == nullptr)
    {
        xTaskCreatePinnedToCore(
            ledStatusTask,
            "BLELedStatus",
            4096,
            this,
            1,
            &ledStatusTaskHandle,
            app_cpu());
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

void BLEManager::scheduleInitialInfo()
{
    xTaskCreate(initialDeviceInfoTask, "InitDeviceInfo", 4096, this, 1, nullptr);
}

void BLEManager::logBLEConnectionInfo()
{
    if (deviceConnected)
    {
        ESP_LOGI(TAG, "BLE connected (conn_id=%u)", connId);
    }
    else
    {
        ESP_LOGI(TAG, "BLE not connected");
    }
}

void BLEManager::logBLEPHYInfo()
{
    ESP_LOGI(TAG, "BLE PHY: 1M, 2M, Coded (S=2, S=8) supported");
}
