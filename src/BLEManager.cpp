#include "BLEManager.h"
#include "LEDController.h"
#include "PowerManager.h"
#include "OTAManager.h"
#include "GameSettings.h"
#include "PiezoSensor.h"
#include "CalibrationStore.h"
#include "LedModuleStore.h"
#include <string.h>
#ifdef BOARD_ESP32S3
#include <string.h>
#include <esp_gap_ble_api.h>
#endif

BLEManager::BLEManager(LEDController *leds, PowerManager *power, OTAManager *ota, PiezoSensor *sensor)
    : ledController(leds), powerManager(power), otaManager(ota), piezoSensor(sensor), pServer(nullptr),
      piezoService(nullptr), piezoCharacteristic(nullptr), batteryCharacteristic(nullptr),
      firmwareVersionCharacteristic(nullptr), calibrationCharacteristic(nullptr),
#ifdef BOARD_ESP32S3
      pMultiAdvertising(nullptr),
#endif
      deviceConnected(false), isInitialized(false),
      hitTime(0), ledStatusTaskHandle(nullptr), commandQueue(nullptr), commandTaskHandle(nullptr)
{
}

bool BLEManager::begin()
{
    if (isInitialized)
    {
        return true;
    }

    // The consumer exists before the first write can arrive.
    commandQueue = xQueueCreate(COMMAND_QUEUE_DEPTH, sizeof(QueuedCommand));
    if (commandQueue == nullptr ||
        xTaskCreatePinnedToCore(commandTask, "BLECommands", COMMAND_TASK_STACK, this, 3,
                                &commandTaskHandle, APP_CPU_NUM) != pdPASS)
    {
        Serial.println("Failed to start BLE command task");
        return false;
    }

    // Initialize BLE
    String deviceName = BLE_SERVER_NAME;
    BLEDevice::init(deviceName.c_str());

#ifdef BOARD_ESP32S3
    if (BLE_CODED_PHY_PREFERRED)
    {
        esp_err_t phyErr = esp_ble_gap_set_prefered_default_phy(
            ESP_BLE_GAP_PHY_CODED_PREF_MASK | ESP_BLE_GAP_PHY_1M_PREF_MASK,
            ESP_BLE_GAP_PHY_CODED_PREF_MASK | ESP_BLE_GAP_PHY_1M_PREF_MASK);
        if (phyErr == ESP_OK)
        {
            Serial.println("PHY preferences set: Coded PHY preferred, 1M PHY fallback");
        }
        else
        {
            Serial.print("Failed to set PHY preferences: ");
            Serial.println(phyErr);
        }
    }

    registerBLEEventCallback();
#endif

    // Create the BLE Server
    pServer = BLEDevice::createServer();
    pServer->setCallbacks(new ServerCallbacks(this));

    // Create the BLE Service and Characteristics
    piezoService = pServer->createService(SERVICE_UUID);

    piezoCharacteristic = piezoService->createCharacteristic(
        CHARACTERISTIC,
        BLECharacteristic::PROPERTY_NOTIFY | BLECharacteristic::PROPERTY_WRITE);

    batteryCharacteristic = piezoService->createCharacteristic(
        BATTERY_CHARACTERISTIC,
        BLECharacteristic::PROPERTY_NOTIFY | BLECharacteristic::PROPERTY_READ);

    firmwareVersionCharacteristic = piezoService->createCharacteristic(
        FIRMWARE_VERSION_CHARACTERISTIC,
        BLECharacteristic::PROPERTY_NOTIFY | BLECharacteristic::PROPERTY_READ);

    calibrationCharacteristic = piezoService->createCharacteristic(
        CALIBRATION_CHARACTERISTIC,
        BLECharacteristic::PROPERTY_NOTIFY | BLECharacteristic::PROPERTY_READ);

    // Add Descriptors and Callbacks
    piezoCharacteristic->addDescriptor(new BLE2902());
    batteryCharacteristic->addDescriptor(new BLE2902());
    firmwareVersionCharacteristic->addDescriptor(new BLE2902());
    calibrationCharacteristic->addDescriptor(new BLE2902());
    piezoCharacteristic->setCallbacks(new WriteCallbacks(this));
    firmwareVersionCharacteristic->setValue(FIRMWARE_VERSION);

    // Start the service
    piezoService->start();

    // Initialize OTA service
    if (otaManager)
    {
        otaManager->begin(pServer);
    }

#ifdef BOARD_ESP32S3
    // Try extended advertising with Coded PHY for long range
    if (BLE_LONG_RANGE_ENABLED && BLE_CODED_PHY_PREFERRED)
    {
        pMultiAdvertising = new BLEMultiAdvertising(1);

        esp_ble_gap_ext_adv_params_t ext_adv_params = {
            .type = ESP_BLE_GAP_SET_EXT_ADV_PROP_CONNECTABLE,
            .interval_min = 0x30,
            .interval_max = 0x60,
            .channel_map = ADV_CHNL_ALL,
            .own_addr_type = BLE_ADDR_TYPE_PUBLIC,
            .filter_policy = ADV_FILTER_ALLOW_SCAN_ANY_CON_ANY,
            .primary_phy = ESP_BLE_GAP_PHY_1M,
            .max_skip = 0,
            .secondary_phy = ESP_BLE_GAP_PHY_CODED,
            .sid = 0,
            .scan_req_notif = false,
        };

        if (!pMultiAdvertising->setAdvertisingParams(0, &ext_adv_params))
        {
            Serial.println("Failed to set extended advertising parameters");
            delete pMultiAdvertising;
            pMultiAdvertising = nullptr;
        }
        else
        {
            BLEUUID piezoUuid(SERVICE_UUID);
            BLEUUID otaUuid(OTA_SERVICE_UUID);

            piezoUuid = piezoUuid.to128();
            otaUuid = otaUuid.to128();

            uint8_t advData[64];
            uint16_t advLen = 0;

            // Flags
            advData[advLen++] = 0x02;
            advData[advLen++] = 0x01;
            advData[advLen++] = 0x06;

            // Device name
            uint8_t nameLen = strlen(deviceName.c_str());
            if (nameLen > 0 && advLen + nameLen + 2 <= sizeof(advData))
            {
                advData[advLen++] = nameLen + 1;
                advData[advLen++] = 0x09;
                memcpy(&advData[advLen], deviceName.c_str(), nameLen);
                advLen += nameLen;
            }

            // Piezo service UUID
            if (advLen + 18 <= sizeof(advData))
            {
                advData[advLen++] = 17;
                advData[advLen++] = 0x06;
                memcpy(&advData[advLen], piezoUuid.getNative()->uuid.uuid128, 16);
                advLen += 16;
            }

            // Scan response: OTA service UUID
            uint8_t scanRspData[64];
            uint16_t scanRspLen = 0;

            if (scanRspLen + 18 <= sizeof(scanRspData))
            {
                scanRspData[scanRspLen++] = 17;
                scanRspData[scanRspLen++] = 0x06;
                memcpy(&scanRspData[scanRspLen], otaUuid.getNative()->uuid.uuid128, 16);
                scanRspLen += 16;
            }

            pMultiAdvertising->setAdvertisingData(0, advLen, advData);
            pMultiAdvertising->setScanRspData(0, scanRspLen, scanRspData);
            pMultiAdvertising->setDuration(0, 0, 0);

            if (!pMultiAdvertising->start())
            {
                Serial.println("Failed to start extended advertising");
                delete pMultiAdvertising;
                pMultiAdvertising = nullptr;
            }
            else
            {
                Serial.println("Extended advertising started: 1M PHY (primary), Coded PHY (secondary)");
                Serial.println("Device should be visible to standard BLE scanners");
                Serial.println("Long range available when connecting with Coded PHY support");
            }
        }
    }

    // Always start legacy advertising for compatibility with standard BLE scanners.
    // On ESP32-S3 this runs alongside extended advertising, so devices that support
    // Coded PHY get long range, while standard scanners still see the device.
    if (!pMultiAdvertising)
    {
        Serial.println("Extended advertising not active, using legacy only");
    }
    else
    {
        Serial.println("Starting legacy advertising alongside extended for compatibility");
    }
#endif
    {
        BLEAdvertising *pAdvertising = BLEDevice::getAdvertising();
        pAdvertising->addServiceUUID(piezoService->getUUID());
        pAdvertising->addServiceUUID(OTA_SERVICE_UUID);
        pAdvertising->setScanResponse(true);

        BLEAdvertisementData advertisementData;
        advertisementData.setName(deviceName.c_str());
        pAdvertising->setAdvertisementData(advertisementData);

        BLEDevice::startAdvertising();
    }

    Serial.println("Waiting for a client connection to notify...");

#ifdef BOARD_ESP32S3
    Serial.println("=== ESP32-S3 BLE INITIALIZATION COMPLETE ===");
    logBLEPHYInfo();
    if (pMultiAdvertising)
    {
        Serial.println("BLE Long Range (Coded PHY) advertising ACTIVE");
    }
    else
    {
        Serial.println("BLE Long Range (Coded PHY) not available, using standard advertising");
    }
    Serial.println("Send 'ble_status' or 'ble_phy' commands to check status");
#endif

    isInitialized = true;
    return true;
}

void BLEManager::onPiezoHit()
{
    hitTime = millis();
}

void BLEManager::sendPiezoValue(int piezoValue)
{
    if (deviceConnected)
    {
        uint8_t data[4];
        uint16_t timeInHundredths = (millis() - hitTime) / 10;
        data[0] = piezoValue & 0xFF;
        data[1] = piezoValue >> 8;
        data[2] = timeInHundredths & 0xFF;
        data[3] = timeInHundredths >> 8;
        piezoCharacteristic->setValue(data, 4);
        piezoCharacteristic->notify();
        Serial.print("Sent piezo value: ");
        Serial.println(piezoValue);
    }
    else
    {
        Serial.println("Device not connected, cannot send notification.");
    }
}

void BLEManager::sendBatteryLevel(int percentage)
{
    if (deviceConnected)
    {
        String valueToSend = String(percentage);
        batteryCharacteristic->setValue(valueToSend.c_str());
        batteryCharacteristic->notify();
        Serial.println("Battery level sent: " + valueToSend);
    }
    else
    {
        Serial.println("Device not connected, cannot send notification.");
    }
}

void BLEManager::sendFirmwareVersion()
{
    if (deviceConnected && firmwareVersionCharacteristic)
    {
        firmwareVersionCharacteristic->setValue(FIRMWARE_VERSION);
        firmwareVersionCharacteristic->notify();
        Serial.println("Firmware version sent: " + String(FIRMWARE_VERSION));
    }
    else
    {
        Serial.println("Device not connected or firmware version characteristic not available.");
    }
}

void BLEManager::sendCalibrationConfig()
{
    if (!calibrationCharacteristic || !piezoSensor)
    {
        return;
    }

    String payload = "calib:" + String(piezoSensor->getThreshold()) + ":" + String(piezoSensor->getDebounceMs());
    calibrationCharacteristic->setValue(payload.c_str());
    if (deviceConnected)
    {
        calibrationCharacteristic->notify();
    }
    Serial.println("Calibration config sent: " + payload);
}

void BLEManager::sendPeakValue(uint16_t peakValue)
{
    if (!calibrationCharacteristic || !deviceConnected)
    {
        return;
    }

    String payload = "peak:" + String(peakValue);
    calibrationCharacteristic->setValue(payload.c_str());
    calibrationCharacteristic->notify();
    Serial.println("Peak sent: " + payload);
}

void BLEManager::sendLedModuleState()
{
    // Sent on the calibration characteristic so no new UUID is needed. The released app's
    // dispatcher there only acts on "calib:" and "peak:" and logs anything else, so this payload
    // never reaches CalibrationConfig.parse in an old app.
    if (!calibrationCharacteristic || !ledController || !deviceConnected)
    {
        return;
    }

    String payload = String("ledswap:") + (ledController->isRedGreenSwapped() ? "1" : "0");
    calibrationCharacteristic->setValue(payload.c_str());
    calibrationCharacteristic->notify();
    Serial.println("LED module state sent: " + payload);
}

BLECommandData BLEManager::parseCommand(const std::string &value)
{
    return BleCommandParser::parse(value);
}

bool BLEManager::parseRgbValues(const std::string &rgbStr, int &red, int &green, int &blue)
{
    return BleCommandParser::parseRgb(rgbStr, red, green, blue);
}

void BLEManager::handleRGBCommand(const std::string &value)
{
    std::string rgbStr = value.substr(4);
    int red, green, blue;

    if (parseRgbValues(rgbStr, red, green, blue))
    {
        if (ledController)
        {
            ledController->setRgbColor(red, green, blue);
        }
    }
    else
    {
        Serial.println("Invalid RGB format. Use: rgb:r,g,b");
    }
}

void BLEManager::handleColorCommand(const std::string &value)
{
    if (value == "off")
    {
        if (ledController)
        {
            ledController->setPredefinedColor("off");
        }
    }
    else if (value.substr(0, 6) == "color:")
    {
        std::string colorName = value.substr(6);
        if (ledController)
        {
            ledController->setPredefinedColor(String(colorName.c_str()));
        }
    }
    else
    {
        Serial.println("Invalid color command. Use: color:red/green/blue/white/yellow/magenta/cyan/off");
    }
}

void BLEManager::handleBlinkColorCommand(const std::string &value)
{
    std::string rgbStr = value.substr(9);
    int red, green, blue;

    if (parseRgbValues(rgbStr, red, green, blue))
    {
        if (ledController)
        {
            ledController->blinkColor(red, green, blue);
        }
    }
    else
    {
        Serial.println("Invalid blink color format. Use: blinkrgb:r,g,b");
    }
}

void BLEManager::handleStartColorCommand(const std::string &value)
{
    std::string rgbStr = value.substr(6);
    int red, green, blue;

    if (parseRgbValues(rgbStr, red, green, blue))
    {
        if (ledController)
        {
            ledController->setRgbColor(red, green, blue);
        }
    }
    else
    {
        Serial.println("Invalid start color format. Use: start:r,g,b");
    }
}

void BLEManager::handleCalibSetCommand(const std::string &value)
{
    // Format: calib:set:<threshold>:<debounceMs>
    std::string payload = value.substr(10);
    size_t sep = payload.find(':');
    if (sep == std::string::npos)
    {
        Serial.println("Invalid calib:set format. Use: calib:set:<threshold>:<debounceMs>");
        return;
    }

    int thr = 0;
    int deb = 0;
    if (!BleCommandParser::parseInt(payload.substr(0, sep), thr) ||
        !BleCommandParser::parseInt(payload.substr(sep + 1), deb))
    {
        Serial.println("Failed to parse calib:set values");
        return;
    }

    if (piezoSensor)
    {
        piezoSensor->setThreshold(static_cast<uint16_t>(thr));
        piezoSensor->setDebounceMs(static_cast<uint16_t>(deb));
        CalibrationStore::saveThreshold(piezoSensor->getThreshold());
        CalibrationStore::saveDebounceMs(piezoSensor->getDebounceMs());
    }
    sendCalibrationConfig();
}

void BLEManager::handleCalibResetCommand()
{
    CalibrationStore::resetToDefaults();
    if (piezoSensor)
    {
        piezoSensor->setThreshold(CalibrationStore::loadThreshold());
        piezoSensor->setDebounceMs(CalibrationStore::loadDebounceMs());
    }
    sendCalibrationConfig();
}

void BLEManager::handlePeakStartCommand()
{
    if (piezoSensor)
    {
        piezoSensor->startMeasurement();
    }
    if (ledController)
    {
        // Solid yellow indicates measurement mode is active
        ledController->setRgbColor(255, 200, 0);
    }
    Serial.println("Peak measurement mode: START");
}

void BLEManager::handlePeakStopCommand()
{
    if (piezoSensor)
    {
        piezoSensor->stopMeasurement();
    }
    if (ledController)
    {
        ledController->turnOff();
    }
    Serial.println("Peak measurement mode: STOP");
}

void BLEManager::handleLedSwapSetCommand(bool swapped)
{
    // The setting is applied only once NVS has confirmed it, so what the user sees now is what the
    // target shows after a restart. A failed write leaves the old setting and the reply carries
    // it, which the app shows as a mismatch instead of a silent success.
    if (LedModuleStore::saveRedGreenSwapped(swapped))
    {
        if (ledController)
        {
            ledController->setRedGreenSwapped(swapped);
        }
    }
    sendLedModuleState();
}

void BLEManager::handleLedTestCommand(bool boardRedPin)
{
    if (ledController)
    {
        ledController->setRawTestColor(boardRedPin);
    }
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

// Static task functions
void BLEManager::initialDeviceInfoTask(void *pvParameters)
{
    BLEManager *bleManager = static_cast<BLEManager *>(pvParameters);
    vTaskDelay(2000 / portTICK_PERIOD_MS);

    if (bleManager->powerManager)
    {
        int percentage = bleManager->powerManager->getBatteryPercentage();
        bleManager->sendBatteryLevel(percentage);
    }

    bleManager->sendFirmwareVersion();
    bleManager->sendCalibrationConfig();

    vTaskDelete(NULL);
}

void BLEManager::ledStatusTask(void *pvParameters)
{
    BLEManager *bleManager = static_cast<BLEManager *>(pvParameters);

    for (;;)
    {
        if (!bleManager->deviceConnected)
        {
            if (bleManager->ledController)
            {
                bleManager->ledController->setRgbColor(0, 0, 0);
                vTaskDelay(1000 / portTICK_PERIOD_MS);
                bleManager->ledController->setRgbColor(0, 0, 255);
                vTaskDelay(1000 / portTICK_PERIOD_MS);
                Serial.println("Device not connected - blinking blue");
            }
        }
        else
        {
            vTaskDelay(100 / portTICK_PERIOD_MS);
        }
    }
}

// WriteCallbacks Implementation
void BLEManager::WriteCallbacks::onWrite(BLECharacteristic *pCharacteristic)
{
    bleManager->enqueueCommand(pCharacteristic->getValue());
}

void BLEManager::enqueueCommand(const std::string &value)
{
    // Runs in the BTC task: no printf here except on the failure paths.
    if (value.size() >= COMMAND_MAX_LEN)
    {
        Serial.print("COMMAND DROPPED (too long): ");
        Serial.println(value.substr(0, COMMAND_MAX_LEN).c_str());
        return;
    }

    QueuedCommand item;
    item.length = static_cast<uint8_t>(value.size());
    memcpy(item.data, value.data(), value.size());

    if (xQueueSend(commandQueue, &item, 0) != pdTRUE)
    {
        Serial.print("COMMAND DROPPED (queue full): ");
        Serial.println(value.c_str());
    }
}

void BLEManager::commandTask(void *pvParameters)
{
    BLEManager *bleManager = static_cast<BLEManager *>(pvParameters);
    QueuedCommand item;

    for (;;)
    {
        if (xQueueReceive(bleManager->commandQueue, &item, portMAX_DELAY) != pdTRUE)
        {
            continue;
        }

        bleManager->executeCommand(std::string(item.data, item.length));

        UBaseType_t freeBytes = uxTaskGetStackHighWaterMark(nullptr);
        if (freeBytes < COMMAND_STACK_WARN_BYTES)
        {
            Serial.printf("WARNING: command task stack low, %u bytes left\n", (unsigned)freeBytes);
        }
    }
}

void BLEManager::executeCommand(const std::string &value)
{
    Serial.print("Received value: ");
    Serial.println(value.c_str());

    BLECommandData cmdData = parseCommand(value);

    switch (cmdData.command)
    {
    case BLECommand::START:
        onPiezoHit();
        if (value.substr(0, 6) == "start:")
        {
            handleStartColorCommand(value);
        }
        break;

    case BLECommand::SLEEP:
        if (powerManager)
        {
            powerManager->goToDeepSleep(SleepReason::Command);
        }
        break;

    case BLECommand::SET_BRIGHTNESS:
        if (ledController)
        {
            ledController->setBrightness(constrain(cmdData.value, 0, 255));
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
            ledController->setRedChannel(constrain(cmdData.value, 0, 255));
        }
        break;

    case BLECommand::SET_GREEN:
        if (ledController)
        {
            ledController->setGreenChannel(constrain(cmdData.value, 0, 255));
        }
        break;

    case BLECommand::SET_BLUE:
        if (ledController)
        {
            ledController->setBlueChannel(constrain(cmdData.value, 0, 255));
        }
        break;

    case BLECommand::SET_AUTO_OFF:
        GameSettings::setAutoLedOffEnabled(cmdData.value != 0);
        break;

    case BLECommand::CALIB_GET:
        sendCalibrationConfig();
        break;

    case BLECommand::CALIB_SET:
        handleCalibSetCommand(value);
        break;

    case BLECommand::CALIB_RESET:
        handleCalibResetCommand();
        break;

    case BLECommand::PEAK_START:
        // Before the queue the disconnect handler could never run ahead of a pending write; now it
        // can, and starting measurement after it would leave the sensor deaf to hits until the
        // next peak:stop. Checked again after the start: onDisconnect (BTC task, other core) clears
        // deviceConnected before it looks at isInMeasurement(), so a disconnect landing between the
        // first check and the start is either seen here or sees the measurement itself.
        if (deviceConnected)
        {
            handlePeakStartCommand();
            if (!deviceConnected)
            {
                handlePeakStopCommand();
            }
        }
        break;

    case BLECommand::PEAK_STOP:
        handlePeakStopCommand();
        break;

    case BLECommand::LED_SWAP_GET:
        sendLedModuleState();
        break;

    case BLECommand::LED_SWAP_SET:
        handleLedSwapSetCommand(cmdData.value != 0);
        break;

    case BLECommand::LED_TEST:
        handleLedTestCommand(cmdData.value != 0);
        break;

#ifdef BOARD_ESP32S3
    case BLECommand::BLE_STATUS:
        logBLEConnectionInfo();
        break;

    case BLECommand::BLE_PHY_INFO:
        logBLEPHYInfo();
        break;
#endif

    default:
        Serial.println("Unknown command received");
        break;
    }
}

// ServerCallbacks Implementation
void BLEManager::ServerCallbacks::onConnect(BLEServer *pServer)
{
    Serial.println("Device connected");
    bleManager->deviceConnected = true;
    if (bleManager->powerManager)
    {
        bleManager->powerManager->setConnected(true);
    }

    pServer->getAdvertising()->stop();
#ifdef BOARD_ESP32S3
    if (bleManager->pMultiAdvertising)
    {
        uint8_t extAdvInst[] = {0};
        bleManager->pMultiAdvertising->stop(1, extAdvInst);
        Serial.println("Advertising stopped while connected (legacy + extended)");
    }
#endif
    if (bleManager->ledController)
    {
        bleManager->ledController->blinkColor(255, 255, 255);
    }

#ifdef BOARD_ESP32S3
    bleManager->logBLEConnectionInfo();
    bleManager->logBLEPHYInfo();
#endif

    vTaskDelay(1000 / portTICK_PERIOD_MS);
    xTaskCreate(initialDeviceInfoTask, "InitDeviceInfo", 8192, bleManager, 1, NULL);
}

void BLEManager::ServerCallbacks::onDisconnect(BLEServer *pServer)
{
    Serial.println("Device disconnected");
    bleManager->deviceConnected = false;
    if (bleManager->powerManager)
    {
        bleManager->powerManager->setConnected(false);
    }
    // Safety: drop out of measurement mode if active so the cel doesn't
    // stay yellow / non-game forever after the phone walks away.
    if (bleManager->piezoSensor && bleManager->piezoSensor->isInMeasurement())
    {
        bleManager->piezoSensor->stopMeasurement();
        if (bleManager->ledController)
        {
            bleManager->ledController->turnOff();
        }
    }
#ifdef BOARD_ESP32S3
    // Restart extended advertising if available
    if (bleManager->pMultiAdvertising)
    {
        bleManager->pMultiAdvertising->start(1, 0);
    }
#endif
    // Always restart legacy advertising for compatibility
    pServer->getAdvertising()->start();
}

void BLEManager::ServerCallbacks::onDisconnect(BLEServer *pServer, esp_ble_gatts_cb_param_t *param)
{
    Serial.print("Disconnect reason: 0x");
    Serial.println(param->disconnect.reason, HEX);
}

#ifdef BOARD_ESP32S3
// BLE monitoring and diagnostics implementation
void BLEManager::registerBLEEventCallback()
{
    Serial.println("BLE event monitoring initialized");
}

void BLEManager::logBLEConnectionInfo()
{
    if (deviceConnected)
    {
        Serial.println("=== CURRENT BLE CONNECTION INFO ===");
        Serial.println("Device is connected");
        Serial.println("BLE 5.0 features enabled");
        Serial.println("Long Range (Coded PHY) support available");
    }
    else
    {
        Serial.println("No BLE connection active");
    }
}

void BLEManager::logBLEPHYInfo()
{
    Serial.println("=== BLE PHY CAPABILITIES ===");
    Serial.println("ESP32-S3 BLE 5.0 Features:");
    Serial.println("- 1M PHY (Standard)");
    Serial.println("- 2M PHY (High Speed)");
    Serial.println("- Coded PHY S=2 (Long Range)");
    Serial.println("- Coded PHY S=8 (Maximum Range)");
    Serial.println("Long Range provides up to 4x range improvement");
}
#endif
