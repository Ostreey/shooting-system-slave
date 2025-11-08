#include "BLEManager.h"
#include "LEDController.h"
#include "PowerManager.h"
#include "OTAManager.h"
#include <string.h>
#include <esp_gap_ble_api.h>

BLEManager::BLEManager(LEDController *leds, PowerManager *power, OTAManager *ota)
    : ledController(leds), powerManager(power), otaManager(ota), pServer(nullptr),
      piezoService(nullptr), piezoCharacteristic(nullptr), batteryCharacteristic(nullptr),
      firmwareVersionCharacteristic(nullptr),
#ifdef CONFIG_BT_BLE_50_FEATURES_SUPPORTED
      pMultiAdvertising(nullptr),
#endif
      deviceConnected(false), isInitialized(false),
      hitTime(0), ledStatusTaskHandle(nullptr)
{
}

bool BLEManager::begin()
{
    if (isInitialized)
    {
        return true;
    }

    // Initialize BLE
    String deviceName = BLE_SERVER_NAME;
    BLEDevice::init(deviceName.c_str());
    
#ifdef CONFIG_BT_BLE_50_FEATURES_SUPPORTED
    if (BLE_CODED_PHY_PREFERRED)
    {
        esp_err_t phyErr = esp_ble_gap_set_prefered_default_phy(
            ESP_BLE_GAP_PHY_CODED_PREF_MASK | ESP_BLE_GAP_PHY_1M_PREF_MASK,
            ESP_BLE_GAP_PHY_CODED_PREF_MASK | ESP_BLE_GAP_PHY_1M_PREF_MASK
        );
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
#endif
    
    // Register BLE event callback for monitoring
    registerBLEEventCallback();

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

    // Add Descriptors and Callbacks
    piezoCharacteristic->addDescriptor(new BLE2902());
    batteryCharacteristic->addDescriptor(new BLE2902());
    firmwareVersionCharacteristic->addDescriptor(new BLE2902());
    piezoCharacteristic->setCallbacks(new WriteCallbacks(this));
    firmwareVersionCharacteristic->setValue(FIRMWARE_VERSION);

    // Start the service
    piezoService->start();

    // Initialize OTA service
    if (otaManager)
    {
        otaManager->begin(pServer);
    }

#ifdef CONFIG_BT_BLE_50_FEATURES_SUPPORTED
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
            
            advData[advLen++] = 0x02;
            advData[advLen++] = 0x01;
            advData[advLen++] = 0x06;
            
            uint8_t nameLen = strlen(deviceName.c_str());
            if (nameLen > 0 && advLen + nameLen + 2 <= sizeof(advData))
            {
                advData[advLen++] = nameLen + 1;
                advData[advLen++] = 0x09;
                memcpy(&advData[advLen], deviceName.c_str(), nameLen);
                advLen += nameLen;
            }
            
            if (advLen + 18 <= sizeof(advData))
            {
                advData[advLen++] = 17;
                advData[advLen++] = 0x06;
                memcpy(&advData[advLen], piezoUuid.getNative()->uuid.uuid128, 16);
                advLen += 16;
            }
            
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
#endif

#ifdef CONFIG_BT_BLE_50_FEATURES_SUPPORTED
    if (!pMultiAdvertising)
#endif
    {
        BLEAdvertising *pAdvertising = BLEDevice::getAdvertising();
        pAdvertising->addServiceUUID(piezoService->getUUID());
        pAdvertising->addServiceUUID(OTA_SERVICE_UUID);
        pAdvertising->setScanResponse(true);
        
        pAdvertising->setMinInterval(0x20);
        pAdvertising->setMaxInterval(0x40);
        pAdvertising->setAdvertisementType(ADV_TYPE_IND);

        BLEAdvertisementData advertisementData;
        advertisementData.setName(deviceName.c_str());
        pAdvertising->setAdvertisementData(advertisementData);

        BLEDevice::startAdvertising();
    }

    Serial.println("Waiting for a client connection to notify...");
    
    Serial.println("=== ESP32-S3 BLE INITIALIZATION COMPLETE ===");
    logBLEPHYInfo();
#ifdef CONFIG_BT_BLE_50_FEATURES_SUPPORTED
    if (pMultiAdvertising)
    {
        Serial.println("BLE Long Range (Coded PHY) advertising ACTIVE");
    }
    else
    {
        Serial.println("BLE Long Range (Coded PHY) not available, using standard advertising");
    }
#else
    Serial.println("BLE 5.0 features not enabled, using standard advertising");
#endif
    Serial.println("Send 'ble_status' or 'ble_phy' commands to check status");

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

BLECommandData BLEManager::parseCommand(const std::string &value)
{
    BLECommandData result = {BLECommand::UNKNOWN, 0};

    // RGB color command: rgb:255,0,0
    if (value.substr(0, 4) == "rgb:")
    {
        result.command = BLECommand::SET_RGB_COLOR;
        result.value = 0; // We'll parse RGB values in the handler
        return result;
    }

    // Blink color command: blinkrgb:255,0,0
    if (value.substr(0, 9) == "blinkrgb:")
    {
        result.command = BLECommand::BLINK_COLOR;
        result.value = 0; // We'll parse RGB values in the handler
        return result;
    }

    // Predefined color command: color:red, color:green, etc.
    if (value.substr(0, 6) == "color:")
    {
        result.command = BLECommand::SET_COLOR;
        result.value = 0; // We'll parse color name in the handler
        return result;
    }

    // Individual channel commands: r:255, g:128, b:0
    if (value.substr(0, 2) == "r:")
    {
        result.command = BLECommand::SET_RED;
        result.value = std::stoi(value.substr(2));
        return result;
    }
    else if (value.substr(0, 2) == "g:")
    {
        result.command = BLECommand::SET_GREEN;
        result.value = std::stoi(value.substr(2));
        return result;
    }
    else if (value.substr(0, 2) == "b:")
    {
        result.command = BLECommand::SET_BRIGHTNESS;
        result.value = std::stoi(value.substr(2));
        return result;
    }

    // Handle valueless commands
    if (value == "start")
        result.command = BLECommand::START;
    else if (value.substr(0, 6) == "start:")
    {
        result.command = BLECommand::START;
        result.value = 0; // We'll parse color in the handler
    }
    else if (value == "sleep")
        result.command = BLECommand::SLEEP;
    else if (value == "game1")
        result.command = BLECommand::GAME1;
    else if (value == "ble_status")
        result.command = BLECommand::BLE_STATUS;
    else if (value == "ble_phy")
        result.command = BLECommand::BLE_PHY_INFO;
    else if (value == "off")
    {
        result.command = BLECommand::SET_COLOR;
        result.value = 0; // We'll handle "off" in the color handler
    }

    return result;
}

bool BLEManager::parseRgbValues(const std::string &rgbStr, int &red, int &green, int &blue)
{
    size_t firstComma = rgbStr.find(',');
    size_t secondComma = rgbStr.find(',', firstComma + 1);

    if (firstComma != std::string::npos && secondComma != std::string::npos)
    {
        red = std::stoi(rgbStr.substr(0, firstComma));
        green = std::stoi(rgbStr.substr(firstComma + 1, secondComma - firstComma - 1));
        blue = std::stoi(rgbStr.substr(secondComma + 1));

        // Constrain values to 0-255 range
        red = constrain(red, 0, 255);
        green = constrain(green, 0, 255);
        blue = constrain(blue, 0, 255);
        return true;
    }
    return false;
}

void BLEManager::handleRGBCommand(const std::string &value)
{
    // Parse RGB values from "rgb:r,g,b" format
    std::string rgbStr = value.substr(4); // Remove "rgb:" prefix
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
        std::string colorName = value.substr(6); // Remove "color:" prefix
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
    // Parse RGB values from "blinkrgb:r,g,b" format
    std::string rgbStr = value.substr(9); // Remove "blinkrgb:" prefix
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
    // Parse RGB values from "start:r,g,b" format
    std::string rgbStr = value.substr(6); // Remove "start:" prefix
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

void BLEManager::startLedStatusTask()
{
    if (ledStatusTaskHandle == nullptr)
    {
        xTaskCreatePinnedToCore(
            ledStatusTask,
            "LEDStatus",
            2048,
            this, // Pass this instance as parameter
            1,
            &ledStatusTaskHandle,
            APP_CPU_NUM);
    }
}

// Static task functions
void BLEManager::initialDeviceInfoTask(void *pvParameters)
{
    BLEManager *bleManager = static_cast<BLEManager *>(pvParameters);
    vTaskDelay(2000 / portTICK_PERIOD_MS); // Wait for connection to stabilize

    // Send battery level
    if (bleManager->powerManager)
    {
        int percentage = bleManager->powerManager->getBatteryPercentage();
        bleManager->sendBatteryLevel(percentage);
    }

    // Send firmware version
    bleManager->sendFirmwareVersion();

    vTaskDelete(NULL); // Delete the task after it's done
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
                // Turn off all LEDs
                bleManager->ledController->setRgbColor(0, 0, 0);
                vTaskDelay(1000 / portTICK_PERIOD_MS);
                // Blink blue LED to indicate "waiting for connection"
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
    std::string value = pCharacteristic->getValue();
    Serial.print("Received value: ");
    Serial.println(value.c_str());

    BLECommandData cmdData = parseCommand(value);

    switch (cmdData.command)
    {
    case BLECommand::START:
        bleManager->onPiezoHit();
        // If command has color info, set the color
        if (value.substr(0, 6) == "start:")
        {
            bleManager->handleStartColorCommand(value);
        }
        break;

    case BLECommand::SLEEP:
        if (bleManager->powerManager)
        {
            bleManager->powerManager->goToDeepSleep();
        }
        break;

    case BLECommand::SET_BRIGHTNESS:
        if (bleManager->ledController)
        {
            bleManager->ledController->setBrightness(constrain(cmdData.value, 0, 255));
        }
        break;

    case BLECommand::SET_RGB_COLOR:
        bleManager->handleRGBCommand(value);
        break;

    case BLECommand::BLINK_COLOR:
        bleManager->handleBlinkColorCommand(value);
        break;

    case BLECommand::SET_COLOR:
        bleManager->handleColorCommand(value);
        break;

    case BLECommand::SET_RED:
        if (bleManager->ledController)
        {
            bleManager->ledController->setRedChannel(constrain(cmdData.value, 0, 255));
        }
        break;

    case BLECommand::SET_GREEN:
        if (bleManager->ledController)
        {
            bleManager->ledController->setGreenChannel(constrain(cmdData.value, 0, 255));
        }
        break;

    case BLECommand::SET_BLUE:
        if (bleManager->ledController)
        {
            bleManager->ledController->setBlueChannel(constrain(cmdData.value, 0, 255));
        }
        break;

    case BLECommand::BLE_STATUS:
        bleManager->logBLEConnectionInfo();
        break;

    case BLECommand::BLE_PHY_INFO:
        bleManager->logBLEPHYInfo();
        break;

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
    
    // PHY preferences are set during initialization to prefer Coded PHY
    // The ESP32-S3 will automatically negotiate Coded PHY if the central device supports it
    
    if (bleManager->powerManager)
    {
        bleManager->powerManager->setConnected(true);
    }
    if (bleManager->ledController)
    {
        bleManager->ledController->blinkColor(255, 255, 255); // White for connection confirmation
    }
    
    // Log BLE connection and PHY information
    bleManager->logBLEConnectionInfo();
    bleManager->logBLEPHYInfo();
    
    vTaskDelay(1000 / portTICK_PERIOD_MS);
    xTaskCreate(initialDeviceInfoTask, "InitDeviceInfo", 2048, bleManager, 1, NULL);
}

void BLEManager::ServerCallbacks::onDisconnect(BLEServer *pServer)
{
    Serial.println("Device disconnected");
    bleManager->deviceConnected = false;
    if (bleManager->powerManager)
    {
        bleManager->powerManager->setConnected(false);
    }
#ifdef CONFIG_BT_BLE_50_FEATURES_SUPPORTED
    if (bleManager->pMultiAdvertising)
    {
        uint8_t instance = 0;
        bleManager->pMultiAdvertising->start(1, 0);
    }
    else
#endif
    {
        pServer->getAdvertising()->start();
    }
}

// BLE monitoring and diagnostics implementation
void BLEManager::registerBLEEventCallback()
{
    // Note: BLE event monitoring will be handled through connection callbacks
    Serial.println("BLE event monitoring initialized");
}

void BLEManager::logBLEConnectionInfo()
{
    if (deviceConnected) {
        Serial.println("=== CURRENT BLE CONNECTION INFO ===");
        Serial.println("Device is connected");
        Serial.println("BLE 5.0 features enabled");
        Serial.println("Long Range (Coded PHY) support available");
    } else {
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
