#include "BLEManager.h"
#include "LEDController.h"
#include "PowerManager.h"
#include "OTAManager.h"

BLEManager::BLEManager(LEDController *leds, PowerManager *power, OTAManager *ota)
    : ledController(leds), powerManager(power), otaManager(ota), pServer(nullptr),
      piezoService(nullptr), piezoCharacteristic(nullptr), batteryCharacteristic(nullptr),
      firmwareVersionCharacteristic(nullptr), deviceConnected(false), isInitialized(false),
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

    // Configure advertising
    BLEAdvertising *pAdvertising = BLEDevice::getAdvertising();
    pAdvertising->addServiceUUID(piezoService->getUUID());
    pAdvertising->addServiceUUID(OTA_SERVICE_UUID);
    pAdvertising->setScanResponse(true);

    // Set device name in advertising data
    BLEAdvertisementData advertisementData;
    advertisementData.setName(deviceName.c_str());
    pAdvertising->setAdvertisementData(advertisementData);

    // Start advertising
    BLEDevice::startAdvertising();

    Serial.println("Waiting for a client connection to notify...");

    isInitialized = true;
    return true;
}

void BLEManager::onPiezoHit()
{
    hitTime = millis();
    if (ledController)
    {
        ledController->setRgbColor(255, 0, 0); // Red for active target
    }
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
    else if (value == "sleep")
        result.command = BLECommand::SLEEP;
    else if (value == "game1")
        result.command = BLECommand::GAME1;
    else if (value == "off")
    {
        result.command = BLECommand::SET_COLOR;
        result.value = 0; // We'll handle "off" in the color handler
    }

    return result;
}

void BLEManager::handleRGBCommand(const std::string &value)
{
    // Parse RGB values from "rgb:r,g,b" format
    std::string rgbStr = value.substr(4); // Remove "rgb:" prefix
    size_t firstComma = rgbStr.find(',');
    size_t secondComma = rgbStr.find(',', firstComma + 1);

    if (firstComma != std::string::npos && secondComma != std::string::npos)
    {
        int red = std::stoi(rgbStr.substr(0, firstComma));
        int green = std::stoi(rgbStr.substr(firstComma + 1, secondComma - firstComma - 1));
        int blue = std::stoi(rgbStr.substr(secondComma + 1));

        // Constrain values to 0-255 range
        red = constrain(red, 0, 255);
        green = constrain(green, 0, 255);
        blue = constrain(blue, 0, 255);

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
    size_t firstComma = rgbStr.find(',');
    size_t secondComma = rgbStr.find(',', firstComma + 1);

    if (firstComma != std::string::npos && secondComma != std::string::npos)
    {
        int red = std::stoi(rgbStr.substr(0, firstComma));
        int green = std::stoi(rgbStr.substr(firstComma + 1, secondComma - firstComma - 1));
        int blue = std::stoi(rgbStr.substr(secondComma + 1));

        // Constrain values to 0-255 range
        red = constrain(red, 0, 255);
        green = constrain(green, 0, 255);
        blue = constrain(blue, 0, 255);

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
    if (bleManager->ledController)
    {
        bleManager->ledController->blinkColor(255, 255, 255); // White for connection confirmation
    }
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
    pServer->getAdvertising()->start();
}
