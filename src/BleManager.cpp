#include "BleManager.h"
#include "LedController.h"
#include "PowerManager.h"
#include "HardwareInterface.h"

// BLE UUIDs from main.cpp
#define SERVICE_UUID "91bad492-b950-4226-aa2b-4ede9fa42fff"
#define CHARACTERISTIC "cba1d466-344c-4be3-ab3f-189f80dd75ff"
#define BATTERY_CHARACTERISTIC "cba1d466-344c-4be3-ab3f-189f80dd76ff"
#define FIRMWARE_VERSION_CHARACTERISTIC "cba1d466-344c-4be3-ab3f-189f80dd77ff"

extern const char *FIRMWARE_VERSION;
extern LedController ledController;
extern PowerManager powerManager;
extern HardwareInterface hardwareInterface;

// Task function for initial device info
void initialDeviceInfoTask(void *pvParameters)
{
    vTaskDelay(2000 / portTICK_PERIOD_MS);

    int percentage = powerManager.getBatteryPercentage();
    bleManager.sendBatteryLevel(percentage);
    bleManager.sendFirmwareVersion();

    vTaskDelete(NULL);
}

// Global instance
BleManager bleManager;

BleManager::BleManager() : pServer(nullptr),
                           piezoService(nullptr),
                           piezoCharacteristic(nullptr),
                           batteryCharacteristic(nullptr),
                           firmwareVersionCharacteristic(nullptr),
                           deviceConnected(false),
                           lastDisconnectTime(0)
{
}

void BleManager::init(const std::string &deviceName)
{
    // Initialize BLE Device
    BLEDevice::init(deviceName.c_str());

    // Create the BLE Server
    pServer = BLEDevice::createServer();
    pServer->setCallbacks(new MyServerCallbacks(this));

    // Setup characteristics
    setupCharacteristics();

    // Configure advertising
    BLEAdvertising *pAdvertising = BLEDevice::getAdvertising();
    pAdvertising->addServiceUUID(piezoService->getUUID());
    pAdvertising->setScanResponse(true);

    // Set device name in advertising data
    BLEAdvertisementData advertisementData;
    advertisementData.setName(deviceName.c_str());
    pAdvertising->setAdvertisementData(advertisementData);

    // Initialize disconnect timer
    lastDisconnectTime = millis();
}

void BleManager::setupCharacteristics()
{
    // Create the BLE Service
    piezoService = pServer->createService(SERVICE_UUID);

    // Create characteristics
    piezoCharacteristic = piezoService->createCharacteristic(
        CHARACTERISTIC,
        BLECharacteristic::PROPERTY_NOTIFY | BLECharacteristic::PROPERTY_WRITE);

    batteryCharacteristic = piezoService->createCharacteristic(
        BATTERY_CHARACTERISTIC,
        BLECharacteristic::PROPERTY_NOTIFY | BLECharacteristic::PROPERTY_READ);

    firmwareVersionCharacteristic = piezoService->createCharacteristic(
        FIRMWARE_VERSION_CHARACTERISTIC,
        BLECharacteristic::PROPERTY_NOTIFY | BLECharacteristic::PROPERTY_READ);

    // Add descriptors and callbacks
    piezoCharacteristic->addDescriptor(new BLE2902());
    batteryCharacteristic->addDescriptor(new BLE2902());
    firmwareVersionCharacteristic->addDescriptor(new BLE2902());

    piezoCharacteristic->setCallbacks(new WriteCallbacks(this));
    firmwareVersionCharacteristic->setValue(FIRMWARE_VERSION);

    // Start the service
    piezoService->start();
}

void BleManager::startAdvertising()
{
    BLEDevice::startAdvertising();
    Serial.println("BLE advertising started");
}

void BleManager::sendPiezoData(int piezoValue, unsigned long hitTime)
{
    if (deviceConnected && piezoCharacteristic)
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

void BleManager::sendBatteryLevel(int percentage)
{
    if (deviceConnected && batteryCharacteristic)
    {
        String valueToSend = String(percentage);
        batteryCharacteristic->setValue(valueToSend.c_str());
        batteryCharacteristic->notify();
        Serial.println("Battery level sent: " + valueToSend);
    }
    else
    {
        Serial.println("Device not connected, cannot send battery notification.");
    }
}

void BleManager::sendFirmwareVersion()
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

BLECommandData BleManager::parseCommand(const std::string &value)
{
    BLECommandData result = {BLECommand::UNKNOWN, 0};

    // Check if the command starts with "b:"
    if (value.substr(0, 2) == "b:")
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
    else if (value == "blink")
        result.command = BLECommand::BLINK;

    return result;
}

// Server callbacks implementation
void MyServerCallbacks::onConnect(BLEServer *pServer)
{
    Serial.println("Device connected");
    bleManager->deviceConnected = true;
    bleManager->lastDisconnectTime = 0;

    ledController.blinkLeds();
    vTaskDelay(1000 / portTICK_PERIOD_MS);

    // Create initial device info task
    xTaskCreate(initialDeviceInfoTask, "InitDeviceInfo", 2048, NULL, 1, NULL);
}

void MyServerCallbacks::onDisconnect(BLEServer *pServer)
{
    Serial.println("Device disconnected");
    bleManager->deviceConnected = false;
    bleManager->lastDisconnectTime = millis();
    pServer->getAdvertising()->start();
}

// Write callbacks implementation
void WriteCallbacks::onWrite(BLECharacteristic *pCharacteristic)
{
    std::string value = pCharacteristic->getValue();
    Serial.print("Received value: ");
    Serial.println(value.c_str());

    BLECommandData cmdData = BleManager::parseCommand(value);

    switch (cmdData.command)
    {
    case BLECommand::START:
        hardwareInterface.onStartCommand();
        break;

    case BLECommand::SLEEP:
        powerManager.goToDeepSleep();
        break;

    case BLECommand::BLINK:
        ledController.blinkLeds();
        break;

    case BLECommand::SET_BRIGHTNESS:
        ledController.setBrightness(constrain(cmdData.value, 0, 255));
        break;

    default:
        break;
    }
}