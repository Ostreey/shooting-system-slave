#pragma once

#include <Arduino.h>
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>

enum class BLECommand
{
    UNKNOWN,
    START,
    SLEEP,
    BLINK,
    GAME1,
    SET_BRIGHTNESS,
};

struct BLECommandData
{
    BLECommand command;
    int value;
};

class BleManager
{
public:
    BleManager();
    void init(const std::string &deviceName);
    void startAdvertising();
    void sendPiezoData(int piezoValue, unsigned long hitTime);
    void sendBatteryLevel(int percentage);
    void sendFirmwareVersion();

    bool isDeviceConnected() const { return deviceConnected; }
    unsigned long getLastDisconnectTime() const { return lastDisconnectTime; }
    void resetDisconnectTime() { lastDisconnectTime = 0; }

    BLEServer *getServer() { return pServer; }

    static BLECommandData parseCommand(const std::string &value);

private:
    BLEServer *pServer;
    BLEService *piezoService;
    BLECharacteristic *piezoCharacteristic;
    BLECharacteristic *batteryCharacteristic;
    BLECharacteristic *firmwareVersionCharacteristic;

    bool deviceConnected;
    unsigned long lastDisconnectTime;

    void setupCharacteristics();

    friend class MyServerCallbacks;
    friend class WriteCallbacks;
};

class MyServerCallbacks : public BLEServerCallbacks
{
public:
    MyServerCallbacks(BleManager *manager) : bleManager(manager) {}
    void onConnect(BLEServer *pServer) override;
    void onDisconnect(BLEServer *pServer) override;

private:
    BleManager *bleManager;
};

class WriteCallbacks : public BLECharacteristicCallbacks
{
public:
    WriteCallbacks(BleManager *manager) : bleManager(manager) {}
    void onWrite(BLECharacteristic *pCharacteristic) override;

private:
    BleManager *bleManager;
};

extern BleManager bleManager;