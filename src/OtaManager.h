#pragma once

#include <BLEDevice.h>
#include <BLEServer.h>
#include <Update.h>
#include <BLE2902.h>
#include <esp_ota_ops.h>
#include <esp_task_wdt.h>

class OtaManager
{
public:
    OtaManager();
    void init(BLEServer *server);
    bool isOtaInProgress() const { return otaInProgress; }
    bool isPendingRestart() const { return pendingRestart; }
    unsigned long getRestartTime() const { return restartTime; }
    void executeRestart();

    static bool isResetAfterOTA();

private:
    // OTA service UUIDs
    static const char *OTA_SERVICE_UUID;
    static const char *OTA_COMMAND_CHAR_UUID;
    static const char *OTA_DATA_CHAR_UUID;
    static const char *OTA_STATUS_CHAR_UUID;

    static const int OTA_CHUNK_SIZE = 256;

    BLEServer *pServer;
    bool otaInProgress;
    size_t otaFirmwareSize;
    size_t otaReceivedSize;
    bool pendingRestart;
    unsigned long restartTime;

    void setupOtaService();

    friend class OTACommandCallbacks;
    friend class OTADataCallbacks;
};

class OTACommandCallbacks : public BLECharacteristicCallbacks
{
public:
    OTACommandCallbacks(OtaManager *manager) : otaManager(manager) {}
    void onWrite(BLECharacteristic *chr) override;

private:
    OtaManager *otaManager;
};

class OTADataCallbacks : public BLECharacteristicCallbacks
{
public:
    OTADataCallbacks(OtaManager *manager) : otaManager(manager) {}
    void onWrite(BLECharacteristic *chr) override;

private:
    OtaManager *otaManager;
};

extern OtaManager otaManager;