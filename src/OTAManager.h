#ifndef OTA_MANAGER_H
#define OTA_MANAGER_H

#include <Arduino.h>
#include <BLEServer.h>
#include <BLEService.h>
#include <BLECharacteristic.h>
#include <BLE2902.h>
#include <Update.h>
#include <esp_ota_ops.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include "Config.h"

class OTAManager
{
private:
    BLEServer *bleServer;
    BLEService *otaService;
    BLECharacteristic *otaCommandChar;
    BLECharacteristic *otaDataChar;
    BLECharacteristic *otaStatusChar;

    bool otaInProgress;
    size_t otaFirmwareSize;
    size_t otaReceivedSize;
    bool pendingRestart;
    unsigned long restartTime;
    bool isInitialized;
    TaskHandle_t otaBeginTaskHandle;
    bool otaBeginInProgress;

    static void otaBeginTask(void *pvParameters);

public:
    OTAManager();

    // Initialization
    bool begin(BLEServer *server);

    // OTA state management
    bool isOTAInProgress() const { return otaInProgress; }
    bool isPendingRestart() const { return pendingRestart; }
    unsigned long getRestartTime() const { return restartTime; }

    // Restart handling
    void checkPendingRestart();

    // Utility functions
    bool isResetAfterOTA();

    // Callback classes as inner classes
    class OTACommandCallbacks : public BLECharacteristicCallbacks
    {
    private:
        OTAManager *otaManager;

    public:
        OTACommandCallbacks(OTAManager *manager) : otaManager(manager) {}
        void onWrite(BLECharacteristic *chr) override;
    };

    class OTADataCallbacks : public BLECharacteristicCallbacks
    {
    private:
        OTAManager *otaManager;

    public:
        OTADataCallbacks(OTAManager *manager) : otaManager(manager) {}
        void onWrite(BLECharacteristic *chr) override;
    };

private:
    // Internal helper methods
    void handleStartCommand(const std::string &cmd);
    void handleEndCommand();
    void processDataChunk(const std::string &chunk);
    void sendOTAStatus(const String &status);
    void scheduleErrorReset();
};

#endif // OTA_MANAGER_H
