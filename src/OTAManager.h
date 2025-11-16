#ifndef OTA_MANAGER_H
#define OTA_MANAGER_H

#include <cstddef>
#include <cstdint>
#include <functional>
#include <string>
#include <esp_ota_ops.h>
#include <esp_system.h>
#include <esp_task_wdt.h>
#include "Config.h"

class OTAManager 
{
private:
    bool otaInProgress;
    size_t otaFirmwareSize;
    size_t otaReceivedSize;
    bool pendingRestart;
    unsigned long restartTime;
    esp_ota_handle_t otaHandle;
    const esp_partition_t* updatePartition;
    std::function<void(const std::string&)> statusCallback;
    
public:
    OTAManager();
    bool isOTAInProgress() const { return otaInProgress; }
    bool isPendingRestart() const { return pendingRestart; }
    unsigned long getRestartTime() const { return restartTime; }
    void checkPendingRestart();
    bool isResetAfterOTA();
    void setStatusCallback(std::function<void(const std::string&)> cb);
    void handleCommand(const std::string& cmd);
    void handleDataChunk(const uint8_t* data, size_t length);
    void resetState();

private:
    void handleStartCommand(const std::string& cmd);
    void handleEndCommand();
    void sendOTAStatus(const std::string& status);
};

#endif
