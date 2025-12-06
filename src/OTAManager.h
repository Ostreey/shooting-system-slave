#ifndef OTA_MANAGER_H
#define OTA_MANAGER_H

#include <cstdint>
#include <functional>
#include <string>

#include "Config.h"
#include "IdfCompat.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

extern "C"
{
#include "esp_ota_ops.h"
}

class OTAManager
{
public:
    using StatusCallback = std::function<void(const std::string &)>;

    OTAManager();

    void begin();

    void setStatusCallback(StatusCallback callback);

    void handleCommand(const std::string &command);
    void handleDataChunk(const uint8_t *data, size_t length);

    void checkPendingRestart();
    bool isResetAfterOTA();

    bool isOTAInProgress() const { return otaInProgress; }

private:
    void handleStartCommand(const std::string &command);
    void handleEndCommand();
    void sendStatus(const std::string &status);
    void scheduleErrorReset();

    StatusCallback statusCallback;
    const esp_partition_t *updatePartition;
    esp_ota_handle_t updateHandle;
    bool otaInProgress;
    size_t otaFirmwareSize;
    size_t otaReceivedSize;
    bool pendingRestart;
    uint64_t restartTimeMs;
    bool initialized;
    TaskHandle_t otaBeginTaskHandle;
    bool otaBeginInProgress;

    static void otaBeginTask(void *pvParameters);
};

#endif // OTA_MANAGER_H
