#include "OTAManager.h"

#include <cstdlib>
#include <cstring>

#include "esp_heap_caps.h"
#include "esp_log.h"
#include "esp_system.h"

namespace
{
    constexpr char TAG[] = "OTAManager";
}

OTAManager::OTAManager()
    : statusCallback(nullptr),
      updatePartition(nullptr),
      updateHandle(0),
      otaInProgress(false),
      otaFirmwareSize(0),
      otaReceivedSize(0),
      pendingRestart(false),
      restartTimeMs(0),
      initialized(false),
      otaBeginTaskHandle(nullptr),
      otaBeginInProgress(false)
{
}

void OTAManager::begin()
{
    if (initialized)
    {
        return;
    }

    updatePartition = esp_ota_get_next_update_partition(nullptr);
    initialized = true;
}

void OTAManager::setStatusCallback(StatusCallback callback)
{
    statusCallback = std::move(callback);
}

void OTAManager::handleCommand(const std::string &command)
{
    ESP_LOGI(TAG, "Received OTA command: '%s' (length=%zu)", command.c_str(), command.length());

    if (command.rfind("START:", 0) == 0)
    {
        ESP_LOGI(TAG, "Processing START command");
        handleStartCommand(command);
    }
    else if (command == "END")
    {
        ESP_LOGI(TAG, "Processing END command");
        handleEndCommand();
    }
    else
    {
        ESP_LOGW(TAG, "Unknown OTA command: '%s'", command.c_str());
        sendStatus("ERROR:UNKNOWN_COMMAND");
    }
}

void OTAManager::handleDataChunk(const uint8_t *data, size_t length)
{
    if (data == nullptr || length == 0)
    {
        return;
    }

    if (otaBeginInProgress)
    {
        ESP_LOGW(TAG, "Data chunk received while OTA begin in progress, ignoring");
        return;
    }

    if (!otaInProgress)
    {
        ESP_LOGW(TAG, "Data chunk received but OTA not in progress, ignoring");
        return;
    }

    size_t freeHeap = esp_get_free_heap_size();
    if (freeHeap < 4096)
    {
        ESP_LOGW(TAG, "Low heap (%zu bytes)", freeHeap);
        sendStatus("ERROR:LOW_MEMORY");
        otaInProgress = false;
        scheduleErrorReset();
        return;
    }

    if (length > OTA_CHUNK_SIZE)
    {
        ESP_LOGW(TAG, "Chunk too large: %zu > %d", length, OTA_CHUNK_SIZE);
        sendStatus("ERROR:CHUNK_TOO_LARGE");
        otaInProgress = false;
        scheduleErrorReset();
        return;
    }

    if ((otaReceivedSize + length) > otaFirmwareSize)
    {
        ESP_LOGW(TAG, "Chunk exceeds firmware size: %zu + %zu > %zu",
                 otaReceivedSize, length, otaFirmwareSize);
        sendStatus("ERROR:SIZE_EXCEEDED");
        otaInProgress = false;
        scheduleErrorReset();
        return;
    }

    esp_err_t err = esp_ota_write(updateHandle, data, length);

    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to write chunk: %s", esp_err_to_name(err));
        sendStatus(std::string("ERROR:OTA_WRITE:") + esp_err_to_name(err));
        otaInProgress = false;
        esp_ota_abort(updateHandle);
        scheduleErrorReset();
        return;
    }

    otaReceivedSize += length;

    if ((otaReceivedSize % (10 * 1024)) == 0)
    {
        ESP_LOGI(TAG, "OTA progress %zu/%zu bytes", otaReceivedSize, otaFirmwareSize);
    }
}

void OTAManager::checkPendingRestart()
{
    if (pendingRestart && millis64() >= restartTimeMs)
    {
        ESP_LOGI(TAG, "Restarting after OTA");
        fflush(stdout);
        esp_restart();
    }
}

bool OTAManager::isResetAfterOTA()
{
    esp_reset_reason_t reason = esp_reset_reason();
    if (reason == ESP_RST_SW)
    {
        const esp_partition_t *running = esp_ota_get_running_partition();
        const esp_partition_t *boot = esp_ota_get_boot_partition();
        if (running != boot)
        {
            ESP_LOGI(TAG, "Reset after OTA detected (running=%s boot=%s)",
                     running ? running->label : "unknown",
                     boot ? boot->label : "unknown");
            return true;
        }
    }
    return false;
}

void OTAManager::handleStartCommand(const std::string &command)
{
    if (otaBeginInProgress || otaInProgress)
    {
        ESP_LOGW(TAG, "OTA already in progress, ignoring START command");
        return;
    }

    if (command.size() <= 6)
    {
        sendStatus("ERROR:INVALID_START");
        return;
    }

    otaFirmwareSize = static_cast<size_t>(std::strtoul(command.substr(6).c_str(), nullptr, 10));
    updatePartition = esp_ota_get_next_update_partition(nullptr);

    if (updatePartition == nullptr)
    {
        ESP_LOGE(TAG, "No OTA partition available");
        sendStatus("ERROR:NO_OTA_PARTITION");
        return;
    }

    if (otaFirmwareSize == 0 || otaFirmwareSize > updatePartition->size)
    {
        ESP_LOGE(TAG, "Firmware too large: %zu > %zu", otaFirmwareSize, (size_t)updatePartition->size);
        sendStatus("ERROR:FIRMWARE_TOO_LARGE");
        return;
    }

    // Kill any existing task
    if (otaBeginTaskHandle != nullptr)
    {
        vTaskDelete(otaBeginTaskHandle);
        otaBeginTaskHandle = nullptr;
    }

    // Start OTA begin in a separate task to avoid blocking BLE handler
    otaBeginInProgress = true;
    ESP_LOGI(TAG, "Starting OTA begin task for size=%zu", otaFirmwareSize);
    xTaskCreate(otaBeginTask, "ota_begin", 4096, this, 5, &otaBeginTaskHandle);
}

void OTAManager::otaBeginTask(void *pvParameters)
{
    OTAManager *self = static_cast<OTAManager *>(pvParameters);

    ESP_LOGI(TAG, "OTA begin task started");
    esp_err_t err = esp_ota_begin(self->updatePartition, self->otaFirmwareSize, &self->updateHandle);

    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "esp_ota_begin failed: %s", esp_err_to_name(err));
        self->sendStatus(std::string("ERROR:OTA_BEGIN:") + esp_err_to_name(err));
        self->otaBeginInProgress = false;
        self->otaBeginTaskHandle = nullptr;
        self->scheduleErrorReset();
        vTaskDelete(nullptr);
        return;
    }

    self->otaInProgress = true;
    self->otaReceivedSize = 0;
    self->otaBeginInProgress = false;
    ESP_LOGI(TAG, "OTA started: size=%zu partition=%s", self->otaFirmwareSize, self->updatePartition->label);

    // Small delay to ensure BLE stack is ready
    vTaskDelay(pdMS_TO_TICKS(50));

    self->sendStatus("READY");
    ESP_LOGI(TAG, "READY status sent to Android");

    self->otaBeginTaskHandle = nullptr;
    vTaskDelete(nullptr);
}

void OTAManager::handleEndCommand()
{
    if (!otaInProgress)
    {
        ESP_LOGW(TAG, "OTA end requested without active session");
        return;
    }

    if (otaReceivedSize != otaFirmwareSize)
    {
        ESP_LOGE(TAG, "Size mismatch: expected %zu, received %zu",
                 otaFirmwareSize, otaReceivedSize);
        sendStatus("ERROR:SIZE_MISMATCH");
        otaInProgress = false;
        esp_ota_abort(updateHandle);
        scheduleErrorReset();
        return;
    }

    esp_err_t err = esp_ota_end(updateHandle);
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "esp_ota_end failed: %s", esp_err_to_name(err));
        sendStatus(std::string("ERROR:OTA_END:") + esp_err_to_name(err));
        otaInProgress = false;
        // OTA end failed, but we can't abort after end - just reset
        scheduleErrorReset();
        return;
    }

    err = esp_ota_set_boot_partition(updatePartition);
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to set boot partition: %s", esp_err_to_name(err));
        sendStatus(std::string("ERROR:BOOT_PART:") + esp_err_to_name(err));
        otaInProgress = false;
        scheduleErrorReset();
        return;
    }

    sendStatus("SUCCESS");
    ESP_LOGI(TAG, "OTA successful, scheduling restart");
    otaInProgress = false;
    pendingRestart = true;
    restartTimeMs = millis64() + 3000;
}

void OTAManager::sendStatus(const std::string &status)
{
    if (statusCallback)
    {
        statusCallback(status);
    }
}

void OTAManager::scheduleErrorReset()
{
    ESP_LOGE(TAG, "OTA error detected, scheduling reset in 2 seconds");
    pendingRestart = true;
    restartTimeMs = millis64() + 2000; // 2 seconds to allow error message to be sent
}
