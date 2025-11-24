#include "OTAManager.h"

#include <cstdlib>
#include <cstring>

#include "esp_heap_caps.h"
#include "esp_log.h"
#include "esp_system.h"
#include "esp_task_wdt.h"

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
      initialized(false)
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
    if (command.rfind("START:", 0) == 0)
    {
        handleStartCommand(command);
    }
    else if (command == "END")
    {
        handleEndCommand();
    }
}

void OTAManager::handleDataChunk(const uint8_t *data, size_t length)
{
    if (!otaInProgress || data == nullptr || length == 0)
    {
        return;
    }

    size_t freeHeap = esp_get_free_heap_size();
    if (freeHeap < 4096)
    {
        ESP_LOGW(TAG, "Low heap (%zu bytes)", freeHeap);
        sendStatus("ERROR:LOW_MEMORY");
        otaInProgress = false;
        return;
    }

    if (length > OTA_CHUNK_SIZE)
    {
        ESP_LOGW(TAG, "Chunk too large: %zu > %d", length, OTA_CHUNK_SIZE);
        sendStatus("ERROR:CHUNK_TOO_LARGE");
        otaInProgress = false;
        return;
    }

    if ((otaReceivedSize + length) > otaFirmwareSize)
    {
        ESP_LOGW(TAG, "Chunk exceeds firmware size: %zu + %zu > %zu",
                 otaReceivedSize, length, otaFirmwareSize);
        sendStatus("ERROR:SIZE_EXCEEDED");
        otaInProgress = false;
        return;
    }

    esp_task_wdt_reset();
    esp_err_t err = esp_ota_write(updateHandle, data, length);
    esp_task_wdt_reset();

    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to write chunk: %s", esp_err_to_name(err));
        sendStatus(std::string("ERROR:OTA_WRITE:") + esp_err_to_name(err));
        otaInProgress = false;
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

    esp_err_t err = esp_ota_begin(updatePartition, otaFirmwareSize, &updateHandle);
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "esp_ota_begin failed: %s", esp_err_to_name(err));
        sendStatus(std::string("ERROR:OTA_BEGIN:") + esp_err_to_name(err));
        return;
    }

    otaInProgress = true;
    otaReceivedSize = 0;
    sendStatus("READY");
    ESP_LOGI(TAG, "OTA started: size=%zu partition=%s", otaFirmwareSize, updatePartition->label);
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
        return;
    }

    esp_err_t err = esp_ota_end(updateHandle);
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "esp_ota_end failed: %s", esp_err_to_name(err));
        sendStatus(std::string("ERROR:OTA_END:") + esp_err_to_name(err));
        otaInProgress = false;
        return;
    }

    err = esp_ota_set_boot_partition(updatePartition);
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to set boot partition: %s", esp_err_to_name(err));
        sendStatus(std::string("ERROR:BOOT_PART:") + esp_err_to_name(err));
        otaInProgress = false;
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
