#include "OTAManager.h"
#include "TimeUtils.h"

#include <cstdlib>
#include <esp_log.h>
#include <esp_heap_caps.h>

namespace
{
const char *TAG = "OTAManager";
}

OTAManager::OTAManager()
    : otaInProgress(false),
      otaFirmwareSize(0),
      otaReceivedSize(0),
      pendingRestart(false),
      restartTime(0),
      otaHandle(0),
      updatePartition(nullptr)
{
}

void OTAManager::setStatusCallback(std::function<void(const std::string &)> cb)
{
    statusCallback = std::move(cb);
}

void OTAManager::resetState()
{
    otaInProgress = false;
    otaFirmwareSize = 0;
    otaReceivedSize = 0;
    updatePartition = nullptr;
    otaHandle = 0;
}

void OTAManager::sendOTAStatus(const std::string &status)
{
    if (statusCallback)
    {
        statusCallback(status);
    }
}

void OTAManager::handleCommand(const std::string &cmd)
{
    if (cmd.rfind("START:", 0) == 0)
    {
        handleStartCommand(cmd);
    }
    else if (cmd == "END")
    {
        handleEndCommand();
    }
}

void OTAManager::handleStartCommand(const std::string &cmd)
{
    otaFirmwareSize = static_cast<size_t>(std::strtoul(cmd.substr(6).c_str(), nullptr, 10));
    updatePartition = esp_ota_get_next_update_partition(nullptr);
    if (updatePartition == nullptr)
    {
        sendOTAStatus("ERROR:NO_OTA_PARTITION");
        ESP_LOGE(TAG, "No OTA partition available");
        return;
    }
    if (otaFirmwareSize > updatePartition->size)
    {
        sendOTAStatus("ERROR:FIRMWARE_TOO_LARGE");
        ESP_LOGE(TAG, "Firmware too large %zu > %zu", otaFirmwareSize, updatePartition->size);
        return;
    }
    if (esp_ota_begin(updatePartition, otaFirmwareSize, &otaHandle) == ESP_OK)
    {
        otaInProgress = true;
        otaReceivedSize = 0;
        sendOTAStatus("READY");
        ESP_LOGI(TAG, "OTA ready size=%zu partition=%s", otaFirmwareSize, updatePartition->label);
    }
    else
    {
        sendOTAStatus("ERROR:OTA_BEGIN");
        ESP_LOGE(TAG, "esp_ota_begin failed");
        resetState();
    }
}

void OTAManager::handleDataChunk(const uint8_t *data, size_t length)
{
    if (!otaInProgress || data == nullptr || length == 0)
    {
        return;
    }

    esp_task_wdt_reset();
    size_t freeHeap = heap_caps_get_free_size(MALLOC_CAP_INTERNAL);
    if (freeHeap < 4096)
    {
        sendOTAStatus("ERROR:LOW_MEMORY");
        ESP_LOGE(TAG, "Low heap %zu", freeHeap);
        resetState();
        return;
    }

    if (length > OTA_CHUNK_SIZE)
    {
        sendOTAStatus("ERROR:CHUNK_TOO_LARGE");
        ESP_LOGE(TAG, "Chunk too large %zu", length);
        resetState();
        return;
    }

    if (otaReceivedSize + length > otaFirmwareSize)
    {
        sendOTAStatus("ERROR:SIZE_EXCEEDED");
        ESP_LOGE(TAG, "Chunk exceeds firmware size");
        resetState();
        return;
    }

    esp_err_t err = esp_ota_write(otaHandle, data, length);
    esp_task_wdt_reset();
    taskYIELD();

    if (err == ESP_OK)
    {
        otaReceivedSize += length;
    }
    else
    {
        sendOTAStatus("ERROR:OTA_WRITE");
        ESP_LOGE(TAG, "esp_ota_write failed %s", esp_err_to_name(err));
        resetState();
    }
}

void OTAManager::handleEndCommand()
{
    if (!otaInProgress)
    {
        return;
    }

    esp_err_t err = esp_ota_end(otaHandle);
    if (err != ESP_OK)
    {
        sendOTAStatus("ERROR:OTA_END");
        ESP_LOGE(TAG, "esp_ota_end failed %s", esp_err_to_name(err));
        resetState();
        return;
    }

    if (otaReceivedSize != otaFirmwareSize)
    {
        sendOTAStatus("ERROR:SIZE_MISMATCH");
        ESP_LOGE(TAG, "Size mismatch expected=%zu got=%zu", otaFirmwareSize, otaReceivedSize);
        resetState();
        return;
    }

    err = esp_ota_set_boot_partition(updatePartition);
    if (err != ESP_OK)
    {
        sendOTAStatus("ERROR:SET_BOOT");
        ESP_LOGE(TAG, "Failed to set boot partition %s", esp_err_to_name(err));
        resetState();
        return;
    }

    sendOTAStatus("SUCCESS");
    pendingRestart = true;
    restartTime = millis() + 3000;
    otaInProgress = false;
    ESP_LOGI(TAG, "OTA success scheduled restart");
}

void OTAManager::checkPendingRestart()
{
    if (pendingRestart && millis() >= restartTime)
    {
        ESP_LOGI(TAG, "Restarting after OTA");
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
            ESP_LOGI(TAG, "Detected OTA reset running=%s boot=%s", running->label, boot->label);
            return true;
        }
    }
    return false;
}
