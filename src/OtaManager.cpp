#include "OtaManager.h"

// OTA service UUIDs
const char *OtaManager::OTA_SERVICE_UUID = "12345678-1234-5678-1234-56789abc0000";
const char *OtaManager::OTA_COMMAND_CHAR_UUID = "12345678-1234-5678-1234-56789abc0001";
const char *OtaManager::OTA_DATA_CHAR_UUID = "12345678-1234-5678-1234-56789abc0002";
const char *OtaManager::OTA_STATUS_CHAR_UUID = "12345678-1234-5678-1234-56789abc0003";

// Global instance
OtaManager otaManager;

OtaManager::OtaManager() : pServer(nullptr),
                           otaInProgress(false),
                           otaFirmwareSize(0),
                           otaReceivedSize(0),
                           pendingRestart(false),
                           restartTime(0)
{
}

void OtaManager::init(BLEServer *server)
{
    pServer = server;
    setupOtaService();
    Serial.println("OTA Manager initialized");
}

void OtaManager::setupOtaService()
{
    BLEService *otaService = pServer->createService(OTA_SERVICE_UUID);

    BLECharacteristic *otaCommandChar = otaService->createCharacteristic(
        OTA_COMMAND_CHAR_UUID,
        BLECharacteristic::PROPERTY_WRITE);

    BLECharacteristic *otaDataChar = otaService->createCharacteristic(
        OTA_DATA_CHAR_UUID,
        BLECharacteristic::PROPERTY_WRITE | BLECharacteristic::PROPERTY_WRITE_NR);

    BLECharacteristic *otaStatusChar = otaService->createCharacteristic(
        OTA_STATUS_CHAR_UUID,
        BLECharacteristic::PROPERTY_NOTIFY | BLECharacteristic::PROPERTY_READ);

    otaCommandChar->setCallbacks(new OTACommandCallbacks(this));
    otaDataChar->setCallbacks(new OTADataCallbacks(this));
    otaStatusChar->addDescriptor(new BLE2902());

    otaService->start();

    // Add OTA service to advertising
    BLEAdvertising *pAdvertising = BLEDevice::getAdvertising();
    pAdvertising->addServiceUUID(OTA_SERVICE_UUID);
}

void OtaManager::executeRestart()
{
    if (pendingRestart && millis() >= restartTime)
    {
        Serial.println("OTA: Executing scheduled restart...");
        Serial.flush();
        ESP.restart();
    }
}

bool OtaManager::isResetAfterOTA()
{
    esp_reset_reason_t reset_reason = esp_reset_reason();

    if (reset_reason == ESP_RST_SW)
    {
        const esp_partition_t *running_partition = esp_ota_get_running_partition();
        const esp_partition_t *boot_partition = esp_ota_get_boot_partition();

        if (running_partition != boot_partition)
        {
            Serial.printf("Reset after OTA detected - Running: %s, Boot: %s\n",
                          running_partition->label, boot_partition->label);
            return true;
        }
    }

    return false;
}

// OTA Command Callbacks Implementation
void OTACommandCallbacks::onWrite(BLECharacteristic *chr)
{
    std::string cmd = chr->getValue();
    if (cmd.rfind("START:", 0) == 0)
    {
        otaManager->otaFirmwareSize = atoi(cmd.substr(6).c_str());

        // Verify OTA partition availability
        const esp_partition_t *update_partition = esp_ota_get_next_update_partition(NULL);
        if (update_partition == NULL)
        {
            BLEService *otaService = otaManager->pServer->getServiceByUUID(OtaManager::OTA_SERVICE_UUID);
            if (otaService)
            {
                BLECharacteristic *statusChar = otaService->getCharacteristic(OtaManager::OTA_STATUS_CHAR_UUID);
                if (statusChar)
                {
                    statusChar->setValue("ERROR:NO_OTA_PARTITION");
                    statusChar->notify();
                    Serial.println("OTA: No OTA partition available");
                }
            }
            return;
        }

        // Check if firmware size fits in partition
        if (otaManager->otaFirmwareSize > update_partition->size)
        {
            BLEService *otaService = otaManager->pServer->getServiceByUUID(OtaManager::OTA_SERVICE_UUID);
            if (otaService)
            {
                BLECharacteristic *statusChar = otaService->getCharacteristic(OtaManager::OTA_STATUS_CHAR_UUID);
                if (statusChar)
                {
                    String error = "ERROR:FIRMWARE_TOO_LARGE:" + String(otaManager->otaFirmwareSize) + ">" + String(update_partition->size);
                    statusChar->setValue(error.c_str());
                    statusChar->notify();
                    Serial.printf("OTA: Firmware too large: %d > %d\n", otaManager->otaFirmwareSize, update_partition->size);
                }
            }
            return;
        }

        Serial.printf("OTA: Starting update - Size: %d bytes, Partition: %s (0x%x)\n",
                      otaManager->otaFirmwareSize, update_partition->label, update_partition->address);

        // Begin the update
        if (Update.begin(otaManager->otaFirmwareSize))
        {
            otaManager->otaInProgress = true;
            otaManager->otaReceivedSize = 0;
            BLEService *otaService = otaManager->pServer->getServiceByUUID(OtaManager::OTA_SERVICE_UUID);
            if (otaService)
            {
                BLECharacteristic *statusChar = otaService->getCharacteristic(OtaManager::OTA_STATUS_CHAR_UUID);
                if (statusChar)
                {
                    statusChar->setValue("READY");
                    statusChar->notify();
                    Serial.println("OTA: Ready to receive firmware");
                }
            }
        }
        else
        {
            BLEService *otaService = otaManager->pServer->getServiceByUUID(OtaManager::OTA_SERVICE_UUID);
            if (otaService)
            {
                BLECharacteristic *statusChar = otaService->getCharacteristic(OtaManager::OTA_STATUS_CHAR_UUID);
                if (statusChar)
                {
                    String errorDetails = "ERROR:OTA_BEGIN:" + String(Update.errorString());
                    statusChar->setValue(errorDetails.c_str());
                    statusChar->notify();
                    Serial.printf("OTA: Failed to begin update - %s\n", Update.errorString());
                }
            }
        }
    }
    else if (cmd == "END")
    {
        Serial.println("OTA: Ending update and performing verification...");

        if (Update.end(true))
        {
            bool verificationPassed = true;
            String errorMsg = "";

            if (!Update.isFinished())
            {
                verificationPassed = false;
                errorMsg = "Update not finished";
                Serial.println("OTA: Verification failed - update not finished");
            }

            if (Update.hasError())
            {
                verificationPassed = false;
                errorMsg = "Update has errors: " + String(Update.errorString());
                Serial.printf("OTA: Verification failed - %s\n", Update.errorString());
            }

            if (otaManager->otaReceivedSize != otaManager->otaFirmwareSize)
            {
                verificationPassed = false;
                errorMsg = "Size mismatch: expected " + String(otaManager->otaFirmwareSize) + " got " + String(otaManager->otaReceivedSize);
                Serial.printf("OTA: Verification failed - size mismatch: expected %d, got %d\n", otaManager->otaFirmwareSize, otaManager->otaReceivedSize);
            }

            BLEService *otaService = otaManager->pServer->getServiceByUUID(OtaManager::OTA_SERVICE_UUID);
            BLECharacteristic *statusChar = nullptr;
            if (otaService)
            {
                statusChar = otaService->getCharacteristic(OtaManager::OTA_STATUS_CHAR_UUID);
            }

            if (verificationPassed)
            {
                if (statusChar)
                {
                    statusChar->setValue("SUCCESS");
                    statusChar->notify();
                    Serial.println("OTA: All verification checks passed. Update successful, scheduling restart...");
                }

                otaManager->pendingRestart = true;
                otaManager->restartTime = millis() + 3000;
            }
            else
            {
                if (statusChar)
                {
                    String fullError = "ERROR:VERIFY_FAILED:" + errorMsg;
                    statusChar->setValue(fullError.c_str());
                    statusChar->notify();
                    Serial.printf("OTA: Verification failed - %s\n", errorMsg.c_str());
                }
                Update.abort();
            }
        }
        else
        {
            BLEService *otaService = otaManager->pServer->getServiceByUUID(OtaManager::OTA_SERVICE_UUID);
            if (otaService)
            {
                BLECharacteristic *statusChar = otaService->getCharacteristic(OtaManager::OTA_STATUS_CHAR_UUID);
                if (statusChar)
                {
                    String errorDetails = "ERROR:OTA_END:" + String(Update.errorString());
                    statusChar->setValue(errorDetails.c_str());
                    statusChar->notify();
                    Serial.printf("OTA: Failed to end update - %s\n", Update.errorString());
                }
            }
        }
        otaManager->otaInProgress = false;
    }
}

// OTA Data Callbacks Implementation
void OTADataCallbacks::onWrite(BLECharacteristic *chr)
{
    if (!otaManager->otaInProgress)
        return;

    esp_task_wdt_reset();

    size_t freeHeap = ESP.getFreeHeap();
    std::string chunk = chr->getValue();
    size_t chunkSize = chunk.size();

    if (otaManager->otaReceivedSize % (10 * 1024) == 0)
    {
        Serial.printf("OTA: Progress %d/%d bytes, heap: %d\n", otaManager->otaReceivedSize, otaManager->otaFirmwareSize, freeHeap);
    }

    if (freeHeap < 4096)
    {
        Serial.printf("OTA: Low memory warning - heap: %d bytes\n", freeHeap);
        BLEService *otaService = otaManager->pServer->getServiceByUUID(OtaManager::OTA_SERVICE_UUID);
        if (otaService)
        {
            BLECharacteristic *statusChar = otaService->getCharacteristic(OtaManager::OTA_STATUS_CHAR_UUID);
            if (statusChar)
            {
                statusChar->setValue("ERROR:LOW_MEMORY");
                statusChar->notify();
            }
        }
        otaManager->otaInProgress = false;
        return;
    }

    if (chunkSize > OtaManager::OTA_CHUNK_SIZE)
    {
        Serial.printf("OTA: Chunk too large (%d bytes), max is %d\n", chunkSize, OtaManager::OTA_CHUNK_SIZE);
        BLEService *otaService = otaManager->pServer->getServiceByUUID(OtaManager::OTA_SERVICE_UUID);
        if (otaService)
        {
            BLECharacteristic *statusChar = otaService->getCharacteristic(OtaManager::OTA_STATUS_CHAR_UUID);
            if (statusChar)
            {
                statusChar->setValue("ERROR:CHUNK_TOO_LARGE");
                statusChar->notify();
            }
        }
        otaManager->otaInProgress = false;
        return;
    }

    if (otaManager->otaReceivedSize + chunkSize > otaManager->otaFirmwareSize)
    {
        Serial.printf("OTA: Chunk would exceed firmware size (%d + %d > %d)\n",
                      otaManager->otaReceivedSize, chunkSize, otaManager->otaFirmwareSize);
        BLEService *otaService = otaManager->pServer->getServiceByUUID(OtaManager::OTA_SERVICE_UUID);
        if (otaService)
        {
            BLECharacteristic *statusChar = otaService->getCharacteristic(OtaManager::OTA_STATUS_CHAR_UUID);
            if (statusChar)
            {
                statusChar->setValue("ERROR:SIZE_EXCEEDED");
                statusChar->notify();
            }
        }
        otaManager->otaInProgress = false;
        return;
    }

    esp_task_wdt_reset();
    yield();

    size_t written = Update.write((uint8_t *)chunk.data(), chunkSize);

    esp_task_wdt_reset();
    yield();

    if (written == chunkSize)
    {
        otaManager->otaReceivedSize += chunkSize;
    }
    else
    {
        Serial.printf("OTA: Failed to write chunk - expected %d, wrote %d bytes\n", chunkSize, written);
        Serial.printf("OTA: Update error: %s\n", Update.errorString());

        BLEService *otaService = otaManager->pServer->getServiceByUUID(OtaManager::OTA_SERVICE_UUID);
        if (otaService)
        {
            BLECharacteristic *statusChar = otaService->getCharacteristic(OtaManager::OTA_STATUS_CHAR_UUID);
            if (statusChar)
            {
                String errorMsg = "ERROR:OTA_WRITE:" + String(Update.errorString());
                statusChar->setValue(errorMsg.c_str());
                statusChar->notify();
            }
        }

        otaManager->otaInProgress = false;
    }

    yield();
}