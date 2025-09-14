#include "OTAManager.h"
#include <esp_task_wdt.h>

OTAManager::OTAManager() 
    : bleServer(nullptr), otaService(nullptr), otaCommandChar(nullptr), 
      otaDataChar(nullptr), otaStatusChar(nullptr), otaInProgress(false), 
      otaFirmwareSize(0), otaReceivedSize(0), pendingRestart(false), 
      restartTime(0), isInitialized(false)
{
}

bool OTAManager::begin(BLEServer* server)
{
    if (isInitialized || !server) {
        return false;
    }
    
    bleServer = server;
    
    // Create OTA service
    otaService = bleServer->createService(OTA_SERVICE_UUID);
    
    // Create characteristics
    otaCommandChar = otaService->createCharacteristic(
        OTA_COMMAND_CHAR_UUID,
        BLECharacteristic::PROPERTY_WRITE);
    
    otaDataChar = otaService->createCharacteristic(
        OTA_DATA_CHAR_UUID,
        BLECharacteristic::PROPERTY_WRITE | BLECharacteristic::PROPERTY_WRITE_NR);
    
    otaStatusChar = otaService->createCharacteristic(
        OTA_STATUS_CHAR_UUID,
        BLECharacteristic::PROPERTY_NOTIFY | BLECharacteristic::PROPERTY_READ);
    
    // Set callbacks
    otaCommandChar->setCallbacks(new OTACommandCallbacks(this));
    otaDataChar->setCallbacks(new OTADataCallbacks(this));
    otaStatusChar->addDescriptor(new BLE2902());
    
    // Start the service
    otaService->start();
    
    isInitialized = true;
    return true;
}

void OTAManager::checkPendingRestart()
{
    if (pendingRestart && millis() >= restartTime) {
        Serial.println("OTA: Executing scheduled restart...");
        Serial.flush();
        ESP.restart();
    }
}

bool OTAManager::isResetAfterOTA()
{
    // Get the reset reason
    esp_reset_reason_t reset_reason = esp_reset_reason();
    
    // Check if it's a software reset (which happens after OTA)
    if (reset_reason == ESP_RST_SW) {
        // Additional check: verify if running partition is different from boot partition
        const esp_partition_t* running_partition = esp_ota_get_running_partition();
        const esp_partition_t* boot_partition = esp_ota_get_boot_partition();
        
        // If they're different, it means we just completed an OTA update
        if (running_partition != boot_partition) {
            Serial.printf("Reset after OTA detected - Running: %s, Boot: %s\n",
                         running_partition->label, boot_partition->label);
            return true;
        }
    }
    
    return false;
}

void OTAManager::sendOTAStatus(const String& status)
{
    if (otaStatusChar) {
        otaStatusChar->setValue(status.c_str());
        otaStatusChar->notify();
    }
}

void OTAManager::handleStartCommand(const std::string& cmd)
{
    otaFirmwareSize = atoi(cmd.substr(6).c_str());
    
    // Step 1: Verify OTA partition availability
    const esp_partition_t* update_partition = esp_ota_get_next_update_partition(NULL);
    if (update_partition == NULL) {
        sendOTAStatus("ERROR:NO_OTA_PARTITION");
        Serial.println("OTA: No OTA partition available");
        return;
    }
    
    // Step 2: Check if firmware size fits in partition
    if (otaFirmwareSize > update_partition->size) {
        String error = "ERROR:FIRMWARE_TOO_LARGE:" + String(otaFirmwareSize) + ">" + String(update_partition->size);
        sendOTAStatus(error);
        Serial.printf("OTA: Firmware too large: %d > %d\n", otaFirmwareSize, update_partition->size);
        return;
    }
    
    Serial.printf("OTA: Starting update - Size: %d bytes, Partition: %s (0x%x)\n",
                 otaFirmwareSize, update_partition->label, update_partition->address);
    
    // Step 3: Begin the update
    if (Update.begin(otaFirmwareSize)) {
        otaInProgress = true;
        otaReceivedSize = 0;
        sendOTAStatus("READY");
        Serial.println("OTA: Ready to receive firmware");
    } else {
        String errorDetails = "ERROR:OTA_BEGIN:" + String(Update.errorString());
        sendOTAStatus(errorDetails);
        Serial.printf("OTA: Failed to begin update - %s\n", Update.errorString());
    }
}

void OTAManager::handleEndCommand()
{
    Serial.println("OTA: Ending update and performing verification...");
    
    // Step 1: End the update process
    if (Update.end(true)) {
        // Step 2: Perform comprehensive verification
        bool verificationPassed = true;
        String errorMsg = "";
        
        // Check if update is finished properly
        if (!Update.isFinished()) {
            verificationPassed = false;
            errorMsg = "Update not finished";
            Serial.println("OTA: Verification failed - update not finished");
        }
        
        // Check if there were any errors during update
        if (Update.hasError()) {
            verificationPassed = false;
            errorMsg = "Update has errors: " + String(Update.errorString());
            Serial.printf("OTA: Verification failed - %s\n", Update.errorString());
        }
        
        // Verify we received the expected amount of data
        if (otaReceivedSize != otaFirmwareSize) {
            verificationPassed = false;
            errorMsg = "Size mismatch: expected " + String(otaFirmwareSize) + " got " + String(otaReceivedSize);
            Serial.printf("OTA: Verification failed - size mismatch: expected %d, got %d\n", otaFirmwareSize, otaReceivedSize);
        }
        
        if (verificationPassed) {
            // All verification checks passed
            sendOTAStatus("SUCCESS");
            Serial.println("OTA: All verification checks passed. Update successful, scheduling restart...");
            
            // Schedule restart for later instead of doing it immediately
            pendingRestart = true;
            restartTime = millis() + 3000; // Restart in 3 seconds
        } else {
            // Verification failed - don't restart
            String fullError = "ERROR:VERIFY_FAILED:" + errorMsg;
            sendOTAStatus(fullError);
            Serial.printf("OTA: Verification failed - %s\n", errorMsg.c_str());
            // Reset OTA state but don't restart with corrupted firmware
            Update.abort();
        }
    } else {
        // Update.end() itself failed
        String errorDetails = "ERROR:OTA_END:" + String(Update.errorString());
        sendOTAStatus(errorDetails);
        Serial.printf("OTA: Failed to end update - %s\n", Update.errorString());
    }
    
    otaInProgress = false;
}

void OTAManager::processDataChunk(const std::string& chunk)
{
    // Feed watchdog to prevent timeout during flash operations
    esp_task_wdt_reset();
    
    // Monitor memory before processing chunk
    size_t freeHeap = ESP.getFreeHeap();
    size_t chunkSize = chunk.size();
    
    // Reduced logging frequency to avoid serial bottlenecks
    if (otaReceivedSize % (10 * 1024) == 0) { // Log every 10KB instead of every chunk
        Serial.printf("OTA: Progress %d/%d bytes, heap: %d\n", otaReceivedSize, otaFirmwareSize, freeHeap);
    }
    
    // Check if we have enough memory to continue safely
    if (freeHeap < 4096) { // Less than 4KB free
        Serial.printf("OTA: Low memory warning - heap: %d bytes\n", freeHeap);
        sendOTAStatus("ERROR:LOW_MEMORY");
        otaInProgress = false;
        return;
    }
    
    // Check if chunk is too large
    if (chunkSize > OTA_CHUNK_SIZE) {
        Serial.printf("OTA: Chunk too large (%d bytes), max is %d\n", chunkSize, OTA_CHUNK_SIZE);
        sendOTAStatus("ERROR:CHUNK_TOO_LARGE");
        otaInProgress = false;
        return;
    }
    
    // Check if this chunk would exceed expected firmware size
    if (otaReceivedSize + chunkSize > otaFirmwareSize) {
        Serial.printf("OTA: Chunk would exceed firmware size (%d + %d > %d)\n",
                     otaReceivedSize, chunkSize, otaFirmwareSize);
        sendOTAStatus("ERROR:SIZE_EXCEEDED");
        otaInProgress = false;
        return;
    }
    
    // Feed watchdog before flash operation
    esp_task_wdt_reset();
    
    // Yield to allow other tasks to run before flash operation
    yield();
    
    // Write chunk to flash
    size_t written = Update.write((uint8_t*)chunk.data(), chunkSize);
    
    // Feed watchdog again after flash operation
    esp_task_wdt_reset();
    
    // Additional yield after flash operation to prevent watchdog timeout
    yield();
    
    if (written == chunkSize) {
        otaReceivedSize += chunkSize;
        // No progress calculation needed - Android handles progress locally
    } else {
        Serial.printf("OTA: Failed to write chunk - expected %d, wrote %d bytes\n", chunkSize, written);
        Serial.printf("OTA: Update error: %s\n", Update.errorString());
        
        String errorMsg = "ERROR:OTA_WRITE:" + String(Update.errorString());
        sendOTAStatus(errorMsg);
        
        // Stop OTA process on write failure
        otaInProgress = false;
    }
    
    // Final yield to allow BLE and other tasks to process
    yield();
}

// OTA Command Callbacks Implementation
void OTAManager::OTACommandCallbacks::onWrite(BLECharacteristic* chr)
{
    std::string cmd = chr->getValue();
    if (cmd.rfind("START:", 0) == 0) {
        otaManager->handleStartCommand(cmd);
    } else if (cmd == "END") {
        otaManager->handleEndCommand();
    }
}

// OTA Data Callbacks Implementation
void OTAManager::OTADataCallbacks::onWrite(BLECharacteristic* chr)
{
    if (!otaManager->otaInProgress) return;
    
    std::string chunk = chr->getValue();
    otaManager->processDataChunk(chunk);
}
