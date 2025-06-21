#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>
#include <Wire.h>
#include <Arduino.h>
#include "PiezoSensor.h"
#include <WiFiManager.h>
#include <esp_adc_cal.h>
#include <Update.h>



// BLE server name
#define bleServerName "SHOOTING TARGET"

#define SERVICE_UUID "91bad492-b950-4226-aa2b-4ede9fa42fff"
#define CHARACTERISTIC "cba1d466-344c-4be3-ab3f-189f80dd75ff"
#define BATTERY_CHARACTERISTIC "cba1d466-344c-4be3-ab3f-189f80dd76ff"

// OTA service UUIDs
#define OTA_SERVICE_UUID          "12345678-1234-5678-1234-56789abc0000"
#define OTA_COMMAND_CHAR_UUID     "12345678-1234-5678-1234-56789abc0001"
#define OTA_DATA_CHAR_UUID        "12345678-1234-5678-1234-56789abc0002"
#define OTA_STATUS_CHAR_UUID      "12345678-1234-5678-1234-56789abc0003"

// Add these constants for ADC calibration
#define DEFAULT_VREF 1100         // Default reference voltage in mV
#define ADC_SAMPLES 64            // Number of samples for averaging
#define ADC_ATTEN ADC_ATTEN_DB_11 // 11dB attenuation for 0-3.3V range

// Add these constants near the top with other definitions
#define PWM_FREQUENCY 5000
#define PWM_RESOLUTION 8 // 8-bit resolution (0-255)
#define PWM_CHANNEL_RED 0
#define PWM_CHANNEL_GREEN 1
#define PWM_CHANNEL_BLUE 2

float temp;

bool deviceConnected = false;








const int ledBlue = 17;
const int ledGreen = 18;
const int ledRed = 19;
const int wifiButton = 15; // New button pin definition
const int batteryPin = 33; // Battery voltage measurement pin
const int ledPowerEnable = 4;

PiezoSensor sensor(32, 400);

BLECharacteristic *piezoCharacteristic;
BLECharacteristic *batteryCharacteristic;

BLEService *piezoService;
BLEServer *pServer;
esp_adc_cal_characteristics_t adc_chars;

// Task handle for battery monitoring
TaskHandle_t batteryTaskHandle = NULL;

// Add this task handle near the top with other global variables
TaskHandle_t ledStatusTaskHandle = NULL;
// Add these constants at the top with other definitions
const int TURN_OFF_TIME = 2000;       // 2 seconds for long press
const int SECRET_PRESS_WINDOW = 1000; // 2 second window for secret combination
const int SECRET_PRESS_COUNT = 10;     // Number of presses needed for secret combination
bool isWebPortalOpen = false, isGoingToSleep = false;
// Add these variables in the global scope
unsigned long lastPressTime = 0;
int pressCount = 0;
int brightness = 255;

unsigned long hitTime = 0;

// OTA state
bool   otaInProgress   = false;
size_t otaFirmwareSize = 0;
size_t otaReceivedSize = 0;
const int OTA_CHUNK_SIZE = 20; // Bezpieczny rozmiar fragmentu BLE

// Add this enum before the WriteCallbacks class
enum class BLECommand
{
  UNKNOWN,
  START,
  SLEEP,
  BLINK,
  GAME1,
  SET_BRIGHTNESS, // New command for LED brightness
};
void ledOff(int piezoValue);
void setLeds(bool on);

// Add a struct to hold command and value
struct BLECommandData
{
  BLECommand command;
  int value;
};

// Update the command parsing function
BLECommandData parseCommand(const std::string &value)
{
  BLECommandData result = {BLECommand::UNKNOWN, 0};

  // Check if the command starts with "b:"
  if (value.substr(0, 2) == "b:")
  {
    result.command = BLECommand::SET_BRIGHTNESS;
    result.value = std::stoi(value.substr(2));
    return result;
  }

  // Handle valueless commands
  if (value == "start")
    result.command = BLECommand::START;
  else if (value == "sleep")
    result.command = BLECommand::SLEEP;
  else if (value == "game1")
    result.command = BLECommand::GAME1;
  else if (value == "blink")
    result.command = BLECommand::BLINK;

  return result;
}

void ledOff(int piezoValue)
{
  setLeds(false);
  if (deviceConnected)
  {
    uint8_t data[4];
    uint16_t timeInHundredths = (millis() - hitTime) / 10;
    data[0] = piezoValue & 0xFF;
    data[1] = piezoValue >> 8;
    data[2] = timeInHundredths & 0xFF;
    data[3] = timeInHundredths >> 8;
    piezoCharacteristic->setValue(data, 4);
    piezoCharacteristic->notify();
    Serial.print("Sent piezo value: ");
    Serial.println(piezoValue);
  }
  else
  {
    Serial.println("Device not connected, cannot send notification.");
  }
}

void sendBatteryLevel(int percentage)
{
  if (deviceConnected)
  {
    String valueToSend = String(percentage);
    batteryCharacteristic->setValue(valueToSend.c_str());
    batteryCharacteristic->notify();
    Serial.println("Battery level sent: " + valueToSend);
  }
  else
  {
    Serial.println("Device not connected, cannot send notification.");
  }
}

void blinkLEDS()
{
  for (int i = 0; i < 3; i++)
  {
    setLeds(true);
    delay(200);
    setLeds(false);
    delay(200);
  }
}

int getBatteryPercentage()
{
  int percentage = 100;
  // Read multiple samples and average them
  uint32_t adc_reading = 0;
  for (int i = 0; i < ADC_SAMPLES; i++)
  {
    adc_reading += analogRead(batteryPin);
  }
  adc_reading /= ADC_SAMPLES;

  // Convert to voltage using calibration
  uint32_t voltage_mv = esp_adc_cal_raw_to_voltage(adc_reading, &adc_chars);
  float voltage = voltage_mv / 1000.0; // Convert to volts

  // Convert to actual battery voltage (multiply by 1.5 due to voltage divider)
  float battery_voltage = voltage * 1.5;

  // Convert to percentage (assuming 4.2V is 100% and 3.6V is 0%)
  percentage = map(battery_voltage * 100, 340, 420, 0, 100);
  percentage = constrain(percentage, 0, 100);

  // // Send through BLE
  // String valueToSend = String(percentage);
  // Serial.print("Raw ADC: ");
  // Serial.print(adc_reading);
  // Serial.print(", Divider Voltage: ");
  // Serial.print(voltage);
  // Serial.print("V, Battery Voltage: ");
  // Serial.print(battery_voltage);
  // Serial.print("V, Percentage: ");
  // Serial.print(percentage);
  // Serial.println("%");
  return percentage;
}

void goToDeepSleep()
{
  isGoingToSleep = true;
  blinkLEDS();
  delay(3000 / portTICK_PERIOD_MS);
  digitalWrite(ledPowerEnable, LOW);
  Serial.println("Going to deep sleep...");
  esp_sleep_enable_ext0_wakeup(GPIO_NUM_15, 0);
  esp_deep_sleep_start();
}


void setLeds(bool on)
{
  if (on)
  {
    ledcWrite(PWM_CHANNEL_RED, brightness);
    ledcWrite(PWM_CHANNEL_GREEN, brightness);
    ledcWrite(PWM_CHANNEL_BLUE, brightness);
  }
  else
  {
    ledcWrite(PWM_CHANNEL_RED, 0);
    ledcWrite(PWM_CHANNEL_GREEN, 0);
    ledcWrite(PWM_CHANNEL_BLUE, 0);
  }
}


// Battery monitoring task
void batteryMonitorTask(void *pvParameters)
{
  // ADC calibration

  esp_adc_cal_characterize(ADC_UNIT_1, ADC_ATTEN, ADC_WIDTH_BIT_12, DEFAULT_VREF, &adc_chars);

  for (;;)
  {

    int percentage = getBatteryPercentage();
    sendBatteryLevel(percentage);
    if (percentage < 5 && !isWebPortalOpen)
    {
      Serial.println("Battery low, going to sleep");
      goToDeepSleep();
    }
    vTaskDelay(30000 / portTICK_PERIOD_MS); // 30 seconds delay
  }
}

// Add this new task function before setup()
void ledStatusTask(void *pvParameters)
{
  for (;;)
  {
    if (!deviceConnected && !isGoingToSleep)
    {
      setLeds(false);
      vTaskDelay(1000 / portTICK_PERIOD_MS);
      setLeds(true);
      vTaskDelay(1000 / portTICK_PERIOD_MS);
      Serial.println("Device not connected");
    }
    else
    {
      vTaskDelay(100 / portTICK_PERIOD_MS);
    }
  }
}

class WriteCallbacks : public BLECharacteristicCallbacks
{
  void onWrite(BLECharacteristic *pCharacteristic)
  {
    std::string value = pCharacteristic->getValue();
    Serial.print("Received value: ");
    Serial.println(value.c_str());

    BLECommandData cmdData = parseCommand(value);

    switch (cmdData.command)
    {
    case BLECommand::START:
      hitTime = millis();
      setLeds(true);
      break;

    case BLECommand::SLEEP:
      goToDeepSleep();
      break;

    case BLECommand::BLINK:
      blinkLEDS();
      break;

    case BLECommand::SET_BRIGHTNESS:
      brightness = constrain(cmdData.value, 0, 255);
      break;

    
    }
  }
};

void initialBatteryTask(void *pvParameters)
{
  vTaskDelay(2000 / portTICK_PERIOD_MS); // Wait for connection to stabilize
  int percentage = getBatteryPercentage();
  sendBatteryLevel(percentage);
  vTaskDelete(NULL); // Delete the task after it's done
}

// Setup callbacks onConnect and onDisconnect
class MyServerCallbacks : public BLEServerCallbacks
{
  void onConnect(BLEServer *pServer)
  {
    Serial.println("Device connected");
    deviceConnected = true;
    blinkLEDS();
    vTaskDelay(1000 / portTICK_PERIOD_MS);
    xTaskCreate(initialBatteryTask, "InitBattery", 2048, NULL, 1, NULL);
  };
  void onDisconnect(BLEServer *pServer)
  {
    deviceConnected = false;
    pServer->getAdvertising()->start();
  }
};

void openWiFiPortal()
{
  // Stop BLE advertising before opening portal
  BLEDevice::getAdvertising()->stop();

  // Visual indication
  for (int i = 0; i < 5; i++)
  {
    setLeds(false);
    delay(50);
    setLeds(true);
    delay(50);
  }

  isWebPortalOpen = true;
  Serial.println("Starting WiFiManager portal");

  WiFiManager wifiManager;
  // Set timeout to 3 minutes (180 seconds)
  wifiManager.setConfigPortalTimeout(180);
  wifiManager.startConfigPortal("ShootingSystemAP");

  // After portal is closed
  isWebPortalOpen = false;

  // Restart BLE advertising
  BLEDevice::getAdvertising()->start();
}

void setup()
{
  delay(100);

  Serial.begin(115200);
  Serial.println("Start");

  pinMode(ledPowerEnable, OUTPUT);
  digitalWrite(ledPowerEnable, HIGH);
  pinMode(ledBlue, OUTPUT);
  digitalWrite(ledBlue, LOW);
  pinMode(ledGreen, OUTPUT);
  digitalWrite(ledGreen, LOW);
  pinMode(ledRed, OUTPUT);
  digitalWrite(ledRed, LOW);
  pinMode(wifiButton, INPUT_PULLUP); // Initialize button pin
  pinMode(batteryPin, INPUT);        // Initialize battery pin

  sensor.begin();
  sensor.setCallback(ledOff);

  // Determine the device name based on sensorNumber
  String deviceName;

  deviceName = bleServerName;
  Serial.print(bleServerName);

  // Initialize BLE Device with the device name
  BLEDevice::init(deviceName.c_str());

  // Create the BLE Server
  pServer = BLEDevice::createServer();
  pServer->setCallbacks(new MyServerCallbacks());

  // Create the BLE Service and Characteristics
  piezoService = pServer->createService(SERVICE_UUID);
  piezoCharacteristic = piezoService->createCharacteristic(
      CHARACTERISTIC,
      BLECharacteristic::PROPERTY_NOTIFY | BLECharacteristic::PROPERTY_WRITE);

  batteryCharacteristic = piezoService->createCharacteristic(
      BATTERY_CHARACTERISTIC,
      BLECharacteristic::PROPERTY_NOTIFY | BLECharacteristic::PROPERTY_READ);

  // Add Descriptor and Callbacks
  piezoCharacteristic->addDescriptor(new BLE2902());
  batteryCharacteristic->addDescriptor(new BLE2902());
  piezoCharacteristic->setCallbacks(new WriteCallbacks());

  // Start the service
  piezoService->start();

  // Configure the advertising data
  BLEAdvertising *pAdvertising = BLEDevice::getAdvertising();
  pAdvertising->addServiceUUID(piezoService->getUUID());
  pAdvertising->addServiceUUID(OTA_SERVICE_UUID);
  pAdvertising->setScanResponse(true);

  // Explicitly set the device name in the advertising data
  BLEAdvertisementData advertisementData;
  advertisementData.setName(deviceName.c_str());
  pAdvertising->setAdvertisementData(advertisementData);

  // Start advertising
  BLEDevice::startAdvertising();

  Serial.println("Waiting for a client connection to notify...");

  // Create battery monitoring task
  xTaskCreatePinnedToCore(
      batteryMonitorTask, // Task function
      "BatteryMonitor",   // Task name
      2048,               // Stack size
      NULL,               // Task parameters
      2,                  // Task priority
      &batteryTaskHandle, // Task handle
      APP_CPU_NUM         // Run on Core 1
  );

  // After ESP32 wakes up from deep sleep
  if (esp_sleep_get_wakeup_cause() == ESP_SLEEP_WAKEUP_EXT0)
  {
    Serial.println("Woken up from deep sleep");
  }

  // In setup(), add this after creating the battery monitoring task
  xTaskCreatePinnedToCore(
      ledStatusTask,        // Task function
      "LEDStatus",          // Task name
      2048,                 // Stack size
      NULL,                 // Task parameters
      1,                    // Task priority
      &ledStatusTaskHandle, // Task handle
      APP_CPU_NUM           // Run on Core 1
  );

  // Configure PWM for LEDs
  ledcSetup(PWM_CHANNEL_RED, PWM_FREQUENCY, PWM_RESOLUTION);
  ledcSetup(PWM_CHANNEL_GREEN, PWM_FREQUENCY, PWM_RESOLUTION);
  ledcSetup(PWM_CHANNEL_BLUE, PWM_FREQUENCY, PWM_RESOLUTION);

  // Attach PWM channels to GPIO pins
  ledcAttachPin(ledRed, PWM_CHANNEL_RED);
  ledcAttachPin(ledGreen, PWM_CHANNEL_GREEN);
  ledcAttachPin(ledBlue, PWM_CHANNEL_BLUE);

  // OTA service
  BLEService* otaService = pServer->createService(OTA_SERVICE_UUID);
  BLECharacteristic* otaCommandChar = otaService->createCharacteristic(
    OTA_COMMAND_CHAR_UUID,
    BLECharacteristic::PROPERTY_WRITE
  );
  BLECharacteristic* otaDataChar = otaService->createCharacteristic(
    OTA_DATA_CHAR_UUID,
    BLECharacteristic::PROPERTY_WRITE
  );
  BLECharacteristic* otaStatusChar = otaService->createCharacteristic(
    OTA_STATUS_CHAR_UUID,
    BLECharacteristic::PROPERTY_NOTIFY | BLECharacteristic::PROPERTY_READ
  );
  otaCommandChar->setCallbacks(new OTACommandCallbacks());
  otaDataChar->setCallbacks(new OTADataCallbacks());
  otaStatusChar->addDescriptor(new BLE2902());
  otaService->start();
}

void loop()
{
  static unsigned long pressStartTime = 0;
  static bool buttonPressed = false;

  // Button handling
  if (digitalRead(wifiButton) == LOW)
  { // Button is pressed
    if (!buttonPressed)
    { // Button was just pressed
      pressStartTime = millis();
      buttonPressed = true;
      Serial.println("Button pressed");

      // Handle secret combination
      unsigned long currentTime = millis();
      if (currentTime - lastPressTime < SECRET_PRESS_WINDOW)
      {
        pressCount++;
        if (pressCount >= SECRET_PRESS_COUNT)
        {
          // Secret combination detected - open WiFi manager
         // openWiFiPortal();
          pressCount = 0; // Reset press count
        }
      }
      else
      {
        // Reset if too much time has passed
        pressCount = 1;
      }
      lastPressTime = currentTime;
    }

    // Check for long press while button is still held
    if ((millis() - pressStartTime) > TURN_OFF_TIME)
    {
      Serial.println("TURN OFF TIME DETECTED");
      goToDeepSleep();
    }
  }
  else if (buttonPressed)
  { // Button was released
    buttonPressed = false;
  }
}

// --- OTA CALLBACKS ---------------------------------------------------------
class OTACommandCallbacks : public BLECharacteristicCallbacks {
  void onWrite(BLECharacteristic* chr) override {
    std::string cmd = chr->getValue();
    if (cmd.rfind("START:", 0) == 0) {
      otaFirmwareSize = atoi(cmd.substr(6).c_str());
      if (Update.begin(otaFirmwareSize)) {
        otaInProgress   = true;
        otaReceivedSize = 0;
        // Find the status characteristic and notify
        BLEService* otaService = pServer->getServiceByUUID(OTA_SERVICE_UUID);
        if (otaService) {
          BLECharacteristic* statusChar = otaService->getCharacteristic(OTA_STATUS_CHAR_UUID);
          if (statusChar) {
            statusChar->setValue("READY");
            statusChar->notify();
            Serial.println("OTA: Ready to receive firmware");
          }
        }
      } else {
        BLEService* otaService = pServer->getServiceByUUID(OTA_SERVICE_UUID);
        if (otaService) {
          BLECharacteristic* statusChar = otaService->getCharacteristic(OTA_STATUS_CHAR_UUID);
          if (statusChar) {
            statusChar->setValue("ERROR:OTA_BEGIN");
            statusChar->notify();
            Serial.println("OTA: Failed to begin update");
          }
        }
      }
    }
    else if (cmd == "END") {
      if (Update.end(true)) {
        BLEService* otaService = pServer->getServiceByUUID(OTA_SERVICE_UUID);
        if (otaService) {
          BLECharacteristic* statusChar = otaService->getCharacteristic(OTA_STATUS_CHAR_UUID);
          if (statusChar) {
            statusChar->setValue("SUCCESS");
            statusChar->notify();
            Serial.println("OTA: Update successful, restarting...");
          }
        }
        delay(1000);
        ESP.restart();
      } else {
        BLEService* otaService = pServer->getServiceByUUID(OTA_SERVICE_UUID);
        if (otaService) {
          BLECharacteristic* statusChar = otaService->getCharacteristic(OTA_STATUS_CHAR_UUID);
          if (statusChar) {
            statusChar->setValue("ERROR:OTA_END");
            statusChar->notify();
            Serial.println("OTA: Failed to end update");
          }
        }
      }
      otaInProgress = false;
    }
  }
};

class OTADataCallbacks : public BLECharacteristicCallbacks {
  void onWrite(BLECharacteristic* chr) override {
    if (!otaInProgress) return;
    
    std::string chunk = chr->getValue();
    size_t chunkSize = chunk.size();
    
    // Sprawdź czy fragment nie jest za duży
    if (chunkSize > OTA_CHUNK_SIZE) {
      Serial.printf("OTA: Chunk too large (%d bytes), max is %d\n", chunkSize, OTA_CHUNK_SIZE);
      BLEService* otaService = pServer->getServiceByUUID(OTA_SERVICE_UUID);
      if (otaService) {
        BLECharacteristic* statusChar = otaService->getCharacteristic(OTA_STATUS_CHAR_UUID);
        if (statusChar) {
          statusChar->setValue("ERROR:CHUNK_TOO_LARGE");
          statusChar->notify();
        }
      }
      return;
    }
    
    if (Update.write((uint8_t*)chunk.data(), chunkSize) == chunkSize) {
      otaReceivedSize += chunkSize;
      int progress = (otaReceivedSize * 100) / otaFirmwareSize;
      
      // Wysyłaj status co 5% postępu (żeby nie spamować)
      static int lastProgress = 0;
      if (progress >= lastProgress + 5 || progress == 100) {
        char buf[20];
        sprintf(buf, "PROGRESS:%d%%", progress);
        
        BLEService* otaService = pServer->getServiceByUUID(OTA_SERVICE_UUID);
        if (otaService) {
          BLECharacteristic* statusChar = otaService->getCharacteristic(OTA_STATUS_CHAR_UUID);
          if (statusChar) {
            statusChar->setValue(buf);
            statusChar->notify();
            Serial.printf("OTA: Progress %d%% (%d/%d bytes)\n", progress, otaReceivedSize, otaFirmwareSize);
          }
        }
        lastProgress = progress;
      }
    } else {
      Serial.println("OTA: Failed to write chunk");
      BLEService* otaService = pServer->getServiceByUUID(OTA_SERVICE_UUID);
      if (otaService) {
        BLECharacteristic* statusChar = otaService->getCharacteristic(OTA_STATUS_CHAR_UUID);
        if (statusChar) {
          statusChar->setValue("ERROR:OTA_WRITE");
          statusChar->notify();
        }
      }
    }
  }
};