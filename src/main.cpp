#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>
#include <Wire.h>
#include <Arduino.h>
#include "PiezoSensor.h"
#include <WiFiManager.h>
#include <esp_adc_cal.h>

// Default Temperature is in Celsius
// Comment the next line for Temperature in Fahrenheit
#define temperatureCelsius

// BLE server name
#define bleServerName "SHOOTING TARGET"

#define SERVICE_UUID "91bad492-b950-4226-aa2b-4ede9fa42fff"
#define CHARACTERISTIC "cba1d466-344c-4be3-ab3f-189f80dd75ff"
#define BATTERY_CHARACTERISTIC "cba1d466-344c-4be3-ab3f-189f80dd76ff"


// Add these constants for ADC calibration
#define DEFAULT_VREF 1100         // Default reference voltage in mV
#define ADC_SAMPLES 64            // Number of samples for averaging
#define ADC_ATTEN ADC_ATTEN_DB_11 // 11dB attenuation for 0-3.3V range

float temp;

bool deviceConnected = false;

const int ledBlue = 17;
const int ledGreen = 18;
const int ledRed = 19;
const int wifiButton = 15; // New button pin definition
const int batteryPin = 13; // Battery voltage measurement pin

PiezoSensor sensor(34, 400);


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
const int TURN_OFF_TIME = 2000; // 2 seconds for long press
const int SECRET_PRESS_WINDOW = 2000; // 2 second window for secret combination
const int SECRET_PRESS_COUNT = 3; // Number of presses needed for secret combination
bool isWebPortalOpen = false,  isGoingToSleep = false;
// Add these variables in the global scope
unsigned long lastPressTime = 0;
int pressCount = 0;

// Add this enum before the WriteCallbacks class
enum class BLECommand {
    UNKNOWN,
    START,
    SLEEP,
    BLINK,
    GAME1,
    // Add more commands as needed
};

// Helper function to convert string to BLECommand
BLECommand stringToCommand(const std::string& value) {
    if (value == "start") return BLECommand::START;
    if (value == "sleep") return BLECommand::SLEEP;
    if (value == "game1") return BLECommand::GAME1;
    if (value == "blink") return BLECommand::BLINK;
    return BLECommand::UNKNOWN;
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

void blinkLEDS(){
   for (int i = 0; i < 3; i++)
    {
      digitalWrite(ledRed, HIGH);
      digitalWrite(ledGreen, HIGH);
      digitalWrite(ledBlue, HIGH);
      delay(200);
      digitalWrite(ledRed, LOW);
      digitalWrite(ledGreen, LOW);
      digitalWrite(ledBlue, LOW);
      delay(200);
    }
}

int getBatteryPercentage() {
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
  Serial.println("Going to deep sleep...");
  esp_sleep_enable_ext0_wakeup(GPIO_NUM_15, 0);
  esp_deep_sleep_start();
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
void ledStatusTask(void *pvParameters) {
    for (;;) {
        if (!deviceConnected && !isGoingToSleep) {
            digitalWrite(ledRed, LOW);
            digitalWrite(ledGreen, LOW);
            digitalWrite(ledBlue, LOW);
            vTaskDelay(1000 / portTICK_PERIOD_MS);
            digitalWrite(ledRed, HIGH);
            digitalWrite(ledGreen, HIGH);
            digitalWrite(ledBlue, HIGH);
            vTaskDelay(1000 / portTICK_PERIOD_MS);
             Serial.println("Device not connected");
        } else {
            vTaskDelay(100 / portTICK_PERIOD_MS); // Small delay when connected
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
    
    BLECommand cmd = stringToCommand(value);
    
    switch (cmd) {
        case BLECommand::START:
            digitalWrite(ledRed, HIGH);
            digitalWrite(ledGreen, HIGH);
            digitalWrite(ledBlue, HIGH);
            break;
            
        case BLECommand::SLEEP:
            goToDeepSleep();
            break;

        case BLECommand::BLINK:
            blinkLEDS();
            break;
            
        case BLECommand::UNKNOWN:
            Serial.println("Unknown command received");
            break;
    }
  }
};

void initialBatteryTask(void *pvParameters) {
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



void ledOff(int piezoValue)
{
  digitalWrite(ledRed, LOW);
  digitalWrite(ledGreen, LOW);
  digitalWrite(ledBlue, LOW);
  if (deviceConnected)
  {

    String valueToSend = String(piezoValue); // Convert to String
    piezoCharacteristic->setValue(valueToSend.c_str());
    piezoCharacteristic->notify();
    Serial.print("Sent piezo value: ");
    Serial.println(valueToSend);
  }
  else
  {
    Serial.println("Device not connected, cannot send notification.");
  }
}

void openWiFiPortal() {
    // Stop BLE advertising before opening portal
    BLEDevice::getAdvertising()->stop();
    
    // Visual indication
    for (int i = 0; i < 5; i++) {
        digitalWrite(ledBlue, LOW);
        digitalWrite(ledGreen, LOW);
        digitalWrite(ledRed, LOW);
        delay(50);
        digitalWrite(ledBlue, HIGH);
        digitalWrite(ledGreen, HIGH);
        digitalWrite(ledRed, HIGH);
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
      ledStatusTask,    // Task function
      "LEDStatus",      // Task name
      2048,            // Stack size
      NULL,            // Task parameters
      1,               // Task priority
      &ledStatusTaskHandle, // Task handle
      APP_CPU_NUM      // Run on Core 1
  );
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
      if (currentTime - lastPressTime < SECRET_PRESS_WINDOW) {
        pressCount++;
        if (pressCount >= SECRET_PRESS_COUNT) {
          // Secret combination detected - open WiFi manager
          openWiFiPortal();
          pressCount = 0; // Reset press count
        }
      } else {
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