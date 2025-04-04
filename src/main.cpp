#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>
#include <Wire.h>
#include <Arduino.h>
#include "PiezoSensor.h"
#include <WiFiManager.h>

//Default Temperature is in Celsius
//Comment the next line for Temperature in Fahrenheit
#define temperatureCelsius

//BLE server name
#define bleServerName1 "SHOOTING TARGET"
#define bleServerName2 "SLAVE_2"
#define bleServerName3 "SLAVE_3"

int sensorNumber = 1;

float temp;

bool deviceConnected = false;

const int ledBlue = 17;
const int ledGreen = 18;
const int ledRed = 2;
const int wifiButton = 15;  // New button pin definition

PiezoSensor sensor(34, 400);


#define SERVICE_UUID1 "91bad492-b950-4226-aa2b-4ede9fa42fff"
#define SERVICE_UUID2 "91bad492-b950-4226-aa2b-4ede9fa42f51"
#define SERVICE_UUID3 "91bad492-b950-4226-aa2b-4ede9fa42333"
#define CHARACTERISTIC1 "cba1d466-344c-4be3-ab3f-189f80dd75ff"
#define CHARACTERISTIC2 "cba1d466-344c-4be3-ab3f-189f80dd7511"
#define CHARACTERISTIC3 "cba1d466-344c-4be3-ab3f-189f80dd7333"

 // BLECharacteristic piezoCharateristic(CHARACTERISTIC2, BLECharacteristic::PROPERTY_NOTIFY| BLECharacteristic::PROPERTY_WRITE);
  //BLEDescriptor piezoDescriptor(BLEUUID((uint16_t)0x2902));

BLECharacteristic *piezoCharacteristic;
BLEDescriptor *piezoDescriptor;
BLEService *piezoService;
BLEServer *pServer;

// Add these constants at the top with other definitions
const int LONG_PRESS_TIME = 3000;  // 3 seconds for long press
const int SHORT_PRESS_TIME = 50;   // 50ms debounce time for short press
bool isInDeepSleep = false;

class WriteCallbacks: public BLECharacteristicCallbacks {
  void onWrite(BLECharacteristic *pCharacteristic) {
    std::string value = pCharacteristic->getValue();
    Serial.print("Received value: ");
    Serial.println(value.c_str());
    // Handle the received value
    if (value == "start") {
      digitalWrite(ledRed, HIGH);
      digitalWrite(ledGreen, HIGH);
      digitalWrite(ledBlue, HIGH);
    }
  }
};



//Setup callbacks onConnect and onDisconnect
class MyServerCallbacks: public BLEServerCallbacks {
  void onConnect(BLEServer* pServer) {
    Serial.println("Device connected");
    deviceConnected = true;
    for(int i = 0; i < 3; i++){
    digitalWrite(ledRed, HIGH); 
    digitalWrite(ledGreen, HIGH);
    digitalWrite(ledBlue, HIGH);
    delay(200);
    digitalWrite(ledRed, LOW);
    digitalWrite(ledGreen, LOW);
    digitalWrite(ledBlue, LOW);
    delay(200);
    }
  };
  void onDisconnect(BLEServer* pServer) {
    deviceConnected = false;
     pServer->getAdvertising()->start();
  }
};

void ledOff(int piezoValue) {
  digitalWrite(ledRed, LOW);
  digitalWrite(ledGreen, LOW);
  digitalWrite(ledBlue, LOW);
  if (deviceConnected) {
  
    String valueToSend = String(piezoValue); // Convert to String
     piezoCharacteristic->setValue(valueToSend.c_str());
    piezoCharacteristic->notify();
   Serial.print("Sent piezo value: ");
    Serial.println(valueToSend);
  } else {
    Serial.println("Device not connected, cannot send notification.");
  }
      
}

void goToDeepSleep() {
  Serial.println("Going to deep sleep...");
 
  esp_sleep_enable_ext0_wakeup(GPIO_NUM_15, 0);
  
  // Go to sleep
  esp_deep_sleep_start();
}

void setup() {
  Serial.begin(9600);
  Serial.println("Start");
  Serial.print("device number: ");
  Serial.println(sensorNumber);

  pinMode(ledBlue, OUTPUT);
  digitalWrite(ledBlue, LOW);
  pinMode(ledGreen, OUTPUT);
  digitalWrite(ledGreen, LOW);
  pinMode(ledRed, OUTPUT);
  digitalWrite(ledRed, LOW); 
  pinMode(wifiButton, INPUT_PULLUP);  // Initialize button pin

  sensor.begin();
  sensor.setCallback(ledOff);

  // Determine the device name based on sensorNumber
  String deviceName;
  if(sensorNumber == 1){
    deviceName = bleServerName1;
    Serial.print("Initialized as SLAVE_1 ");
  } else if (sensorNumber == 2){
    deviceName = bleServerName2;
    Serial.print("Initialized as SLAVE_2 ");
  } else if (sensorNumber == 3){
    deviceName = bleServerName3;
    Serial.print("Initialized as SLAVE_3 ");
  }

  // Initialize BLE Device with the device name
  BLEDevice::init(deviceName.c_str());

  // Create the BLE Server
  pServer = BLEDevice::createServer();
  pServer->setCallbacks(new MyServerCallbacks());

  // Create the BLE Service and Characteristic
  if(sensorNumber == 1){
    piezoService = pServer->createService(SERVICE_UUID1);
    piezoCharacteristic = piezoService->createCharacteristic(
                            CHARACTERISTIC1,
                            BLECharacteristic::PROPERTY_NOTIFY | BLECharacteristic::PROPERTY_WRITE
                          );
  } else if (sensorNumber == 2){
    piezoService = pServer->createService(SERVICE_UUID2);
    piezoCharacteristic = piezoService->createCharacteristic(
                            CHARACTERISTIC2,
                            BLECharacteristic::PROPERTY_NOTIFY | BLECharacteristic::PROPERTY_WRITE
                          );
  } else if (sensorNumber == 3){
    piezoService = pServer->createService(SERVICE_UUID3);
    piezoCharacteristic = piezoService->createCharacteristic(
                            CHARACTERISTIC3,
                            BLECharacteristic::PROPERTY_NOTIFY | BLECharacteristic::PROPERTY_WRITE
                          );
  }

  // Add Descriptor and Callbacks
  piezoCharacteristic->addDescriptor(new BLE2902());
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

  // After ESP32 wakes up from deep sleep
  if (esp_sleep_get_wakeup_cause() == ESP_SLEEP_WAKEUP_EXT0) {
    Serial.println("Woken up from deep sleep");
  }
}

void loop() {
  static unsigned long pressStartTime = 0;
  static bool buttonPressed = false;
  
  // Button handling
  if (digitalRead(wifiButton) == LOW) {  // Button is pressed
    if (!buttonPressed) {  // Button was just pressed
      pressStartTime = millis();
      buttonPressed = true;
      Serial.println("Button pressed");
    }
    
    // Check for long press while button is still held
    if ((millis() - pressStartTime) > LONG_PRESS_TIME) {
      Serial.println("Long press detected");
       for(int i = 0; i < 3; i++){
  digitalWrite(ledBlue, LOW);
  digitalWrite(ledGreen, LOW);
  digitalWrite(ledRed, LOW);
  delay(200);
  digitalWrite(ledBlue, HIGH);
  digitalWrite(ledGreen, HIGH);
  digitalWrite(ledRed, HIGH);
  delay(200);
  }
  
      vTaskDelay(3000 / portTICK_PERIOD_MS);
      goToDeepSleep();
    }
  }
  else if (buttonPressed) {  // Button was released
    if ((millis() - pressStartTime) > SHORT_PRESS_TIME &&
        (millis() - pressStartTime) < LONG_PRESS_TIME) {
      // Short press detected
      Serial.println("WiFi button pressed, starting WiFiManager portal");
      WiFiManager wifiManager;
      wifiManager.startConfigPortal("ShootingSystemAP");
    }
    buttonPressed = false;
  }

  if (!deviceConnected) {
    Serial.println("Device not connected");
    digitalWrite(ledRed, LOW); 
    digitalWrite(ledGreen, LOW);
    digitalWrite(ledBlue, LOW);
    vTaskDelay(1000 / portTICK_PERIOD_MS);
    digitalWrite(ledRed, HIGH);
    digitalWrite(ledGreen, HIGH);
    digitalWrite(ledBlue, HIGH);
    vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
   
}