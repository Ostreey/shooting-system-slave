
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>
#include <Wire.h>
#include <Arduino.h>
#include "PiezoSensor.h"

//Default Temperature is in Celsius
//Comment the next line for Temperature in Fahrenheit
#define temperatureCelsius

//BLE server name
#define bleServerName1 "SLAVE_1"
#define bleServerName2 "SLAVE_2"
#define bleServerName3 "SLAVE_3"

int sensorNumber = 2;

float temp;

bool deviceConnected = false;

const int activeLed = 2;
const int connectedLed = 26;

PiezoSensor sensor(25, 400);


#define SERVICE_UUID1 "91bad492-b950-4226-aa2b-4ede9fa42f59"
#define SERVICE_UUID2 "91bad492-b950-4226-aa2b-4ede9fa42f51"
#define SERVICE_UUID3 "91bad492-b950-4226-aa2b-4ede9fa42333"
#define CHARACTERISTIC1 "cba1d466-344c-4be3-ab3f-189f80dd7518"
#define CHARACTERISTIC2 "cba1d466-344c-4be3-ab3f-189f80dd7511"
#define CHARACTERISTIC3 "cba1d466-344c-4be3-ab3f-189f80dd7333"

 // BLECharacteristic piezoCharateristic(CHARACTERISTIC2, BLECharacteristic::PROPERTY_NOTIFY| BLECharacteristic::PROPERTY_WRITE);
  //BLEDescriptor piezoDescriptor(BLEUUID((uint16_t)0x2902));

BLECharacteristic *piezoCharacteristic;
BLEDescriptor *piezoDescriptor;
BLEService *piezoService;
BLEServer *pServer;



class WriteCallbacks: public BLECharacteristicCallbacks {
  void onWrite(BLECharacteristic *pCharacteristic) {
    std::string value = pCharacteristic->getValue();
    Serial.print("Received value: ");
    Serial.println(value.c_str());
    // Handle the received value
    if (value == "start") {
      digitalWrite(connectedLed, LOW);
    }
  }
};



//Setup callbacks onConnect and onDisconnect
class MyServerCallbacks: public BLEServerCallbacks {
  void onConnect(BLEServer* pServer) {
    deviceConnected = true;
    digitalWrite(connectedLed, HIGH); 
    delay(200);
    digitalWrite(connectedLed, LOW);
    delay(200);
    digitalWrite(connectedLed, HIGH);
    delay(200);
    digitalWrite(connectedLed, LOW);
    delay(200);
    digitalWrite(connectedLed, HIGH);
    delay(200);
    digitalWrite(connectedLed, LOW);
    delay(200);
    digitalWrite(connectedLed, HIGH);
  };
  void onDisconnect(BLEServer* pServer) {
    deviceConnected = false;
     pServer->getAdvertising()->start();
  }
};

void ledOff(int piezoValue) {
  digitalWrite(connectedLed, HIGH); 
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

void setup() {
  Serial.begin(9600);
  Serial.println("Start");
  Serial.print("device number: ");
  Serial.println(sensorNumber);

  pinMode(activeLed, OUTPUT);
  digitalWrite(activeLed, LOW); 
  pinMode(connectedLed, OUTPUT);
  digitalWrite(connectedLed, HIGH); 

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
}

void loop() {
  

  if (!deviceConnected) {
    digitalWrite(connectedLed, LOW); 
    delay(500);
    digitalWrite(connectedLed, HIGH);
    delay(500);
  }
  delay(100);
}