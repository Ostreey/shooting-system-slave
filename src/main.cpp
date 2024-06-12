
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

int sensorNumber = 1;

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

  BLECharacteristic piezoCharateristic(CHARACTERISTIC1, BLECharacteristic::PROPERTY_NOTIFY| BLECharacteristic::PROPERTY_WRITE);
  BLEDescriptor piezoDescriptor(BLEUUID((uint16_t)0x2902));


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

void ledOff() {
  digitalWrite(connectedLed, HIGH); 
  if (deviceConnected) {
  
    piezoCharateristic.setValue("got hit");
    piezoCharateristic.notify();
    Serial.println("Sent 'got hit' notification.");
  } else {
    Serial.println("Device not connected, cannot send notification.");
  }
      
}

void setup() {

  Serial.begin(9600);
  Serial.println("Start");

 pinMode(activeLed, OUTPUT);
  digitalWrite(activeLed, LOW); 
  pinMode(connectedLed, OUTPUT);
  digitalWrite(connectedLed, HIGH); 

  sensor.begin();

  // Create the BLE Device
  if(sensorNumber == 1){
    BLEDevice::init(bleServerName1);
  } else if (sensorNumber == 2){
    BLEDevice::init(bleServerName2);
  } 
  else if (sensorNumber == 3){
    BLEDevice::init(bleServerName3);
  }
  

  // Create the BLE Server
  BLEServer *pServer = BLEDevice::createServer();
  pServer->setCallbacks(new MyServerCallbacks());

  // Create the BLE Service
  BLEService *piezoService;
  if(sensorNumber == 1){
    piezoService = pServer->createService(SERVICE_UUID1);
  } else if (sensorNumber == 2){
    piezoService = pServer->createService(SERVICE_UUID2);
  } else if (sensorNumber == 3){
    piezoService = pServer->createService(SERVICE_UUID3);
  }

    piezoService->addCharacteristic(&piezoCharateristic);

    piezoCharateristic.addDescriptor(&piezoDescriptor);
    piezoCharateristic.setCallbacks(new WriteCallbacks());
 
  
  // Start the service
  piezoService->start();
 sensor.setCallback(ledOff);
  // Start advertising
  BLEAdvertising *pAdvertising = BLEDevice::getAdvertising();
  if(sensorNumber == 1){
    pAdvertising->addServiceUUID(SERVICE_UUID1);
  } else if (sensorNumber == 2){
    pAdvertising->addServiceUUID(SERVICE_UUID2);
  }
  else if (sensorNumber == 3){
    pAdvertising->addServiceUUID(SERVICE_UUID3);
  }
  pServer->getAdvertising()->start();
  Serial.println("Waiting a client connection to notify...");
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