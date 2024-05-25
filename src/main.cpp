#include <Arduino.h>
#include "PiezoSensor.h"

const int ledPin = 2;     // Digital pin for the LED

PiezoSensor sensor(15, 400);


void ledOff() {
  digitalWrite(ledPin, LOW); 
}

void setup() {
  Serial.begin(9600);


  pinMode(ledPin, OUTPUT);
  digitalWrite(ledPin, HIGH); 

  sensor.setCallback(ledOff);
  sensor.begin();
}

void loop() {
}
