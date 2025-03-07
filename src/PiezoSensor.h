#ifndef PIEZOSENSOR_H
#define PIEZOSENSOR_H

#include <Arduino.h>


typedef void (*HitCallback)(int piezoValue);


class PiezoSensor {
  private:
    int pin;
    int threshold;
    int cnt;
    HitCallback callback;
    TaskHandle_t taskHandle;
    unsigned long lastHitTime;      // Track time of last hit
    const int debounceTime = 200;   // Debounce period in ms

 
    static void sensorTask(void *pvParameters) {
      PiezoSensor *sensor = static_cast<PiezoSensor*>(pvParameters);
      for (;;) {
        sensor->update(); 
        vTaskDelay(5 / portTICK_PERIOD_MS);
      }
    }

  public:

    PiezoSensor(int pin, int threshold) 
      {
        this->pin = pin;
        this->threshold = threshold;
        cnt = 0;
        callback = nullptr;
        taskHandle = NULL;
        lastHitTime = 0;
      }


    void begin() {
      for (int i = 0; i < 10; i++) {
        analogRead(pin);
        delay(50); 
      }

     
      xTaskCreate(
        sensorTask,          // Task function
        "Sensor Task",       // Task name
        10000,               // Stack size
        this,                // Parameters
        1,                   // Priority
        &taskHandle          // Task handle
      );
    }


    void setCallback(HitCallback callback) {
      this->callback = callback;
    }

   
    void update() {
      int piezoValue = analogRead(pin);
      unsigned long currentTime = millis();
    
      if (cnt > 50) {
       
        if (piezoValue > threshold&& (currentTime - lastHitTime > debounceTime)) {
          Serial.println(piezoValue);
          if (callback) {
            callback(piezoValue);
          }
          lastHitTime = currentTime; 
        }
      } else {
        cnt++;
      }
    }
};

#endif
