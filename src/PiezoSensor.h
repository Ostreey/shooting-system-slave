#ifndef PIEZOSENSOR_H
#define PIEZOSENSOR_H

#include <Arduino.h>
#include "CalibrationStore.h"


typedef void (*HitCallback)(int piezoValue);
typedef void (*PeakCallback)(uint16_t peakValue);


class PiezoSensor {
  private:
    static constexpr uint16_t NOISE_FLOOR = 30;

    int pin;
    volatile uint16_t threshold;
    volatile uint16_t debounceMs;
    volatile bool measurementMode;
    int cnt;
    HitCallback callback;
    PeakCallback peakCallback;
    TaskHandle_t taskHandle;
    unsigned long lastHitTime;
    uint16_t peakBuffer;
    bool peakActive;


    static void sensorTask(void *pvParameters) {
      PiezoSensor *sensor = static_cast<PiezoSensor*>(pvParameters);
      for (;;) {
        sensor->update();
        vTaskDelay(2 / portTICK_PERIOD_MS);
      }
    }

  public:

    PiezoSensor(int pin)
      {
        this->pin = pin;
        this->threshold = CalibrationStore::THRESHOLD_DEFAULT;
        this->debounceMs = CalibrationStore::DEBOUNCE_DEFAULT_MS;
        this->measurementMode = false;
        cnt = 0;
        callback = nullptr;
        peakCallback = nullptr;
        taskHandle = NULL;
        lastHitTime = 0;
        peakBuffer = 0;
        peakActive = false;
      }


    void begin() {
      for (int i = 0; i < 10; i++) {
        analogRead(pin);
        delay(50);
      }


      xTaskCreatePinnedToCore(
        sensorTask,          // Task function
        "Sensor Task",       // Task name
        10000,               // Stack size
        this,                // Parameters
        3,                   // Priority (increased from 1 to 3)
        &taskHandle,         // Task handle
        PRO_CPU_NUM          // Run on Core 0
      );
    }


    void setCallback(HitCallback callback) {
      this->callback = callback;
    }

    void setPeakCallback(PeakCallback cb) {
      this->peakCallback = cb;
    }

    void setThreshold(uint16_t value) {
      threshold = CalibrationStore::clampThreshold(value);
    }

    void setDebounceMs(uint16_t value) {
      debounceMs = CalibrationStore::clampDebounceMs(value);
    }

    uint16_t getThreshold() const {
      return threshold;
    }

    uint16_t getDebounceMs() const {
      return debounceMs;
    }

    void startMeasurement() {
      peakBuffer = 0;
      peakActive = false;
      measurementMode = true;
    }

    void stopMeasurement() {
      measurementMode = false;
      peakBuffer = 0;
      peakActive = false;
    }

    bool isInMeasurement() const {
      return measurementMode;
    }


    void update() {
      int piezoValue = analogRead(pin);
      unsigned long currentTime = millis();

      if (cnt <= 50) {
        cnt++;
        return;
      }

      if (measurementMode) {
        if (piezoValue > NOISE_FLOOR) {
          if ((uint16_t)piezoValue > peakBuffer) {
            peakBuffer = (uint16_t)piezoValue;
          }
          peakActive = true;
        } else if (peakActive) {
          if (currentTime - lastHitTime > debounceMs) {
            if (peakCallback) {
              peakCallback(peakBuffer);
            }
            lastHitTime = currentTime;
          }
          peakBuffer = 0;
          peakActive = false;
        }
        return;
      }

      if (piezoValue > threshold && (currentTime - lastHitTime > debounceMs)) {
        Serial.println(piezoValue);
        if (callback) {
          callback(piezoValue);
        }
        lastHitTime = currentTime;
      }
    }
};

#endif
