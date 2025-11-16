#ifndef PIEZOSENSOR_H
#define PIEZOSENSOR_H

#include <cstdint>
#include <functional>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <driver/adc.h>
#include "TimeUtils.h"

using HitCallback = std::function<void(int)>;

class PiezoSensor {
  private:
    int pin;
    int threshold;
    int warmupCount;
    HitCallback callback;
    TaskHandle_t taskHandle;
    uint32_t lastHitTime;
    static constexpr int debounceTimeMs = 200;
    static constexpr adc1_channel_t channel = ADC1_CHANNEL_1;

    static void sensorTask(void *pvParameters) {
      PiezoSensor *sensor = static_cast<PiezoSensor*>(pvParameters);
      for (;;) {
        sensor->update();
        vTaskDelay(pdMS_TO_TICKS(2));
      }
    }

  public:
    PiezoSensor(int pin, int threshold)
        : pin(pin),
          threshold(threshold),
          warmupCount(0),
          callback(nullptr),
          taskHandle(nullptr),
          lastHitTime(0) {}

    void begin() {
      adc1_config_width(ADC_WIDTH_BIT_12);
      adc1_config_channel_atten(channel, ADC_ATTEN_DB_11);
      for (int i = 0; i < 10; ++i) {
        adc1_get_raw(channel);
        delayMs(50);
      }
      xTaskCreatePinnedToCore(
          sensorTask,
          "SensorTask",
          4096,
          this,
          3,
          &taskHandle,
          PRO_CPU_NUM);
    }

    void setCallback(HitCallback cb) {
      callback = cb;
    }

    void update() {
      int piezoValue = adc1_get_raw(channel);
      uint32_t currentTime = millis();

      if (warmupCount > 50) {
        if (piezoValue > threshold && (currentTime - lastHitTime > debounceTimeMs)) {
          if (callback) {
            callback(piezoValue);
          }
          lastHitTime = currentTime;
        }
      } else {
        warmupCount++;
      }
    }
};

#endif
