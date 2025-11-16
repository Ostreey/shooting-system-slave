#ifndef PIEZOSENSOR_H
#define PIEZOSENSOR_H

#include <cstdint>

#include "Config.h"
#include "IdfCompat.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/adc.h"

typedef void (*HitCallback)(int piezoValue);

class PiezoSensor
{
private:
    int pin;
    adc1_channel_t adcChannel;
    int threshold;
    int warmupCount;
    HitCallback callback;
    TaskHandle_t taskHandle;
    uint64_t lastHitTimeMs;
    static constexpr int debounceTimeMs = 200;

    static void sensorTask(void *pvParameters)
    {
        auto *sensor = static_cast<PiezoSensor *>(pvParameters);
        for (;;)
        {
            sensor->update();
            vTaskDelay(pdMS_TO_TICKS(10));
        }
    }

public:
    PiezoSensor(int pin, int threshold)
        : pin(pin),
          adcChannel(PIEZO_ADC_CHANNEL),
          threshold(threshold),
          warmupCount(0),
          callback(nullptr),
          taskHandle(nullptr),
          lastHitTimeMs(0)
    {
    }

    void begin()
    {
        adc1_config_width(ADC_WIDTH_BIT_12);
        adc1_config_channel_atten(adcChannel, ADC_ATTEN);

        for (int i = 0; i < 10; ++i)
        {
            adc1_get_raw(adcChannel);
            delay_ms(50);
        }

        xTaskCreatePinnedToCore(
            sensorTask,
            "SensorTask",
            4096,
            this,
            3,
            &taskHandle,
            app_cpu());
    }

    void setCallback(HitCallback newCallback)
    {
        callback = newCallback;
    }

    void update()
    {
        int piezoValue = adc1_get_raw(adcChannel);
        uint64_t currentTime = millis64();

        if (warmupCount > 50)
        {
            if (piezoValue > threshold && (currentTime - lastHitTimeMs) > debounceTimeMs)
            {
                if (callback)
                {
                    callback(piezoValue);
                }
                lastHitTimeMs = currentTime;
            }
        }
        else
        {
            warmupCount++;
        }
    }
};

#endif
