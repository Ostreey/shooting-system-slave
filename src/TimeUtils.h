#ifndef TIME_UTILS_H
#define TIME_UTILS_H

#include <cstdint>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <esp_timer.h>

inline uint32_t millis()
{
    return static_cast<uint32_t>(esp_timer_get_time() / 1000);
}

inline void delayMs(uint32_t ms)
{
    vTaskDelay(pdMS_TO_TICKS(ms));
}

#endif
