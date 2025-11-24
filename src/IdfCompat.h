#pragma once

#include <algorithm>
#include <cstdint>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_timer.h"

#ifdef __cplusplus
extern "C" {
#endif
#include "sdkconfig.h"
#ifdef __cplusplus
}
#endif

inline uint64_t millis64()
{
    return esp_timer_get_time() / 1000ULL;
}

inline uint32_t millis32()
{
    return static_cast<uint32_t>(millis64() & 0xFFFFFFFFULL);
}

inline void delay_ms(uint32_t ms)
{
    vTaskDelay(pdMS_TO_TICKS(ms));
}

template <typename T>
inline T clamp_value(T value, T min_value, T max_value)
{
    return std::clamp(value, min_value, max_value);
}

inline int map_value(int x, int in_min, int in_max, int out_min, int out_max)
{
    if (in_max == in_min)
    {
        return out_min;
    }
    int64_t numerator = static_cast<int64_t>(x - in_min) * static_cast<int64_t>(out_max - out_min);
    int64_t result = static_cast<int64_t>(out_min) + numerator / static_cast<int64_t>(in_max - in_min);
    return static_cast<int>(result);
}

inline BaseType_t app_cpu()
{
#if CONFIG_FREERTOS_UNICORE
    return 0;
#else
    return 1;
#endif
}
