#include "BatteryScale.h"
#include "Config.h"

namespace
{
    int voltageToPercent(int vx100)
    {
        // Li-Ion discharge curve: {voltage*100, percent}
        static const struct { uint16_t v; uint8_t p; } lut[] = {
            {420, 100}, {415, 95}, {411, 90}, {408, 85},
            {402, 80},  {398, 75}, {395, 70}, {391, 65},
            {387, 60},  {383, 55}, {379, 50}, {375, 45},
            {371, 40},  {367, 35}, {363, 30}, {359, 25},
            {355, 20},  {350, 15}, {345, 10}, {340,  5},
            {330,  2},  {300,  0},
        };
        static const int N = sizeof(lut) / sizeof(lut[0]);

        if (vx100 >= lut[0].v)   return 100;
        if (vx100 <= lut[N-1].v) return 0;

        for (int i = 0; i < N - 1; i++)
        {
            if (vx100 >= lut[i + 1].v)
                return lut[i + 1].p + (vx100 - lut[i + 1].v) * (lut[i].p - lut[i + 1].p) / (lut[i].v - lut[i + 1].v);
        }
        return 0;
    }
}

namespace BatteryScale
{
    int32_t cellMvFromAdcMv(int32_t adcMv)
    {
        return adcMv * 3 / 2 + ADC_BATTERY_OFFSET_MV;
    }

    int percentFromMv(int32_t batteryMv)
    {
        int percent = voltageToPercent((int)(batteryMv / 10));
        if (percent < 0)
            return 0;
        if (percent > 100)
            return 100;
        return percent;
    }
}
