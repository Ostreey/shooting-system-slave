#ifndef CALIBRATIONSTORE_H
#define CALIBRATIONSTORE_H

#include <Arduino.h>

class CalibrationStore
{
public:
    static constexpr uint16_t THRESHOLD_DEFAULT = 400;
    static constexpr uint16_t DEBOUNCE_DEFAULT_MS = 200;

    static constexpr uint16_t THRESHOLD_MIN = 50;
    static constexpr uint16_t THRESHOLD_MAX = 3500;
    static constexpr uint16_t DEBOUNCE_MIN_MS = 20;
    static constexpr uint16_t DEBOUNCE_MAX_MS = 1000;

    static void begin();

    static uint16_t loadThreshold();
    static uint16_t loadDebounceMs();

    static void saveThreshold(uint16_t threshold);
    static void saveDebounceMs(uint16_t debounceMs);

    static void resetToDefaults();

    static uint16_t clampThreshold(uint16_t value);
    static uint16_t clampDebounceMs(uint16_t value);
};

#endif
