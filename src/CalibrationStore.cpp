#include "CalibrationStore.h"
#include <Preferences.h>

namespace
{
    constexpr const char *NS = "piezo";
    constexpr const char *KEY_THRESHOLD = "thr";
    constexpr const char *KEY_DEBOUNCE = "deb";

    Preferences prefs;
}

void CalibrationStore::begin()
{
    // Touch the namespace once so it exists, then close. Each accessor
    // re-opens scoped to avoid keeping the NVS handle held during runtime.
    prefs.begin(NS, false);
    prefs.end();
}

uint16_t CalibrationStore::loadThreshold()
{
    prefs.begin(NS, true);
    uint16_t value = prefs.getUShort(KEY_THRESHOLD, THRESHOLD_DEFAULT);
    prefs.end();
    return clampThreshold(value);
}

uint16_t CalibrationStore::loadDebounceMs()
{
    prefs.begin(NS, true);
    uint16_t value = prefs.getUShort(KEY_DEBOUNCE, DEBOUNCE_DEFAULT_MS);
    prefs.end();
    return clampDebounceMs(value);
}

void CalibrationStore::saveThreshold(uint16_t threshold)
{
    uint16_t clamped = clampThreshold(threshold);
    prefs.begin(NS, false);
    prefs.putUShort(KEY_THRESHOLD, clamped);
    prefs.end();
    Serial.printf("CalibrationStore: threshold saved = %u\n", clamped);
}

void CalibrationStore::saveDebounceMs(uint16_t debounceMs)
{
    uint16_t clamped = clampDebounceMs(debounceMs);
    prefs.begin(NS, false);
    prefs.putUShort(KEY_DEBOUNCE, clamped);
    prefs.end();
    Serial.printf("CalibrationStore: debounce saved = %u ms\n", clamped);
}

void CalibrationStore::resetToDefaults()
{
    prefs.begin(NS, false);
    prefs.remove(KEY_THRESHOLD);
    prefs.remove(KEY_DEBOUNCE);
    prefs.end();
    Serial.println("CalibrationStore: reset to defaults");
}

uint16_t CalibrationStore::clampThreshold(uint16_t value)
{
    if (value < THRESHOLD_MIN) return THRESHOLD_MIN;
    if (value > THRESHOLD_MAX) return THRESHOLD_MAX;
    return value;
}

uint16_t CalibrationStore::clampDebounceMs(uint16_t value)
{
    if (value < DEBOUNCE_MIN_MS) return DEBOUNCE_MIN_MS;
    if (value > DEBOUNCE_MAX_MS) return DEBOUNCE_MAX_MS;
    return value;
}
