#ifndef BATTERY_LATCH_STORE_H
#define BATTERY_LATCH_STORE_H

#include <Arduino.h>

// Persistent "this cell is flat" latch. Written only on the start path, before any LED and
// before sleep, so the decision survives a brownout reset from the TPS3823-33 supervisor in the
// middle of the sequence.
class BatteryLatchStore
{
public:
    enum class ClearedBy : uint8_t
    {
        Voltage,
        Button,
    };

    static void begin();

    static bool isFlat();

    // Both writers are no-ops when the stored value already matches: NVS must never be written
    // in a loop. They return false when the write could not be confirmed by a read-back, so a
    // failure is never logged as a success.
    static bool setFlat(int batteryMv);
    static bool clearFlat(int batteryMv, ClearedBy by);
};

#endif // BATTERY_LATCH_STORE_H
