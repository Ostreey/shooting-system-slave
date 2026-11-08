#include "BatteryLatchStore.h"
#include <Preferences.h>

namespace
{
    constexpr const char *NS = "battguard";
    constexpr const char *KEY_FLAT = "flat";

    Preferences prefs;

    bool cachedFlat = false;
    bool cacheValid = false;

    bool readFlat(bool &ok)
    {
        if (!prefs.begin(NS, true))
        {
            // Namespace missing is the normal state of a target that never latched; any other
            // open failure is indistinguishable from it here, so the caller is told it is not
            // a trustworthy read.
            ok = false;
            return false;
        }
        bool value = prefs.getBool(KEY_FLAT, false);
        prefs.end();
        ok = true;
        return value;
    }

    // Writes and then reads the value back. Without the read-back a failed write is logged as a
    // success, and the only artifact of this whole guard would claim protection that is not
    // there.
    bool writeFlatVerified(bool value)
    {
#ifdef MIGRATION_TEST_NO_BATTERY
        // The no-battery test build never touches the latch. `pio run -t upload` does not erase
        // NVS, so a latch written during a debugging session would outlive the reflash and the
        // next QA scenario would meet a target that "refuses to boot" - a false failure in the
        // test protocol, which is the exact kind of quiet wrong result this branch is about.
        (void)value;
        Serial.println("LATCH write skipped (no-battery test build)");
        return false;
#else
        if (!prefs.begin(NS, false))
        {
            return false;
        }
        size_t written = prefs.putBool(KEY_FLAT, value);
        bool stored = prefs.getBool(KEY_FLAT, !value);
        prefs.end();
        return written > 0 && stored == value;
#endif
    }
}

void BatteryLatchStore::begin()
{
    // Read-only open on purpose: the start path must not write to flash before it has decided
    // whether the target is allowed to run at all. The namespace is created by the first real
    // latch write instead.
    bool ok = false;
    cachedFlat = readFlat(ok);
    cacheValid = true;
}

bool BatteryLatchStore::isFlat()
{
    if (!cacheValid)
    {
        bool ok = false;
        cachedFlat = readFlat(ok);
        cacheValid = true;
    }
    return cachedFlat;
}

bool BatteryLatchStore::setFlat(int batteryMv)
{
    if (isFlat())
    {
        return true;
    }

    if (!writeFlatVerified(true))
    {
        Serial.printf("LATCH set FAILED mv=%d (NVS write not confirmed)\n", batteryMv);
        return false;
    }

    cachedFlat = true;
    Serial.printf("LATCH set mv=%d\n", batteryMv);
    return true;
}

bool BatteryLatchStore::clearFlat(int batteryMv, ClearedBy by)
{
    if (!isFlat())
    {
        return true;
    }

    const char *reason = (by == ClearedBy::Button) ? "button" : "voltage";

    if (!writeFlatVerified(false))
    {
        Serial.printf("LATCH clear FAILED mv=%d by=%s (NVS write not confirmed)\n",
                      batteryMv, reason);
        return false;
    }

    cachedFlat = false;
    Serial.printf("LATCH clear mv=%d by=%s\n", batteryMv, reason);
    return true;
}
