#include "LedModuleStore.h"
#include <Preferences.h>

namespace
{
    constexpr const char *NS = "ledmod";
    constexpr const char *KEY_SWAP = "rgswap";

    Preferences prefs;
}

bool LedModuleStore::loadRedGreenSwapped()
{
    // Read-only open: a target that never received the command has no namespace, the open fails
    // and the old-module default applies without a flash write on the start path.
    if (!prefs.begin(NS, true))
    {
        return false;
    }
    bool value = prefs.getBool(KEY_SWAP, false);
    prefs.end();
    return value;
}

bool LedModuleStore::saveRedGreenSwapped(bool swapped)
{
    if (loadRedGreenSwapped() == swapped)
    {
        return true;
    }

    if (!prefs.begin(NS, false))
    {
        Serial.println("LedModuleStore: NVS open FAILED");
        return false;
    }
    size_t written = prefs.putBool(KEY_SWAP, swapped);
    bool stored = prefs.getBool(KEY_SWAP, !swapped);
    prefs.end();

    if (written == 0 || stored != swapped)
    {
        Serial.printf("LedModuleStore: rgswap=%d write FAILED (not confirmed)\n", (int)swapped);
        return false;
    }

    Serial.printf("LedModuleStore: rgswap saved = %d\n", (int)swapped);
    return true;
}
