#ifndef LED_MODULE_STORE_H
#define LED_MODULE_STORE_H

#include <Arduino.h>

// Persistent setting of the external LED module: the newer modules have the red and green pins
// physically swapped and the plug cannot be changed. Default (key absent) is the old module, so a
// target that never received the command behaves exactly as before.
class LedModuleStore
{
public:
    static bool loadRedGreenSwapped();

    // No-op when the stored value already matches, so a repeated command does not wear the flash.
    // Returns false when the write could not be confirmed by a read-back.
    static bool saveRedGreenSwapped(bool swapped);
};

#endif // LED_MODULE_STORE_H
