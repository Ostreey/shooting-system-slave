#ifndef BATTERY_GUARD_H
#define BATTERY_GUARD_H

#include <stdint.h>

// Pure start/run decisions for the flat-cell guard: no Arduino, no Preferences, no ADC.
// Measurements come in as arguments so the rules can be exercised away from the hardware.
//
// Three rules live here and nowhere else:
//  1. the button wins over a sleep branch only in the latch hysteresis band (flat threshold up to
//     the clear threshold); below the flat threshold the target does not start at all;
//  2. one sample never decides - a reading below the flat threshold needs a confirmation;
//  3. the latch is lifted only by BATTERY_LATCH_CLEAR_MV or by the button, and it is written
//     only on the start path, where the cell is measured with the LED rail off and BLE down.
namespace BatteryGuard
{
    enum class StartAction : uint8_t
    {
        Start,
        ClearLatchAndStart,
        LatchAndSleep,
        SleepLatched,
    };

    enum class RuntimeAction : uint8_t
    {
        Continue,
        SleepLowVoltage,
        SleepCharger,
    };

    struct StartInputs
    {
        int32_t batteryMv;  // estimated cell voltage, same conversion as the reported level
        int32_t confirmMv;  // second reading; equals batteryMv when no confirmation was needed
        bool latched;       // "cell is flat" latch read from NVS
    };

    // `overridden`: the guard was heading for sleep and the button turned it around - drives the
    // confirmation flash and the "by=button" log.
    // `overrideRefused`: the cell is below the flat threshold on both readings, the button was
    // not asked and the decision is sleep. Below that threshold only the charger gets the target
    // back (owner decision 2026-11-07, variant A).
    struct StartDecision
    {
        StartAction action;
        bool overridden;
        bool overrideRefused;
    };

    struct Thresholds
    {
        int32_t flatMv;
        int32_t latchClearMv;
        int32_t loadedFloorMv;
        int32_t loadedFloorSamples;
    };

    // Asked only when the decision would otherwise end in sleep, so a healthy start never pays
    // for it. Taking the probe as an argument is what keeps rule 1 impossible to lose at a call
    // site: no sleep action is reached without either consulting the button or setting
    // overrideRefused, which only happens below the flat threshold.
    typedef bool (*ButtonHoldProbe)(void *context);

    Thresholds defaultThresholds();

    StartDecision decideStart(const StartInputs &in, const Thresholds &th,
                              ButtonHoldProbe probe, void *context);

    // Consecutive run-time cycles in which no unloaded reading was possible and the loaded reading
    // sat below the loaded floor. Any unloaded reading, or a loaded reading at or above the floor,
    // starts the count over.
    int32_t nextLoadedLowStreak(int32_t previous, int32_t batteryMv, bool unloadedKnown,
                                const Thresholds &th);

    // Run time never writes the latch: a dip under load has to be able to end in a plain sleep
    // the user can undo with a normal button press.
    //
    // `unloadedKnown` false means the confirming reading could not be taken this cycle, because
    // the LED output was on and the caller refuses to switch it off (that would fight the game
    // logic for the LEDs). Then only the loaded floor can decide, and only after
    // loadedFloorSamples consecutive cycles.
    RuntimeAction decideRuntime(int32_t batteryMv, int32_t unloadedMv, bool unloadedKnown,
                                int32_t loadedLowStreak, bool chargerEvidence,
                                const Thresholds &th);

    const char *actionName(StartAction action);
    const char *actionName(RuntimeAction action);
}

#endif // BATTERY_GUARD_H
