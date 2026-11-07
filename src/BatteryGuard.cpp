#include "BatteryGuard.h"
#include "Config.h"

namespace BatteryGuard
{
    Thresholds defaultThresholds()
    {
        Thresholds th;
        th.flatMv = BATTERY_FLAT_THRESHOLD_MV;
        th.latchClearMv = BATTERY_LATCH_CLEAR_MV;
        th.loadedFloorMv = BATTERY_LOADED_FLOOR_MV;
        th.loadedFloorSamples = BATTERY_LOADED_FLOOR_SAMPLES;
        return th;
    }

    StartDecision decideStart(const StartInputs &in, const Thresholds &th,
                              ButtonHoldProbe probe, void *context)
    {
        StartDecision decision;
        decision.overridden = false;
        decision.overrideRefused = false;
        bool wantsSleep;

        if (in.latched)
        {
            // Only a voltage a flat cell cannot reach by itself lifts the latch. A charger being
            // present does not: starting while TP4056 is still in its low trickle current puts
            // the load back on the cell and the charging deadlock returns.
            wantsSleep = in.batteryMv < th.latchClearMv;
        }
        else
        {
            // Both readings have to be low. One sample is not evidence of an empty cell - it can
            // be a dip under load or a rail still settling after a supervisor reset.
            wantsSleep = in.batteryMv < th.flatMv && in.confirmMv < th.flatMv;
        }

        if (!wantsSleep)
        {
            decision.action = in.latched ? StartAction::ClearLatchAndStart : StartAction::Start;
            return decision;
        }

        // Rule 1, bounded from below (owner decision 2026-11-07, variant A). The button exists to
        // rescue a falsely latched cell in the 3500-3899 mV hysteresis band. Below the flat
        // threshold the target does not start at all and the only way out is the charger: the
        // button is not even asked, because a probe costs its debounce delay on every turn of a
        // brownout loop. The bound is the flat threshold itself, not a constant of its own, so
        // tuning that threshold after the live test cannot open a band where the button starts a
        // target the run-time guard puts straight back to sleep. Both readings must be below, as
        // everywhere else - one sample never decides. On the unlatched path wantsSleep is the same
        // test, so there the button is never asked.
        if (in.batteryMv < th.flatMv && in.confirmMv < th.flatMv)
        {
            decision.overrideRefused = true;
            decision.action = in.latched ? StartAction::SleepLatched : StartAction::LatchAndSleep;
            return decision;
        }

        // A person holding the target is a better authority than this heuristic (owner decision
        // 2026-11-06). Here the latch stands and the cell is at or above the flat threshold.
        if (probe != nullptr && probe(context))
        {
            decision.overridden = true;
            decision.action = in.latched ? StartAction::ClearLatchAndStart : StartAction::Start;
            return decision;
        }

        decision.action = in.latched ? StartAction::SleepLatched : StartAction::LatchAndSleep;
        return decision;
    }

    int32_t nextLoadedLowStreak(int32_t previous, int32_t batteryMv, bool unloadedKnown,
                                const Thresholds &th)
    {
        if (unloadedKnown || batteryMv >= th.loadedFloorMv)
        {
            return 0;
        }
        return previous + 1;
    }

    RuntimeAction decideRuntime(int32_t batteryMv, int32_t unloadedMv, bool unloadedKnown,
                                int32_t loadedLowStreak, bool chargerEvidence,
                                const Thresholds &th)
    {
        if (unloadedKnown && batteryMv < th.flatMv && unloadedMv < th.flatMv)
        {
            return RuntimeAction::SleepLowVoltage;
        }

        // The loaded floor alone, and only when it held for consecutive cycles: a single sag
        // under the LED load must never end a match. The floor is far enough under the flat
        // threshold that a healthy cell does not reach it under that load.
        if (!unloadedKnown && th.loadedFloorSamples > 0 &&
            loadedLowStreak >= th.loadedFloorSamples && batteryMv < th.loadedFloorMv)
        {
            return RuntimeAction::SleepLowVoltage;
        }

        if (chargerEvidence)
        {
            return RuntimeAction::SleepCharger;
        }

        return RuntimeAction::Continue;
    }

    const char *actionName(StartAction action)
    {
        switch (action)
        {
        case StartAction::Start:
            return "start";
        case StartAction::ClearLatchAndStart:
            return "clear-latch-start";
        case StartAction::LatchAndSleep:
            return "latch-and-sleep";
        case StartAction::SleepLatched:
            return "sleep-latched";
        }
        return "unknown";
    }

    const char *actionName(RuntimeAction action)
    {
        switch (action)
        {
        case RuntimeAction::Continue:
            return "continue";
        case RuntimeAction::SleepLowVoltage:
            return "sleep-lowvolt";
        case RuntimeAction::SleepCharger:
            return "sleep-charger";
        }
        return "unknown";
    }
}
