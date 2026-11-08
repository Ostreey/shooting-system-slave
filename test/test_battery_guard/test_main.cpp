#include <unity.h>
#include <stdio.h>

#include "BatteryGuard.h"
#include "BatteryScale.h"
#include "Config.h"

// Host tests for the two pieces of this fix that have no hardware in them: the start decision and
// the battery level reported over BLE. Everything else about the flat-cell guard - voltage sag
// under load, the TPS3823-33 supervisor, NVS surviving a brownout, the 58 mA charging deadlock -
// needs a target on a bench and lives in the QA protocol, not here.
//
// Deliberately plain: a table and a loop, no fixtures, no ordering between cases.

namespace
{
    int probeCalls = 0;
    bool probeAnswer = false;

    bool fakeButtonProbe(void *context)
    {
        (void)context;
        probeCalls++;
        return probeAnswer;
    }

    struct GuardCase
    {
        const char *name;
        int32_t batteryMv;
        int32_t confirmMv;
        bool latched;
        bool buttonHeld;
        BatteryGuard::StartAction expected;
        int expectedProbeCalls;  // asked once on the way to sleep, never below the flat threshold
        bool expectedOverridden; // the button turned a sleep into a start - drives the flash
        bool expectedRefused;    // below the flat threshold: no start at all, charger only
    };

    const GuardCase GUARD_CASES[] = {
        {"latch + low voltage", 3300, 3300, true, false,
         BatteryGuard::StartAction::SleepLatched, 0, false, true},

        {"latch + voltage above the clear threshold", 3950, 3950, true, false,
         BatteryGuard::StartAction::ClearLatchAndStart, 0, false, false},

        {"latch + voltage exactly at the clear threshold", 3900, 3900, true, false,
         BatteryGuard::StartAction::ClearLatchAndStart, 0, false, false},

        {"latch + one mV below the clear threshold", 3899, 3899, true, false,
         BatteryGuard::StartAction::SleepLatched, 1, false, false},

        // Owner decision 2026-11-07 (variant A): below the flat threshold the button does nothing.
        {"latch + button held on a low cell", 3300, 3300, true, true,
         BatteryGuard::StartAction::SleepLatched, 0, false, true},

        // The hysteresis band is the only place the button works: the latch stands, the cell is
        // healthy, the button clears the latch and the target comes up normally.
        {"latch + button held in the 3500-3900 band", 3750, 3750, true, true,
         BatteryGuard::StartAction::ClearLatchAndStart, 1, true, false},

        {"latch + button held exactly at the flat threshold", 3500, 3500, true, true,
         BatteryGuard::StartAction::ClearLatchAndStart, 1, true, false},

        {"latch + button held one mV below the flat threshold", 3499, 3499, true, true,
         BatteryGuard::StartAction::SleepLatched, 0, false, true},

        // A charger is not an input to this decision on purpose: starting while TP4056 still
        // trickles 58 mA puts the load back on the cell and the charging deadlock returns. All a
        // charger can do is raise the cell, and only past the clear threshold does that count.
        {"latch + charger raised the cell to 3.6V, still below clear", 3600, 3600, true, false,
         BatteryGuard::StartAction::SleepLatched, 1, false, false},

        {"no latch + low voltage confirmed twice", 3400, 3400, false, false,
         BatteryGuard::StartAction::LatchAndSleep, 0, false, true},

        {"no latch + voltage exactly at the flat threshold", 3500, 3500, false, false,
         BatteryGuard::StartAction::Start, 0, false, false},

        {"no latch + low first sample, healthy confirmation", 3400, 3600, false, false,
         BatteryGuard::StartAction::Start, 0, false, false},

        {"no latch + healthy first sample, low confirmation", 3600, 3400, false, false,
         BatteryGuard::StartAction::Start, 0, false, false},

        {"no latch + button held on a low cell", 3400, 3400, false, true,
         BatteryGuard::StartAction::LatchAndSleep, 0, false, true},
    };

    struct PercentCase
    {
        int32_t batteryMv;
        int expectedPercent;
    };

    // Boundaries of the discharge table reported over BLE. These are the 1.0.8 numbers and they
    // must not move: the released Android app blocks OTA at batteryLevel <= 40.
    const PercentCase PERCENT_CASES[] = {
        {4200, 100},
        {3700, 38},
        {3500, 15},
        {3370, 4},
        {3300, 2},
        {3000, 0},
    };
}

void setUp(void) {}
void tearDown(void) {}

void test_start_decision_table(void)
{
    const BatteryGuard::Thresholds th = BatteryGuard::defaultThresholds();
    const int caseCount = (int)(sizeof(GUARD_CASES) / sizeof(GUARD_CASES[0]));

    for (int i = 0; i < caseCount; i++)
    {
        const GuardCase &c = GUARD_CASES[i];

        probeCalls = 0;
        probeAnswer = c.buttonHeld;

        BatteryGuard::StartInputs in;
        in.batteryMv = c.batteryMv;
        in.confirmMv = c.confirmMv;
        in.latched = c.latched;

        BatteryGuard::StartDecision decision =
            BatteryGuard::decideStart(in, th, fakeButtonProbe, nullptr);

        char message[160];
        snprintf(message, sizeof(message), "case '%s': got %s", c.name,
                 BatteryGuard::actionName(decision.action));
        TEST_ASSERT_EQUAL_INT_MESSAGE((int)c.expected, (int)decision.action, message);
        TEST_ASSERT_EQUAL_INT_MESSAGE(c.expectedProbeCalls, probeCalls, message);
        TEST_ASSERT_EQUAL_INT_MESSAGE((int)c.expectedOverridden, (int)decision.overridden,
                                      message);
        TEST_ASSERT_EQUAL_INT_MESSAGE((int)c.expectedRefused, (int)decision.overrideRefused,
                                      message);
    }
}

// Every sleep decision is either offered to the button or refused outright - there is no third
// kind, so a sleep branch cannot quietly skip the person holding the target.
void test_every_sleep_is_offered_to_the_button_or_refused(void)
{
    const BatteryGuard::Thresholds th = BatteryGuard::defaultThresholds();
    const int caseCount = (int)(sizeof(GUARD_CASES) / sizeof(GUARD_CASES[0]));

    for (int i = 0; i < caseCount; i++)
    {
        const GuardCase &c = GUARD_CASES[i];

        probeCalls = 0;
        probeAnswer = false;

        BatteryGuard::StartInputs in;
        in.batteryMv = c.batteryMv;
        in.confirmMv = c.confirmMv;
        in.latched = c.latched;

        BatteryGuard::StartDecision decision =
            BatteryGuard::decideStart(in, th, fakeButtonProbe, nullptr);

        bool sleeps = decision.action == BatteryGuard::StartAction::SleepLatched ||
                      decision.action == BatteryGuard::StartAction::LatchAndSleep;

        char message[160];
        snprintf(message, sizeof(message), "case '%s' ends in %s", c.name,
                 BatteryGuard::actionName(decision.action));
        if (sleeps)
        {
            TEST_ASSERT_EQUAL_INT_MESSAGE(decision.overrideRefused ? 0 : 1, probeCalls, message);
        }
        else
        {
            TEST_ASSERT_FALSE_MESSAGE(decision.overrideRefused, message);
        }
    }
}

namespace
{
    BatteryGuard::StartDecision decideWithButton(int32_t batteryMv, int32_t confirmMv, bool latched,
                                                 bool held)
    {
        probeCalls = 0;
        probeAnswer = held;

        BatteryGuard::StartInputs in;
        in.batteryMv = batteryMv;
        in.confirmMv = confirmMv;
        in.latched = latched;

        return BatteryGuard::decideStart(in, BatteryGuard::defaultThresholds(), fakeButtonProbe,
                                         nullptr);
    }
}

// Exactly at the flat threshold, latch standing: the button still wins.
void test_button_overrides_the_latch_at_the_flat_threshold(void)
{
    BatteryGuard::StartDecision decision =
        decideWithButton(BATTERY_FLAT_THRESHOLD_MV, BATTERY_FLAT_THRESHOLD_MV, true, true);
    TEST_ASSERT_EQUAL_INT((int)BatteryGuard::StartAction::ClearLatchAndStart, (int)decision.action);
    TEST_ASSERT_TRUE(decision.overridden);
    TEST_ASSERT_FALSE(decision.overrideRefused);
    TEST_ASSERT_EQUAL_INT(1, probeCalls);
}

// Owner decision 2026-11-07 (variant A): below the flat threshold the target does not start, not
// even with the button held - only the charger brings it back. The button is not asked at all,
// and the latch stays (or gets written).
void test_override_refused_below_the_flat_threshold(void)
{
    const int32_t below = BATTERY_FLAT_THRESHOLD_MV - 1;

    BatteryGuard::StartDecision decision = decideWithButton(below, below, true, true);
    TEST_ASSERT_EQUAL_INT((int)BatteryGuard::StartAction::SleepLatched, (int)decision.action);
    TEST_ASSERT_FALSE(decision.overridden);
    TEST_ASSERT_TRUE(decision.overrideRefused);
    TEST_ASSERT_EQUAL_INT(0, probeCalls);

    decision = decideWithButton(below, below, false, true);
    TEST_ASSERT_EQUAL_INT((int)BatteryGuard::StartAction::LatchAndSleep, (int)decision.action);
    TEST_ASSERT_FALSE(decision.overridden);
    TEST_ASSERT_TRUE(decision.overrideRefused);
    TEST_ASSERT_EQUAL_INT(0, probeCalls);

    // Lowest value the ADC path can report.
    decision = decideWithButton(0, 0, true, true);
    TEST_ASSERT_EQUAL_INT((int)BatteryGuard::StartAction::SleepLatched, (int)decision.action);
    TEST_ASSERT_TRUE(decision.overrideRefused);
    TEST_ASSERT_EQUAL_INT(0, probeCalls);
}

// One sample never decides, also for the refusal: a low first reading with a healthy
// confirmation is a normal start, with no button needed.
void test_refusal_needs_both_readings(void)
{
    const int32_t below = BATTERY_FLAT_THRESHOLD_MV - 100;
    const int32_t above = BATTERY_FLAT_THRESHOLD_MV + 100;

    BatteryGuard::StartDecision decision = decideWithButton(below, above, false, false);
    TEST_ASSERT_EQUAL_INT((int)BatteryGuard::StartAction::Start, (int)decision.action);
    TEST_ASSERT_FALSE(decision.overridden);
    TEST_ASSERT_FALSE(decision.overrideRefused);
    TEST_ASSERT_EQUAL_INT(0, probeCalls);
}

// The refusal never touches a healthy start: a charged cell with a stale latch, and a healthy
// unlatched cell.
void test_refusal_never_blocks_a_healthy_start(void)
{
    BatteryGuard::StartDecision decision = decideWithButton(4100, 4100, true, false);
    TEST_ASSERT_EQUAL_INT((int)BatteryGuard::StartAction::ClearLatchAndStart, (int)decision.action);
    TEST_ASSERT_FALSE(decision.overrideRefused);

    decision = decideWithButton(3600, 3600, false, false);
    TEST_ASSERT_EQUAL_INT((int)BatteryGuard::StartAction::Start, (int)decision.action);
    TEST_ASSERT_FALSE(decision.overrideRefused);
    TEST_ASSERT_EQUAL_INT(0, probeCalls);
}

// A single cycle without an unloaded confirmation must never sleep the target: the battery task
// refuses to switch the LED output off for a measurement, so "I could not check" has to mean
// "wait another cycle" - unless the loaded floor has held for consecutive cycles.
void test_runtime_never_sleeps_without_a_confirmed_reading(void)
{
    const BatteryGuard::Thresholds th = BatteryGuard::defaultThresholds();

    TEST_ASSERT_EQUAL_INT((int)BatteryGuard::RuntimeAction::Continue,
                          (int)BatteryGuard::decideRuntime(3100, 3100, false, 1, false, th));
    TEST_ASSERT_EQUAL_INT((int)BatteryGuard::RuntimeAction::SleepLowVoltage,
                          (int)BatteryGuard::decideRuntime(3100, 3100, true, 0, false, th));

    // A charger still gets the load off the cell even when no confirmation was possible.
    TEST_ASSERT_EQUAL_INT((int)BatteryGuard::RuntimeAction::SleepCharger,
                          (int)BatteryGuard::decideRuntime(3100, 3100, false, 1, true, th));

    // And a loaded dip with a healthy unloaded reading keeps the target running.
    TEST_ASSERT_EQUAL_INT((int)BatteryGuard::RuntimeAction::Continue,
                          (int)BatteryGuard::decideRuntime(3400, 3700, true, 0, false, th));
}

// A target the app keeps lit while the phone stays connected never gets an unloaded reading. The
// loaded floor is the only thing standing between that target and the 2.4V DW01A cutoff.
void test_loaded_floor_sleeps_a_target_that_stays_lit(void)
{
    const BatteryGuard::Thresholds th = BatteryGuard::defaultThresholds();
    const int32_t low = BATTERY_LOADED_FLOOR_MV - 1;

    int32_t streak = 0;
    for (int32_t i = 1; i < BATTERY_LOADED_FLOOR_SAMPLES; i++)
    {
        streak = BatteryGuard::nextLoadedLowStreak(streak, low, false, th);
        TEST_ASSERT_EQUAL_INT((int)BatteryGuard::RuntimeAction::Continue,
                              (int)BatteryGuard::decideRuntime(low, low, false, streak, false, th));
    }

    streak = BatteryGuard::nextLoadedLowStreak(streak, low, false, th);
    TEST_ASSERT_EQUAL_INT(BATTERY_LOADED_FLOOR_SAMPLES, (int)streak);
    TEST_ASSERT_EQUAL_INT((int)BatteryGuard::RuntimeAction::SleepLowVoltage,
                          (int)BatteryGuard::decideRuntime(low, low, false, streak, false, th));
}

// A healthy cell sagging under the LED load during a match: below the flat threshold, at the
// floor. However long it lasts, it must not sleep the target.
void test_loaded_sag_above_floor_never_sleeps(void)
{
    const BatteryGuard::Thresholds th = BatteryGuard::defaultThresholds();
    const int32_t sag = BATTERY_LOADED_FLOOR_MV;

    int32_t streak = 0;
    for (int i = 0; i < 100; i++)
    {
        streak = BatteryGuard::nextLoadedLowStreak(streak, sag, false, th);
        TEST_ASSERT_EQUAL_INT(0, (int)streak);
        TEST_ASSERT_EQUAL_INT((int)BatteryGuard::RuntimeAction::Continue,
                              (int)BatteryGuard::decideRuntime(sag, sag, false, streak, false, th));
    }
}

// The streak counts consecutive cycles only: one reading at or above the floor, or one unloaded
// reading, starts it over.
void test_loaded_floor_streak_resets(void)
{
    const BatteryGuard::Thresholds th = BatteryGuard::defaultThresholds();
    const int32_t low = BATTERY_LOADED_FLOOR_MV - 100;

    TEST_ASSERT_EQUAL_INT(1, (int)BatteryGuard::nextLoadedLowStreak(0, low, false, th));
    TEST_ASSERT_EQUAL_INT(0, (int)BatteryGuard::nextLoadedLowStreak(1, BATTERY_LOADED_FLOOR_MV,
                                                                   false, th));
    TEST_ASSERT_EQUAL_INT(0, (int)BatteryGuard::nextLoadedLowStreak(1, low, true, th));
}

// The no-battery test build zeroes both thresholds (a button override no longer does);
// nothing may sleep the target on voltage then, however long the low readings last.
void test_zeroed_thresholds_mute_both_voltage_paths(void)
{
    BatteryGuard::Thresholds th = BatteryGuard::defaultThresholds();
    th.flatMv = 0;
    th.loadedFloorMv = 0;

    int32_t streak = 0;
    for (int i = 0; i < 10; i++)
    {
        streak = BatteryGuard::nextLoadedLowStreak(streak, 2500, false, th);
    }
    TEST_ASSERT_EQUAL_INT(0, (int)streak);
    TEST_ASSERT_EQUAL_INT((int)BatteryGuard::RuntimeAction::Continue,
                          (int)BatteryGuard::decideRuntime(2500, 2500, false, streak, false, th));
    TEST_ASSERT_EQUAL_INT((int)BatteryGuard::RuntimeAction::Continue,
                          (int)BatteryGuard::decideRuntime(2500, 2500, true, streak, false, th));
}

void test_loaded_floor_sits_below_flat_threshold(void)
{
    TEST_ASSERT_TRUE(BATTERY_LOADED_FLOOR_MV < BATTERY_FLAT_THRESHOLD_MV);
    TEST_ASSERT_TRUE(BATTERY_LOADED_FLOOR_SAMPLES >= 2);
}

void test_percent_from_mv_boundaries(void)
{
    const int caseCount = (int)(sizeof(PERCENT_CASES) / sizeof(PERCENT_CASES[0]));

    for (int i = 0; i < caseCount; i++)
    {
        const PercentCase &c = PERCENT_CASES[i];

        char message[96];
        snprintf(message, sizeof(message), "%d mV", (int)c.batteryMv);
        TEST_ASSERT_EQUAL_INT_MESSAGE(c.expectedPercent,
                                      BatteryScale::percentFromMv(c.batteryMv), message);
    }
}

void test_adc_mv_to_cell_mv(void)
{
    // 100k/200k divider (x1.5) plus the systematic underread compensation.
    TEST_ASSERT_EQUAL_INT(ADC_BATTERY_OFFSET_MV, (int)BatteryScale::cellMvFromAdcMv(0));
    TEST_ASSERT_EQUAL_INT(3100, (int)BatteryScale::cellMvFromAdcMv(2000));
    TEST_ASSERT_EQUAL_INT(4300, (int)BatteryScale::cellMvFromAdcMv(2800));
}

void test_ota_gate_still_reachable(void)
{
    // The released app needs batteryLevel >= 41 to start an OTA, so this fix can only be
    // delivered to a cell well above the flat threshold. Pinned here so a future change to the
    // discharge table cannot quietly lock targets out of updates.
    TEST_ASSERT_TRUE(BatteryScale::percentFromMv(3800) > 40);
    TEST_ASSERT_TRUE(BatteryScale::percentFromMv(4100) > 40);
    TEST_ASSERT_TRUE(BatteryScale::percentFromMv(BATTERY_FLAT_THRESHOLD_MV) <= 40);
}

int main(int argc, char **argv)
{
    (void)argc;
    (void)argv;

    UNITY_BEGIN();
    RUN_TEST(test_start_decision_table);
    RUN_TEST(test_every_sleep_is_offered_to_the_button_or_refused);
    RUN_TEST(test_button_overrides_the_latch_at_the_flat_threshold);
    RUN_TEST(test_override_refused_below_the_flat_threshold);
    RUN_TEST(test_refusal_needs_both_readings);
    RUN_TEST(test_refusal_never_blocks_a_healthy_start);
    RUN_TEST(test_runtime_never_sleeps_without_a_confirmed_reading);
    RUN_TEST(test_loaded_floor_sleeps_a_target_that_stays_lit);
    RUN_TEST(test_loaded_sag_above_floor_never_sleeps);
    RUN_TEST(test_loaded_floor_streak_resets);
    RUN_TEST(test_zeroed_thresholds_mute_both_voltage_paths);
    RUN_TEST(test_loaded_floor_sits_below_flat_threshold);
    RUN_TEST(test_percent_from_mv_boundaries);
    RUN_TEST(test_adc_mv_to_cell_mv);
    RUN_TEST(test_ota_gate_still_reachable);
    return UNITY_END();
}
