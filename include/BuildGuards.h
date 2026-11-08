#ifndef BUILD_GUARDS_H
#define BUILD_GUARDS_H

// Tripwire for the no-battery test build (PlatformIO env:migration-test).
//
// MIGRATION_TEST_NO_BATTERY turns off the voltage triggers and the idle auto-sleep, so a target
// can stay awake on bench power through a long migration session. On a target WITH a cell that
// build runs the cell down to the 2.4 V DW01A cutoff, which is the failure this branch exists to
// fix. The original mistake was not the flag, though: it was that a build carrying it reported
// the same firmware version as a release build, so nobody could tell what was on a target.
//
// Hence: the flag stays usable, but only from the environment that also declares the opt-in
// token below, and that environment reports its version as 1.0.9nobatt-esp32s3 (see Config.h) and
// is kept out of default_envs so CI never builds it.
//
// The marker sits in the version member, BEFORE the board dash, and moving it to the end breaks
// the app: the released version reads the board with substringAfter("-") and matches it against a
// list of exactly "esp32"/"esp32s3" (FirmwareRepositoryImpl.extractBoardFromVersion). A marker
// after the board member yields "esp32s3-nobatt", misses the list, falls back to the default
// "esp32" and the app quietly downloads the firmware for the wrong board.
#if defined(MIGRATION_TEST_NO_BATTERY) && !defined(MIGRATION_TEST_OPT_IN)
#error "MIGRATION_TEST_NO_BATTERY must not be set in a release build: it disables the low-voltage sleep and runs the cell down to the 2.4 V DW01A cutoff. Need a target that stays awake on bench power? Build the dedicated environment instead: pio run -e migration-test (it is excluded from default_envs, reports its version as 1.0.9nobatt-esp32s3 and never writes the flat-cell latch to NVS). Do not add this flag to env:esp32 or env:esp32-s3."
#endif

#endif // BUILD_GUARDS_H
