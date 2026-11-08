#ifndef BATTERY_SCALE_H
#define BATTERY_SCALE_H

#include <stdint.h>

// The battery level reported over BLE, kept free of Arduino so the discharge curve can be
// exercised on the host.
//
// This is a frozen contract with the released Android app: it blocks OTA at batteryLevel <= 40
// (MainViewModel.kt, OtaScreen.kt), so a target in the field would refuse this very fix if these
// numbers moved down. The table and the ADC offset are the ones from firmware 1.0.8, copied
// verbatim. The new flat-cell thresholds are internal to BatteryGuard and do not touch this path.
namespace BatteryScale
{
    // Divider on BATT+ is 100k/200k (x1.5), plus the systematic ADC underread compensation.
    int32_t cellMvFromAdcMv(int32_t adcMv);

    int percentFromMv(int32_t batteryMv);
}

#endif // BATTERY_SCALE_H
