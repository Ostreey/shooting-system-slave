#include "GameSettings.h"

static bool autoLedOffEnabled = true;

void GameSettings::setAutoLedOffEnabled(bool enabled)
{
    autoLedOffEnabled = enabled;
}

bool GameSettings::isAutoLedOffEnabled()
{
    return autoLedOffEnabled;
}
