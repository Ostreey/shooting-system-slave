#ifndef BLE_COMMAND_PARSER_H
#define BLE_COMMAND_PARSER_H

#include <string>

// Text command parser for the piezo characteristic. Kept free of Arduino and BLE headers so the
// host tests (env:native) can check it: a malformed number used to reach std::stoi, and the
// uncaught exception there aborts and restarts the target - any client in BLE range could do it.

enum class BLECommand
{
    UNKNOWN,
    START,
    SLEEP,
    GAME1,
    SET_BRIGHTNESS,
    SET_RGB_COLOR,
    SET_COLOR,
    SET_RED,
    SET_GREEN,
    SET_BLUE,
    SET_AUTO_OFF,
    BLINK_COLOR,
    CALIB_GET,
    CALIB_SET,
    CALIB_RESET,
    PEAK_START,
    PEAK_STOP,
    LED_SWAP_GET,
    LED_SWAP_SET,
    LED_TEST,
#ifdef BOARD_ESP32S3
    BLE_STATUS,
    BLE_PHY_INFO,
#endif
};

struct BLECommandData
{
    BLECommand command;
    int value;
};

namespace BleCommandParser
{
    BLECommandData parse(const std::string &value);

    // Same accepted input as std::stoi (leading whitespace, optional sign, digits, trailing text
    // ignored), but a missing or out-of-range number returns false instead of throwing.
    bool parseInt(const std::string &text, int &out);

    // "r,g,b", each clamped to 0-255. False when a comma or a number is missing.
    bool parseRgb(const std::string &text, int &red, int &green, int &blue);
}

#endif // BLE_COMMAND_PARSER_H
