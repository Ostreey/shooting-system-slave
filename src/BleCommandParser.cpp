#include "BleCommandParser.h"

#include <cerrno>
#include <climits>
#include <cstdlib>

namespace
{
    int clampChannel(int value)
    {
        if (value < 0)
            return 0;
        if (value > 255)
            return 255;
        return value;
    }

    bool startsWith(const std::string &value, const char *prefix, size_t prefixLen)
    {
        return value.compare(0, prefixLen, prefix) == 0;
    }

    BLECommandData numeric(BLECommand command, const std::string &number)
    {
        BLECommandData result = {BLECommand::UNKNOWN, 0};
        int parsed = 0;
        if (BleCommandParser::parseInt(number, parsed))
        {
            result.command = command;
            result.value = parsed;
        }
        return result;
    }
}

bool BleCommandParser::parseInt(const std::string &text, int &out)
{
    const char *begin = text.c_str();
    char *end = nullptr;
    errno = 0;
    long parsed = std::strtol(begin, &end, 10);
    if (end == begin || errno == ERANGE || parsed < INT_MIN || parsed > INT_MAX)
    {
        return false;
    }
    out = static_cast<int>(parsed);
    return true;
}

bool BleCommandParser::parseRgb(const std::string &text, int &red, int &green, int &blue)
{
    size_t firstComma = text.find(',');
    if (firstComma == std::string::npos)
    {
        return false;
    }
    size_t secondComma = text.find(',', firstComma + 1);
    if (secondComma == std::string::npos)
    {
        return false;
    }

    int r = 0, g = 0, b = 0;
    if (!parseInt(text.substr(0, firstComma), r) ||
        !parseInt(text.substr(firstComma + 1, secondComma - firstComma - 1), g) ||
        !parseInt(text.substr(secondComma + 1), b))
    {
        return false;
    }

    red = clampChannel(r);
    green = clampChannel(g);
    blue = clampChannel(b);
    return true;
}

BLECommandData BleCommandParser::parse(const std::string &value)
{
    BLECommandData result = {BLECommand::UNKNOWN, 0};

    // Order matters and is the 1.0.9 order: "rgb:" before "r:", "blinkrgb:" before "b:".
    if (startsWith(value, "rgb:", 4))
    {
        result.command = BLECommand::SET_RGB_COLOR;
        return result;
    }
    if (startsWith(value, "blinkrgb:", 9))
    {
        result.command = BLECommand::BLINK_COLOR;
        return result;
    }
    if (startsWith(value, "color:", 6))
    {
        result.command = BLECommand::SET_COLOR;
        return result;
    }

    if (startsWith(value, "r:", 2))
        return numeric(BLECommand::SET_RED, value.substr(2));
    if (startsWith(value, "g:", 2))
        return numeric(BLECommand::SET_GREEN, value.substr(2));
    if (startsWith(value, "b:", 2))
        return numeric(BLECommand::SET_BRIGHTNESS, value.substr(2));
    if (startsWith(value, "autooff:", 8))
        return numeric(BLECommand::SET_AUTO_OFF, value.substr(8));

    if (value == "calib:get")
    {
        result.command = BLECommand::CALIB_GET;
        return result;
    }
    if (value == "calib:reset")
    {
        result.command = BLECommand::CALIB_RESET;
        return result;
    }
    if (startsWith(value, "calib:set:", 10))
    {
        result.command = BLECommand::CALIB_SET;
        return result;
    }

    // Exact matches only for the LED module commands.
    if (value == "ledswap:get")
    {
        result.command = BLECommand::LED_SWAP_GET;
        return result;
    }
    if (value == "ledswap:0" || value == "ledswap:1")
    {
        result.command = BLECommand::LED_SWAP_SET;
        result.value = (value == "ledswap:1") ? 1 : 0;
        return result;
    }
    if (value == "ledtest:red" || value == "ledtest:green")
    {
        result.command = BLECommand::LED_TEST;
        result.value = (value == "ledtest:red") ? 1 : 0;
        return result;
    }

    if (value == "peak:start")
    {
        result.command = BLECommand::PEAK_START;
        return result;
    }
    if (value == "peak:stop")
    {
        result.command = BLECommand::PEAK_STOP;
        return result;
    }

    if (value == "start" || startsWith(value, "start:", 6))
        result.command = BLECommand::START;
    else if (value == "sleep")
        result.command = BLECommand::SLEEP;
    else if (value == "game1")
        result.command = BLECommand::GAME1;
#ifdef BOARD_ESP32S3
    else if (value == "ble_status")
        result.command = BLECommand::BLE_STATUS;
    else if (value == "ble_phy")
        result.command = BLECommand::BLE_PHY_INFO;
#endif
    else if (value == "off")
        result.command = BLECommand::SET_COLOR;

    return result;
}
