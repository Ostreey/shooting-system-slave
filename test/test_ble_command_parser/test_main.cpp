#include <unity.h>
#include <string>

#include "BleCommandParser.h"

// Host tests for the BLE text command parser. The point of the malformed-number cases: before
// 1.0.9 these strings reached std::stoi, threw, and the uncaught exception restarted the target.

namespace
{
    struct ParseCase
    {
        const char *input;
        BLECommand expected;
        int expectedValue;
    };

    const ParseCase CASES[] = {
        // LED module commands - exact matches only
        {"ledswap:get", BLECommand::LED_SWAP_GET, 0},
        {"ledswap:0", BLECommand::LED_SWAP_SET, 0},
        {"ledswap:1", BLECommand::LED_SWAP_SET, 1},
        {"ledswap:2", BLECommand::UNKNOWN, 0},
        {"ledswap:", BLECommand::UNKNOWN, 0},
        {"ledswap:1 ", BLECommand::UNKNOWN, 0},
        {"ledtest:red", BLECommand::LED_TEST, 1},
        {"ledtest:green", BLECommand::LED_TEST, 0},
        {"ledtest:blue", BLECommand::UNKNOWN, 0},

        // Valid numbers behave as with std::stoi
        {"b:128", BLECommand::SET_BRIGHTNESS, 128},
        {"b: 64", BLECommand::SET_BRIGHTNESS, 64},
        {"b:200x", BLECommand::SET_BRIGHTNESS, 200},
        {"r:-5", BLECommand::SET_RED, -5},
        {"g:255", BLECommand::SET_GREEN, 255},
        {"autooff:0", BLECommand::SET_AUTO_OFF, 0},
        {"autooff:1", BLECommand::SET_AUTO_OFF, 1},

        // Malformed numbers: UNKNOWN, never a throw
        {"b:", BLECommand::UNKNOWN, 0},
        {"b:x", BLECommand::UNKNOWN, 0},
        {"r:", BLECommand::UNKNOWN, 0},
        {"g:abc", BLECommand::UNKNOWN, 0},
        {"autooff:", BLECommand::UNKNOWN, 0},
        {"b:99999999999999999999", BLECommand::UNKNOWN, 0},

        // Prefix order of 1.0.9
        {"rgb:255,0,0", BLECommand::SET_RGB_COLOR, 0},
        {"blinkrgb:0,0,255", BLECommand::BLINK_COLOR, 0},
        {"color:red", BLECommand::SET_COLOR, 0},
        {"off", BLECommand::SET_COLOR, 0},
        {"calib:get", BLECommand::CALIB_GET, 0},
        {"calib:reset", BLECommand::CALIB_RESET, 0},
        {"calib:set:400:200", BLECommand::CALIB_SET, 0},
        {"peak:start", BLECommand::PEAK_START, 0},
        {"peak:stop", BLECommand::PEAK_STOP, 0},
        {"start", BLECommand::START, 0},
        {"start:255,0,0", BLECommand::START, 0},
        {"sleep", BLECommand::SLEEP, 0},
        {"", BLECommand::UNKNOWN, 0},
        {"garbage", BLECommand::UNKNOWN, 0},
    };
}

void test_parse_table()
{
    for (const ParseCase &c : CASES)
    {
        BLECommandData result = BleCommandParser::parse(c.input);
        TEST_ASSERT_EQUAL_INT_MESSAGE((int)c.expected, (int)result.command, c.input);
        TEST_ASSERT_EQUAL_INT_MESSAGE(c.expectedValue, result.value, c.input);
    }
}

void test_parse_rgb_valid_and_clamped()
{
    int r = -1, g = -1, b = -1;
    TEST_ASSERT_TRUE(BleCommandParser::parseRgb("255,0,10", r, g, b));
    TEST_ASSERT_EQUAL_INT(255, r);
    TEST_ASSERT_EQUAL_INT(0, g);
    TEST_ASSERT_EQUAL_INT(10, b);

    TEST_ASSERT_TRUE(BleCommandParser::parseRgb("300,-4,7", r, g, b));
    TEST_ASSERT_EQUAL_INT(255, r);
    TEST_ASSERT_EQUAL_INT(0, g);
    TEST_ASSERT_EQUAL_INT(7, b);
}

void test_parse_rgb_malformed_is_rejected()
{
    int r = 1, g = 2, b = 3;
    const char *bad[] = {"", "255", "255,0", "a,b,c", "255,,0", ",0,0", "255,0,", "1,2,x"};
    for (const char *input : bad)
    {
        TEST_ASSERT_FALSE_MESSAGE(BleCommandParser::parseRgb(input, r, g, b), input);
    }
    TEST_ASSERT_EQUAL_INT(1, r);
    TEST_ASSERT_EQUAL_INT(2, g);
    TEST_ASSERT_EQUAL_INT(3, b);
}

int main(int argc, char **argv)
{
    (void)argc;
    (void)argv;
    UNITY_BEGIN();
    RUN_TEST(test_parse_table);
    RUN_TEST(test_parse_rgb_valid_and_clamped);
    RUN_TEST(test_parse_rgb_malformed_is_rejected);
    return UNITY_END();
}
