/* Copyright (c) 2019 Skyward Experimental Rocketry
 * Author: Luca Erbetta
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 */

// Depends on Active Object which is not supported from miosix host
#ifndef COMPILE_FOR_HOST

#ifdef STANDALONE_CATCH1_TEST
#include "catch-tests-entry.cpp"
#endif

#include <Singleton.h>
#include <miosix.h>
#include <utils/ButtonHandler.h>
#include <utils/Debug.h>

#include <catch2/catch.hpp>

using namespace Boardcore;
using namespace miosix;
using std::bind;

class ButtonMock : public Singleton<ButtonMock>
{
    friend class Singleton<ButtonMock>;

public:
    // Mocked methods
    static void mode(miosix::Mode::Mode_ mode)
    {
        getInstance().inputMode = mode == Mode::INPUT;
    }

    static int value()
    {
        const ButtonMock& instance = getInstance();
        if (instance.inputMode)
            return getInstance().btnValue;

        TRACE("Returning 0\n");
        return 0;
    }

    // Test methods
    void setValue(int value) { btnValue = value; }

    void reset()
    {
        btnValue  = 0;
        inputMode = false;
    }

    void press(unsigned int duration)
    {
        setValue(1);
        Thread::sleep(duration);
        setValue(0);
        Thread::sleep(TICK_LENGTH * 2);
    }

private:
    int btnValue   = 0;
    bool inputMode = false;
};

ButtonMock& btn = ButtonMock::getInstance();

using ButtonHandler_t = ButtonHandler<ButtonMock>;

class ButtonHandlerTestFixture
{

public:
    ButtonHandlerTestFixture() : handler(nullptr)
    {
        registerCallback();
        handler->start();
    }

    ~ButtonHandlerTestFixture()
    {
        if (handler)
        {
            handler->stop();
            delete handler;
        }

        btn.reset();
    }

protected:
    void registerCallback()
    {
        using namespace std::placeholders;
        ButtonHandler_t::ButtonCallback cb =
            bind(&ButtonHandlerTestFixture::callback, this, _1, _2);
        handler = new ButtonHandler_t(0x5E, cb);
    }

    void callback(int id, ButtonPress pressType)
    {
        if (id != 0x5E)
        {
            return;
        }
        switch (pressType)
        {
            case ButtonPress::DOWN:
                ++downCount;
                break;
            case ButtonPress::UP:
                ++upCount;
                break;
            case ButtonPress::SHORT:
                ++shortPressCount;
                break;
            case ButtonPress::LONG:
                ++longPressCount;
                break;
            case ButtonPress::VERY_LONG:
                ++veryLongPressCount;
                break;
            default:
                break;
        }
    }

    unsigned int downCount          = 0;
    unsigned int upCount            = 0;
    unsigned int shortPressCount    = 0;
    unsigned int longPressCount     = 0;
    unsigned int veryLongPressCount = 0;
    ButtonHandler_t* handler;
};

TEST_CASE_METHOD(ButtonHandlerTestFixture,
                 "ButtonHandler - Nothing happens if button is not pressed")
{
    Thread::sleep((VERY_LONG_PRESS_TICKS + 1) * TICK_LENGTH);

    REQUIRE(downCount == 0);
    REQUIRE(upCount == 0);
    REQUIRE(shortPressCount == 0);
    REQUIRE(longPressCount == 0);
    REQUIRE(veryLongPressCount == 0);
}

TEST_CASE_METHOD(ButtonHandlerTestFixture,
                 "ButtonHandler - Handle each button press")
{
    SECTION("Callback")
    {
        btn.press(TICK_LENGTH + 5);

        REQUIRE(downCount == 1);
        REQUIRE(upCount == 1);

        CHECK(longPressCount == 0);
        CHECK(veryLongPressCount == 0);
        REQUIRE(shortPressCount == 1);

        btn.press(TICK_LENGTH * (LONG_PRESS_TICKS + 1));

        REQUIRE(downCount == 2);
        REQUIRE(upCount == 2);
        CHECK(shortPressCount == 1);
        CHECK(veryLongPressCount == 0);
        REQUIRE(longPressCount == 1);

        btn.press(TICK_LENGTH * (VERY_LONG_PRESS_TICKS + 1));

        REQUIRE(downCount == 3);
        REQUIRE(upCount == 3);
        CHECK(shortPressCount == 1);
        CHECK(longPressCount == 1);
        REQUIRE(veryLongPressCount == 1);
    }
}

TEST_CASE_METHOD(ButtonHandlerTestFixture,
                 "ButtonHandler - Just short of each button press")
{
    REQUIRE(longPressCount == 0);
    REQUIRE(veryLongPressCount == 0);
    REQUIRE(shortPressCount == 0);
    REQUIRE(downCount == 0);
    REQUIRE(upCount == 0);

    btn.press(TICK_LENGTH * (LONG_PRESS_TICKS - 1));

    REQUIRE(downCount == 1);
    REQUIRE(upCount == 1);
    REQUIRE(shortPressCount == 1);
    REQUIRE(veryLongPressCount == 0);
    REQUIRE(longPressCount == 0);

    btn.press(TICK_LENGTH * (VERY_LONG_PRESS_TICKS - 1));

    REQUIRE(downCount == 2);
    REQUIRE(upCount == 2);
    REQUIRE(shortPressCount == 1);
    REQUIRE(longPressCount == 1);
    REQUIRE(veryLongPressCount == 0);
}

#endif
