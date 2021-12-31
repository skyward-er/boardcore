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
        getInstance().input_mode = mode == Mode::INPUT;
    }

    static int value()
    {
        const ButtonMock& instance = getInstance();
        if (instance.input_mode)
            return getInstance().btn_value;

        TRACE("Returning 0\n");
        return 0;
    }

    // Test methods
    void setValue(int value) { btn_value = value; }

    void reset()
    {
        btn_value  = 0;
        input_mode = false;
    }

    void press(unsigned int duration)
    {
        setValue(1);
        Thread::sleep(duration);
        setValue(0);
        Thread::sleep(TICK_LENGTH * 2);
    }

private:
    int btn_value   = 0;
    bool input_mode = false;
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

    void callback(int id, ButtonPress press_type)
    {
        if (id != 0x5E)
        {
            return;
        }
        switch (press_type)
        {
            case ButtonPress::DOWN:
                ++down_count;
                break;
            case ButtonPress::UP:
                ++up_count;
                break;
            case ButtonPress::SHORT:
                ++short_press_count;
                break;
            case ButtonPress::LONG:
                ++long_press_count;
                break;
            case ButtonPress::VERY_LONG:
                ++very_long_press_count;
                break;
            default:
                break;
        }
    }

    unsigned int down_count            = 0;
    unsigned int up_count              = 0;
    unsigned int short_press_count     = 0;
    unsigned int long_press_count      = 0;
    unsigned int very_long_press_count = 0;
    ButtonHandler_t* handler;
};

TEST_CASE_METHOD(ButtonHandlerTestFixture,
                 "ButtonHandler - Nothing happens if button is not pressed")
{
    Thread::sleep((VERY_LONG_PRESS_TICKS + 1) * TICK_LENGTH);

    REQUIRE(down_count == 0);
    REQUIRE(up_count == 0);
    REQUIRE(short_press_count == 0);
    REQUIRE(long_press_count == 0);
    REQUIRE(very_long_press_count == 0);
}

TEST_CASE_METHOD(ButtonHandlerTestFixture,
                 "ButtonHandler - Handle each button press")
{
    SECTION("Callback")
    {
        btn.press(TICK_LENGTH + 5);

        REQUIRE(down_count == 1);
        REQUIRE(up_count == 1);

        CHECK(long_press_count == 0);
        CHECK(very_long_press_count == 0);
        REQUIRE(short_press_count == 1);

        btn.press(TICK_LENGTH * (LONG_PRESS_TICKS + 1));

        REQUIRE(down_count == 2);
        REQUIRE(up_count == 2);
        CHECK(short_press_count == 1);
        CHECK(very_long_press_count == 0);
        REQUIRE(long_press_count == 1);

        btn.press(TICK_LENGTH * (VERY_LONG_PRESS_TICKS + 1));

        REQUIRE(down_count == 3);
        REQUIRE(up_count == 3);
        CHECK(short_press_count == 1);
        CHECK(long_press_count == 1);
        REQUIRE(very_long_press_count == 1);
    }
}

TEST_CASE_METHOD(ButtonHandlerTestFixture,
                 "ButtonHandler - Just short of each button press")
{
    REQUIRE(long_press_count == 0);
    REQUIRE(very_long_press_count == 0);
    REQUIRE(short_press_count == 0);
    REQUIRE(down_count == 0);
    REQUIRE(up_count == 0);

    btn.press(TICK_LENGTH * (LONG_PRESS_TICKS - 1));

    REQUIRE(down_count == 1);
    REQUIRE(up_count == 1);
    REQUIRE(short_press_count == 1);
    REQUIRE(very_long_press_count == 0);
    REQUIRE(long_press_count == 0);

    btn.press(TICK_LENGTH * (VERY_LONG_PRESS_TICKS - 1));

    REQUIRE(down_count == 2);
    REQUIRE(up_count == 2);
    REQUIRE(short_press_count == 1);
    REQUIRE(long_press_count == 1);
    REQUIRE(very_long_press_count == 0);
}
