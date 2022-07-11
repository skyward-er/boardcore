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

#include <miosix.h>
#include <utils/PinObserver/PinObserver.h>

#include <functional>

using namespace Boardcore;
using namespace miosix;
using namespace std;
using namespace std::placeholders;

static constexpr unsigned int POLL_INTERVAL = 20;

static constexpr unsigned int BTN_PORT  = GPIOA_BASE;
static constexpr unsigned char BTN_PIN  = 0;
static constexpr unsigned int PIN1_PORT = GPIOE_BASE;
static constexpr unsigned char PIN1_PIN = 5;
static constexpr unsigned int PIN2_PORT = GPIOE_BASE;
static constexpr unsigned char PIN2_PIN = 6;

void onTransition(GpioPin pin, PinTransition transition)
{
    if (pin.getPort() == BTN_PORT && pin.getNumber() == BTN_PIN)
        printf("BTN transition: ");
    if (pin.getPort() == PIN1_PORT && pin.getNumber() == PIN1_PIN)
        printf("PIN1 transition: ");
    if (pin.getPort() == PIN2_PORT && pin.getNumber() == PIN2_PIN)
        printf("PIN2 transition: ");

    if (transition == PinTransition::FALLING_EDGE)
        printf("FALLING_EDGE\n");
    else
        printf("RISING_EDGE\n");
}

int main()
{
    auto btn  = GpioPin(BTN_PORT, BTN_PIN);
    auto pin1 = GpioPin(PIN1_PORT, PIN1_PIN);
    auto pin2 = GpioPin(PIN2_PORT, PIN2_PIN);

    btn.mode(Mode::INPUT);
    pin1.mode(Mode::INPUT_PULL_DOWN);
    pin2.mode(Mode::INPUT_PULL_UP);

    PinObserver::getInstance().registerPinCallback(
        btn, std::bind(onTransition, btn, _1), 10);
    PinObserver::getInstance().registerPinCallback(
        pin1, std::bind(onTransition, pin1, _1), 10);
    PinObserver::getInstance().registerPinCallback(
        pin2, std::bind(onTransition, pin2, _1), 10);

    while (true)
        Thread::sleep(10000);
}
