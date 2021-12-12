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
#include <utils/PinObserver.h>

using namespace Boardcore;
using namespace miosix;

static constexpr unsigned int POLL_INTERVAL = 20;

static constexpr unsigned int BTN_PORT  = GPIOA_BASE;
static constexpr unsigned char BTN_PIN  = 0;
static constexpr unsigned int PIN1_PORT = GPIOE_BASE;
static constexpr unsigned char PIN1_PIN = 5;
static constexpr unsigned int PIN2_PORT = GPIOE_BASE;
static constexpr unsigned char PIN2_PIN = 6;

void onStateChange(unsigned int p, unsigned char n, int state)
{
    if (p == BTN_PORT && n == BTN_PIN)
    {
        printf("BTN state change: %d\n", state);
    }
    if (p == PIN1_PORT && n == PIN1_PIN)
    {
        printf("PIN1 state change: %d\n", state);
    }
    if (p == PIN2_PORT && n == PIN2_PIN)
    {
        printf("PIN2 state change: %d\n", state);
    }
}

void onTransition(unsigned int p, unsigned char n)
{
    if (p == BTN_PORT && n == BTN_PIN)
    {
        printf("BTN transition.\n");
    }
    if (p == PIN1_PORT && n == PIN1_PIN)
    {
        printf("PIN1 transition.\n");
    }
    if (p == PIN2_PORT && n == PIN2_PIN)
    {
        printf("PIN2 transition.\n");
    }
}

int main()
{
    GpioPin(BTN_PORT, BTN_PIN).mode(Mode::INPUT);
    GpioPin(PIN1_PORT, PIN1_PIN).mode(Mode::INPUT_PULL_DOWN);
    GpioPin(PIN2_PORT, PIN2_PIN).mode(Mode::INPUT_PULL_UP);

    PinObserver observer;

    observer.observePin(BTN_PORT, BTN_PIN, PinObserver::Transition::RISING_EDGE,
                        &onTransition, 1, onStateChange);

    observer.observePin(PIN1_PORT, PIN1_PIN,
                        PinObserver::Transition::FALLING_EDGE, &onTransition,
                        1000 / POLL_INTERVAL, onStateChange);

    observer.observePin(PIN2_PORT, PIN2_PIN,
                        PinObserver::Transition::RISING_EDGE, &onTransition,
                        1000 / POLL_INTERVAL, onStateChange);

    observer.start();
    for (;;)
    {
        Thread::sleep(10000);
    }
}
