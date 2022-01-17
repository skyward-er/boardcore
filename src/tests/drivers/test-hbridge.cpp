/* Copyright (c) 2020 Skyward Experimental Rocketry
 * Authors: Luca Erbetta, Luca Conterio
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

#include <drivers/hbridge/HBridge.h>
#include <miosix.h>
#include <utils/Debug.h>

#include <iostream>
#include <sstream>

using namespace Boardcore;
using namespace miosix;
using namespace std;

static constexpr int PWM_DURATION = 60 * 1000;

static const PWM::Timer HBRIDGE_TIM{
    TIM3, &(RCC->APB1ENR), RCC_APB1ENR_TIM3EN,
    ClockUtils::getAPBFrequecy(ClockUtils::APB::APB2)};

static const TimerUtils::Channel HBRIDGE_PWM_CHANNEL =
    TimerUtils::Channel::CHANNEL_2;

GpioPin hbridgeIn(GPIOB_BASE, 5);       // pwm pin
GpioPin hbridgeInhibit(GPIOB_BASE, 7);  // inhibit pin for the hbridge

bool print = true;  // print the elapsed time or not

long long measuredTime = 0;
void wait()
{
    long long t  = getTick();
    long long t0 = t;

    while (t < t0 + PWM_DURATION)
    {
        Thread::sleep(50);

        t = getTick();

        if (print)
        {
            TRACE("Elapsed time : %.2f \n", (t - t0) / 1000.0);
        }
    }

    measuredTime = t - t0;
}

int main()
{
    {
        FastInterruptDisableLock l;
        hbridgeIn.mode(Mode::ALTERNATE);
        hbridgeIn.alternateFunction(2);
        hbridgeInhibit.mode(Mode::OUTPUT);
        hbridgeInhibit.low();
    }

    for (;;)
    {
        uint32_t freq = 0;
        float duty    = 0;
        string temp;

        do
        {
            cout << "Insert h-bridge frequency (1-30000 Hz): \n";
            getline(cin, temp);
            stringstream(temp) >> freq;

            cout << "Insert h-bridge duty cycle (0-100 %): \n";
            getline(cin, temp);
            stringstream(temp) >> duty;

            if (!(freq >= 1 && freq <= 30000 && duty >= 0.0f && duty <= 100.0f))
            {
                TRACE("Invalid inputs!\n");
            }
        } while (
            !(freq >= 1 && freq <= 30000 && duty >= 0.0f && duty <= 100.0f));

        do
        {
            cout << "READY!\nWrite 'yeet' to begin:\n";
            getline(cin, temp);
        } while (temp != "yeet");

        HBridge hbridge(hbridgeInhibit, HBRIDGE_TIM, HBRIDGE_PWM_CHANNEL, freq,
                        duty / 100);

        hbridge.enable();
        TRACE("Hbridge enabled\n");

        wait();

        hbridge.disable();
        TRACE("Hbridge disabled\n");

        Thread::sleep(500);

        TRACE("Elapsed time: %.2f s\n", (measuredTime) / 1000.0f);
        TRACE("Done!\n\n");
    }

    return 0;
}
