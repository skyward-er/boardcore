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

#include <drivers/timer/GeneralPurposeTimer.h>
#include <miosix.h>

#include <iostream>
#include <sstream>

#include "Common.h"
#include "drivers/hbridge/HBridge.h"

using namespace miosix;
using namespace std;

static constexpr int PWM_DURATION = 60 * 1000;

static const PWM::Timer HBRIDGE_TIM{
    TIM3, &(RCC->APB1ENR), RCC_APB1ENR_TIM3EN,
    timer::TimerUtils::getPrescalerInputFrequency(
        timer::TimerUtils::InputClock::APB2)};

static const PWMChannel HBRIDGE_PWM_CHANNEL = PWMChannel::CH2;

GpioPin hbridge_in(GPIOB_BASE, 5);       // pwm pin
GpioPin hbridge_inhibit(GPIOB_BASE, 7);  // inhibit pin for the hbridge

bool print = true;  // print the elapsed time or not

long long measured_time = 0;
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

    measured_time = t - t0;
}

int main()
{
    {
        FastInterruptDisableLock l;
        hbridge_in.mode(Mode::ALTERNATE);
        hbridge_in.alternateFunction(2);
        hbridge_inhibit.mode(Mode::OUTPUT);
        hbridge_inhibit.low();
    }

    timer::TimestampTimer::enableTimestampTimer();

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

        HBridge hbridge(hbridge_inhibit, HBRIDGE_TIM, HBRIDGE_PWM_CHANNEL, freq,
                        duty / 100);

        hbridge.enable();
        TRACE("Hbridge status : timestamp = %llu - state = %d \n",
              hbridge.getStatus().timestamp, hbridge.getStatus().state);

        wait();

        hbridge.disable();
        TRACE("Hbridge status : timestamp = %llu - state = %d \n",
              hbridge.getStatus().timestamp, hbridge.getStatus().state);

        Thread::sleep(500);

        TRACE("Elapsed time: %.2f s\n", (measured_time) / 1000.0f);
        TRACE("Done!\n\n");
    }

    return 0;
}
