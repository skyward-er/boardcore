/*
 * Copyright (c) 2018 Skyward Experimental Rocketry
 * Authors: Luca Erbetta
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
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.  IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 */

#include <miosix.h>

#include "ActiveObject.h"
#include "drivers/adc/ADC.h"

#include "boards/Homeone/DeploymentController/ThermalCutter/Cutter.h"

using namespace HomeoneBoard;
using namespace miosix;

typedef miosix::Gpio<GPIOA_BASE, 0> btn;
typedef miosix::Gpio<GPIOA_BASE, 5> csense;

static const int BUTTON_SLEEP  = 10;
static const int SAMPLE_FREQ   = 10;
static const int SAMPLE_PERIOD = 1000 / SAMPLE_FREQ;

using miosix::Thread;

using ADC_t = SensorADC<1, 5, csense>;
float adcToI(uint16_t adc_in)
{
    float v    = (adc_in * 3.3f) / 4096;
    float iout = v / 525;
    return (iout - 0.000030) * 10000;
}

void awaitButton(int time)
{
    int pressed_for = 0;

    // Wait until button released
    while (btn::value() == 1)
    {
        miosix::Thread::sleep(BUTTON_SLEEP);
    }

    do
    {
        if (btn::value() == 1)
        {
            pressed_for += BUTTON_SLEEP;
        }
        else
        {
            if (pressed_for >= time)
            {
                break;
            }
            pressed_for = 0;
        }
        Thread::sleep(BUTTON_SLEEP);
    } while (true);
}

class Runner : public ActiveObject
{
protected:
    void run() override
    {
        Cutter cutter;

        printf("Press the button for 1 second to enable the cutter\n");

        awaitButton(1000);

        printf("Cutter enabled\n");
        miosix::ledOn();
        cutter.startCutDrogue();  // TIM4-CH1

        awaitButton(50);
        printf("Cutter disabled\n");
        cutter.stopCutDrogue();
        miosix::ledOff();
    }
};
int main()
{
    // csense::mode(Mode::INPUT_ANALOG);

    ADC_t adc(SAMPLE_FREQ);
    adc.init();

    miosix::ledOff();
    btn::mode(miosix::Mode::INPUT);

    Runner r;
    r.start();

    for (;;)
    {
        adc.updateParams();
        uint16_t adcval = adc.getValue();
        float ival = adcToI(adcval);

        printf("%f,\t%d\n", ival, (int)adcval);
        Thread::sleep(SAMPLE_PERIOD);
    }
}