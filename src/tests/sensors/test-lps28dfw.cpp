/* Copyright (c) 2022 Skyward Experimental Rocketry
 * Author: Emilio Corigliano
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

#include <drivers/interrupt/external_interrupts.h>

#include <iostream>

#include "miosix.h"
#include "sensors/LPS28DFW/LPS28DFW.h"
#include "sensors/LPS28DFW/LPS28DFWData.h"
#include "string"
#include "string.h"
#include "thread"

using namespace std;
using namespace miosix;
using namespace Boardcore;

// I2C1
typedef Gpio<GPIOB_BASE, 8> i1scl;
typedef Gpio<GPIOB_BASE, 9> i1sda;

I2C i2c(I2C1, i1scl::getPin(), i1sda::getPin());
LPS28DFW lps28dfw(i2c, false);
uint8_t nSamples = 10;
/**
 * ONE_SHOT | AVG4   : 161.6 us/samp (W1+R1+W2+W1+R5)   @1Hz: 1.7  uA [FROM DS]
 * ONE_SHOT | AVG512 : 162 us/samp (W1+R1+W2+W1+R5)     @1Hz: 32.2 uA [FROM DS]
 * ODR1     | AVG4   : 127.8 us/samp (W1+R1+W1+R5)      @1Hz: 2.5  uA [FROM DS]
 * ODR1     | AVG512 : 127.8 us/samp (W1+R1+W1+R5)      @1Hz: 32.8 uA [FROM DS]
 */
miosix::Thread *waiting = 0;
bool sampleAvailable    = false;
void __attribute__((used)) EXTI5_IRQHandlerImpl()
{
    if (waiting)
    {
        sampleAvailable = true;
        waiting->IRQwakeup();
    }
}

void sampleOneShotMode(LPS28DFW &lps28dfw)
{
    printf("Start One-Shot\n");
    LPS28DFW::SensorConfig lps28dfwConfig{
        LPS28DFW::FullScaleRange::FS_1260, LPS28DFW::AVG::AVG_64,
        LPS28DFW::Mode::ONE_SHOT_MODE, LPS28DFW::ODR::ONE_SHOT, false};
    lps28dfw.setConfig(lps28dfwConfig);

    for (uint8_t i = 0; i < nSamples; i++)
    {
        lps28dfw.sample();

        if (lps28dfw.getLastError() == SensorErrors::NO_ERRORS)
        {
            lps28dfw.getLastSample().print(std::cout);
        }
        else
        {
            printf("Error: %d\n", lps28dfw.getLastError());
        }
        Thread::sleep(100);
    }

    printf("End One-Shot\n\n");
}

void sampleContinuousMode(LPS28DFW &lps28dfw)
{
    printf("Start Continuous\n");
    LPS28DFW::SensorConfig lps28dfwConfig{
        LPS28DFW::FullScaleRange::FS_1260, LPS28DFW::AVG::AVG_64,
        LPS28DFW::Mode::CONTINUOUS_MODE, LPS28DFW::ODR::ODR_10, false};
    lps28dfw.setConfig(lps28dfwConfig);

    for (uint8_t i = 0; i < nSamples; i++)
    {
        lps28dfw.sample();

        if (lps28dfw.getLastError() == SensorErrors::NO_ERRORS)
        {
            lps28dfw.getLastSample().print(std::cout);
        }
        else
        {
            printf("Error: %d\n", lps28dfw.getLastError());
        }

        Thread::sleep(100);
    }

    printf("End Continuous\n\n");
}

void sampleInterruptMode(LPS28DFW &lps28dfw)
{
    printf("Start Interrupt\n");
    LPS28DFW::SensorConfig lps28dfwConfig{
        LPS28DFW::FullScaleRange::FS_1260, LPS28DFW::AVG::AVG_64,
        LPS28DFW::Mode::CONTINUOUS_MODE, LPS28DFW::ODR::ODR_10, true};
    lps28dfw.setConfig(lps28dfwConfig);

    for (uint8_t i = 0; i < nSamples; i++)
    {

        while (!sampleAvailable)
        {
            waiting->wait();
        }

        sampleAvailable = false;
        lps28dfw.sample();

        if (lps28dfw.getLastError() == SensorErrors::NO_ERRORS)
        {
            lps28dfw.getLastSample().print(std::cout);
        }
        else
        {
            printf("Error: %d\n", lps28dfw.getLastError());
        }
    }

    printf("End Interrupt\n\n");
}

int main()
{
    waiting = Thread::getCurrentThread();
    enableExternalInterrupt(GPIOE_BASE, 5, InterruptTrigger::RISING_EDGE);

    if (!lps28dfw.init())
    {
        printf("Error initialization of sensor\n");
        return 0;
    }

    for (;;)
    {
        printf("\nNew set of samples\n");
        sampleOneShotMode(lps28dfw);
        sampleContinuousMode(lps28dfw);
        sampleInterruptMode(lps28dfw);
    }

    return 0;
}
