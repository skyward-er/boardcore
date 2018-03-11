/* Copyright (c) 2015-2016 Skyward Experimental Rocketry
 * Authors: Silvano Seva
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
 
#include <Common.h>
#include <BusTemplate.h>
#include <sensors/MPL3115.h>
#include <Leds.h>
#include <drivers/stm32f2_f4_i2c.h>
// #include <util/software_i2c.h>

using namespace miosix;

int main()
{
    sensors::sda::mode(Mode::ALTERNATE_OD);
    sensors::sda::alternateFunction(4);
    sensors::scl::mode(Mode::ALTERNATE_OD);
    sensors::scl::alternateFunction(4);
    
    MPL3115<ProtocolI2C<I2C1Driver>> baro;
    
    printf("%d\n", baro.selfTest() ? 1 : 0);
    baro.setMode(baro.MODE_ALTIMETER);
    baro.setOversampleRatio(128);
    
    while(1)
    {
        baro.onSimpleUpdate();
        printf("press %f, temp %f, alt %f\n",
                *(baro.pressureDataPtr()),
                *(baro.tempDataPtr()), 
                *(baro.altitudeDataPtr())
        );

        Thread::sleep(1000);
    }
    
    return 0;
}
