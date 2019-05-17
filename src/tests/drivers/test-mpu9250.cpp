/* Copyright (c) 2018 Skyward Experimental Rocketry
 * Authors: Nuno Barcellos
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
#include <drivers/BusTemplate.h>
#include <interfaces-impl/hwmapping.h>
#include <sensors/MPU9250/MPU9250.h>

#include <drivers/spi/SensorSpi.h>
#include <sensors/SensorSampling.h>

using namespace miosix;
using namespace miosix::interfaces;

typedef BusSPI<1,spi1::mosi,spi1::miso,spi1::sck> busSPI1;
typedef ProtocolSPI<busSPI1,sensors::mpu9250::cs> spiMPU9250_a;
typedef MPU9250<spiMPU9250_a> mpu_t;

int main()
{   
    SimpleSensorSampler sampler;

    spiMPU9250_a::init();
    mpu_t* mpu = new mpu_t(1, 1);

    Thread::sleep(100);
    
    if(mpu->init()){
        printf("MPU9250 Init succeeded\n" );
        sampler.AddSensor(mpu);
    }
    else {
        printf("MPU9250 Init failed\n");

        while(!mpu->init()) {
            printf("MPU9250 Init failed\n");
            Thread::sleep(1000);
        }
    }

    Thread::sleep(100);

    while(true)
    {
        // sampler.Update();
        mpu->updateMagneto();

        const Vec3* last_data = mpu->compassDataPtr();
        printf("%f %f %f\n", last_data->getX(), last_data->getY(),
               last_data->getZ());

        Thread::sleep(100);
    }
}