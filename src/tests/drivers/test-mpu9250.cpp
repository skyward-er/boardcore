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
#include <sensors/SensorSampling.h>

using namespace miosix;
using namespace miosix::interfaces;

//typedef Gpio<GPIOC_BASE, 3> CS_MPU9250;

// SPI1 binding to the sensor
typedef BusSPI<1,spi1::mosi,spi1::miso,spi1::sck> busSPI1; //Create SPI1
typedef ProtocolSPI<busSPI1,sensors::mpu9250::cs> spiMPU9250_a; //La lego al Chip Select 1 per la IMU 1
// typedef ProtocolSPI<busSPI1,CS_MPU9250> spiMPU9250_a; //La lego al Chip Select 1 per la IMU 1
typedef MPU9250<spiMPU9250_a> mpu_t; //Passo il bus creato al sensore

int main()
{	
	// CS_MPU9250::mode(miosix::Mode::OUTPUT);
	// CS_MPU9250::high();

    DMASensorSampler sampler;

	spiMPU9250_a::init();
	mpu_t* mpu = new mpu_t(1, 1);

	Thread::sleep(1000);
	
	if(mpu->init()){
		printf("MPU9250 Init succeeded\n" );
        sampler.AddSensor(mpu);
    }
	else {
		printf("MPU9250 Init failed\n");
        for(;;);
    }

	Thread::sleep(100);


    while(true)
    {
        sampler.Update();

        const Vec3* last_data = mpu->accelDataPtr();
        printf("%f %f %f\n", last_data->getX(), last_data->getY(),
               last_data->getZ());
        Thread::sleep(100);
    }
}
