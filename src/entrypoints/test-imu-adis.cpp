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
#include <sensors/ADIS16405.h>
#include <sensors/MPU9250.h>
#include <sensors/bme280.h>

#include <interfaces-impl/hwmapping.h>

using namespace miosix;
using namespace miosix::interfaces;
// SPI1
/*typedef Gpio<GPIOA_BASE, 5> GpioSck;
typedef Gpio<GPIOA_BASE, 6> GpioMiso;
typedef Gpio<GPIOA_BASE, 7> GpioMosi;
typedef Gpio<GPIOC_BASE, 3> CS_MPU9250;*/

// Testing the pressure sensor on MPU's place
// SPI1 binding al sensore
typedef BusSPI<1,spi1::mosi,spi1::miso,spi1::sck> busSPI1; //Creo la SPI1
typedef ProtocolSPI<busSPI1,sensors::mpu9250::cs> spiMPU9250_a; //La lego al Chip Select 1 per la IMU 1
typedef BME280<spiMPU9250_a> mpu1_t; //Passo il bus creato al sensore

// typedef BusSPI<1,spi1::mosi,spi1::miso,spi1::sck> busSPI1; //Creo la SPI1
// typedef ProtocolSPI<busSPI1,sensors::max21105::cs> spiMAX21105_a; //La lego al Chip Select 1 per la IMU 1
// typedef MPU9250<spiMPU9250_a> max1_t; //Passo il bus creato al sensore

// SPI1
/*typedef Gpio<GPIOA_BASE, 5> GpioSck;
typedef Gpio<GPIOA_BASE, 6> GpioMiso;
typedef Gpio<GPIOA_BASE, 7> GpioMosi;
typedef Gpio<GPIOC_BASE, 3> CS_MPU9250;*/

// SPI1 binding al sensore
/*typedef BusSPI<1,GpioMosi,GpioMiso,GpioSck> busSPI1; //Creo la SPI1
typedef ProtocolSPI<busSPI1,CS_MPU9250> spiMPU9250_a; //La lego al Chip Select 1 per la IMU 1
typedef MPU9250<spiMPU9250_a> mpu1_t; //Passo il bus creato al sensore*/


int main()
{
	// MPU9250
	spiMPU9250_a::init();
	mpu1_t* mpu1 = new mpu1_t();
	// mpu1_t* mpu1 = new mpu1_t(mpu1_t::ACC_FS_2G, mpu1_t::GYRO_FS_250);
	// AccelSensor *myMPU = mpu1;
	if(mpu1->init())
		printf("MPU9250 Init succeeded\n" );
	else
		printf("MPU9250 Init failed\n");

	// // MAX21105
	// spiMAX21105_a::init();
	// max1_t* max1 = new max1_t(max1_t::ACC_FS_2G, max1_t::GYRO_FS_250);
	// AccelSensor *myMAX = max1;
	// if(max1->init())
	// 	printf("MAX21105 Init succeeded\n" );
	// else
	// 	printf("MAX21105 Init failed\n");

	Thread::sleep(2000);

    while(true)
    {
    	if(mpu1->init())
		printf("MPU9250 Init succeeded\n" );
		else
			printf("MPU9250 Init failed\n");

  //   	const Vec3* last_data = myMPU->accelDataPtr();
  //   	// const Vec3* last_data = myMAX->accelDataPtr();

  //   	printf("new max data:\n" );

  //   	printf("%f %f %f\n", last_data->getX(),last_data->getY(),last_data->getZ());
		// Thread::sleep(2000);
    }
}