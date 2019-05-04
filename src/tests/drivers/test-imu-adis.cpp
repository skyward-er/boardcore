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
#include <sensors/ADIS16405/ADIS16405.h>
#include <sensors/SensorSampling.h>
#include <math/Stats.h>
#include <diagnostic/CpuMeter.h>
#include <drivers/spi/SensorSpi.h>

using namespace miosix;
using namespace miosix::interfaces;

// Reset pin
typedef Gpio<GPIOD_BASE, 5> rstPin;

// SPI1 binding to the sensor
typedef BusSPI<1,spi1::mosi,spi1::miso,spi1::sck> busSPI1; //Create SPI1
typedef ProtocolSPI<busSPI1,miosix::sensors::adis16405::cs> spiADIS16405; //La lego al Chip Select 1 per la IMU 1
typedef ADIS16405<spiADIS16405,rstPin> adis_t; //Passo il bus creato al sensore

int main()
{	
	spiADIS16405::init();
	auto& spi = SPIDriver::instance();

	Thread::sleep(1000);
	adis_t adis(adis_t::GYRO_FS_300);

	if(adis.init())
		printf("[ADIS16405] Init succeeded\n" );
	else
		printf("[ADIS16405] Init failed\n");
	
	if(adis.selfTest())
		printf("[ADIS16405] Self test succeeded\n" );
	else
		printf("[ADIS16405] Self test failed\n");

	auto req = adis.buildDMARequest();

    
    auto sample = [&]() {
	    if (spi.transaction(req) == false)
	        puts("DMA error");
        for (auto& r : req)
        {
            adis.onDMAUpdate(r);
            printf("ID: %d  --> ", r.id());
            auto& resp = r.readResponseFromPeripheral();
            memDump(resp.data(), resp.size());
        }
    };

	// SimpleSensorSampler sampler;
	// sampler.AddSensor(&adis);

	// DMASensorSampler sampler;
	// sampler.AddSensor(&adis);

	// StatsResult statResult;
	// Stats stats;

	// int counter = 0;






    while(true)
    {
		// sampler.Update();

		// stats.add(averageCpuUtilization());


		// if(counter == 2500){
		// 	statResult = stats.getStats();
		// 	printf("CPU usage: %f\n", statResult.mean);
		// 	counter = 0;
		// }
		// counter++;

		// Thread::sleep(2);

		sample();

    	const Vec3* last_data = adis.accelDataPtr();
    	printf("%f %f %f\n", last_data->getX(), last_data->getY(),
               last_data->getZ());
		Thread::sleep(100);
    }
}