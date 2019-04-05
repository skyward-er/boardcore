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

using namespace miosix;
using namespace miosix::interfaces;

// SPI1
// typedef Gpio<GPIOA_BASE, 5> GpioSck;
// typedef Gpio<GPIOA_BASE, 6> GpioMiso;
// typedef Gpio<GPIOA_BASE, 7> GpioMosi;
// typedef Gpio<GPIOA_BASE, 8> CS_ADIS16405;

typedef Gpio<GPIOD_BASE, 5> rstPin;

// SPI1 binding to the sensor
typedef BusSPI<1,spi1::mosi,spi1::miso,spi1::sck> busSPI1; //Create SPI1
typedef ProtocolSPI<busSPI1,miosix::sensors::adis16405::cs> spiADIS16405; //La lego al Chip Select 1 per la IMU 1
typedef ADIS16405<spiADIS16405,rstPin> adis_t; //Passo il bus creato al sensore

int main()
{	
	// CS_ADIS16405::mode(miosix::Mode::OUTPUT);
	// CS_ADIS16405::high();

	// ADIS
	spiADIS16405::init();

	Thread::sleep(1000);
	adis_t* adis = new adis_t();

	if(adis->init())
		printf("ADIS Init succeeded\n" );
	else
		printf("ADIS Init failed\n");

	Thread::sleep(1000);
	
	if(adis->selfTest())
		printf("Self test succeeded\n" );
	else
		printf("Self test failed\n");


    while(true)
    {
    	// adis->readTest();
    	adis->burstTest();
		Thread::sleep(100);
    }
}