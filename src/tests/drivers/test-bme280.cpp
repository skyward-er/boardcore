/**
 * Copyright (c) 2019 Skyward Experimental Rocketry
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

#include <sensors/BME280/BME280.h>
#include <miosix.h>
#include <drivers/BusTemplate.h>

// SPI1
typedef miosix::Gpio<GPIOA_BASE, 5> GpioSck;
typedef miosix::Gpio<GPIOB_BASE, 4> GpioMiso;
typedef miosix::Gpio<GPIOA_BASE, 7> GpioMosi;
typedef miosix::Gpio<GPIOC_BASE, 1> CS;

// SPI1 binding al sensore
typedef BusSPI<1, GpioMosi, GpioMiso, GpioSck> busSPI1;  // Creo la SPI1
typedef ProtocolSPI<busSPI1, CS> spiProt;   // La lego al Chip Select 1 per la IMU 1

//typedef BME280<spiProt> bme_t;  // Passo il bus creato al sensore

int main ()
{
    BME280<spiProt> bme;
    bme.init();
    
    while(1)
    {
        bme.onSimpleUpdate();
        printf("%d,%f,%f\n", bme.getDataPtr()->timestamp, bme.getDataPtr()->temperature, bme.getDataPtr()->pressure);

        miosix::Thread::sleep(500);
    }
}