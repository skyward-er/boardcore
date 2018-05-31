/* Copyright (c) 2015-2018 Skyward Experimental Rocketry
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
#ifndef SRC_SHARED_BOARDS_HOMEONE_SENSORMANAGER_SENSORMANAGERCONFIG_H
#define SRC_SHARED_BOARDS_HOMEONE_SENSORMANAGER_SENSORMANAGERCONFIG_H

#include <Common.h>
#include <drivers/BusTemplate.h>
#include <drivers/stm32f2_f4_i2c.h>

using miosix::Gpio;

typedef ProtocolI2C<miosix::I2C1Driver> busI2C1;

// SPI1
typedef Gpio<GPIOA_BASE, 5> GpioSck;
typedef Gpio<GPIOA_BASE, 6> GpioMiso;
typedef Gpio<GPIOA_BASE, 7> GpioMosi;
typedef BusSPI<1, GpioMosi, GpioMiso, GpioSck> busSPI1;

// CSs
typedef Gpio<GPIOC_BASE, 3> csMPU9250;
typedef Gpio<GPIOC_BASE, 1> csMAX21105;

// Spi protocol defs
typedef ProtocolSPI<busSPI1, csMPU9250> spiMPU9250;
typedef ProtocolSPI<busSPI1, csMAX21105> spiMAX21105;

static const uint8_t AD7994_I2C_ADDRESS = 0x24;  // Todo: Update with real value

#endif /* SRC_SHARED_BOARDS_HOMEONE_SENSORMANAGER_SENSORMANAGERCONFIG_H */
