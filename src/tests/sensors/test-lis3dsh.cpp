/* Copyright (c) 2020 Skyward Experimental Rocketry
 * Author: Luca Conterio
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

#include <drivers/spi/SPIDriver.h>
#include <drivers/timer/TimestampTimer.h>
#include <miosix.h>
#include <sensors/LIS3DSH/LIS3DSH.h>

using namespace Boardcore;
using namespace std;
using namespace miosix;

SPIBus bus(SPI1);

GpioPin spiSck(GPIOA_BASE, 5);
GpioPin spiMiso(GPIOA_BASE, 6);
GpioPin spiMosi(GPIOA_BASE, 7);
GpioPin cs(GPIOE_BASE, 3);

int main()
{
    spiSck.mode(miosix::Mode::ALTERNATE);
    spiSck.alternateFunction(5);
    spiMiso.mode(miosix::Mode::ALTERNATE);
    spiMiso.alternateFunction(5);
    spiMosi.mode(miosix::Mode::ALTERNATE);
    spiMosi.alternateFunction(5);

    cs.mode(miosix::Mode::OUTPUT);
    cs.high();

    auto streamRx = DMADriver::instance().acquireStreamForPeripheral(
        DMADefs::Peripherals::PE_SPI1_RX);
    if (!streamRx.isValid())
    {
        printf("Cannot acquire streamRx\n");
        return 1;
    }

    auto streamTx = DMADriver::instance().acquireStreamForPeripheral(
        DMADefs::Peripherals::PE_SPI1_TX);
    if (!streamTx.isValid())
    {
        printf("Cannot acquire streamTx\n");
        return 1;
    }

    LIS3DSH sensor(bus, cs, &streamTx, &streamRx, sensor.ODR_100_HZ, sensor.UPDATE_AFTER_READ_MODE,
                   sensor.FULL_SCALE_4G);
    // LIS3DSH sensor(bus, cs, sensor.ODR_100_HZ, sensor.UPDATE_AFTER_READ_MODE,
    //                sensor.FULL_SCALE_4G);

    LIS3DSHData data;

    // sensor not initialized, should give the error
    sensor.sample();
    if (sensor.getLastError() != SensorErrors::NOT_INIT)
    {
        printf("\nTEST FAILED: sensor not initialized \n");
        return -1;
    }

    // initialize imu
    if (!sensor.init())
    {
        if (sensor.getLastError() == SensorErrors::INVALID_WHOAMI)
            printf("TEST FAILED: invalid WHO_AM_I value, init failed \n");
        else
            printf("TEST FAILED: init failed \n");
        return -1;
    }

    // check if the sensor is properly working
    if (!sensor.selfTest())
    {
        printf("\nTEST FAILED: self-test failed \n");
        return -1;
    }

    Thread::sleep(500);

    // sample some data from the sensor
    for (int i = 0; i < 2; i++)
    {
        // sensor intitialized, should return error if no new data exist
        sensor.sample();

        if (sensor.getLastError() == SensorErrors::NO_NEW_DATA)
            printf("\nWarning: no new data to be read \n");

        data = sensor.getLastSample();

        printf("\nTimestamp: %llu \n", data.accelerationTimestamp);
        printf("Acc: x: %f | y: %f | z: %f \n", data.accelerationX,
               data.accelerationY, data.accelerationZ);
        printf("Temp: %.2f C \n", data.temperature);

        Thread::sleep(200);
    }

    printf("\nLIS3DSH TEST OK ! \n\n\n");


    printf("Beginning readRegisters() test\n");
    if (!sensor.testDmaReadRegisters())
    {
        printf("readRegisters() test FAILED\n");
        return 1;
    }
    printf("readRegisters() test SUCCESSFUL\n\n");


    printf("Beginning writeRegisters() test\n");
    if(!sensor.testDmaWriteRegisters())
    {
        printf("writeRegisters() test FAILED\n");
        return 1;
    }
    printf("writeRegisters() test SUCCESSFUL\n\n");


    printf("Beginning write16() test\n");
    if(!sensor.testDmaWrite16())
    {
        printf("write16() test FAILED\n");
        return 1;
    }
    printf("write16() test SUCCESSFUL\n\n");


    printf("Beginning write24() test\n");
    if(!sensor.testDmaWrite24())
    {
        printf("write24() test FAILED\n");
        return 1;
    }
    printf("write24() test SUCCESSFUL\n\n");


    printf("Beginning write32() test\n");
    if(!sensor.testDmaWrite32())
    {
        printf("write32() test FAILED\n");
        return 1;
    }
    printf("write32() test SUCCESSFUL\n\n");


    printf("Beginning writeBuffer() test\n");
    if(!sensor.testDmaWriteBuffer())
    {
        printf("writeBuffer() test FAILED\n");
        return 1;
    }
    printf("writeBuffer() test SUCCESSFUL\n\n");


    printf("Beginning transferBuffer() test\n");
    if(!sensor.testDmaTransferBuffer())
    {
        printf("transferBuffer() test FAILED\n");
        return 1;
    }
    printf("transferBuffer() test SUCCESSFUL\n\n");


    printf("Beginning transfer24() test\n");
    if(!sensor.testDmaTransfer24())
    {
        printf("transfer24() test FAILED\n");
        return 1;
    }
    printf("transfer24() test SUCCESSFUL\n\n");


    printf("Beginning transfer32() test\n");
    if(!sensor.testDmaTransfer32())
    {
        printf("transfer32() test FAILED\n");
        return 1;
    }
    printf("transfer32() test SUCCESSFUL\n\n");


    printf("Beginning readRegister16() test\n");
    if(!sensor.testDmaReadRegister16())
    {
        printf("readRegister16() test FAILED\n");
        return 1;
    }
    printf("readRegister16() test SUCCESSFUL\n\n");


    printf("Beginning readRegister24() test\n");
    if(!sensor.testDmaReadRegister24())
    {
        printf("readRegister24() test FAILED\n");
        return 1;
    }
    printf("readRegister24() test SUCCESSFUL\n\n");


    printf("Beginning readRegister32() test\n");
    if(!sensor.testDmaReadRegister32())
    {
        printf("readRegister32() test FAILED\n");
        return 1;
    }
    printf("readRegister32() test SUCCESSFUL\n\n");


    printf("Beginning writeRegister16() test\n");
    if(!sensor.testDmaWriteRegister16())
    {
        printf("writeRegister16() test FAILED\n");
        return 1;
    }
    printf("writeRegister16() test SUCCESSFUL\n\n");


    printf("Beginning writeRegister24() test\n");
    if(!sensor.testDmaWriteRegister24())
    {
        printf("writeRegister24() test FAILED\n");
        return 1;
    }
    printf("writeRegister24() test SUCCESSFUL\n\n");


    printf("Beginning writeRegister32() test\n");
    if(!sensor.testDmaWriteRegister32())
    {
        printf("writeRegister32() test FAILED\n");
        return 1;
    }
    printf("writeRegister32() test SUCCESSFUL\n\n");


    printf("Beginning write16buffer() test\n");
    if(!sensor.testDmaWrite16Buffer())
    {
        printf("write16buffer() test FAILED\n");
        return 1;
    }
    printf("write16buffer() test SUCCESSFUL\n\n");


    printf("Beginning transfer16buffer() test\n");
    if(!sensor.testDmaTransfer16Buffer())
    {
        printf("transfer16buffer() test FAILED\n");
        return 1;
    }
    printf("transfer16buffer() test SUCCESSFUL\n\n");


    printf("ALL TESTS TERMINATED CORRECTLY\n");
    return 0;
}
