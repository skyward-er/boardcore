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

#include "drivers/spi/SPIDriver.h"
#include "sensors/L3GD20.h"
#include "diagnostic/CpuMeter.h"

using namespace miosix;
typedef Gpio<GPIOF_BASE, 7> GpioSck;
typedef Gpio<GPIOF_BASE, 8> GpioMiso;
typedef Gpio<GPIOF_BASE, 9> GpioMosi;

static constexpr bool FIFO_ENABLED = true;
static constexpr int NUM_SAMPLES   = 10000;

struct GyroSample
{
    float timestamp;
    Vec3 data;
    int level;
    float cpu;
};

// 364 KB buffer to store up to 30 seconds of data @ 760 Hz
GyroSample data[22800];
int data_counter = 0;

int main()
{
    Thread::sleep(100);
    GpioPin cs(GPIOC_BASE, 1);

    {
        FastInterruptDisableLock dLock;

        RCC->APB2ENR |= RCC_APB2ENR_SPI5EN;

        GpioSck::mode(Mode::ALTERNATE);
        GpioMiso::mode(Mode::ALTERNATE);
        GpioMosi::mode(Mode::ALTERNATE);

        GpioSck::alternateFunction(5);
        GpioMiso::alternateFunction(5);
        GpioMosi::alternateFunction(5);
        cs.mode(Mode::OUTPUT);
    }
    cs.high();

    SPIBus bus(SPI5);
    L3GD20 gyro(bus, cs, L3GD20::FullScaleRange::FS_500,
                L3GD20::OutPutDataRate::ODR_760, 0x03, FIFO_ENABLED);

    while(!gyro.init())
    {

    }

    Thread::sleep(500);

    uint32_t first_tick = miosix::getTick();
    while (data_counter < NUM_SAMPLES)
    {
        uint64_t last_tick = miosix::getTick();

        if (FIFO_ENABLED)
        {
            // Wait until fifo has about 24 samples
            Thread::sleepUntil(last_tick + 32);
            // Extract data from the fifo
            gyro.onSimpleUpdate();
            uint8_t level = gyro.getLastFifoSize();
            Vec3* fifo    = gyro.getLastFifo();

            for (int i = 0; i < level; i++)
            {
                data[data_counter++] = {
                    (last_tick - first_tick) / 1000.0f + (i + 1) / 760.0f,
                    fifo[i], level, averageCpuUtilization()};
                if (data_counter >= NUM_SAMPLES)
                {
                    break;
                }
            }
        }
        else
        {
            // Sample sensor @ 500 Hz
            gyro.onSimpleUpdate();
            
            data[data_counter++] = {(last_tick - first_tick) / 1000.0f,
                                    *(gyro.gyroDataPtr()), 0, averageCpuUtilization()};
            Thread::sleepUntil(last_tick + 2);
        }
    }
    // Dump buffer content as CSV
    for (int i = 0; i < data_counter; i++)
    {
        printf("%f,%f,%f,%f,%d,%.2f\n", data[i].timestamp, data[i].data.getX(),
               data[i].data.getY(), data[i].data.getZ(),data[i].level, data[i].cpu);
    }

    printf("\n\n\nend.\n");
    for (;;)
    {
        Thread::sleep(1000);
    }
}