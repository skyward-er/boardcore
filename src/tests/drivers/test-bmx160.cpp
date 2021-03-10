/**
 * Copyright (c) 2020 Skyward Experimental Rocketry
 * Authors: Davide Mor
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

/*
Wiring on STM32F407VG Discovery:
| BMX Shuttle | Discovery       |
| ----------- | --------------- |
| MISO (4)    | SPI3_MISO (PB4) |
| MOSI (5)    | SPI3_MOSI (PB5) |
| SCK  (6)    | SPI3_SCK  (PB3) |
| CS   (7)    | GPIO      (PB7) |
| INT1 (20)   | GPIO      (PB8) |
*/

#include <Common.h>
#include <drivers/HardwareTimer.h>
#include <drivers/interrupt/external_interrupts.h>
#include <sensors/BMX160/BMX160.h>

SPIBus bus(SPI3);
GpioPin cs(GPIOB_BASE, 7);

GpioPin spi_sck(GPIOB_BASE, 3);
GpioPin spi_miso(GPIOB_BASE, 4);
GpioPin spi_mosi(GPIOB_BASE, 5);

BMX160 *sensor = nullptr;
uint32_t tick  = 0;

HardwareTimer<uint32_t> hrclock{
    TIM5, TimerUtils::getPrescalerInputFrequency(TimerUtils::InputClock::APB1)};

void __attribute__((used)) EXTI8_IRQHandlerImpl()
{
    tick = hrclock.toIntMicroSeconds(hrclock.tick());
    if (sensor)
    {
        sensor->IRQupdateTimestamp(tick);
    }
}

int main()
{
    TimestampTimer::enableTimestampTimer();

    {
        miosix::FastInterruptDisableLock _lock;

        // Enable TIM5 and SPI3 bus
        RCC->APB1ENR |= RCC_APB1ENR_SPI3EN | RCC_APB1ENR_TIM5EN;

        // Setup correct alternate functions for SPI3 bus
        spi_sck.mode(miosix::Mode::ALTERNATE);
        spi_miso.mode(miosix::Mode::ALTERNATE);
        spi_mosi.mode(miosix::Mode::ALTERNATE);

        spi_sck.alternateFunction(6);
        spi_miso.alternateFunction(6);
        spi_mosi.alternateFunction(6);

        // Setup CS
        cs.mode(miosix::Mode::OUTPUT);
    };

    cs.high();

    hrclock.setPrescaler(382);
    hrclock.start();

    enableExternalInterrupt(GPIOB_BASE, 8, InterruptTrigger::FALLING_EDGE);

    BMX160Config config;
    config.fifo_mode    = BMX160Config::FifoMode::HEADER;
    config.fifo_int     = BMX160Config::FifoInt::PIN_INT1;
    config.temp_divider = 1;

    sensor = new BMX160(bus, cs, config);

    TRACE("Initializing BMX160...\n");

    if (!sensor->init())
    {
        TRACE("Init failed! (code: %d)\n", sensor->getLastError());
        return -1;
    }

    TRACE("Performing self-test...\n");

    if (!sensor->selfTest())
    {
        TRACE("Self-test failed! (code: %d)\n", sensor->getLastError());
        return -1;
    }

    TRACE("Self-test successful!\n");

    while (1)
    {
        miosix::Thread::sleep(5000);

        printf("----------------------------\n");

        uint32_t now = hrclock.toIntMicroSeconds(hrclock.tick());

        sensor->sample();
        if (sensor->getLastError() != SensorErrors::NO_ERRORS)
        {
            TRACE("Failed to read data!\n");
            continue;
        }

        printf("Tick: %.4f s, Now: %.4f s\n", tick / 1000000.0f,
               now / 1000000.0f);
        printf("Temp: %.2f deg\n", sensor->getTemperature());
        printf("Fill: %d\n", sensor->getLastFifoSize());

        printf("----------------------------\n");
        uint8_t len = std::min(sensor->getLastFifoSize(), (uint8_t)5);

        for (uint8_t i = 0; i < len; i++)
        {
            BMX160Data data = sensor->getFifoElement(i);
            printf("Mag [%.4f s]:\t%.2f\t%.2f\t%.2f\n",
                   data.mag_timestamp / 1000000.0f, data.mag_x, data.mag_y,
                   data.mag_z);

            printf("Gyr [%.4f s]:\t%.2f\t%.2f\t%.2f\n",
                   data.gyro_timestamp / 1000000.0f, data.gyro_x, data.gyro_y,
                   data.gyro_z);

            printf("Acc [%.4f s]:\t%.2f\t%.2f\t%.2f\n",
                   data.accel_timestamp / 1000000.0f, data.accel_x, data.accel_y,
                   data.accel_z);
        }
    }

    return 0;
}