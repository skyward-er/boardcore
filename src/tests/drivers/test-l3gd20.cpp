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

/**
 * Polls the sensor at an high rate (1000 Hz), while measuring a precise
 * timestamp for each sample using the DataReady interrupt provided by the
 * sensor. This interrupt is used to measure the timestamp of the last sample
 * read. After reading a certain number of samples, dumps them on the serial in
 * CSV format.
 */

#include <array>

#include "diagnostic/CpuMeter.h"
#include "drivers/HardwareTimer.h"
#include "drivers/interrupt/external_interrupts.h"
#include "drivers/spi/SPIDriver.h"
#include "sensors/L3GD20.h"

using namespace miosix;
using std::array;

// GPIOs
typedef Gpio<GPIOF_BASE, 7> GpioSck;
typedef Gpio<GPIOF_BASE, 8> GpioMiso;
typedef Gpio<GPIOF_BASE, 9> GpioMosi;
typedef Gpio<GPIOA_BASE, 2> GpioINT2;

// L3GD20 SPI
SPIBus bus(SPI5);
GpioPin cs(GPIOC_BASE, 1);

L3GD20* gyro;

static constexpr int SAMPLE_RATE = 1000;

static constexpr int NUM_SAMPLES = SAMPLE_RATE * 5;

struct GyroSample
{
    uint64_t timestamp;
    uint64_t sample_delta;
    L3GD20Data data;
    float cpu;
};

GyroSample data[NUM_SAMPLES];
int data_counter = 0;

// High resolution hardware timer using TIM5
HardwareTimer<uint32_t> hrclock{
    TIM5, TimerUtils::getPrescalerInputFrequency(TimerUtils::InputClock::APB1)};

// Last interrupt received timer tick
uint32_t last_sample_tick;  // Stores the high-res tick of the last
                            // interrupt (L3GD20 watermark event)
uint32_t sample_delta;      // Tick delta between the last 2 watermark
                            // events

/**
 * Interrupt handling routine. Called each time a new sample is available from
 * the gyroscope (Data Ready function on INT2 on the sensor).
 * Stores the timestamp of the last sample and calculates the time delta from
 * the previous sample, then provides the timestamp (in microseconds) to the
 * sensor.
 */
void __attribute__((used)) EXTI2_IRQHandlerImpl()
{
    // Current high resolution tick
    uint32_t tick    = hrclock.tick();
    sample_delta     = tick - last_sample_tick;
    last_sample_tick = tick;

    // Pass timestamp to sensor
    if (gyro != nullptr)
        gyro->IRQupdateTimestamp(hrclock.toIntMicroSeconds(tick));
}

void configure()
{
    {
        FastInterruptDisableLock dLock;

        // Enable SPI5 and TIM5 peripheral clocks
        RCC->APB1ENR |= RCC_APB1ENR_TIM5EN;
        RCC->APB2ENR |= RCC_APB2ENR_SPI5EN;

        // Set SPI pins to correct alternate mode
        GpioSck::mode(Mode::ALTERNATE);
        GpioMiso::mode(Mode::ALTERNATE);
        GpioMosi::mode(Mode::ALTERNATE);

        GpioSck::alternateFunction(5);
        GpioMiso::alternateFunction(5);
        GpioMosi::alternateFunction(5);

        // Setup interrupt pin (sensors pulls INT2 up, so we set it to PULL_DOWN
        // to avoid spurious interrupts)
        GpioINT2::mode(Mode::INPUT_PULL_DOWN);

        // Set chip select pin to OUTPUT
        cs.mode(Mode::OUTPUT);
    }

    // Chip select starts high (not asserted)
    cs.high();

    // Enable rising-edge interrupt detection on PA2
    enableExternalInterrupt(GPIOA_BASE, 2, InterruptTrigger::RISING_EDGE);

    // High resolution clock configuration: Sets the prescaler as to obtain
    // 1.8 hours run time and 1.5 microseconds of resolution
    hrclock.setPrescaler(127);
    hrclock.start();
}

int main()
{
    configure();

    gyro = new L3GD20(bus, cs, L3GD20::FullScaleRange::FS_250,
                      L3GD20::OutPutDataRate::ODR_760, 0x03);

    // Init the gyro
    while (!gyro->init())
    {
        printf("Gyro initialization failure!\n");
        Thread::sleep(100);
    }

    Thread::sleep(500);

    // Collect NUM_SAMPLE samples
    while (data_counter < NUM_SAMPLES)
    {
        long last_tick = miosix::getTick();

        // Read data from the sensor
        gyro->sample();

        // Obtain the data
        L3GD20Data d = gyro->getLastSample();

        // Store the sample in the array, togheter with other useful data
        data[data_counter++] = {d.gyro_timestamp, sample_delta, d,
                                averageCpuUtilization()};

        // Wait until SAMPLE_PERIOD milliseconds from the start of this
        // iteration have passed (SAMPLE_PERIOD = 1000 / SAMPLE_RATE)
        Thread::sleepUntil(last_tick + 1000 / SAMPLE_RATE);
    }

    // Dump buffer content as CSV on the serial (might take a while)
    printf("FIFO_num,timestamp,int_delta,sample_delta,x,y,z,cpu\n");
    for (int i = 1; i < data_counter; i++)
    {
        // clang-format off
         printf("%d,%llu,%llu,%llu,%f,%f,%f,%.2f\n", 
                0,
                data[i].timestamp, 
                hrclock.toIntMicroSeconds(data[i].sample_delta),
                (data[i].timestamp - data[i - 1].timestamp),
                data[i].data.gyro_x, 
                data[i].data.gyro_y, 
                data[i].data.gyro_z,
                data[i].cpu);
        // clang-format on
    }

    printf("\n\n\nend.\n");
    for (;;)
    {
        Thread::sleep(1000);
    }
}
