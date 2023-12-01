/* Copyright (c) 2019 Skyward Experimental Rocketry
 * Author: Luca Erbetta
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

/**
 * Polls the sensor at an high rate (1000 Hz), while measuring a precise
 * timestamp for each sample using the DataReady interrupt provided by the
 * sensor. This interrupt is used to measure the timestamp of the last sample
 * read. After reading a certain number of samples, dumps them on the serial in
 * CSV format.
 */

#include <diagnostic/CpuMeter/CpuMeter.h>
#include <drivers/interrupt/external_interrupts.h>
#include <drivers/spi/SPIDriver.h>
#include <drivers/timer/TimestampTimer.h>
#include <sensors/L3GD20/L3GD20.h>
#include <utils/Constants.h>

#include <array>

using namespace Boardcore;
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
    uint64_t sampleDelta;
    L3GD20Data data;
    float cpu;
};

GyroSample data[NUM_SAMPLES];
int dataCounter = 0;

// Last interrupt received timer tick
uint32_t lastSampleTick;  // Stores the high-res tick of the last
                          // interrupt (L3GD20 watermark event)
uint32_t sampleDelta;     // Tick delta between the last 2 watermark
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
    uint64_t currentTimestamp = TimestampTimer::getTimestamp();
    sampleDelta               = currentTimestamp - lastSampleTick;
    lastSampleTick            = currentTimestamp;

    // Pass timestamp to sensor
    if (gyro != nullptr)
    {
        gyro->IRQupdateTimestamp(currentTimestamp);
    }
}

void configure()
{
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

    // Chip select starts high (not asserted)
    cs.high();

    // Enable rising-edge interrupt detection on PA2
    enableExternalInterrupt(GPIOA_BASE, 2, InterruptTrigger::RISING_EDGE);
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
    while (dataCounter < NUM_SAMPLES)
    {
        long lastTime = miosix::getTime();

        // Read data from the sensor
        gyro->sample();

        // Obtain the data
        L3GD20Data d = gyro->getLastSample();

        // Store the sample in the array, togheter with other useful data
        data[dataCounter++] = {d.angularSpeedTimestamp, sampleDelta, d,
                               CpuMeter::getCpuStats().mean};

        // Wait until SAMPLE_PERIOD milliseconds from the start of this
        // iteration have passed (SAMPLE_PERIOD = 1000 / SAMPLE_RATE)
        Thread::nanoSleepUntil(lastTime +
                               (1000 / SAMPLE_RATE) * Constants::NS_IN_MS);
    }

    // Dump buffer content as CSV on the serial (might take a while)
    printf("FIFO_num,timestamp,int_delta,sampleDelta,x,y,z,cpu\n");
    for (int i = 1; i < dataCounter; i++)
    {
        // clang-format off
         printf("%d,%llu,%llu,%llu,%f,%f,%f,%.2f\n",
                0,
                data[i].timestamp,
                TimerUtils::toIntMicroSeconds(
                    TimestampTimer::timestampTimer.getTimer(), data[i].sampleDelta),
                (data[i].timestamp - data[i - 1].timestamp),
                data[i].data.angularSpeedX,
                data[i].data.angularSpeedY,
                data[i].data.angularSpeedZ,
                data[i].cpu);
        // clang-format on
    }

    printf("\n\n\nend.\n");
    for (;;)
    {
        Thread::sleep(1000);
    }
}
