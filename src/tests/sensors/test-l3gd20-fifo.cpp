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
 * Reads data at the highest possible rate from the sensor (~760 Hz) using the
 * FIFO on the sensor as a buffer to avoid polling too frequently.
 * We perform polling to maintain compatibility with the SensorManager and
 * simplify sampling multiple sensors on the same thread.
 *
 * We read the FIFO at an interval such that it fills up to about 25 samples, to
 * leave us a bit of wiggle room (fifo size is 32 samples). A watermark
 * interrupt is set at about half the samples we plan to read (12 samples), in
 * order to estimate the timestamp of the samples as precisely as possible. The
 * watermark is intentionally set in the "middle" of the fifo as to minimize
 * the possibility of desyncronizations between the sensor and the polling loop:
 * If the watermark is set too close to the end of the fifo, the watermark
 * interrupt may arrive *after* we start emptying the fifo, causing the driver
 * to use an incorrect timestamp for the data. A similar case happens if the
 * timestamp is too close to the beginning of the FIFO.
 *
 * The timestamp for each sample in the fifo is calculated as follows (inside
 * the sensor driver): Knowing the timestamp of the 12th sample (thanks to the
 * watermark interrup) we can estimate the timestamp of every other (previous or
 * subsequent) sample in the FIFO, by doing
 *
 * timestamp(i) = timestamp(12) + SAMPLE_PERIOD*(i-12)
 *
 * After we have collected enough samples, dump them on the serial in CSV format
 * togheter with other useful data.
 */

#include <diagnostic/CpuMeter.h>
#include <drivers/interrupt/external_interrupts.h>
#include <drivers/spi/SPIDriver.h>
#include <drivers/timer/GeneralPurposeTimer.h>
#include <drivers/timer/TimestampTimer.h>
#include <sensors/L3GD20/L3GD20.h>

#include <array>

using namespace Boardcore;
using namespace miosix;
using std::array;

typedef Gpio<GPIOF_BASE, 7> GpioSck;
typedef Gpio<GPIOF_BASE, 8> GpioMiso;
typedef Gpio<GPIOF_BASE, 9> GpioMosi;
typedef Gpio<GPIOA_BASE, 2> GpioINT2;

// L3GD20 SPI
SPIBus bus(SPI5);
GpioPin cs(GPIOC_BASE, 1);

L3GD20* gyro = nullptr;

// Send the watermark interrupt when the FIFO contains FIFO_WATERMARK samples.
static constexpr unsigned int FIFO_WATERMARK = 12;

// Expected frequency from the datasheet is 760 Hz, but due to clock
// misalignment / temperature errors and other factors, the observed clock (and
// output data rate) is the following:
static constexpr float SAMPLE_FREQUENCY = 782.3f;

struct GyroSample
{
    int fifoNum;
    L3GD20Data gyro;
    int level;
    uint64_t wtmDelta;
    float cpu;
    uint64_t update;
};

// How many samples to collect
static constexpr int NUM_SAMPLES = SAMPLE_FREQUENCY * 20;

GyroSample data[NUM_SAMPLES];
int dataCounter = 0;

// Last interrupt received timer tick
volatile uint64_t lastWatermarkTick;  // Stores the high-res tick of the last
                                      // interrupt (L3GD20 watermark event)
volatile uint64_t watermarkDelta;     // Tick delta between the last 2 watermark
                                      // events

/**
 * Interrupt handling routine. Called each time the fifo is filled with
 * FIFO_WATERMARK samples. Stores the timestamp of the interrupt, representing
 * the time the 12th sample was collected. From this, knowing the sample rate,
 * we can obtain the timestamp of all previous and subsequent samples in the
 * FIFO. Also calculates the time delta from the previous watermark, then
 * provides the timestamp (in microseconds) to the sensor.
 */
void __attribute__((used)) EXTI2_IRQHandlerImpl()
{
    // Current high resolution tick
    uint64_t currentTimestamp = TimestampTimer::getInstance().getTimestamp();
    watermarkDelta            = currentTimestamp - lastWatermarkTick;
    lastWatermarkTick         = currentTimestamp;

    // Pass timestamp to the sensor
    if (gyro != nullptr)
    {
        gyro->IRQupdateTimestamp(currentTimestamp);
    }
}

void configure()
{
    GpioSck::mode(Mode::ALTERNATE);
    GpioMiso::mode(Mode::ALTERNATE);
    GpioMosi::mode(Mode::ALTERNATE);

    GpioSck::alternateFunction(5);
    GpioMiso::alternateFunction(5);
    GpioMosi::alternateFunction(5);

    // Interrupt
    GpioINT2::mode(Mode::INPUT_PULL_DOWN);

    cs.mode(Mode::OUTPUT);

    cs.high();

    enableExternalInterrupt(GPIOA_BASE, 2, InterruptTrigger::RISING_EDGE);
}

int main()
{
    configure();

    // Setup sensor
    gyro = new L3GD20(bus, cs, L3GD20::FullScaleRange::FS_250,
                      L3GD20::OutPutDataRate::ODR_760, 0x03);

    // Enable fifo with the specified watermark before calling init()
    gyro->enableFifo(FIFO_WATERMARK);

    // Init the gyro
    while (!gyro->init())
    {
        printf("Gyro initialization failure!\n");
        Thread::sleep(100);
    }

    // Sample NUM_SAMPLES data
    int fifoNum = 0;
    while (dataCounter < NUM_SAMPLES)
    {
        long lastTick = miosix::getTick();

        // Read the fifo
        uint64_t update = TimestampTimer::getInstance().getTimestamp();
        gyro->sample();

        // Measure how long we take to read the fifo
        update = TimestampTimer::getInstance().getTimestamp() - update;

        uint8_t level =
            gyro->getLastFifoSize();  // Current number of samples in the FIFO

        // Obtain the FIFO
        const array<L3GD20Data, L3GD20_FIFO_SIZE>& fifo = gyro->getLastFifo();

        // Store everything in the data buffer
        for (int i = 0; i < level; i++)
        {
            // data[dataCounter++] = fifo[i];
            data[dataCounter++] = {
                fifoNum,
                fifo[i],
                level,
                TimerUtils::toIntMicroSeconds(
                    TimestampTimer::getInstance().getTimer(), watermarkDelta),
                averageCpuUtilization(),
                TimerUtils::toIntMicroSeconds(
                    TimestampTimer::getInstance().getTimer(), update)};

            // Stop if we have enough data
            if (dataCounter >= NUM_SAMPLES)
            {
                break;
            }
        }
        ++fifoNum;

        // Wait until fifo has about 25 samples. The fifo size is 32 samples, so
        // we have 7 samples (~ 9 ms) of wiggle room before we start losing
        // data, in case we sleep a bit too much (may happen if an higher
        // priority thread has a long task to perform)
        Thread::sleepUntil(lastTick + 25.5 * 1000 / SAMPLE_FREQUENCY);
    }

    // Dump buffer content as CSV on the serial (might take a while)
    // printf("t,x,y,z\n");
    printf("FIFO_num,timestamp,int_delta,read_time,sample_delta,x,y,z,cpu\n");

    for (int i = 1; i < dataCounter; i++)
    {
        // clang-format off
        printf("%d,%llu,%llu,%llu,%llu,%f,%f,%f,%.2f\n",
                data[i].fifoNum,
                data[i].gyro.angularVelocityTimestamp,
                data[i].wtmDelta,
                data[i].update,
                (data[i].gyro.angularVelocityTimestamp - data[i - 1].gyro.angularVelocityTimestamp),
                data[i].gyro.angularVelocityX,
                data[i].gyro.angularVelocityY,
                data[i].gyro.angularVelocityZ,
                data[i].cpu);
        // clang-format on
    }

    printf("\n\n\nend.\n");
    for (;;)
    {
        Thread::sleep(1000);
    }
}
