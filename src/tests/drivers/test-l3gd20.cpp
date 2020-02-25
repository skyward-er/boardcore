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

#include <array>

#include "diagnostic/CpuMeter.h"
#include "drivers/HardwareTimer.h"
#include "drivers/spi/SPIDriver.h"
#include "sensors/L3GD20.h"

using namespace miosix;
using std::array;

typedef Gpio<GPIOF_BASE, 7> GpioSck;
typedef Gpio<GPIOF_BASE, 8> GpioMiso;
typedef Gpio<GPIOF_BASE, 9> GpioMosi;

typedef Gpio<GPIOA_BASE, 2> GpioINT2;

static constexpr bool FIFO_ENABLED           = true;
static constexpr unsigned int FIFO_WATERMARK = 24;
// Expected frequency from the datasheet is 760 Hz, but due to clock
// misalignment / temperature errors and other factors, the observed clock (and
// output data rate) is the following:
static constexpr float SAMPLE_FREQUENCY = 782.3f;
static constexpr int NUM_SAMPLES        = 10000;

void enableInterrupt();

struct GyroSample
{
    int fifo_num;
    float timestamp;
    Vec3 data;
    int level;
    float cpu;
};

// 364 KB buffer to store up to 30 seconds of data @ 760 Hz
GyroSample data[22800];
int data_counter = 0;

// High resolution hardware timer using TIM5
HardwareTimer<uint32_t> hrclock{
    TIM5, TimerUtils::getPrescalerInputFrequency(TimerUtils::InputClock::APB1)};

// Last interrupt received timer tick
volatile uint32_t last_watermark_tick;  // Stores the high-res tick of the last
                                        // interrupt (L3GD20 watermark event)
volatile uint32_t watermark_delta;  // Tick delta between the last 2 watermark
                                    // events

// L3GD20 SPI
SPIBus bus(SPI5);
GpioPin cs(GPIOC_BASE, 1);
SPIBusConfig cfg;
uint8_t buf[192];

// Interrupt handlers
void __attribute__((naked)) EXTI2_IRQHandler()
{
    saveContext();
    asm volatile("bl _Z20EXTI2_IRQHandlerImplv");
    restoreContext();
}

void __attribute__((used)) EXTI2_IRQHandlerImpl()
{
    uint32_t tick       = hrclock.tick();
    watermark_delta     = tick - last_watermark_tick;
    last_watermark_tick = tick;

    EXTI->PR |= EXTI_PR_PR2;  // Reset pending register
}

int main()
{
    cfg.br = SPIBaudRate::DIV_64;

    {
        FastInterruptDisableLock dLock;

        RCC->APB1ENR |= RCC_APB1ENR_TIM5EN;
        RCC->APB2ENR |= RCC_APB2ENR_SPI5EN;

        GpioSck::mode(Mode::ALTERNATE);
        GpioMiso::mode(Mode::ALTERNATE);
        GpioMosi::mode(Mode::ALTERNATE);

        GpioSck::alternateFunction(5);
        GpioMiso::alternateFunction(5);
        GpioMosi::alternateFunction(5);

        // Interrupt
        GpioINT2::mode(Mode::INPUT_PULL_DOWN);

        cs.mode(Mode::OUTPUT);
    }
    // High resolution clock configuration
    // 1.8 hours run time, 1.5 us resolution
    hrclock.setPrescaler(127);
    hrclock.start();

    enableInterrupt();
    cs.high();

    L3GD20 gyro(bus, cs, L3GD20::FullScaleRange::FS_250,
                L3GD20::OutPutDataRate::ODR_760, 0x03, FIFO_ENABLED,
                FIFO_WATERMARK);

    while (!gyro.init())
    {
    }

    Thread::sleep(500);

    long long first_tick = miosix::getTick();

    // Sample NUM_SAMPLES data
    int fifo_num = 0;
    while (data_counter < NUM_SAMPLES)
    {
        long last_tick = miosix::getTick();

        if (FIFO_ENABLED)
        {
            // Precise timestamp of the last sample in the FIFO
            float wtm_timestamp = hrclock.toSeconds(last_watermark_tick);

            // Read the fifo
            gyro.onSimpleUpdate();

            uint8_t level =
                gyro.getLastFifoSize();  // Number of samples in the FIFO

            const array<Vec3, 32>& fifo = gyro.getLastFifo();

            for (int i = 0; i < level; i++)
            {
                // Assign a timestamp to each sample in the FIFO
                // Samples before the watermark are older, after the watermark
                // are younger. Time delta between samples is
                // (1 / SAMPLE_FREQUENCY) seconds
                float ts = wtm_timestamp +
                           (i - (int)FIFO_WATERMARK) / SAMPLE_FREQUENCY;

                data[data_counter++] = {fifo_num, ts, fifo[i], level,
                                        averageCpuUtilization()};
                if (data_counter >= NUM_SAMPLES)
                {
                    break;
                }
            }
            ++fifo_num;

            // Wait until fifo has about 25 samples
            Thread::sleepUntil(last_tick + 34);
        }
        else
        {
            // Sample sensor @ 500 Hz
            gyro.onSimpleUpdate();

            data[data_counter++] = {0, (last_tick - first_tick) / 1000.0f,
                                    *(gyro.gyroDataPtr()), 0,
                                    averageCpuUtilization()};
            Thread::sleepUntil(last_tick + 2);
        }
    }
    // Dump buffer content as CSV on the serial (might take a while)
    printf("FIFO_num,timestamp,watermark_delta,sample_delta,x,y,z,level,cpu");
    for (int i = 1; i < data_counter; i++)
    {
        printf("%d,%.3f,%.3f,%.3f,%f,%f,%f,%d,%.2f\n", data[i].fifo_num,
               data[i].timestamp * 1000,
               hrclock.toMilliSeconds(watermark_delta),
               (data[i].timestamp - data[i - 1].timestamp) * 1000,
               data[i].data.getX(), data[i].data.getY(), data[i].data.getZ(),
               data[i].level, data[i].cpu);
    }

    printf("\n\n\nend.\n");
    for (;;)
    {
        Thread::sleep(1000);
    }
}

void enableInterrupt()
{
    {
        FastInterruptDisableLock l;
        RCC->APB2ENR |= RCC_APB2ENR_SYSCFGEN;
    }
    // Refer to the datasheet for a detailed description on the procedure and
    // interrupt registers

    // Clear the mask on the wanted line
    EXTI->IMR |= EXTI_IMR_MR2;

    // Trigger the interrupt on a falling edge
    // EXTI->FTSR |= EXTI_FTSR_TR2;

    // Trigger the interrupt on a rising edge
    EXTI->RTSR |= EXTI_RTSR_TR2;

    EXTI->PR |= EXTI_PR_PR2;  // Reset pending register

    // Enable interrupt on PA2 in SYSCFG
    SYSCFG->EXTICR[0] &= 0xFFFFF0FF;

    // // Enable the interrput in the interrupt controller
    NVIC_EnableIRQ(EXTI2_IRQn);
    NVIC_SetPriority(EXTI2_IRQn, 15);
}