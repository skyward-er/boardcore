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

static constexpr unsigned int FIFO_WATERMARK = 12;

// Expected frequency from the datasheet is 760 Hz, but due to clock
// misalignment / temperature errors and other factors, the observed clock (and
// output data rate) is the following:
static constexpr float SAMPLE_FREQUENCY = 782.3f;
static constexpr int NUM_SAMPLES        = SAMPLE_FREQUENCY * 120;

// FD
void enableInterrupt();
void configure();

struct GyroSample
{
    int fifo_num;
    L3GD20Data gyro;
    int level;
    uint64_t wtm_delta;
    float cpu;
    uint64_t update;
};

// GyroSample data[NUM_SAMPLES];
GyroSample data[NUM_SAMPLES];
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

L3GD20* gyro = nullptr;

// Interrupt handlers
void __attribute__((naked)) EXTI2_IRQHandler()
{
    saveContext();
    asm volatile("bl _Z20EXTI2_IRQHandlerImplv");
    restoreContext();
}

void __attribute__((used)) EXTI2_IRQHandlerImpl()
{
    // Current high resolution tick
    uint32_t tick       = hrclock.tick();
    watermark_delta     = tick - last_watermark_tick;
    last_watermark_tick = tick;
    // Pass tick microseconds to sensor
    if(gyro != nullptr)
        gyro->IRQupdateTimestamp(hrclock.toIntMicroSeconds(tick));

    EXTI->PR |= EXTI_PR_PR2;  // Reset pending register
}

int main()
{
    Thread::sleep(5000);

    configure();

    // Setup sensor
    gyro = new L3GD20(bus, cs, L3GD20::FullScaleRange::FS_250,
                      L3GD20::OutPutDataRate::ODR_760, 0x03);

    gyro->enableFifo(FIFO_WATERMARK);

    // Init
    while (!gyro->init())
    {
    }

    // Sample NUM_SAMPLES data
    int fifo_num = 0;
    while (data_counter < NUM_SAMPLES)
    {
        long last_tick = miosix::getTick();

        // Read the fifo
        uint32_t update = hrclock.tick();
        gyro->onSimpleUpdate();
        update = hrclock.tick() - update;

        uint8_t level =
            gyro->getLastFifoSize();  // Number of samples in the FIFO

        // Get the FIFO
        const array<L3GD20Data, 32>& fifo = gyro->getLastFifo();

        // Store everything in the data buffer
        for (int i = 0; i < level; i++)
        {
            // data[data_counter++] = fifo[i];
            data[data_counter++] = {fifo_num,
                                    fifo[i],
                                    level,
                                    hrclock.toIntMicroSeconds(watermark_delta),
                                    averageCpuUtilization(), 
                                    hrclock.toIntMicroSeconds(update)};

            // Stop if we have enough data
            if (data_counter >= NUM_SAMPLES)
            {
                break;
            }
        }
        ++fifo_num;

        // Wait until fifo has about 25 samples
        Thread::sleepUntil(last_tick + 25.5 * 1000 / SAMPLE_FREQUENCY);
    }

    // Dump buffer content as CSV on the serial (might take a while)
    // printf("t,x,y,z\n");
    printf("FIFO_num,timestamp,int_delta,read_time,sample_delta,x,y,z,cpu\n");

    for (int i = 1; i < data_counter; i++)
    {
        // clang-format off
        printf("%d,%llu,%llu,%llu,%llu,%f,%f,%f,%.2f\n", 
                data[i].fifo_num,
                data[i].gyro.timestamp, 
                data[i].wtm_delta,
                data[i].update,
                (data[i].gyro.timestamp - data[i - 1].gyro.timestamp),
                data[i].gyro.gyro.getX(), 
                data[i].gyro.gyro.getY(), 
                data[i].gyro.gyro.getZ(),
                data[i].cpu);
        // clang-format on
    }

    printf("\n\n\nend.\n");
    for (;;)
    {
        Thread::sleep(1000);
    }
}

void configure()
{
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

    cs.high();

    enableInterrupt();

    // High resolution clock configuration
    hrclock.setPrescaler(382);
    hrclock.start();
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