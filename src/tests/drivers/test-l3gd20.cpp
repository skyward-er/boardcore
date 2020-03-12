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

static constexpr int NUM_SAMPLES = 500 * 5;  // 120 seconds of data

void enableInterrupt();
void configure();

struct GyroSample
{
    uint64_t timestamp;
    uint64_t sample_delta;
    Vec3 data;
    float cpu;
};

GyroSample data[NUM_SAMPLES];
int data_counter = 0;

// High resolution hardware timer using TIM5
HardwareTimer<uint32_t> hrclock{
    TIM5, TimerUtils::getPrescalerInputFrequency(TimerUtils::InputClock::APB1)};

// Last interrupt received timer tick
volatile uint32_t last_sample_tick;  // Stores the high-res tick of the last
                                     // interrupt (L3GD20 watermark event)
volatile uint32_t sample_delta;      // Tick delta between the last 2 watermark
                                     // events

// L3GD20 SPI
SPIBus bus(SPI5);
GpioPin cs(GPIOC_BASE, 1);

L3GD20* gyro;

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
    uint32_t tick    = hrclock.tick();
    sample_delta     = tick - last_sample_tick;
    last_sample_tick = tick;

    // Pass tick microseconds to sensor for timestamp estimation
    if (gyro != nullptr)
        gyro->IRQupdateTimestamp(hrclock.toIntMicroSeconds(tick));

    EXTI->PR |= EXTI_PR_PR2;  // Reset pending register
}

int main()
{
    Thread::sleep(5000);

    configure();

    gyro = new L3GD20(bus, cs, L3GD20::FullScaleRange::FS_250,
                      L3GD20::OutPutDataRate::ODR_760, 0x03);

    // gyro->setRealSampleInterval(
    //     static_cast<uint64_t>(1000000 / SAMPLE_FREQUENCY));

    while (!gyro->init())
    {
    }

    Thread::sleep(500);

    while (data_counter < NUM_SAMPLES)
    {
        long last_tick = miosix::getTick();

        // Sample sensor @ 500 Hz
        gyro->onSimpleUpdate();
        L3GD20Data d = gyro->getLastSample();

        data[data_counter++] = {d.timestamp, sample_delta, d.gyro,
                                averageCpuUtilization()};

        Thread::sleepUntil(last_tick + 1);
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
                data[i].data.getX(), 
                data[i].data.getY(), 
                data[i].data.getZ(),
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
    // 1.8 hours run time, 1.5 us resolution
    hrclock.setPrescaler(127);
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