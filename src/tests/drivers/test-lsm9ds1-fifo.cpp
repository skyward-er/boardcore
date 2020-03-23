/**
 * test LSM9DS1 axel + gyro
 * Copyright (c) 2020 Skyward Experimental Rocketry
 * Authors: Andrea Milluzzo
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
#include <iostream>
#include "drivers/HardwareTimer.h"
#include "drivers/spi/SPIDriver.h"
#include "sensors/LSM9DS1/LSM9DS1_AxelGyro.h"

using namespace miosix;
using namespace std;

typedef Gpio<GPIOA_BASE, 5> GpioSck;  // questi sono i pin SPI per
                                      // f407_discovery
typedef Gpio<GPIOA_BASE, 6> GpioMiso;
typedef Gpio<GPIOA_BASE, 7> GpioMosi;

typedef Gpio<GPIOA_BASE, 1> GpioINT1;

static const bool FIFO_ENABLED      = true;
static const uint8_t FIFO_WATERMARK = 12;
static const uint8_t FIFO_SAMPLES   = 5;

// SPI
SPIBus bus(SPI1);
GpioPin cs_XLG(GPIOE_BASE, 7);

// LED just for init
GpioPin LED1(GPIOD_BASE, 15);

// SPI read flag
volatile bool flagSPIReadRequest = false;

// High Resolution hardware timer using TIM5
HardwareTimer<uint32_t> hrclock{
    TIM5, TimerUtils::getPrescalerInputFrequency(TimerUtils::InputClock::APB1)};

// Last interrupt tick & delta
volatile uint32_t last_tick;
volatile uint32_t delta;

//LSM9DS1 obj
LSM9DS1_XLG* lsm9ds1 = nullptr; 

// Interrupt handlers
void __attribute__((naked)) EXTI1_IRQHandler()
{
    saveContext();
    asm volatile("bl _Z20EXTI1_IRQHandlerImplv");
    restoreContext();
}

void __attribute__((used)) EXTI1_IRQHandlerImpl()
{
    // Computing delta beetween interrupts
    uint32_t tick = hrclock.tick();
    delta         = tick - last_tick;
    last_tick     = tick;

    // Set read flag
    flagSPIReadRequest = true;

    // Built-in LED on
    LED1.high();

    // Clear pending interrupt register
    EXTI->PR |= EXTI_PR_PR1;
}

void gpioConfig();
void timer5Config();
void EXTI1Config();

int main()
{

    uint8_t fifo_counter = 0;
    uint32_t dt[FIFO_SAMPLES];
    //array<Vec3, 32> axelData[FIFO_SAMPLES], gyroData[FIFO_SAMPLES];

    gpioConfig();

    
    Thread::sleep(4000);
    LED1.low();

    timer5Config();
    EXTI1Config();

    std::cout << "Before allocation" << std::endl; //FA ANCORA STACK OVERFLOW DOPO QUI GRRRR
    
    lsm9ds1 = new LSM9DS1_XLG(bus, cs_XLG, LSM9DS1_XLG::AxelFSR::FS_8,
                        LSM9DS1_XLG::GyroFSR::FS_245, LSM9DS1_XLG::ODR::ODR_15,
                        FIFO_ENABLED, FIFO_WATERMARK);

    std::cout << "After allocation" << std::endl;

    while (!lsm9ds1->init());

    lsm9ds1->clearFIFO();

    for (;;)
    {
        if (flagSPIReadRequest && fifo_counter < FIFO_SAMPLES)
        {
            flagSPIReadRequest = false;
            dt[fifo_counter]   = delta;
            lsm9ds1->onSimpleUpdate();
            //axelData[fifo_counter] = lsm9ds1->getAxelFIFO();
            //gyroData[fifo_counter] = lsm9ds1->getGyroFIFO();
            LED1.low();
            fifo_counter++;
        }

        if (fifo_counter == FIFO_SAMPLES)
        {
            break;
        }
    }

    for (uint8_t i = 0; i < FIFO_SAMPLES; i++)
    {
        std::cout << dt[i] << std::endl;
    }

    return 0;
}

void gpioConfig()
{
    {
        FastInterruptDisableLock dLock;

        // Enable SPI1 Peripheral
        RCC->APB2ENR |= RCC_APB2ENR_SPI1EN;

        // Select GPIO mode (ALTERNATE 5 for SPI1 on PA5, PA6, PA7)
        GpioSck::mode(Mode::ALTERNATE);
        GpioMiso::mode(Mode::ALTERNATE);
        GpioMosi::mode(Mode::ALTERNATE);

        GpioSck::alternateFunction(5);
        GpioMiso::alternateFunction(5);
        GpioMosi::alternateFunction(5);

        // Select Speed for clk (needs precision)
        GpioSck::speed(Speed::_25MHz);

        // Select GPIO mode for Chip Select
        cs_XLG.mode(Mode::OUTPUT);

        // Select GPIO mode for INTERRUPT (PULL-DOWN because there's no
        // pull-down in HW)
        GpioINT1::mode(Mode::INPUT_PULL_DOWN);

        // Select LED built in GPIO mode
        LED1.mode(Mode::OUTPUT);
    }

    cs_XLG.high();
}

void timer5Config()
{
    {
        FastInterruptDisableLock dl;
        // Enable high resolution TIM5
        RCC->APB1ENR |= RCC_APB1ENR_TIM5EN;
    }
}

void EXTI1Config() //PC13
{
    // Enable SYSCFG for setting interrupts
    {
        FastInterruptDisableLock dl;
        RCC->APB2ENR |= RCC_APB2ENR_SYSCFGEN;
    }

    // Configure mask bit of 1st interrupt line
    EXTI->IMR |= EXTI_IMR_MR1;

    // Configure trigger selection bit (rising edge)
    EXTI->RTSR |= EXTI_RTSR_TR1;

    // Clear pending interrupt register before enable
    EXTI->PR |= EXTI_PR_PR1;

    // Select PB1 as interrupt source in line 1
    SYSCFG->EXTICR[0] &= 0xFFFFFF0F;

    // Enable the interrupt in the interrupt controller
    NVIC_EnableIRQ(EXTI1_IRQn);
    NVIC_SetPriority(EXTI1_IRQn, 14);
}