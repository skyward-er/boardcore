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

#include "drivers/HardwareTimer.h"
#include "drivers/spi/SPIDriver.h"
#include "logger/Logger.h"
#include "sensors/LSM9DS1/LSM9DS1_AxelGyro.h"
#include "sensors/LSM9DS1/LSM9DS1_Magneto.h"

using namespace miosix;
using namespace std;

// SPI1 f407
typedef Gpio<GPIOA_BASE, 5> GpioSck;
typedef Gpio<GPIOA_BASE, 6> GpioMiso;
typedef Gpio<GPIOA_BASE, 7> GpioMosi;

// INT1 A/G LSM9DS1
typedef Gpio<GPIOC_BASE, 13> GpioINT1;

// SPI bus & cs
SPIBus bus(SPI1);
GpioPin cs_XLG(GPIOE_BASE, 7);
GpioPin cs_M(GPIOE_BASE, 9);

// LED just for visual reference
GpioPin LED1(GPIOD_BASE, 15);
GpioPin LED2(GPIOD_BASE, 13);
GpioPin LED3(GPIOD_BASE, 14);

// USR pushbutton
GpioPin PUSHBUTTON(GPIOA_BASE, 0);

// SPI read flag
volatile bool flagSPIReadRequest = false;

// IMU obj variables
static const uint8_t FIFO_WATERMARK       = 12;
static const uint16_t FIFO_SAMPLES        = 1000;
static const uint8_t TEMP_DIV_FREQ        = 20;
static const uint16_t MAG_SAMPLING_PERIOD = 10;  // 100Hz
static const uint16_t FIFO_SAMPLING_PERIOD = 25.5 * 1000 / 476.0f;

// High Resolution hardware timer using TIM5
HardwareTimer<uint32_t> hrclock(
    TIM5, TimerUtils::getPrescalerInputFrequency(TimerUtils::InputClock::APB1));

// Last interrupt tick & delta
volatile uint32_t last_tick;
volatile uint32_t delta;

// LSM9DS1 obj
LSM9DS1_XLG* lsm9ds1_xlg = nullptr;
LSM9DS1_M* lsm9ds1_m     = nullptr;

// Interrupt handlers
void __attribute__((naked)) EXTI13_IRQHandler()
{
    saveContext();
    asm volatile("bl _Z20EXTI13_IRQHandlerImplv");
    restoreContext();
}

void __attribute__((used)) EXTI13_IRQHandlerImpl()
{
    // Update timestamp
    lsm9ds1_xlg->updateTimestamp(hrclock.toMicroSeconds(hrclock.tick()));

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
void printStats(void*);

int main()
{

    uint64_t lastMagtick   = 0;
    uint64_t lastFifotick  = 0;
    uint16_t lastTempcount = 0;

    // Spawn thread for loggings logger stats
    Thread::create(printStats, 4096);

    // start logger
    Logger& logger = Logger::instance();
    logger.start();

    gpioConfig();
    timer5Config();
    EXTI1Config();

    lsm9ds1_xlg = new LSM9DS1_XLG(bus, cs_XLG, LSM9DS1_XLG::AxelFSR::FS_8,
                                  LSM9DS1_XLG::GyroFSR::FS_245,
                                  LSM9DS1_XLG::ODR::ODR_952, TEMP_DIV_FREQ);

    lsm9ds1_m = new LSM9DS1_M(bus, cs_M, LSM9DS1_M::MagFSR::FS_8,
                              LSM9DS1_M::ODR::ODR_80);

    // enable FIFO
    lsm9ds1_xlg->enable_fifo(FIFO_WATERMARK);

    // perform self-tests
    lsm9ds1_xlg->selfTest();
    lsm9ds1_m->selfTest();

    // initialize sensor
    while (!lsm9ds1_xlg->init())
        ;
    while (!lsm9ds1_m->init())
        ;

    // sampling until you push the button
    while (!PUSHBUTTON.value())
    {
        // ACCELEROMETER + GYROSCOPE + UPDATE (FIFO)
        // TEMPERATURE UPDATE
        // an interrupt is set: time to dump the FIFO
        
        //if (miosix::getTick() - lastFifotick >= FIFO_SAMPLING_PERIOD)
        if (flagSPIReadRequest)
        {
            flagSPIReadRequest = false;
            //lastFifotick = miosix::getTick();
            // dump the fifo
            lsm9ds1_xlg->onSimpleUpdate();

            // log each sample
            for (int i = 0; i < lsm9ds1_xlg->getFIFOdepth() ; i++)
            {
                lsm9ds1XLGSample XLGsample = lsm9ds1_xlg->getFIFO()[i];
                LED2.high();
                logger.log(XLGsample);
                LED2.low();
            }
                //lsm9ds1debug debugstats = lsm9ds1_xlg->getFIFOStats();
                //logger.log(debugstats);

            if (lastTempcount == TEMP_DIV_FREQ)
            {
                lastTempcount          = 0;
                lsm9ds1TSample Tsample = lsm9ds1_xlg->getTSample();
                //logger.log(Tsample);
            }
            lastTempcount++;

            LED1.low();
        }

        // MAGNETOMETER UPDATE (SIMPLE)
        if (miosix::getTick() - lastMagtick >= MAG_SAMPLING_PERIOD)
        {
            lastMagtick = miosix::getTick();

            // get sample from the sensor
            lsm9ds1_m->onSimpleUpdate();

            // log the sample
            lsm9ds1MSample MAGsample = lsm9ds1_m->getSample();
            //logger.log(MAGsample);
        }

        //Thread::sleep(FIFO_SAMPLING_PERIOD);

    }

    // stop log
    logger.stop();

    LED1.low();
    LED2.low();

    Thread::sleep(10000);
    miosix::reboot();

    while (1)
        ;

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
        cs_M.mode(Mode::OUTPUT);

        // Select GPIO mode for INTERRUPT (PULL-DOWN because there's no
        // pull-down in HW)
        GpioINT1::mode(Mode::INPUT_PULL_DOWN);

        // Select LED built in GPIO mode
        LED1.mode(Mode::OUTPUT);
        LED2.mode(Mode::OUTPUT);
        LED3.mode(Mode::OUTPUT);

        // Select USR pushbutton
        PUSHBUTTON.mode(Mode::INPUT);
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
    hrclock.setPrescaler(382);
    hrclock.start();
}

void EXTI1Config()  // PC13
{
    // Enable SYSCFG for setting interrupts
    {
        FastInterruptDisableLock dl;
        RCC->APB2ENR |= RCC_APB2ENR_SYSCFGEN;
    }

    // Configure mask bit of 1st interrupt line
    EXTI->IMR |= EXTI_IMR_MR13;

    // Configure trigger selection bit (rising edge)
    EXTI->RTSR |= EXTI_RTSR_TR13;

    // Clear pending interrupt register before enable
    EXTI->PR |= EXTI_PR_PR13;

    // Select PC13 as interrupt source in line 13 (EXTICR4)
    SYSCFG->EXTICR[3] &= 0xFFFFFF0F;
    SYSCFG->EXTICR[3] |= 0xFFFFFF2F;

    // Enable the interrupt in the interrupt controller
    NVIC_EnableIRQ(IRQn_Type::EXTI15_10_IRQn);
    NVIC_SetPriority(IRQn_Type::EXTI15_10_IRQn, 47);
}

void printStats(void*)
{
    Logger& log = Logger::instance();
    while (!log.isStarted())
        ;

    while (log.isStarted())
    {
        LogStats stats = log.getLogStats();

        stats.setTimestamp(miosix::getTick());
        log.log(stats);

        if (stats.statWriteError)
        {
            LED3.high();
        }
        else
        {
            LED3.low();
        }

        Thread::sleep(1000);
    }
}