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
#include "sensors/LSM9DS1/LSM9DS1_AxelGyro.h"
#include "sensors/LSM9DS1/LSM9DS1_Magneto.h"
#include "diagnostic/CpuMeter.h"
#include "logger/Logger.h"

using namespace miosix;
using namespace std;

typedef Gpio<GPIOA_BASE, 5> GpioSck;  //SPI1 f407
typedef Gpio<GPIOA_BASE, 6> GpioMiso;
typedef Gpio<GPIOA_BASE, 7> GpioMosi;

typedef Gpio<GPIOC_BASE, 13> GpioINT1; //INT1 A/G

// SPI
SPIBus bus(SPI1);
GpioPin cs_XLG(GPIOE_BASE, 7);
GpioPin cs_M(GPIOE_BASE, 9);

// LED just for init
GpioPin LED1(GPIOD_BASE, 15);
GpioPin LED2(GPIOD_BASE, 13);
GpioPin LED3(GPIOD_BASE, 14);

//USR pushbutton
GpioPin PUSHBUTTON(GPIOA_BASE, 0);


// SPI read flag
volatile bool flagSPIReadRequest = false;

//IMU obj data
static const bool FIFO_ENABLED      = true;
static const uint8_t FIFO_WATERMARK = 20;
static const uint16_t FIFO_SAMPLES   = 1000;
static const uint16_t MAG_SAMPLING_PERIOD = 10; //100Hz
static const uint16_t TEMP_SAMPLING_PERIOD = 100; //10Hz 

// High Resolution hardware timer using TIM5
HardwareTimer<uint32_t> hrclock(
    TIM5, TimerUtils::getPrescalerInputFrequency(TimerUtils::InputClock::APB1));

// Last interrupt tick & delta
volatile uint32_t last_tick;
volatile uint32_t delta;

// LSM9DS1 obj
LSM9DS1_XLG* lsm9ds1_xlg = nullptr;
LSM9DS1_M*   lsm9ds1_m   = nullptr; 

// Interrupt handlers
void __attribute__((naked)) EXTI13_IRQHandler()
{
    saveContext();
    asm volatile("bl _Z20EXTI13_IRQHandlerImplv");
    restoreContext();
}

void __attribute__((used)) EXTI13_IRQHandlerImpl()
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
void printStats(void*);

uint16_t fifo_counter = 0;

int main()
{

    uint32_t dt;
    uint64_t XLGtimestamp = 0;
    uint64_t lastMagtick = 0;
    uint64_t lastTemptick = 0;

    Thread::create(printStats,4096);

    Logger& logger = Logger::instance();
    logger.start();

    gpioConfig();
    timer5Config();
    EXTI1Config();

    lsm9ds1_xlg = new LSM9DS1_XLG(bus, cs_XLG, LSM9DS1_XLG::AxelFSR::FS_8, 
                            LSM9DS1_XLG::GyroFSR::FS_245,LSM9DS1_XLG::ODR::ODR_238, 
                            FIFO_ENABLED, FIFO_WATERMARK);

    lsm9ds1_m = new LSM9DS1_M(bus,cs_M, LSM9DS1_M::MagFSR::FS_8,LSM9DS1_M::ODR::ODR_40);

    while (!lsm9ds1_xlg->init());
    while (!lsm9ds1_m->init());
    LED2.high(); //init OK
    
    lsm9ds1_xlg->clearFIFO();  //just to be sure to intercept the first interrupt

    //start sampling
    while(!PUSHBUTTON.value())
    {

        if(flagSPIReadRequest)
        {
            flagSPIReadRequest = false;
            dt = hrclock.toMicroSeconds(delta)/FIFO_WATERMARK; //delta of each sample
            lsm9ds1_xlg->onSimpleUpdate();
            for(int i=0 ; i < FIFO_WATERMARK; i++)
            {
                lsm9ds1XLGSample XLGsample = lsm9ds1_xlg->getLsm9ds1FIFO()[i];
                XLGtimestamp += dt; 
                XLGsample.timestamp = XLGtimestamp; 
                logger.log(XLGsample);
            }
            LED1.low();
            fifo_counter++;
        }

        if(miosix::getTick() - lastMagtick >= MAG_SAMPLING_PERIOD)
        {
            lastMagtick = miosix::getTick();
            lsm9ds1_m->onSimpleUpdate();
            lsm9ds1MSample MAGsample;
            MAGsample.magData = *(lsm9ds1_m->compassDataPtr());
            MAGsample.timestamp = lastMagtick;
            logger.log(MAGsample);
        }

        if(miosix::getTick() - lastTemptick >= TEMP_SAMPLING_PERIOD)
        {
            lastTemptick = miosix::getTick();
            lsm9ds1_xlg->temperatureUpdate();
            lsm9ds1TSample Tsample;
            Tsample.tempData = *(lsm9ds1_xlg->tempDataPtr());
            Tsample.timestamp = lastTemptick;
            logger.log(Tsample);
        }

    }

    logger.stop();

    LED1.low();
    LED2.low();

    Thread::sleep(10000);
    reboot();

    while(1);
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

        //Select USR pushbutton 
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
    Logger& log=Logger::instance();
    while(!log.isStarted());
    while(log.isStarted())
    {
        LogStats stats = log.getLogStats();

        stats.setTimestamp(miosix::getTick());
        log.log(stats);

        if(stats.statWriteError)
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