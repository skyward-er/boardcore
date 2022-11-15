/* Copyright (c) 2022 Skyward Experimental Rocketry
 * Author: Emilio Corigliano
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

#include "I2C.h"

#include <kernel/scheduler/scheduler.h>
#include <utils/ClockUtils.h>

// MACRO in order to avoid repeated pattern: This waits until the thread isn't
// waken up by an I2C interrupt (EV or ERR). This MACRO shuould be called in a
// block where interrupts are disabled; this handles the waiting and yielding
// and the management of the flags for the interrupts.
// [WARNING] If the flag
// identified in REG at the end is false or the error interrupt has been invoked
// the macro makes the OUTER (!) function return with the value passed in RETVAL
#define WAIT_FOR_REGISTER_CHANGE(REG, DLOCK, RETVAL)   \
    waiting = miosix::Thread::getCurrentThread();      \
    while (waiting)                                    \
    {                                                  \
        waiting->IRQwait();                            \
        i2c->CR2 |= I2C_CR2_ITEVTEN | I2C_CR2_ITERREN; \
        miosix::InterruptEnableLock eLock(DLOCK);      \
        waiting->yield();                              \
    }                                                  \
                                                       \
    if (error || !(REG))                               \
    {                                                  \
        error = false;                                 \
        i2c->CR1 |= I2C_CR1_STOP;                      \
        return RETVAL;                                 \
    }

Boardcore::I2C *Boardcore::I2C::ports[N_I2C_PORTS];

inline void wakeUpWaitingThread(miosix::Thread *waiting)
{
    if (waiting)
    {
        waiting->IRQwakeup();
        if (waiting->IRQgetPriority() >
            miosix::Thread::IRQgetCurrentThread()->IRQgetPriority())
        {
            miosix::Scheduler::IRQfindNextThread();
        }
    }
}

//** I2C1 **//
/**
 * I2C address sent interrupt
 */
void __attribute__((naked)) I2C1_EV_IRQHandler()
{
    saveContext();
    asm volatile("bl _Z15I2C1HandlerImplv");
    restoreContext();
}

/**
 * I2C address sent interrupt actual implementation
 */
void __attribute__((used)) I2C1HandlerImpl()
{
    Boardcore::I2C *port = Boardcore::I2C::ports[0];
    if (port)
        port->IRQhandleInterrupt();
}

/**
 * I2C error interrupt
 */
void __attribute__((naked)) I2C1_ER_IRQHandler()
{
    saveContext();
    asm volatile("bl _Z18I2C1errHandlerImplv");
    restoreContext();
}

/**
 * I2C error interrupt actual implementation
 */
void __attribute__((used)) I2C1errHandlerImpl()
{
    Boardcore::I2C *port = Boardcore::I2C::ports[0];
    if (port)
        port->IRQhandleErrInterrupt();
}

#if defined(STM32F429xx) || defined(STM32F407xx) || defined(STM32F746xx) || \
    defined(STM32F767xx)
//** I2C2 **//
/**
 * I2C address sent interrupt
 */
void __attribute__((naked)) I2C2_EV_IRQHandler()
{
    saveContext();
    asm volatile("bl _Z15I2C2HandlerImplv");
    restoreContext();
}

/**
 * I2C address sent interrupt actual implementation
 */
void __attribute__((used)) I2C2HandlerImpl()
{
    Boardcore::I2C *port = Boardcore::I2C::ports[1];
    if (port)
        port->IRQhandleInterrupt();
}

/**
 * I2C error interrupt
 */
void __attribute__((naked)) I2C2_ER_IRQHandler()
{
    saveContext();
    asm volatile("bl _Z18I2C2errHandlerImplv");
    restoreContext();
}

/**
 * I2C error interrupt actual implementation
 */
void __attribute__((used)) I2C2errHandlerImpl()
{
    Boardcore::I2C *port = Boardcore::I2C::ports[1];
    if (port)
        port->IRQhandleErrInterrupt();
}

//** I2C3 **//
/**
 * I2C address sent interrupt
 */
void __attribute__((naked)) I2C3_EV_IRQHandler()
{
    saveContext();
    asm volatile("bl _Z15I2C3HandlerImplv");
    restoreContext();
}

/**
 * I2C address sent interrupt actual implementation
 */
void __attribute__((used)) I2C3HandlerImpl()
{
    Boardcore::I2C *port = Boardcore::I2C::ports[2];
    if (port)
        port->IRQhandleInterrupt();
}

/**
 * I2C error interrupt
 */
void __attribute__((naked)) I2C3_ER_IRQHandler()
{
    saveContext();
    asm volatile("bl _Z18I2C3errHandlerImplv");
    restoreContext();
}

/**
 * I2C error interrupt actual implementation
 */
void __attribute__((used)) I2C3errHandlerImpl()
{
    Boardcore::I2C *port = Boardcore::I2C::ports[2];
    if (port)
        port->IRQhandleErrInterrupt();
}

#if defined(STM32F746xx) || defined(STM32F767xx)
//** I2C4 **//
/**
 * I2C address sent interrupt
 */
void __attribute__((naked)) I2C4_EV_IRQHandler()
{
    saveContext();
    asm volatile("bl _Z15I2C4HandlerImplv");
    restoreContext();
}

/**
 * I2C address sent interrupt actual implementation
 */
void __attribute__((used)) I2C4HandlerImpl()
{
    Boardcore::I2C *port = Boardcore::I2C::ports[3];
    if (port)
        port->IRQhandleInterrupt();
}

/**
 * I2C error interrupt
 */
void __attribute__((naked)) I2C4_ER_IRQHandler()
{
    saveContext();
    asm volatile("bl _Z18I2C4errHandlerImplv");
    restoreContext();
}

/**
 * I2C error interrupt actual implementation
 */
void __attribute__((used)) I2C4errHandlerImpl()
{
    Boardcore::I2C *port = Boardcore::I2C::ports[3];
    if (port)
        port->IRQhandleErrInterrupt();
}
#endif
#endif

namespace Boardcore
{

I2C::I2C(I2CType *i2c, Speed speed, Addressing addressing, uint16_t address)
    : i2c(i2c), speed(speed), addressing(addressing), address(address),
      mutex(miosix::FastMutex::Options::RECURSIVE)
{
    // Setting the id and irqn of the right i2c peripheral
    switch (reinterpret_cast<uint32_t>(i2c))
    {
        case I2C1_BASE:
            this->id = 1;
            irqnEv   = I2C1_EV_IRQn;
            irqnErr  = I2C1_ER_IRQn;
            break;
#if defined(STM32F429xx) || defined(STM32F407xx) || defined(STM32F746xx) || \
    defined(STM32F767xx)
        case I2C2_BASE:
            this->id = 2;
            irqnEv   = I2C2_EV_IRQn;
            irqnErr  = I2C2_ER_IRQn;
            break;
        case I2C3_BASE:
            this->id = 3;
            irqnEv   = I2C3_EV_IRQn;
            irqnErr  = I2C3_ER_IRQn;
            break;
#if defined(STM32F746xx) || defined(STM32F767xx)
        case I2C4_BASE:
            this->id = 4;
            irqnEv   = I2C4_EV_IRQn;
            irqnErr  = I2C4_ER_IRQn;
            break;
#endif
#endif
    }

    // Enabling the peripheral on the right APB
    ClockUtils::enablePeripheralClock(i2c);
}

I2C::~I2C()
{
    // removing the relative i2c port from the array
    I2C::ports[id - 1] = 0;

    // Disabling the interrupts (Ev and Err) in the NVIC for the relative i2c
    NVIC_DisableIRQ(irqnEv);
    NVIC_DisableIRQ(irqnErr);

    // Disabling the peripheral
    i2c->CR1 &= ~I2C_CR1_PE;

    // Disabling the peripheral on the bus
    ClockUtils::disablePeripheralClock(i2c);
}

bool I2C::init()
{
    if (initialized)
    {
        LOG_ERR(logger, "Peripheral already initialized!");
        return false;
    }

    // Assuring that the peripheral is disabled
    i2c->CR1 = 0;

    /* setting the peripheral input clock */
    // Retrieving the frequency of the APB relative to the I2C peripheral [MHz]
    // (I2C peripherals are always connected to APB1, Low speed bus)
    uint32_t f =
        ClockUtils::getAPBFrequency(ClockUtils::APB::APB1) / 1000000 / 2;

    // frequency higher than 50MHz not allowed
    if (f > 50)
    {
        LOG_ERR(logger, "APB frequency too high!");
        return false;
    }

    // frequency < 2MHz in standard mode or < 4MHz in fast mode not allowed
    if ((speed == STANDARD && f < 2) || (speed == FAST && f < 4))
    {
        LOG_ERR(logger, "APB frequency too low!");
        return false;
    }

    I2C1->CR1 = I2C_CR1_SWRST;
    I2C1->CR1 = 0;

    // Programming the input clock in order to generate correct timings +
    // enabling generation of all interrupts
    i2c->CR2 = (f & I2C_CR2_FREQ) |  // setting FREQ bits
               I2C_CR2_ITBUFEN;      // enabling interupts for rx/tx byte

    // Configuring the Clock Control Register
    if (speed == Speed::STANDARD)
    {
        // if STANDARD mode, this is the divider to the peripheral clock to
        // reach the wanted frequency; It's divided by 2 because in reality it
        // uses this value to calculate the time that the clock needs to be in
        // the "set" state. [* 1000 KHz / (100 KHz * 2) = *5]
        i2c->CCR = f * 5;  // setting the CCR bits (implicit Standard mode)
    }
    else
    {
        // [WARNING] hardcocded to use DUTY = 0
        i2c->CCR = I2C_CCR_FS |  // selecting Fast mode
                   f * 10 / 3;   // setting the CCR bits

        // for DUTY = 1
        // i2c->CCR = I2C_CCR_FS |    // selecting Fast mode
        //            I2C_CCR_DUTY |  // selecting dutycycle of 9 - 16
        //            f * 2 / 5;      // setting the CCR bits (f * 10 / 25)
    }

    // Configuring the TRISE
    i2c->TRISE = (f & I2C_CR2_FREQ) + 1;

    // Enabling the interrupts (Ev and Err) in the NVIC for the relative i2c
    NVIC_SetPriority(irqnEv, 15);
    NVIC_ClearPendingIRQ(irqnEv);
    NVIC_EnableIRQ(irqnEv);
    NVIC_SetPriority(irqnErr, 15);
    NVIC_ClearPendingIRQ(irqnErr);
    NVIC_EnableIRQ(irqnErr);

    // Setting the Own Address Register
    i2c->OAR1 = address |            // Setting the own address
                (addressing << 15);  // Selecting the addressing mode

    // Add to the array of i2c peripherals so that the interrupts can see it
    I2C::ports[id - 1] = this;

    // Finally enabling the peripheral
    i2c->CR1 |= I2C_CR1_PE;

    initialized = true;

    return true;
}

int I2C::read(uint16_t slaveAddress, void *buffer, size_t nBytes,
              bool generateStopSignal = true)
{
    uint8_t *buff = static_cast<uint8_t *>(buffer);

    // Synchronization because of the single bus
    miosix::Lock<miosix::FastMutex> lock(mutex);

    // Enabling option to generate ACK
    i2c->CR1 |= I2C_CR1_ACK;

    // Sending prologue until the channel is clear
    if (!prologue(slaveAddress, false, nBytes))
    {
        return 0;
    }

    // reading the nBytes
    for (size_t i = 0; i < nBytes; i++)
    {
        {
            miosix::InterruptDisableLock dLock;
            // waiting for the reception of another byte
            WAIT_FOR_REGISTER_CHANGE(i2c->SR1 & I2C_SR1_RXNE, dLock, 0)
        }

        // checking if a byte has been lost (TODO: see what to do)
        if (i2c->SR1 & I2C_SR1_BTF)
            ;

        buff[i] = I2C1->DR;

        // clearing the ACK flag and setting the STOP flag in order to send a
        // NACK on the last byte that will be read and generating the stop
        // condition
        if (i == nBytes - 2)
        {
            i2c->CR1 &= ~I2C_CR1_ACK;
        }
    }

    // generate the stop condition
    if (generateStopSignal)
        i2c->CR1 |= I2C_CR1_STOP;

    return nBytes;
};

int I2C::write(uint16_t slaveAddress, void *buffer, size_t nBytes,
               bool generateStopSignal = true)
{
    uint8_t *buff = static_cast<uint8_t *>(buffer);

    // Synchronization because of the single bus
    miosix::Lock<miosix::FastMutex> lock(mutex);

    if (!prologue(slaveAddress, true, nBytes))
    {
        return 0;
    }

    // sending the nBytes
    for (size_t i = 0; i < nBytes; i++)
    {
        miosix::InterruptDisableLock dLock;
        i2c->DR = buff[i];

        // waiting for the sending of the byte
        WAIT_FOR_REGISTER_CHANGE(i2c->SR1 & I2C_SR1_TXE, dLock, 0)
    }

    // if we are on the last byte, generate the stop condition
    if (generateStopSignal)
        i2c->CR1 |= I2C_CR1_STOP;

    return nBytes;
};

bool I2C::prologue(uint16_t slaveAddress, bool writeOperation, size_t nBytes)
{
    // Generating start condition if bus not busy -> passing in Master mode
    // [WARNING] BUSY WAIT
    while (i2c->SR2 & I2C_SR2_BUSY)
        ;

    {
        miosix::InterruptDisableLock dLock;
        i2c->CR1 |= I2C_CR1_START | I2C_CR1_ACK;

        // waiting for reception of start signal
        WAIT_FOR_REGISTER_CHANGE(i2c->SR1 & I2C_SR1_SB, dLock, false)

        if (!(i2c->SR2 & I2C_SR2_MSL))
        {
            // peripheral not changed to master mode
            i2c->CR1 |= I2C_CR1_STOP;
            return false;
        }
    }

    // Sending (header + ) slave address
    if (addressing == Addressing::BIT7)
    {
        miosix::InterruptDisableLock dLock;
        // setting the LSB if we want to enter receiver mode
        i2c->DR = slaveAddress | (writeOperation ? 0 : 1);

        // // Checking if a slave matched his address
        WAIT_FOR_REGISTER_CHANGE(i2c->SR1 & I2C_SR1_ADDR, dLock, false)

        if (!(I2C1->SR2 & I2C_SR2_MSL))
        {
            return false;
        }
    }
    else  // addressing == Addressing::BIT10
    {
        // Header generated (composed of 11110xx0 bits with xx as the 9th and
        // 8th bits of the address)
        const uint8_t header = 0b11110000 | ((slaveAddress >> 7) & 0b110);

        {
            miosix::InterruptDisableLock dLock;
            // sending header
            i2c->DR = header;

            // Checking if the header has been sent
            WAIT_FOR_REGISTER_CHANGE(i2c->SR1 & I2C_SR1_ADD10, dLock, false)
        }

        {
            miosix::InterruptDisableLock dLock;
            // sending address ((1 << 8) - 1) = 0xff
            i2c->DR = (slaveAddress & 0xff);

            // Checking if a slave matched his address
            WAIT_FOR_REGISTER_CHANGE(i2c->SR1 & I2C_SR1_ADDR, dLock, false)
        }

        // if we want to enter in receiver mode
        if (!writeOperation)
        {
            // Checking if the channel is busy (clearing ADDR flag)
            if (i2c->SR2 & I2C_SR2_BUSY)
                ;

            {
                miosix::InterruptDisableLock dLock;
                // Repeated start
                i2c->CR1 |= I2C_CR1_START;

                // waiting for reception of start signal
                WAIT_FOR_REGISTER_CHANGE(i2c->SR1 & I2C_SR1_SB, dLock, false)
            }

            // sending modified header
            i2c->DR = header | 1;

            // TODO: reset ACK flag
        }
    }

    // Disabling the generation of the ACK if reading only 1 byte
    if (!writeOperation && nBytes == 1)
        i2c->CR1 &= ~I2C_CR1_ACK;

    // Checking if the channel is busy (clearing ADDR flag)
    if (i2c->SR2 & I2C_SR2_BUSY)
        ;

    return true;
}

void I2C::IRQhandleInterrupt()
{
    // disabling the regeneration of the interrupt; if we don't disable the
    // interrupts we will enter in an infinite loop of interrupts
    i2c->CR2 &= ~(I2C_CR2_ITEVTEN | I2C_CR2_ITERREN);

    // waking up the waiting thread
    wakeUpWaitingThread(waiting);
    waiting = 0;
}

void I2C::IRQhandleErrInterrupt()
{
    // disabling the regeneration of the interrupt; if we don't disable the
    // interrupts we will enter in an infinite loop of interrupts
    i2c->CR2 &= ~(I2C_CR2_ITEVTEN | I2C_CR2_ITERREN);

    // notifying an error
    error = true;

    // if (i2c->SR1 & I2C_SR1_TIMEOUT)

    // clearing all the errors in the register
    i2c->SR1 &= ~(I2C_SR1_SMBALERT | I2C_SR1_TIMEOUT | I2C_SR1_PECERR |
                  I2C_SR1_OVR | I2C_SR1_AF | I2C_SR1_ARLO | I2C_SR1_BERR);

    // waking up the waiting thread
    wakeUpWaitingThread(waiting);
    waiting = 0;
}

}  // namespace Boardcore