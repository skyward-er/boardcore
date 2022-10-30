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

#include <utils/ClockUtils.h>

Boardcore::I2C *Boardcore::I2C::ports[N_I2C_PORTS];

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
void __attribute__((used)) I2C1errHandlerImpl() {}

#if defined(STM32F429xx) || defined(STM32F407xx)
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
void __attribute__((used)) I2C2errHandlerImpl() {}

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
void __attribute__((used)) I2C3errHandlerImpl() {}
#endif

namespace Boardcore
{

I2C::I2C(I2CType *i2c, Speed speed, Addressing addressing, uint16_t address)
    : i2c(i2c), speed(speed), addressing(addressing),
      header(0b11110000 & ((address >> 7) & 0b110)), address(address)
{
    // Setting the id of the serial port
    switch (reinterpret_cast<uint32_t>(i2c))
    {
        case I2C1_BASE:
            this->id = 1;
            irqnEv   = I2C1_EV_IRQn;
            irqnErr  = I2C1_ER_IRQn;
            break;
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
    }

    // Enabling the peripheral on the right APB
    ClockUtils::enablePeripheralClock(i2c);
}

I2C::~I2C() {}

bool I2C::init()
{
    if (initialized)
    {
        LOG_ERR(logger, "Peripheral already initialized!");
        return false;
    }

    // Assuring that the peripheral is disabled
    if (i2c->CR1 & I2C_CR1_PE)
    {
        LOG_INFO(logger,
                 "Can't set CCR if the peripheral is enabled! Disabling it.");
        i2c->CR1 &= ~I2C_CR1_PE;
    }

    /* setting the peripheral input clock */
    // frequency of the APB relative to the I2C peripheral in MHz
    uint32_t f = ClockUtils::getAPBFrequency(ClockUtils::APB::APB1) >>
                 20;  // I2C peripherals are always connected to
                      // APB1 (Low speed bus)

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

    i2c->CCR = 0;
    i2c->CCR |= (speed ? I2C_CCR_FS : 0);  // setting F/S speed bits

    if (speed == Speed::STANDARD)
    {
        // if STANDARD mode, this is the divider to the peripheral clock to
        // reach the wanted frequency; It's divided by 2 because in reality it
        // uses this value to calculate the time that the clock needs to be in
        // the "set" state. [* 1000 KHz / (100 KHz * 2) = *5]
        i2c->CCR |= f * 5;  // setting the CCR bits
    }
    else
    {
        // [WARNING] hardcocded to use DUTY = 0
        i2c->CCR |= f * 10 / 3;  // setting the CCR bits

        // for DUTY = 1
        // i2c->CCR |= I2C_CCR_DUTY;
        // i2c->CCR |= f * 10 / 25;  // setting the CCR bits
        // i2c->CCR |= f * 2 / 5;  // setting the CCR bits
    }

    i2c->CR2 = 0;
    i2c->CR2 |= (f & I2C_CR2_FREQ);  // setting FREQ bits
    i2c->CR2 |= I2C_CR2_ITEVTEN;     // enabling interupts for different phases

    // Enabling the interrupts (Ev and Err) in the NVIC for the relative i2c
    NVIC_SetPriority(irqnEv, 15);
    NVIC_EnableIRQ(irqnEv);
    NVIC_SetPriority(irqnErr, 15);
    NVIC_EnableIRQ(irqnErr);

    /* Setting the Own Address Register */
    i2c->OAR1 = 0;
    i2c->OAR1 |= (addressing << 15);
    i2c->OAR1 |= (address << 1);

    /* Setting the TRISE */
    i2c->TRISE = 0;
    i2c->TRISE |= (f & I2C_CR2_FREQ) + 1;

    // Add to the array of i2c peripherals so that the interrupts can see it
    I2C::ports[id - 1] = this;

    i2c->CR1 |= I2C_CR1_PE;

    return true;
}

/**
 * @brief Blocking read operation to read nBytes or till the data transfer
 * is complete.
 */
int I2C::read(uint16_t slaveAddress, void *buffer, size_t nBytes)
{
    // TODO: Synchronize
    waiting = miosix::Thread::getCurrentThread();
    while (!prologue(slaveAddress, false))
        ;

    // TODO: implement the read operation
    return 0;
};

/**
 * @brief Blocking write operation.
 */
int I2C::write(uint16_t slaveAddress, void *buf, size_t nChars)
{
    // TODO: Synchronize
    waiting = miosix::Thread::getCurrentThread();
    while (!prologue(slaveAddress, true))
        ;

    // TODO: implement the write operation
    return 0;
};

/**
 * @brief Prologue of any read/write operation.
 * @returns True if the channel is clear; False otherwise.
 */
bool I2C::prologue(uint16_t slaveAddress, bool writeOperation)
{
    /* Generating start signal */
    // Generating start condition -> passing in Master mode
    i2c->CR1 |= I2C_CR1_START;

    // waiting for reception of start signal
    while (!(i2c->SR1 & I2C_SR1_SB))
        miosix::Thread::wait();

    /* Sending (header + ) slave address */
    if (addressing == Addressing::BIT7)
    {
        // setting the LSB if we want to enter receiver mode
        i2c->DR = address | (writeOperation ? 0 : 1);

        // Checking if a slave matched his address
        while (!(i2c->SR1 & I2C_SR1_ADDR))
            miosix::Thread::wait();
    }
    else  // addressing == Addressing::BIT10
    {
        // sending header
        i2c->DR = header;

        // Checking if the header has been sent
        while (!(i2c->SR1 & I2C_SR1_ADD10))
            miosix::Thread::wait();

        // sending address ((1 << 8) - 1) = 0xff
        i2c->DR = (address & 0xff);

        // Checking if a slave matched his address
        while (!(i2c->SR1 & I2C_SR1_ADDR))
            miosix::Thread::wait();

        // if we want to enter in receiver mode
        if (writeOperation)
        {
            // Repeated start
            i2c->CR1 |= I2C_CR1_START;

            // waiting for reception of start signal
            while (!(i2c->SR1 & I2C_SR1_SB))
                miosix::Thread::wait();

            // sending modified header
            i2c->DR = header | 1;
        }
        // TODO: send the repeated Start condition with changed header
    }

    // Checking if the channel is busy
    if (i2c->SR2 & I2C_SR2_BUSY)
        return false;

    return true;
}

void I2C::IRQhandleInterrupt() { waiting->wakeup(); }

}  // namespace Boardcore