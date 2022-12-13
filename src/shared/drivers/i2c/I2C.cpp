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

#include <assert.h>
#include <kernel/scheduler/scheduler.h>
#include <utils/ClockUtils.h>
#include <utils/Debug.h>

static Boardcore::I2C *ports[N_I2C_PORTS] =
    {};  ///< Pointer to serial port classes to
         ///< let interrupts access the classes

static const int MAX_N_POLLING =
    2000;  ///< Maximum number of cycles for polling
static const int N_SCL_BITBANG =
    16;  ///< Number of clocks created for slave locked bus recovery
static const uint8_t I2C_ADDRESS_READ  = 0x1;  ///< LSB of address to read
static const uint8_t I2C_ADDRESS_WRITE = 0x0;  ///< LSB of address to read

#ifdef I2C1
/**
 * I2C1 event interrupt
 */
void __attribute__((naked)) I2C1_EV_IRQHandler()
{
    saveContext();
    asm volatile("bl _Z15I2C1HandlerImplv");
    restoreContext();
}

/**
 * I2C1 event interrupt actual implementation
 */
void __attribute__((used)) I2C1HandlerImpl()
{
    auto *port = ports[0];
    if (port)
    {
        port->IRQhandleInterrupt();
    }
}

/**
 * I2C1 error interrupt
 */
void __attribute__((naked)) I2C1_ER_IRQHandler()
{
    saveContext();
    asm volatile("bl _Z18I2C1errHandlerImplv");
    restoreContext();
}

/**
 * I2C1 error interrupt actual implementation
 */
void __attribute__((used)) I2C1errHandlerImpl()
{
    auto *port = ports[0];
    if (port)
    {
        port->IRQhandleErrInterrupt();
    }
}
#endif

#ifdef I2C2
/**
 * I2C2 event interrupt
 */
void __attribute__((naked)) I2C2_EV_IRQHandler()
{
    saveContext();
    asm volatile("bl _Z15I2C2HandlerImplv");
    restoreContext();
}

/**
 * I2C2 event interrupt actual implementation
 */
void __attribute__((used)) I2C2HandlerImpl()
{
    auto *port = ports[1];
    if (port)
    {
        port->IRQhandleInterrupt();
    }
}

/**
 * I2C2 error interrupt
 */
void __attribute__((naked)) I2C2_ER_IRQHandler()
{
    saveContext();
    asm volatile("bl _Z18I2C2errHandlerImplv");
    restoreContext();
}

/**
 * I2C2 error interrupt actual implementation
 */
void __attribute__((used)) I2C2errHandlerImpl()
{
    auto *port = ports[1];
    if (port)
    {
        port->IRQhandleErrInterrupt();
    }
}
#endif

#ifdef I2C3
/**
 * I2C3 event interrupt
 */
void __attribute__((naked)) I2C3_EV_IRQHandler()
{
    saveContext();
    asm volatile("bl _Z15I2C3HandlerImplv");
    restoreContext();
}

/**
 * I2C3 event interrupt actual implementation
 */
void __attribute__((used)) I2C3HandlerImpl()
{
    auto *port = ports[2];
    if (port)
    {
        port->IRQhandleInterrupt();
    }
}

/**
 * I2C3 error interrupt
 */
void __attribute__((naked)) I2C3_ER_IRQHandler()
{
    saveContext();
    asm volatile("bl _Z18I2C3errHandlerImplv");
    restoreContext();
}

/**
 * I2C3 error interrupt actual implementation
 */
void __attribute__((used)) I2C3errHandlerImpl()
{
    auto *port = ports[2];
    if (port)
    {
        port->IRQhandleErrInterrupt();
    }
}
#endif

#ifdef I2C4
/**
 * I2C4 event interrupt
 */
void __attribute__((naked)) I2C4_EV_IRQHandler()
{
    saveContext();
    asm volatile("bl _Z15I2C4HandlerImplv");
    restoreContext();
}

/**
 * I2C4 event interrupt actual implementation
 */
void __attribute__((used)) I2C4HandlerImpl()
{
    auto *port = ports[3];
    if (port)
    {
        port->IRQhandleInterrupt();
    }
}

/**
 * I2C4 error interrupt
 */
void __attribute__((naked)) I2C4_ER_IRQHandler()
{
    saveContext();
    asm volatile("bl _Z18I2C4errHandlerImplv");
    restoreContext();
}

/**
 * I2C4 error interrupt actual implementation
 */
void __attribute__((used)) I2C4errHandlerImpl()
{
    auto *port = ports[3];
    if (port)
    {
        port->IRQhandleErrInterrupt();
    }
}
#endif

namespace Boardcore
{
I2C::I2C(I2C_TypeDef *i2c, Speed speed, Addressing addressing)
    : i2c(i2c), speed(speed), addressing(addressing)
{
    // Setting the id and irqn of the right i2c peripheral
    switch (reinterpret_cast<uint32_t>(i2c))
    {
#ifdef I2C1
        case I2C1_BASE:
            this->id = 1;
            irqnEv   = I2C1_EV_IRQn;
            irqnErr  = I2C1_ER_IRQn;
            break;
#endif
#ifdef I2C2
        case I2C2_BASE:
            this->id = 2;
            irqnEv   = I2C2_EV_IRQn;
            irqnErr  = I2C2_ER_IRQn;
            break;
#endif
#ifdef I2C3
        case I2C3_BASE:
            this->id = 3;
            irqnEv   = I2C3_EV_IRQn;
            irqnErr  = I2C3_ER_IRQn;
            break;
#endif
#ifdef I2C4
        case I2C4_BASE:
            this->id = 4;
            irqnEv   = I2C4_EV_IRQn;
            irqnErr  = I2C4_ER_IRQn;
            break;
#endif
        default:
            // checking that the peripheral is present in this architecture
            D(assert(false &&
                     "I2C peripheral not present in this architecture"));
            break;
    }

    // Checking that this parcticular I2C port hasn't been already instantiated
    D(assert(id > 0));
    D(assert(ports[id - 1] == nullptr));

    init();

    // Add to the array of i2c peripherals so that the interrupts can see it
    ports[id - 1] = this;

    // Enabling the interrupts (Ev and Err) in the NVIC for the relative i2c
    NVIC_SetPriority(irqnEv, 15);
    NVIC_ClearPendingIRQ(irqnEv);
    NVIC_EnableIRQ(irqnEv);
    NVIC_SetPriority(irqnErr, 15);
    NVIC_ClearPendingIRQ(irqnErr);
    NVIC_EnableIRQ(irqnErr);
}

I2C::~I2C()
{
    // removing the relative i2c port from the array
    ports[id - 1] = nullptr;

    // Disabling the interrupts (Ev and Err) in the NVIC for the relative i2c
    NVIC_DisableIRQ(irqnEv);
    NVIC_DisableIRQ(irqnErr);

    // Disabling the peripheral
    i2c->CR1 &= ~I2C_CR1_PE;

    // Disabling the peripheral on the bus
    ClockUtils::disablePeripheralClock(i2c);
}

void I2C::init()
{
    // Enabling the peripheral on the right APB
    ClockUtils::enablePeripheralClock(i2c);

    // Assuring that the peripheral is disabled
    i2c->CR1 = 0;

    /* setting the peripheral input clock */
    // Retrieving the frequency of the APB relative to the I2C peripheral [MHz]
    // (I2C peripherals are always connected to APB1, Low speed bus)
    uint32_t f{ClockUtils::getAPBPeripheralsClock(ClockUtils::APB::APB1) /
               1000000};

    // frequency higher than 50MHz not allowed by I2C peripheral
    D(assert(f <= 50));

    // frequency < 2MHz in standard mode or < 4MHz in fast mode not allowed
    D(assert((speed == STANDARD && f >= 2) || (speed == FAST && f >= 4)));

    // Resetting the I2C peripheral before setting the registers
    i2c->CR1 = I2C_CR1_SWRST;
    i2c->CR1 = 0;  // cppcheck-suppress redundantAssignment

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
                   f * 5 / 6;    // setting the CCR bits

        // for DUTY = 1
        // i2c->CCR = I2C_CCR_FS |    // selecting Fast mode
        //            I2C_CCR_DUTY |  // selecting dutycycle of 9 - 16
        //            f * 2 / 5;      // setting the CCR bits (f * 10 / 25)
    }

    // Configuring the TRISE
    i2c->TRISE = (f & I2C_CR2_FREQ) + 1;

    // Setting the addressing mode
    i2c->OAR1 = (addressing << 15);

    // Finally enabling the peripheral
    i2c->CR1 |= I2C_CR1_PE;
}

bool I2C::read(uint16_t slaveAddress, void *buffer, size_t nBytes)
{
    auto *buff = static_cast<uint8_t *>(buffer);

    // Enabling option to generate ACK
    i2c->CR1 |= I2C_CR1_ACK;

    // Sending prologue when the channel isn't busy (LSB set to signal this is a
    // read)
    if (!prologue(slaveAddress << 1 | I2C_ADDRESS_READ))
    {
        return false;
    }

    // Disabling the generation of the ACK if reading only 1 byte
    if (nBytes == 1)
    {
        i2c->CR1 &= ~I2C_CR1_ACK;
    }

    // reading the nBytes
    for (size_t i = 0; i < nBytes; i++)
    {
        {
            miosix::FastInterruptDisableLock dLock;

            // waiting for the reception of another byte
            if (!IRQwaitForRegisterChange(dLock) || !(i2c->SR1 & I2C_SR1_RXNE))
            {
                i2c->CR1 |= I2C_CR1_STOP;
                return false;
            }
        }

        // checking if a byte has been lost
        if (i2c->SR1 & I2C_SR1_BTF)
            ;

        buff[i] = i2c->DR;

        // clearing the ACK flag in order to send a NACK on the last byte that
        // will be read
        if (i == nBytes - 2)
        {
            i2c->CR1 &= ~I2C_CR1_ACK;
        }
    }

    // Generate the stop condition after the read transaction
    i2c->CR1 |= I2C_CR1_STOP;

    return true;
};

bool I2C::write(uint16_t slaveAddress, const void *buffer, size_t nBytes)
{
    auto *buff = static_cast<const uint8_t *>(buffer);

    // Sending prologue when the channel isn't busy
    if (!prologue(slaveAddress << 1 | I2C_ADDRESS_WRITE))
    {
        return false;
    }

    // sending the nBytes
    for (size_t i = 0; i < nBytes; i++)
    {
        miosix::FastInterruptDisableLock dLock;
        i2c->DR = buff[i];

        // waiting for the sending of the byte
        if (!IRQwaitForRegisterChange(dLock) || !(i2c->SR1 & I2C_SR1_TXE))
        {
            i2c->CR1 |= I2C_CR1_STOP;
            return false;
        }
    }

    // if we are on the last byte, generate the stop condition
    i2c->CR1 |= I2C_CR1_STOP;

    return true;
};

bool I2C::prologue(uint16_t slaveAddress)
{
    // if already detected a locked state return directly without loosing time
    if (lockedState)
    {
        return false;
    }

    uint32_t i{0};
    for (; (i < MAX_N_POLLING) && (i2c->SR2 & I2C_SR2_BUSY); ++i)
        ;

    // locked state detected after N polling cycles
    if (i == MAX_N_POLLING)
    {
        lockedState = true;
        LOG_ERR(logger, fmt::format("I2C{} bus locked state detected", id));
        return false;
    }

    {
        miosix::FastInterruptDisableLock dLock;
        i2c->CR1 |= I2C_CR1_START;

        // waiting for reception of start signal and change to master mode
        if (!IRQwaitForRegisterChange(dLock) || !(i2c->SR1 & I2C_SR1_SB) ||
            !(i2c->SR2 & I2C_SR2_MSL))
        {
            i2c->CR1 |= I2C_CR1_STOP;
            return false;
        }
    }

    // Sending (header + ) slave address
    if (addressing == Addressing::BIT7)
    {
        miosix::FastInterruptDisableLock dLock;

        // setting the LSB if we want to enter receiver mode
        i2c->DR = slaveAddress;

        // Checking if a slave matched his address
        if (!IRQwaitForRegisterChange(dLock) || !(i2c->SR1 & I2C_SR1_ADDR))
        {
            i2c->CR1 |= I2C_CR1_STOP;
            return false;
        }
    }
    else  // addressing == Addressing::BIT10
    {
        // Header generated (composed of 11110xx0 bits with xx as the 9th
        // and 8th bits of the address)
        const uint8_t header = 0b11110000 | ((slaveAddress >> 7) & 0b110);

        {
            miosix::FastInterruptDisableLock dLock;
            // sending header
            i2c->DR = header;

            // Checking if the header has been sent
            if (!IRQwaitForRegisterChange(dLock) || !(i2c->SR1 & I2C_SR1_ADD10))
            {
                i2c->CR1 |= I2C_CR1_STOP;
                return false;
            }
        }

        {
            miosix::FastInterruptDisableLock dLock;
            // sending address ((1 << 8) - 1) = 0xff
            i2c->DR = (slaveAddress & 0xff);

            // Checking if a slave matched his address
            if (!IRQwaitForRegisterChange(dLock) || !(i2c->SR1 & I2C_SR1_ADDR))
            {
                i2c->CR1 |= I2C_CR1_STOP;
                return false;
            }
        }

        // if we want to enter in receiver mode
        if (slaveAddress & I2C_ADDRESS_READ)
        {
            // Checking if the peripheral is in Master mode (clearing ADDR
            // flag with a read on SR2 register)
            if (!(i2c->SR2 & I2C_SR2_MSL))
            {
                i2c->CR1 |= I2C_CR1_STOP;
                return false;
            }

            {
                miosix::FastInterruptDisableLock dLock;
                // Repeated start
                i2c->CR1 |= I2C_CR1_START;

                // waiting for reception of start signal
                if (!IRQwaitForRegisterChange(dLock) ||
                    !(i2c->SR1 & I2C_SR1_SB))
                {
                    i2c->CR1 |= I2C_CR1_STOP;
                    return false;
                }
            }

            // sending modified header
            i2c->DR = header | I2C_ADDRESS_READ;
        }
    }

    // clearing ADDR flag
    if (!(i2c->SR2 & I2C_SR2_BUSY) ||  // channel should be busy
        !(i2c->SR2 & I2C_SR2_MSL) ||  // the peripheral should be in master mode
        !((i2c->SR2 & I2C_SR2_TRA) !=
          (slaveAddress & I2C_ADDRESS_READ)))  // Tx or Rx mode
    {
        i2c->CR1 |= I2C_CR1_STOP;
        return false;
    }

    return true;
}

void I2C::flushBus(miosix::GpioPin scl, unsigned char af)
{
    // If there isn't any locked state return immediately
    if (!lockedState)
    {
        return;
    }

    // set the period of the bit-banged clock
    uint8_t toggleDelay = (speed == Speed::STANDARD ? 5 : 2);

    {
        miosix::FastInterruptDisableLock dLock;
        // Recovery from the locked state due to a stuck Slave.
        // We bit-bang 16 clocks on the scl line in order to restore pending
        // packets of the slaves.
        scl.mode(miosix::Mode::OUTPUT);
    }

    for (size_t c = 0; c < N_SCL_BITBANG; c++)
    {
        scl.low();
        miosix::delayUs(toggleDelay);
        scl.high();
        miosix::delayUs(toggleDelay);
    }

    {
        miosix::FastInterruptDisableLock dLock;
        // we set again the scl pin to the correct Alternate function
        scl.mode(miosix::Mode::ALTERNATE_OD);
        scl.alternateFunction(af);
    }

    // Reinitializing the peripheral in order to avoid inconsistent state
    init();

    // assuming the locked state is solved. If it is not the case, only when it
    // will be the case it will be detected again
    lockedState = false;

    LOG_WARN(logger, fmt::format("I2C{} Bus flushed", id));
}

inline bool I2C::IRQwaitForRegisterChange(
    miosix::FastInterruptDisableLock &dLock)
{
    waiting = miosix::Thread::IRQgetCurrentThread();

    while (waiting)
    {
        waiting->IRQwait();
        i2c->CR2 |= I2C_CR2_ITEVTEN | I2C_CR2_ITERREN;
        miosix::FastInterruptEnableLock eLock(dLock);
        waiting->yield();
    }

    if (error)
    {
        error = false;
        return false;
    }

    return true;
}

inline void I2C::IRQwakeUpWaitingThread()
{
    if (waiting)
    {
        waiting->IRQwakeup();

        if (waiting->IRQgetPriority() >
            miosix::Thread::IRQgetCurrentThread()->IRQgetPriority())
        {
            miosix::Scheduler::IRQfindNextThread();
        }

        waiting = 0;
    }
}

void I2C::IRQhandleInterrupt()
{
    // disabling the regeneration of the interrupt; if we don't disable the
    // interrupts we will enter in an infinite loop of interrupts
    i2c->CR2 &= ~(I2C_CR2_ITEVTEN | I2C_CR2_ITERREN);

    // waking up the waiting thread
    IRQwakeUpWaitingThread();
}

void I2C::IRQhandleErrInterrupt()
{
    // disabling the regeneration of the interrupt; if we don't disable the
    // interrupts we will enter in an infinite loop of interrupts
    i2c->CR2 &= ~(I2C_CR2_ITEVTEN | I2C_CR2_ITERREN);

    // notifying an error
    error = true;

    // clearing all the errors in the register
    i2c->SR1 = 0;

    // waking up the waiting thread
    IRQwakeUpWaitingThread();
}

SyncedI2C::SyncedI2C(I2C_TypeDef *i2c, Speed speed, Addressing addressing)
    : I2C(i2c, speed, addressing)
{
}

bool SyncedI2C::read(uint16_t slaveAddress, void *buffer, size_t nBytes)
{
    miosix::Lock<miosix::FastMutex> lock(mutex);

    return I2C::read(slaveAddress, buffer, nBytes);
}

bool SyncedI2C::write(uint16_t slaveAddress, const void *buffer, size_t nBytes)
{
    miosix::Lock<miosix::FastMutex> lock(mutex);

    return I2C::write(slaveAddress, buffer, nBytes);
}

void SyncedI2C::flushBus(miosix::GpioPin scl, unsigned char af)
{
    miosix::Lock<miosix::FastMutex> lock(mutex);

    return I2C::flushBus(scl, af);
}

}  // namespace Boardcore