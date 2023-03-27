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

#ifdef _ARCH_CORTEXM4_STM32F4

#include <assert.h>
#include <kernel/scheduler/scheduler.h>
#include <utils/ClockUtils.h>
#include <utils/Debug.h>

#include "I2CDriver.h"

namespace I2CConsts
{
static Boardcore::I2CDriver *ports[N_I2C_PORTS] =
    {};  ///< Pointer to serial port classes to
         ///< let interrupts access the classes
static const int MAX_N_POLLING =
    2000;  ///< Maximum number of cycles for polling
static const int N_SCL_BITBANG =
    16;  ///< Number of clocks created for slave locked bus recovery
static const uint8_t I2C_PIN_ALTERNATE_FUNCTION =
    4;             ///< Alternate Function number of the I2C peripheral pins
static uint8_t f;  ///< APB peripheral clock frequency
}  // namespace I2CConsts

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
    auto *port = I2CConsts::ports[0];
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
    auto *port = I2CConsts::ports[0];
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
    auto *port = I2CConsts::ports[1];
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
    auto *port = I2CConsts::ports[1];
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
    auto *port = I2CConsts::ports[2];
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
    auto *port = I2CConsts::ports[2];
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
    auto *port = I2CConsts::ports[3];
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
    auto *port = I2CConsts::ports[3];
    if (port)
    {
        port->IRQhandleErrInterrupt();
    }
}
#endif

namespace Boardcore
{

I2CDriver::I2CDriver(I2C_TypeDef *i2c, miosix::GpioPin scl, miosix::GpioPin sda)
    : i2c(i2c), scl(scl), sda(sda), transaction()
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
            // Checking that the peripheral is present in this architecture
            D(assert(false &&
                     "I2C peripheral not present in this architecture"));
            break;
    }

    {
        miosix::FastInterruptDisableLock dLock;

        // Initializing the alternate function and mode of the pins so we won't
        // forget the open-drain mode, avoiding eventual short-circuits between
        // master and slaves when they both drive the same bus on two different
        // logical values.
        // scl.alternateFunction(I2CConsts::I2C_PIN_ALTERNATE_FUNCTION);
        // sda.alternateFunction(I2CConsts::I2C_PIN_ALTERNATE_FUNCTION);
        // scl.mode(miosix::Mode::ALTERNATE_OD_PULL_UP);
        // sda.mode(miosix::Mode::ALTERNATE_OD_PULL_UP);
    }

    // Checking that this particular I2C port hasn't been already instantiated
    D(assert(id > 0));
    D(assert(I2CConsts::ports[id - 1] == nullptr));

    // Enabling the peripheral's clock
    ClockUtils::enablePeripheralClock(i2c);

    init();

    // Add to the array of i2c peripherals so that the interrupts can see it
    I2CConsts::ports[id - 1] = this;

    // Enabling the interrupts (Ev and Err) in the NVIC
    NVIC_SetPriority(irqnEv, 15);
    NVIC_ClearPendingIRQ(irqnEv);
    NVIC_EnableIRQ(irqnEv);
    NVIC_SetPriority(irqnErr, 15);
    NVIC_ClearPendingIRQ(irqnErr);
    NVIC_EnableIRQ(irqnErr);
}

I2CDriver::~I2CDriver()
{
    // Removing the relative i2c port from the array
    I2CConsts::ports[id - 1] = nullptr;

    // Disabling the interrupts (Ev and Err) in the NVIC
    NVIC_DisableIRQ(irqnEv);
    NVIC_DisableIRQ(irqnErr);

    // Disabling the peripheral
    i2c->CR1 &= ~I2C_CR1_PE;

    // Disabling the peripheral on the bus
    ClockUtils::disablePeripheralClock(i2c);
}

void I2CDriver::init()
{
    // Resetting the I2C peripheral before setting the registers
    i2c->CR1 = I2C_CR1_SWRST;
    i2c->CR1 = 0;  // cppcheck-suppress redundantAssignment

    // Retrieving the frequency of the APB relative to the I2C peripheral
    // [MHz] (I2C peripherals are always connected to APB1, Low speed bus)
    I2CConsts::f =
        ClockUtils::getAPBPeripheralsClock(ClockUtils::APB::APB1) / 1000000;

    // Frequency higher than 50MHz not allowed by I2C peripheral
    D(assert(I2CConsts::f <= 50));

    // Programming the input clock in order to generate correct timings +
    // enabling generation of all interrupts
    i2c->CR2 = (I2CConsts::f & I2C_CR2_FREQ) |  // setting FREQ bits
               I2C_CR2_ITBUFEN;  // enabling interrupts for rx/tx byte
}

void I2CDriver::setupPeripheral(const I2CSlaveConfig &slaveConfig)
{
    // Frequency < 2MHz in standard mode or < 4MHz in fast mode not allowed by
    // the peripheral
    D(assert((slaveConfig.speed == STANDARD && I2CConsts::f >= 2) ||
             (slaveConfig.speed == FAST && I2CConsts::f >= 4)));

    // Disabling the I2C peripheral before setting the registers
    i2c->CR1 &= ~I2C_CR1_PE;

    // Configuring the Clock Control Register
    if (slaveConfig.speed == Speed::STANDARD)
    {
        // If STANDARD mode, this is the divider to the peripheral clock to
        // reach the wanted frequency. It's divided by 2 because in reality
        // it uses this value to calculate the time that the clock needs to
        // be in the "set" state. [* 1000 KHz / (100 KHz * 2) = *5]
        i2c->CCR =
            I2CConsts::f * 5;  // Setting the CCR bits (implicit Standard mode)
    }
    else if (slaveConfig.speed == Speed::FAST)
    {
        // [WARNING] Hardcoded to use DUTY = 0
        i2c->CCR = I2C_CCR_FS |           // Selecting Fast mode
                   I2CConsts::f * 5 / 6;  // Setting the CCR bits

        // For DUTY = 1
        // i2c->CCR = I2C_CCR_FS |    // Selecting Fast mode
        //            I2C_CCR_DUTY |  // Selecting duty cycle of 9 - 16
        //            f * 2 / 5;      // Setting the CCR bits (f * 10 / 25)
    }
    else
    {
        D(assert(false && "speed not supported!"));
    }

    // Configuring the TRISE
    i2c->TRISE = (I2CConsts::f & I2C_CR2_FREQ) + 1;

    // Setting the addressing mode
    i2c->OAR1 = (slaveConfig.addressing << 15);

    // Finally enabling the peripheral
    i2c->CR1 |= I2C_CR1_PE;
}

bool I2CDriver::read(const I2CSlaveConfig &slaveConfig, void *buffer,
                     const size_t &nBytes)
{
    // Setting up the read transaction
    transaction.operation    = Operation::READ;
    transaction.buffWrite    = nullptr;
    transaction.buffRead     = static_cast<uint8_t *>(buffer);
    transaction.nBytes       = nBytes;
    transaction.nBytesDone   = 0;
    transaction.generateStop = true;

    // Disabling the generation of the ACK if reading only 1 byte, otherwise
    // enable it
    i2c->CR1 =
        ((nBytes <= 1) ? (i2c->CR1 & ~I2C_CR1_ACK) : (i2c->CR1 | I2C_CR1_ACK));

    // Sending doOperation when the channel isn't busy
    return doOperation(slaveConfig);
};

bool I2CDriver::write(const I2CSlaveConfig &slaveConfig, const void *buffer,
                      const size_t &nBytes, bool generateStop)
{
    // Setting up the write transaction
    transaction.operation    = Operation::WRITE;
    transaction.buffWrite    = static_cast<const uint8_t *>(buffer);
    transaction.buffRead     = nullptr;
    transaction.nBytes       = nBytes;
    transaction.nBytesDone   = 0;
    transaction.generateStop = generateStop;

    // Sending doOperation when the channel isn't busy
    return doOperation(slaveConfig);
};

bool I2CDriver::doOperation(const I2CSlaveConfig &slaveConfig)
{
    // Not yet supported
    D(assert(slaveConfig.addressing == Addressing::BIT7 &&
             "Only 7-bit addressing supported!"));

    // If already detected a locked state return directly without loosing time
    if (lastError & Errors::BUS_LOCKED)
    {
        reStarting = false;
        return false;
    }

    // If starting a new transaction (so STOP bit sent in previous transaction),
    // wait for the bus to be clear
    if (!reStarting)
    {
        // Waiting for the bus to be clear
        uint32_t i{0};
        for (; (i < I2CConsts::MAX_N_POLLING) && (i2c->SR2 & I2C_SR2_BUSY); ++i)
            ;

        // Locked state detected after N polling cycles
        if (i == I2CConsts::MAX_N_POLLING)
        {
            lastError = Errors::BUS_LOCKED;
            LOG_ERR(logger, fmt::format("I2C{} bus locked state detected", id));
            return false;
        }

        // Setting up the peripheral when the bus is clear in order to
        // communicate in the mode wanted by the slave device
        setupPeripheral(slaveConfig);
    }

    reStarting = false;

    // Starting the transaction with the START bit
    // From the wait till the end of transaction it will all be executed in the
    // event ISR
    {
        miosix::PauseKernelLock dLock;
        uint32_t i{0};

        // Sending the start condition
        // We are waiting on the Start Bit using polling because using
        // interrupts would have lead to a messy and less reliable code. The
        // only downside regards the usage of polling instead of interrupts; the
        // maximum number of cycles used in the tests for waiting for a start
        // condition were more or less 950. Anyway this is preferred to the risk
        // of having a deadlock.
        i2c->CR1 |= I2C_CR1_START;

        // Waiting for START condition to be sent
        for (; (i < I2CConsts::MAX_N_POLLING) &&
               (!(i2c->SR1 & I2C_SR1_SB) || !(i2c->SR2 & I2C_SR2_MSL));
             ++i)
            ;

        // START condition not sent after N polling cycles
        if (i == I2CConsts::MAX_N_POLLING)
        {
            lastError |= Errors::SB_NOT_SENT;
            LOG_ERR(logger,
                    fmt::format("I2C{} bus didn't send the start bit", id));
            return false;
        }
    }

    transaction.nBytesDone = 0;

    // Sending slave address
    {
        miosix::FastInterruptDisableLock dLock;

        // Setting the LSB if we want to enter receiver mode
        i2c->DR = ((slaveConfig.slaveAddress << 1) | transaction.operation);

        // Making the thread wait for the operation completion. The next steps
        // will be performed in the ISR while the thread stays in waiting state.
        // The only way the thread will be waken up are the completion of the
        // operation or an error during the transaction.
        return IRQwaitForOperationCompletion(dLock);
    }
}

void I2CDriver::flushBus()
{
    // If there isn't any locked state return immediately
    if (!(lastError & Errors::BUS_LOCKED))
    {
        return;
    }

    // Set the period of the bit-banged clock (Default to standard mode)
    uint8_t toggleDelay = 5;

    {
        miosix::FastInterruptDisableLock dLock;

        // Recovery from the locked state due to a stuck Slave.
        // We bit-bang 16 clocks on the scl line in order to restore pending
        // packets of the slaves.
        // scl.mode(miosix::Mode::OPEN_DRAIN_PULL_UP);
    }

    for (size_t c = 0; c < I2CConsts::N_SCL_BITBANG; c++)
    {
        scl.low();
        miosix::delayUs(toggleDelay);
        scl.high();
        miosix::delayUs(toggleDelay);
    }

    {
        miosix::FastInterruptDisableLock dLock;

        // We set again the scl pin to the correct Alternate function
        // scl.mode(miosix::Mode::ALTERNATE_OD_PULL_UP);
        scl.alternateFunction(I2CConsts::I2C_PIN_ALTERNATE_FUNCTION);
    }

    // Re-initializing the peripheral in order to avoid inconsistent state
    init();

    // Assuming the locked state is solved. If it is not the case, only when
    // it will be the case it will be detected again
    lastError = Errors::NO_ERROR;

    LOG_WARN(logger, fmt::format("I2C{} Bus flushed", id));
}

uint16_t I2CDriver::getLastError() { return lastError; }

inline bool I2CDriver::IRQwaitForOperationCompletion(
    miosix::FastInterruptDisableLock &dLock)
{
    // Saving the current thread in order to be waken up by interrupts
    waiting = miosix::Thread::IRQgetCurrentThread();

    // flag thread as waiting, enable interrupts in I2C peripheral and yield
    // till an interrupt doesn't wake up the thread
    while (waiting)
    {
        waiting->IRQwait();
        i2c->CR2 |= I2C_CR2_ITEVTEN | I2C_CR2_ITERREN;
        miosix::FastInterruptEnableLock eLock(dLock);
        waiting->yield();
    }

    // If error occurred, parse it to notify the error(s); otherwise reset
    // lastError parameter
    if (error)
    {
        lastError |= ((error & I2C_SR1_OVR) ? Errors::OVR : 0) |
                     ((error & I2C_SR1_BERR) ? Errors::BERR : 0) |
                     ((error & I2C_SR1_ARLO) ? Errors::ARLO : 0) |
                     ((error & I2C_SR1_AF) ? Errors::AF : 0);
        error = 0;
        return false;
    }
    else
    {
        lastError = Errors::NO_ERROR;
    }

    return true;
}

inline void I2CDriver::IRQwakeUpWaitingThread()
{
    // Disabling the regeneration of the interrupt; if we don't disable the
    // interrupts we will enter in an infinite loop of interrupts
    i2c->CR2 &= ~(I2C_CR2_ITEVTEN | I2C_CR2_ITERREN);

    // invalidating the buffer pointers (avoiding to keep a pointer to an old
    // memory location)
    transaction.buffRead  = nullptr;
    transaction.buffWrite = nullptr;

    if (waiting)
    {
        waiting->IRQwakeup();

        if (waiting->IRQgetPriority() >
            miosix::Thread::IRQgetCurrentThread()->IRQgetPriority())
        {
            miosix::Scheduler::IRQfindNextThread();
        }

        waiting = nullptr;
    }
}

void I2CDriver::IRQhandleInterrupt()
{
    // Address sending
    if ((i2c->SR1 & I2C_SR1_ADDR))
    {
        // Clearing ADDR flag
        if (!(i2c->SR2 & I2C_SR2_BUSY) ||  // Channel should be busy
            !(i2c->SR2 & I2C_SR2_MSL) ||   // Should be in Master mode
            !((i2c->SR2 & I2C_SR2_TRA) != transaction.operation))
        {
            // "reserved" bit in the SR1 register, so we don't collide with
            // other fields
            error = 1 << 13;
            lastError |= ADDR_ERROR;
            IRQwakeUpWaitingThread();
            return;
        }
    }

    // Performing the read/write
    if (transaction.operation == Operation::READ)
    {
        // READ
        if (i2c->SR1 & (I2C_SR1_BTF | I2C_SR1_RXNE))
        {
            // Clearing the ACK flag in order to send a NACK on the last byte
            // that will be read
            if (transaction.nBytesDone >= transaction.nBytes - 2)
            {
                i2c->CR1 &= ~I2C_CR1_ACK;
            }

            if (transaction.nBytesDone < transaction.nBytes)
            {
                transaction.buffRead[transaction.nBytesDone] = i2c->DR;
                transaction.nBytesDone++;
            }
        }
    }
    else
    {
        // WRITE
        if (i2c->SR1 & (I2C_SR1_BTF | I2C_SR1_TXE))
        {
            if (transaction.nBytesDone < transaction.nBytes)
            {
                i2c->DR = transaction.buffWrite[transaction.nBytesDone];
                transaction.nBytesDone++;
                return;
            }
        }
    }

    // Sending STOP condition and wake up thread
    if (transaction.nBytesDone >= transaction.nBytes)
    {
        // If we are on the last byte, generate the stop condition if we have to
        // end the communication
        if (transaction.generateStop)
        {
            i2c->CR1 |= I2C_CR1_STOP;
            reStarting = false;
        }
        else
        {
            reStarting = true;
        }

        // waking up the waiting thread
        IRQwakeUpWaitingThread();
    }
}

void I2CDriver::IRQhandleErrInterrupt()
{
    error = i2c->SR1;

    // Clearing all the errors in the register
    i2c->SR1 = 0;

    // In case of arbitration lost, the hardware releases automatically the
    // lines. Do not send STOP condition. In the other cases, the software must
    // issue the STOP condition.
    if (!(error & I2C_SR1_ARLO))
    {
        i2c->CR1 |= I2C_CR1_STOP;
    }

    // Waking up the waiting thread
    IRQwakeUpWaitingThread();
}

}  // namespace Boardcore

#endif  // _ARCH_CORTEXM4_STM32F4