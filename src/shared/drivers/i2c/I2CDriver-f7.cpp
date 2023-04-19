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

#ifdef _ARCH_CORTEXM7_STM32F7

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
        scl.alternateFunction(I2CConsts::I2C_PIN_ALTERNATE_FUNCTION);
        sda.alternateFunction(I2CConsts::I2C_PIN_ALTERNATE_FUNCTION);
        scl.mode(miosix::Mode::ALTERNATE_OD);
        sda.mode(miosix::Mode::ALTERNATE_OD);
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
    // Resetting the I2C peripheral before setting the registers (resetting PE
    // bit resets the peripheral)
    i2c->CR1 = 0;

    // Retrieving the frequency of the APB relative to the I2C peripheral
    // [MHz] (I2C peripherals are always connected to APB1, Low speed bus). In
    // fact by default the I2C peripheral is clocked by the APB1 bus; anyway HSI
    // and SYSCLK can be chosen.
    I2CConsts::f =
        ClockUtils::getAPBPeripheralsClock(ClockUtils::APB::APB1) / 1000000;

    // I2CCLK < (t_low - t_filters) / 4
    // I2CCLK < t_high
    // I2CCLK < 4/3 * t_SCL
}

bool I2CDriver::read(const I2CSlaveConfig &slaveConfig, void *buffer,
                     size_t nBytes)
{
    // Setting up the read transaction
    transaction.operation    = Operation::READ;
    transaction.buffWrite    = nullptr;
    transaction.buffRead     = static_cast<uint8_t *>(buffer);
    transaction.nBytes       = nBytes;
    transaction.nBytesDone   = 0;
    transaction.generateStop = true;

    // enabling only the RX interrupts
    i2c->CR1 &= ~I2C_CR1_TXIE;
    i2c->CR1 |= I2C_CR1_RXIE;

    // Sending doOperation when the channel isn't busy
    return doOperation(slaveConfig);
};

bool I2CDriver::write(const I2CSlaveConfig &slaveConfig, const void *buffer,
                      size_t nBytes, bool generateStop)
{
    // Setting up the write transaction
    transaction.operation    = Operation::WRITE;
    transaction.buffWrite    = static_cast<const uint8_t *>(buffer);
    transaction.buffRead     = nullptr;
    transaction.nBytes       = nBytes;
    transaction.nBytesDone   = 0;
    transaction.generateStop = generateStop;

    // enabling only the TX interrupts
    i2c->CR1 &= ~I2C_CR1_RXIE;
    i2c->CR1 |= I2C_CR1_TXIE;

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
        for (; (i < I2CConsts::MAX_N_POLLING) && (i2c->ISR & I2C_ISR_BUSY); ++i)
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

    // Setting up transaction
    setupTransaction();

    // Sending STOP condition and wake up thread
    reStarting = false;

    // clearing pending flags
    i2c->ICR |= (I2C_ICR_ADDRCF | I2C_ICR_ARLOCF | I2C_ICR_BERRCF |
                 I2C_ICR_OVRCF | I2C_ICR_STOPCF);

    // Starting the transaction with the START bit
    // From the wait till the end of transaction it will all be executed in the
    // event Interrupt Service Routine
    {
        miosix::FastInterruptDisableLock dLock;

        // Sending the start condition
        i2c->CR2 |= I2C_CR2_START;

        // setting automatic stop generation if we have to generate STOP
        // condition. This must be done after the START condition generation
        // since, in case of a reStart, this would immediately end the previous
        // transaction before the start condition is generated.
        if (transaction.generateStop)
        {
            i2c->CR2 |= I2C_CR2_AUTOEND;
        }
        else
        {
            i2c->CR2 &= ~I2C_CR2_AUTOEND;
        }

        // Making the thread wait for the operation completion. The next steps
        // will be performed in the ISR while the thread stays in waiting state.
        // The only way the thread will be waken up are the completion of the
        // operation or an error during the transaction.
        return IRQwaitForOperationCompletion(dLock);
    }
}

void I2CDriver::setupPeripheral(const I2CSlaveConfig &slaveConfig)
{
    // Disabling the I2C peripheral before setting the registers
    i2c->CR1 &= ~I2C_CR1_PE;

    // setting the SCLH and SCLL bits in I2C_TIMINGR register to generate
    // correct timings for each speed modes
    if (slaveConfig.speed == Speed::STANDARD)
    {
        // PRESC = 0xb
        // SCLL = 0x13
        // SCLH = 0xf
        // SDADEL = 0x2
        // SCLDEL = 0x4
        i2c->TIMINGR = (0xb << I2C_TIMINGR_PRESC_Pos) |   // PRESC
                       (0x13 << I2C_TIMINGR_SCLL_Pos) |   // SCLL
                       (0xf << I2C_TIMINGR_SCLH_Pos) |    // SCLH
                       (0x2 << I2C_TIMINGR_SDADEL_Pos) |  // SDADEL
                       (0x4 << I2C_TIMINGR_SCLDEL_Pos);   // SCLDEL
    }
    else if (slaveConfig.speed == Speed::FAST)
    {
        // PRESC = 0x5
        // SCLL = 0x9
        // SCLH = 0x3
        // SDADEL = 0x3
        // SCLDEL = 0x3
        i2c->TIMINGR = (0x5 << I2C_TIMINGR_PRESC_Pos) |   // PRESC
                       (0x9 << I2C_TIMINGR_SCLL_Pos) |    // SCLL
                       (0x3 << I2C_TIMINGR_SCLH_Pos) |    // SCLH
                       (0x3 << I2C_TIMINGR_SDADEL_Pos) |  // SDADEL
                       (0x3 << I2C_TIMINGR_SCLDEL_Pos);   // SCLDEL
    }
    else if (slaveConfig.speed == Speed::FAST_PLUS)
    {
        // PRESC = 0x5
        // SCLL = 0x3
        // SCLH = 0x1
        // SDADEL = 0x0
        // SCLDEL = 0x1
        i2c->TIMINGR = (0x5 << I2C_TIMINGR_PRESC_Pos) |   // PRESC
                       (0x3 << I2C_TIMINGR_SCLL_Pos) |    // SCLL
                       (0x1 << I2C_TIMINGR_SCLH_Pos) |    // SCLH
                       (0x0 << I2C_TIMINGR_SDADEL_Pos) |  // SDADEL
                       (0x1 << I2C_TIMINGR_SCLDEL_Pos);   // SCLDEL
    }
    else
    {
        D(assert(false && "speed not supported"));
    }

    // setting addressing mode, read or write mode and slave address
    i2c->CR2 =
        (slaveConfig.addressing == Addressing::BIT10 ? I2C_CR2_ADD10 : 0) |
        (slaveConfig.slaveAddress << (I2C_CR2_SADD_Pos + 1));

    // Finally enabling the peripheral
    i2c->CR1 |= I2C_CR1_PE;
}

void I2CDriver::setupTransaction()
{
    // Setting the direction of the transaction
    if (transaction.operation == Operation::READ)
    {
        i2c->CR2 |= I2C_CR2_RD_WRN;
    }
    else
    {
        i2c->CR2 &= ~I2C_CR2_RD_WRN;
    }

    // setting registers for the remaining bytes
    setupReload();
}

void I2CDriver::setupReload()
{
    if ((transaction.nBytes - transaction.nBytesDone) <= 0xffu)
    {
        i2c->CR2 &= ~I2C_CR2_RELOAD;
        i2c->CR2 |= ((transaction.nBytes - transaction.nBytesDone)
                     << I2C_CR2_NBYTES_Pos);
    }
    else
    {
        i2c->CR2 |=
            (I2C_CR2_RELOAD |                // There must be a reload
             (0xff << I2C_CR2_NBYTES_Pos));  // maximum bytes that can be sent
    }
}

void I2CDriver::flushBus()
{
    // If there isn't any locked state return immediately
    if (!((lastError & (Errors::BUS_LOCKED | Errors::BERR)) &&
          ((i2c->ISR & I2C_ISR_BUSY))))
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
        scl.mode(miosix::Mode::OPEN_DRAIN);
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
        scl.mode(miosix::Mode::ALTERNATE_OD);
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

    // enabling interrupts for errors
    i2c->CR1 |= (I2C_CR1_ERRIE |   // interrupt for errors
                 I2C_CR1_NACKIE |  // interrupt for NACKs
                 I2C_CR1_TCIE |    // interrupt for TC and TCR
                 I2C_CR1_STOPIE);  // interrupt for STOP detected

    // flag thread as waiting, enable interrupts in I2C peripheral and yield
    // till an interrupt doesn't wake up the thread
    while (waiting)
    {
        waiting->IRQwait();
        miosix::FastInterruptEnableLock eLock(dLock);
        waiting->yield();
    }

    // If error occurred, parse it to notify the error(s); otherwise reset
    // lastError parameter
    if (error)
    {
        lastError |= ((error & I2C_ISR_OVR) ? Errors::OVR : 0) |
                     ((error & I2C_ISR_BERR) ? Errors::BERR : 0) |
                     ((error & I2C_ISR_ARLO) ? Errors::ARLO : 0);
        error = 0;

        return false;
    }

    lastError = Errors::NO_ERROR;
    return true;
}

inline void I2CDriver::IRQwakeUpWaitingThread()
{
    // invalidating the buffer pointers (avoiding to keep a pointer to an old
    // memory location)
    transaction.buffRead  = nullptr;
    transaction.buffWrite = nullptr;

    // disabling interrupts for errors
    i2c->CR1 &= ~(I2C_CR1_ERRIE |   // interrupt for errors
                  I2C_CR1_NACKIE |  // interrupt for NACKs
                  I2C_CR1_TCIE |    // interrupt for TC and TCR
                  I2C_CR1_STOPIE |  // interrupt for STOP detected
                  I2C_CR1_TXIE |    // interrupt for tx buffer
                  I2C_CR1_RXIE);    // interrupt for rx buffer

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
    // If NACK reception, return error
    if (i2c->ISR & I2C_ISR_NACKF)
    {
        // "reserved" bit in the ISR register, so we don't collide with
        // other fields
        error = 1 << 14;
        lastError |= Errors::AF;
        i2c->ICR |= I2C_ICR_NACKCF;
        IRQwakeUpWaitingThread();
        return;
    }

    if (transaction.nBytesDone < transaction.nBytes)
    {
        // Performing the read/write
        if (i2c->ISR & I2C_ISR_RXNE)
        {
            // READ
            transaction.buffRead[transaction.nBytesDone] = i2c->RXDR;
            transaction.nBytesDone++;
        }
        else if (i2c->ISR & (I2C_ISR_TXIS | I2C_ISR_TXE))
        {
            // WRITE
            i2c->TXDR = transaction.buffWrite[transaction.nBytesDone];
            transaction.nBytesDone++;
        }
    }

    // Transfer complete reload: setting registers for the remaining bytes
    if (i2c->ISR & I2C_ISR_TCR)
    {
        setupReload();
    }

    // when stop detected on the bus
    if (i2c->ISR & I2C_ISR_STOPF)
    {
        // clearing STOPF
        i2c->ICR |= I2C_ICR_STOPCF;

        if (transaction.nBytesDone < transaction.nBytes)
        {
            error = 1 << 14;
            lastError |= Errors::BERR;
        }

        // waking up the waiting thread
        IRQwakeUpWaitingThread();
        return;
    }

    // Transfer complete (RELOAD = 0, AUTOEND = 0, NBYTES transferred)
    if ((i2c->ISR & I2C_ISR_TC) &&
        (transaction.nBytesDone >= transaction.nBytes))
    {
        if (transaction.generateStop)
        {
            // stop and wait for STOPF event
            i2c->CR2 |= I2C_CR2_STOP;
        }
        else
        {
            reStarting = true;
            // waking up the waiting thread
            IRQwakeUpWaitingThread();
            return;
        }
    }
}

void I2CDriver::IRQhandleErrInterrupt()
{
    error = (i2c->ISR | (1 << 24));

    // Clearing all the errors in the register
    i2c->ICR |= (I2C_ICR_ADDRCF | I2C_ICR_ARLOCF | I2C_ICR_BERRCF |
                 I2C_ICR_OVRCF | I2C_ICR_STOPCF);

    // Waking up the waiting thread
    IRQwakeUpWaitingThread();
}

}  // namespace Boardcore

#endif  // _ARCH_CORTEXM7_STM32F7