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
    4;  ///< Alternate Function number of the I2C peripheral pins
}  // namespace I2CConsts

/**
 * @brief Struct that collects all the timing parameters for the clock
 * generation.
 */
typedef struct
{
    uint8_t presc;   ///< Timing prescaler
    uint8_t sclh;    ///< SCL high period
    uint8_t scll;    ///< SCL low period
    uint8_t scldel;  ///< Data setup time
    uint8_t sdadel;  ///< Data hold time
} I2CTimings;

/**
 * @brief Helper function for calculating the timing parameters for the
 * peripheral
 *
 * The formula for the clock is:
 * t_SCL = [(SCLL + 1) + (SCLH + 1)] * (PRESC + 1) * t_I2CCLK + t_sync
 * @param f Peripheral timer clock frequency in kHz.
 * @param fi2c I2C clock frequency in kHz.
 * @return Struct with the timings for the wanted i2c frequency.
 */
I2CTimings calculateTimings(uint32_t f, uint32_t fi2c)
{
    I2CTimings i2cTimings;
    // calculating the "smallest" prescaler so that we can handle in a more
    // refined way (with SCLL and SCLH) the length of high and low phases. We
    // "limit" SCLL and SCLH to 64 because like so we can have acceptable values
    // for SCLDEL and SDADEL
    uint32_t temp_presc = f / (64 * fi2c);

#if defined(_BOARD_STM32F756ZG_NUCLEO)
    const uint16_t correction = 10;
#elif defined(_BOARD_STM32F767ZI_COMPUTE_UNIT) ||         \
    defined(_BOARD_STM32F767ZI_GEMINI_GS) ||              \
    defined(_BOARD_STM32F767ZI_SKYWARD_DEATH_STACK_V4) || \
    defined(_BOARD_STM32F767ZI_NUCLEO)
    const uint16_t correction = 7;
#else
    const uint16_t correction = 0;
#warning \
    "I2C Timings not corrected, actual frequency could be lower than nominal"
#endif

    // presc is 4 bit long, so avoiding overflow
    if (temp_presc >= 16)
    {
        i2cTimings.presc = 15;
    }
    else if (temp_presc == 0)
    {
        i2cTimings.presc = 0;
    }
    else
    {
        i2cTimings.presc = temp_presc - 1;
    }

    // calculating SCLL and SCLH in order to have duty cycle of 50%. Also,
    // correcting for the 250ns delay of the peripheral (250ns = 12 I2C clocks)
    // distributing the correction on SCLL and SCLH
    i2cTimings.sclh = i2cTimings.scll =
        (f / (fi2c * 2 * (i2cTimings.presc + 1)) - 1) -
        (correction / (i2cTimings.presc + 1));

    // SCLDEL >= (t_r + t_su) / ((PRESC+1)*t_i2c) - 1 ; approximated without
    // subtracting 1. scldly and sdadly are calculated using values taken from
    // the reference manual
    uint32_t scldly = 0, sdadly = 0;
    if (fi2c == 100)
    {
        scldly = 1250 * f / (i2cTimings.presc + 1) / 1000000;
        sdadly = (3450 - 1000 - 260) * f / (i2cTimings.presc + 1) / 1000000;
    }
    else if (fi2c == 400)
    {
        scldly = 400 * f / (i2cTimings.presc + 1) / 1000000;
        sdadly = (900 - 300 - 260) * f / (i2cTimings.presc + 1) / 1000000;
    }
    else if (fi2c == 1000)
    {
        scldly = 170 * f / (i2cTimings.presc + 1) / 1000000;
        sdadly = (450 - 120 - 260) * f / (i2cTimings.presc + 1) / 1000000;
    }

    // max value of scldel is 15
    i2cTimings.scldel = ((scldly < 16) ? (scldly - 1) : 15);

    // max value of sdadel is 15
    i2cTimings.sdadel = ((sdadly < 16) ? (sdadly - 1) : 15);

    return i2cTimings;
}

/**
 * @brief Used to get the timing parameters for each speed mode. If the speed
 * mode isn't supported, communication in release mode will take place with
 * minimum speed (standard mode).
 * @param speed Speed mode for which we want the timing parameters.
 * @return The struct with the timing parameters for the speed mode passed.
 */
const I2CTimings &getTimings(Boardcore::I2CDriver::Speed speed)
{
    // Retrieving the frequency of the APB relative to the I2C peripheral
    // [kHz] (I2C peripherals are always connected to APB1, Low speed bus). In
    // fact by default the I2C peripheral is clocked by the APB1 bus; anyway HSI
    // and SYSCLK can be chosen.
    static const uint32_t f = Boardcore::ClockUtils::getAPBPeripheralsClock(
                                  Boardcore::ClockUtils::APB::APB1) /
                              1000;
    static const I2CTimings i2cTimingsStd      = calculateTimings(f, 100);
    static const I2CTimings i2cTimingsFast     = calculateTimings(f, 400);
    static const I2CTimings i2cTimingsFastPlus = calculateTimings(f, 1000);

    // Calculating for all the speed modes the clock parameters (so we won't
    // have to calculate them again for every transaction)
    switch (speed)
    {
        case Boardcore::I2CDriver::Speed::STANDARD:;
            return i2cTimingsStd;
        case Boardcore::I2CDriver::Speed::FAST:
            return i2cTimingsFast;
        case Boardcore::I2CDriver::Speed::FAST_PLUS:
            return i2cTimingsFastPlus;
        default:
            D(assert(false && "Speed mode not supported!"));
            return i2cTimingsStd;
    }
}

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
        scl.mode(miosix::Mode::ALTERNATE_OD_PULL_UP);
        sda.mode(miosix::Mode::ALTERNATE_OD_PULL_UP);
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

    // Calling getTimings so that all his static variables are initialized
    getTimings(I2CDriver::Speed::STANDARD);

    // Enabling the peripheral after initialization
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

    // enabling only the RX interrupts
    i2c->CR1 &= ~I2C_CR1_TXIE;
    i2c->CR1 |= I2C_CR1_RXIE;

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

    // enabling only the TX interrupts
    i2c->CR1 &= ~I2C_CR1_RXIE;
    i2c->CR1 |= I2C_CR1_TXIE;

    // Sending doOperation when the channel isn't busy
    return doOperation(slaveConfig);
};

bool I2CDriver::doOperation(const I2CSlaveConfig &slaveConfig)
{
    // 10-bit addressing not supported
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

    // Now proceeding as every other transaction (re-starting specific code
    // already executed)
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
        // [WARNING]: In F7 the START condition is not generated immediately
        // when set the flag, but the peripheral waits that the bus is free for
        // sending the START condition + slave address. This could be bad since
        // the peripheral isn't deterministic in time.
        i2c->CR2 |= I2C_CR2_START;

        // Setting automatic stop generation if we have to generate STOP
        // condition. This MUST be done after the START condition generation
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
    // Disabling the I2C peripheral before setting the registers.
    // This will also perform a software reset, useful to restore the internal
    // state machines and flags in order to start with a clear environment
    i2c->CR1 &= ~I2C_CR1_PE;

    const auto i2cTimings = getTimings(slaveConfig.speed);

    // Setting PRESC, SCLH, SCLL, SCLDEL and SDADEL bits in I2C_TIMINGR register
    // to generate correct timings for each speed mode
    i2c->TIMINGR = (i2cTimings.presc << I2C_TIMINGR_PRESC_Pos) |    // PRESC
                   (i2cTimings.scll << I2C_TIMINGR_SCLL_Pos) |      // SCLL
                   (i2cTimings.sclh << I2C_TIMINGR_SCLH_Pos) |      // SCLH
                   (i2cTimings.scldel << I2C_TIMINGR_SCLDEL_Pos) |  // SCLDEL
                   (i2cTimings.sdadel << I2C_TIMINGR_SDADEL_Pos);   // SDADEL

    // setting addressing mode and slave address (for 7-Bit addressing the 7
    // bits have to be set on SADD[7:1])
    i2c->CR2 =
        (slaveConfig.addressing == Addressing::BIT10 ? I2C_CR2_ADD10 : 0) |
        (slaveConfig.slaveAddress << (I2C_CR2_SADD_Pos + 1));

    // Re-enabling the peripheral
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
    // If we are left with a communication of less than 256 bytes, we can do it
    // without reloading, otherwise we must perform the transaction with the
    // first 255 bytes and then perform a reload operation
    if ((transaction.nBytes - transaction.nBytesDone) <= 0xffu)
    {
        i2c->CR2 =
            (i2c->CR2 & ~(I2C_CR2_NBYTES_Msk |
                          I2C_CR2_RELOAD)) |  // clearing NBYTES and RELOAD
            ((transaction.nBytes - transaction.nBytesDone)
             << I2C_CR2_NBYTES_Pos);  // Reloading right number of bytes
    }
    else
    {
        i2c->CR2 |= (I2C_CR2_RELOAD |      // There must be a reload
                     I2C_CR2_NBYTES_Msk);  // maximum bytes that can be sent
    }
}

void I2CDriver::flushBus()
{
    // If there isn't any locked state or bus error return immediately
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
        scl.mode(miosix::Mode::OPEN_DRAIN_PULL_UP);
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
        scl.mode(miosix::Mode::ALTERNATE_OD_PULL_UP);
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
        // other fields. Set to force an error
        error = 1 << 14;
        lastError |= Errors::AF;
        i2c->ICR |= I2C_ICR_NACKCF;
        IRQwakeUpWaitingThread();
        return;
    }

    // Transfer complete reload: setting registers for the remaining bytes
    if (i2c->ISR & I2C_ISR_TCR)
    {
        setupReload();
    }

    if (transaction.nBytesDone < transaction.nBytes)
    {
        // Performing the read/write
        if (i2c->ISR & I2C_ISR_RXNE)
        {
            // READ
            transaction.buffRead[transaction.nBytesDone] =
                static_cast<uint8_t>(i2c->RXDR);
            transaction.nBytesDone++;
        }
        else if (i2c->ISR & (I2C_ISR_TXIS | I2C_ISR_TXE))
        {
            // WRITE
            i2c->TXDR = static_cast<uint32_t>(
                transaction.buffWrite[transaction.nBytesDone]);
            transaction.nBytesDone++;
        }
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