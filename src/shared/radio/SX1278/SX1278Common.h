/* Copyright (c) 2022 Skyward Experimental Rocketry
 * Author: Davide Mor
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

#pragma once

#include <drivers/spi/SPIDriver.h>
#include <miosix.h>
#include <radio/Transceiver.h>

#include <memory>

#include "SX1278Defs.h"

namespace Boardcore
{

namespace SX1278
{

/**
 * @brief Represents a set of Dio
 */
class DioMask
{
public:
    DioMask() : mask(0) {}

    bool test(Dio dio) const
    {
        return (mask & (1 << static_cast<int>(dio))) != 0;
    }
    void set(Dio dio) { mask |= (1 << static_cast<int>(dio)); }
    void reset(Dio dio) { mask &= ~(1 << static_cast<int>(dio)); }

private:
    uint8_t mask;
};

using DioMapping = RegDioMapping::Mapping;

/**
 * @brief Shared interface between all SX1278 drivers
 */
class ISX1278 : public Transceiver
{
protected:
    /*
     * Stuff used internally by SX1278Common
     */

    using IrqFlags = int;
    using Mode     = int;

    virtual void setMode(Mode mode)             = 0;
    virtual void setMapping(DioMapping mapping) = 0;

    virtual IrqFlags getIrqFlags()             = 0;
    virtual void resetIrqFlags(IrqFlags flags) = 0;

    virtual DioMask getDioMaskFromIrqFlags(IrqFlags flags, Mode mode,
                                           DioMapping mapping) = 0;
};

/**
 * @brief Shared interface between all SX1278 frontends
 */
class ISX1278Frontend
{
public:
    /**
     * @brief Is this frontend connected to PA_BOOST or RFO_LF/_HF?
     */
    virtual bool isOnPaBoost() = 0;

    /**
     * @brief What is the maximum power supported by this frontend?
     */
    virtual int maxInPower() = 0;

    virtual void enableRx()  = 0;
    virtual void disableRx() = 0;
    virtual void enableTx()  = 0;
    virtual void disableTx() = 0;
};

class SX1278Common : public ISX1278
{
private:
    struct DeviceState
    {
        // Current device mode (dummy number to signal no mode)
        Mode mode = -1;
        // Current Dio mapping
        DioMapping mapping = DioMapping();
        // Thread waiting listening for interrupts
        miosix::Thread *irq_wait_thread = nullptr;
        // What DIOs are we waiting on
        DioMask waiting_dio_mask = DioMask();
        // True if the RX frontend is enabled
        bool is_rx_frontend_on = false;
        // True if the TX frontend is enabled
        bool is_tx_frontend_on = false;
    };

public:
    using Dio = SX1278::Dio;

    /**
     * @brief Handle generic DIO irq.
     */
    void handleDioIRQ(Dio dio);

protected:
    explicit SX1278Common(SPIBus &bus, miosix::GpioPin cs,
                          SPI::ClockDivider clock_divider,
                          std::unique_ptr<ISX1278Frontend>)
        : slave(SPISlave(bus, cs, getSpiBusConfig(clock_divider)))
    {
    }

    /**
     * @brief RAII scoped bus lock guard.
     */
    class Lock
    {
    public:
        explicit Lock(SX1278Common &driver) : driver(driver) { driver.lock(); }

        ~Lock() { driver.unlock(); }

    private:
        SX1278Common &driver;
    };

    /**
     * @brief RAII scoped mode lock, requires a previous lock.
     */
    class LockMode
    {
    public:
        LockMode(SX1278Common &driver, Lock &lock, Mode mode,
                 DioMapping mapping, bool set_tx_frontend_on = false,
                 bool set_rx_frontend_on = false)
            : driver(driver), lock(lock)
        {
            // cppcheck-suppress useInitializationList
            old_state = driver.lockMode(mode, mapping, set_tx_frontend_on,
                                        set_rx_frontend_on);
        }

        ~LockMode() { driver.unlockMode(old_state); }

    private:
        SX1278Common &driver;
        Lock &lock;
        DeviceState old_state;
    };

    /**
     * @brief Set default device mode.
     *
     * WARNING: This will lock the mutex.
     */
    void setDefaultMode(Mode mode, DioMapping mapping, bool set_tx_frontend_on,
                        bool set_rx_frontend_on);

    /**
     * @brief Wait for generic irq.
     */
    void waitForIrq(LockMode &guard, IrqFlags irq, bool unlock = false);

    /**
     * @brief Busy waits for an interrupt by polling the irq register.
     *
     * USE ONLY DURING INITIALIZATION! BAD THINGS *HAVE* HAPPENED DUE TO THIS!
     */
    bool waitForIrqBusy(LockMode &guard, IrqFlags irq, int timeout);

    /**
     * @brief Returns if an interrupt happened, and clears it if it did.
     */
    bool checkForIrqAndReset(IrqFlags irq);

    ISX1278Frontend &getFrontend();

    SPISlave &getSpiSlave();

private:
    DeviceState lockMode(Mode mode, DioMapping mapping, bool set_tx_frontend_on,
                         bool set_rx_frontend_on);
    void unlockMode(DeviceState old_state);

    void lock();
    void unlock();

    void enterMode(Mode mode, DioMapping mapping, bool set_tx_frontend_on,
                   bool set_rx_frontend_on);

    miosix::FastMutex mutex;
    DeviceState state;
    std::unique_ptr<ISX1278Frontend> frontend;
    SPISlave slave;
};

}  // namespace SX1278

}  // namespace Boardcore
