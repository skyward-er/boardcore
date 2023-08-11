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

#include <drivers/interrupt/external_interrupts.h>
#include <drivers/spi/SPIDriver.h>
#include <miosix.h>
#include <radio/Transceiver.h>

#include <cmath>
#include <memory>

#include "SX1278Defs.h"

namespace Boardcore
{

namespace SX1278
{

using DioMapping = RegDioMapping::Mapping;

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

class SX1278Common : public Transceiver
{
protected:
    using IrqFlags = int;
    using Mode     = int;

private:
    struct DeviceState
    {
        // Current device mode (dummy number to signal no mode)
        Mode mode = -1;
        // Current Dio mapping
        DioMapping mapping = DioMapping();
        // Thread waiting listening for interrupts
        miosix::Thread *irq_wait_thread = nullptr;
        // True if the RX frontend is enabled
        bool is_rx_frontend_on = false;
        // True if the TX frontend is enabled
        bool is_tx_frontend_on = false;
        // Mode of trigger for dio1
        InterruptTrigger dio1_trigger = InterruptTrigger::RISING_EDGE;
    };

    // This is reasonably the maximum we should wait for an interrupt
    static constexpr int IRQ_TIMEOUT = 100;

public:
    /**
     * @brief Handle generic DIO irq.
     */
    void handleDioIRQ();

    /**
     * @brief Get the number of times this device has been reset.
     */
    int resetCount();

    /**
     * @brief Get the RSSI in dBm, during last packet receive.
     */
    virtual float getLastRxRssi() = 0;

    /**
     * @brief Get the frequency error index in Hz, during last packet receive
     * (NaN if not available).
     */
    virtual float getLastRxFei() { return std::nanf(""); }

    /**
     * @brief Get the signal to noise ratio, during last packet receive (NaN if
     * not available).
     */
    virtual float getLastRxSnr() { return std::nanf(""); }

protected:
    explicit SX1278Common(SPIBus &bus, miosix::GpioPin cs, miosix::GpioPin dio0,
                          miosix::GpioPin dio1, miosix::GpioPin dio3,
                          SPI::ClockDivider clock_divider,
                          std::unique_ptr<ISX1278Frontend> frontend)
        : slave(SPISlave(bus, cs, getSpiBusConfig(clock_divider))), dio0(dio0),
          dio1(dio1), dio3(dio3), frontend(std::move(frontend))
    {
        enableIrqs();
    }

    ~SX1278Common() { disableIrqs(); }

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
        LockMode(SX1278Common &driver, Lock &guard, Mode mode,
                 DioMapping mapping, InterruptTrigger dio1_trigger,
                 bool set_tx_frontend_on = false,
                 bool set_rx_frontend_on = false)
            : driver(driver), guard(guard), reset_count(driver.reset_count)
        {
            // cppcheck-suppress useInitializationList
            old_state = driver.lockMode(guard, mode, mapping, dio1_trigger,
                                        set_tx_frontend_on, set_rx_frontend_on);
        }

        ~LockMode()
        {
            // Restore device mode ONLY if the lock is valid!
            if (!wasDeviceInvalidated())
            {
                driver.unlockMode(guard, old_state);
            }
        }

        /**
         * @brief Returns the parent simple lock.
         */
        Lock &parent() { return guard; }

        /**
         * @brief Returns true if this LockMode is still valid, and the device
         * hasn't failed while holding the lock.
         */
        bool wasDeviceInvalidated()
        {
            return driver.reset_count != reset_count;
        }

    private:
        SX1278Common &driver;
        Lock &guard;
        int reset_count;
        DeviceState old_state;
    };

    /**
     * @brief Set default device mode.
     *
     * WARNING: This will lock the mutex.
     */
    void setDefaultMode(Lock &guard, Mode mode, DioMapping mapping,
                        InterruptTrigger dio1_trigger, bool set_tx_frontend_on,
                        bool set_rx_frontend_on);

    /**
     * @brief Wait for generic irq.
     */
    IrqFlags waitForIrq(LockMode &guard, IrqFlags set_irq, IrqFlags reset_irq,
                        bool unlock = false);

    /**
     * @brief Busy waits for an interrupt by polling the irq register.
     *
     * USE ONLY DURING INITIALIZATION! BAD THINGS *HAVE* HAPPENED DUE TO THIS!
     */
    IrqFlags waitForIrqBusy(LockMode &guard, IrqFlags set_irq,
                            IrqFlags reset_irq, int timeout);

    /**
     * @brief Returns a mask containing triggered interrupts.
     *
     * NOTE: This function checks both set irqs (rising edge), and reset irqs
     * (falling edge). But it only resets set interrupts.
     *
     * @param set_irq Mask containing set (rising edge) interrupts.
     * @param reset_irq Mask containing reset (falling edge) interrupts.
     * @return Mask containing all triggered interrupts (both rising and
     * falling)
     */
    IrqFlags checkForIrqAndReset(LockMode &guard, IrqFlags set_irq,
                                 IrqFlags reset_irq);

    ISX1278Frontend &getFrontend();

    SPISlave &getSpiSlave(Lock &guard);

protected:
    virtual bool checkDeviceFailure(Lock &guard) = 0;
    virtual void reconfigure(Lock &guard)        = 0;

    virtual void setMode(Lock &guard, Mode mode)             = 0;
    virtual void setMapping(Lock &guard, DioMapping mapping) = 0;

    virtual IrqFlags getIrqFlags(Lock &guard)               = 0;
    virtual void resetIrqFlags(Lock &guard, IrqFlags flags) = 0;

private:
    void enableIrqs();
    void disableIrqs();

    bool waitForIrqInner(LockMode &guard, bool unlock);

    DeviceState lockMode(Lock &guard, Mode mode, DioMapping mapping,
                         InterruptTrigger dio1_trigger, bool set_tx_frontend_on,
                         bool set_rx_frontend_on);
    void unlockMode(Lock &guard, DeviceState old_state);

    void lock();
    void unlock();

    void enterMode(Lock &guard, Mode mode, DioMapping mapping,
                   InterruptTrigger dio1_trigger, bool set_tx_frontend_on,
                   bool set_rx_frontend_on);

    miosix::FastMutex mutex;
    DeviceState state;
    SPISlave slave;
    miosix::GpioPin dio0;
    miosix::GpioPin dio1;
    miosix::GpioPin dio3;
    std::unique_ptr<ISX1278Frontend> frontend;
    int reset_count = 0;
};

}  // namespace SX1278

}  // namespace Boardcore
