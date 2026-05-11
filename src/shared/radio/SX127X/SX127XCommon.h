#pragma once

#include <drivers/interrupt/external_interrupts.h>
#include <drivers/spi/SPIDriver.h>
#include <miosix.h>
#include <radio/Transceiver.h>

#include <cmath>
#include <memory>

#include "SX127XDefs.h"

namespace Boardcore
{

namespace SX127x
{

static constexpr auto MIN_IRQ_PRIORITY = miosix::PRIORITY_MAX - 2;

using DioMapping = RegDioMapping::Mapping;

class ISX127X : public Transceiver
{
public:
    virtual ~ISX127X() = default;

    virtual void handleDioIRQ() = 0;
    virtual float getLastRxRssi() = 0;
    virtual float getLastRxFei() { return std::nanf(""); }
    virtual float getLastRxSnr() { return std::nanf(""); }
    virtual bool sendTimeout(uint8_t* packet, size_t packetLength,
                             int timeoutMs) = 0;
    virtual ssize_t receiveTimeout(uint8_t* packet, size_t packetLength,
                                   int timeoutMs) = 0;
    virtual bool setFrequency(uint32_t freqHz) = 0;

protected:
    using IrqFlags = int;
    using Mode     = int;

    virtual void setMode(Mode mode)             = 0;
    virtual void setMapping(DioMapping mapping) = 0;

    virtual IrqFlags getIrqFlags()             = 0;
    virtual void resetIrqFlags(IrqFlags flags) = 0;
};

class ISX127XFrontend
{
public:
    virtual ~ISX127XFrontend() = default;

    virtual bool isOnPaBoost() = 0;
    virtual int maxInPower()   = 0;

    virtual void enableRx()  = 0;
    virtual void disableRx() = 0;
    virtual void enableTx()  = 0;
    virtual void disableTx() = 0;
};

class SX127XCommon : public ISX127X
{
private:
    struct DeviceState
    {
        Mode mode                        = -1;
        DioMapping mapping               = DioMapping();
        miosix::Thread* irq_wait_thread  = nullptr;
        bool is_rx_frontend_on           = false;
        bool is_tx_frontend_on           = false;
        InterruptTrigger dio1_trigger    = InterruptTrigger::RISING_EDGE;
    };

    static constexpr int IRQ_TIMEOUT = 100;

public:
    void handleDioIRQ() override;

    bool setFrequency(uint32_t freqHz) override;

protected:
    explicit SX127XCommon(SPIBus& bus, miosix::GpioPin cs, miosix::GpioPin dio0,
                          miosix::GpioPin dio1, miosix::GpioPin dio3,
                          SPI::ClockDivider clock_divider,
                          std::unique_ptr<ISX127XFrontend> frontend)
        : slave(SPISlave(bus, cs, getSpiBusConfig(clock_divider))), dio0(dio0),
          dio1(dio1), dio3(dio3), frontend(std::move(frontend))
    {
        enableIrqs();
    }

    ~SX127XCommon() { disableIrqs(); }

    class Lock
    {
    public:
        explicit Lock(SX127XCommon& driver) : driver(driver) { driver.lock(); }

        ~Lock() { driver.unlock(); }

    private:
        SX127XCommon& driver;
    };

    class LockMode
    {
    public:
        LockMode(SX127XCommon& driver, Lock& lock, Mode mode,
                 DioMapping mapping, InterruptTrigger dio1_trigger,
                 bool set_tx_frontend_on = false,
                 bool set_rx_frontend_on = false)
            : driver(driver), lock(lock)
        {
            old_state = driver.lockMode(mode, mapping, dio1_trigger,
                                        set_tx_frontend_on, set_rx_frontend_on);
        }

        ~LockMode() { driver.unlockMode(old_state); }

    private:
        SX127XCommon& driver;
        Lock& lock;
        DeviceState old_state;
    };

    void setDefaultMode(Mode mode, DioMapping mapping,
                        InterruptTrigger dio1_trigger, bool set_tx_frontend_on,
                        bool set_rx_frontend_on);

    IrqFlags waitForIrq(LockMode& guard, IrqFlags set_irq, IrqFlags reset_irq,
                        bool unlock = false);

    IrqFlags waitForIrqBusy(LockMode& guard, IrqFlags set_irq,
                            IrqFlags reset_irq, int timeout);

    IrqFlags waitForIrqTimeout(LockMode& guard, IrqFlags set_irq,
                               IrqFlags reset_irq, int timeout_ms,
                               bool unlock = false);

    IrqFlags checkForIrqAndReset(IrqFlags set_irq, IrqFlags reset_irq);

    ISX127XFrontend& getFrontend();

    SPISlave& getSpiSlave();

private:
    void enableIrqs();
    void disableIrqs();

    bool waitForIrqInner(LockMode& guard, bool unlock, int timeout_ms);

    DeviceState lockMode(Mode mode, DioMapping mapping,
                         InterruptTrigger dio1_trigger, bool set_tx_frontend_on,
                         bool set_rx_frontend_on);
    void unlockMode(DeviceState old_state);

    static constexpr Mode MODE_STDBY = 0b001;

    void lock();
    void unlock();

    void enterMode(Mode mode, DioMapping mapping, InterruptTrigger dio1_trigger,
                   bool set_tx_frontend_on, bool set_rx_frontend_on);

    miosix::FastMutex mutex;
    DeviceState state;
    SPISlave slave;
    miosix::GpioPin dio0;
    miosix::GpioPin dio1;
    miosix::GpioPin dio3;
    std::unique_ptr<ISX127XFrontend> frontend;
};

}  // namespace SX127x

}  // namespace Boardcore
