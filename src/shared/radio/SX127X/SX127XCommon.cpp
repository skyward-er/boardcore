#include "SX127XCommon.h"

#include <kernel/scheduler/scheduler.h>
#include <utils/KernelTime.h>

namespace Boardcore
{

namespace SX127x
{

void SX127XCommon::handleDioIRQ()
{
    if (state.irq_wait_thread)
    {
        state.irq_wait_thread->IRQwakeup();
        if (state.irq_wait_thread->IRQgetPriority() >
            miosix::Thread::IRQgetCurrentThread()->IRQgetPriority())
        {
            miosix::Scheduler::IRQfindNextThread();
        }

        state.irq_wait_thread = nullptr;
    }
}

void SX127XCommon::enableIrqs()
{
    enableExternalInterrupt(dio0, InterruptTrigger::RISING_EDGE);
    enableExternalInterrupt(dio1, InterruptTrigger::RISING_EDGE);
    enableExternalInterrupt(dio3, InterruptTrigger::RISING_EDGE);
}

void SX127XCommon::disableIrqs()
{
    disableExternalInterrupt(dio0);
    disableExternalInterrupt(dio1);
    disableExternalInterrupt(dio3);
}

void SX127XCommon::setDefaultMode(Mode mode, DioMapping mapping,
                                  InterruptTrigger dio1_trigger,
                                  bool tx_frontend, bool rx_frontend)
{
    miosix::Lock<miosix::FastMutex> lock(mutex);
    enterMode(mode, mapping, dio1_trigger, tx_frontend, rx_frontend);
}

SX127XCommon::IrqFlags SX127XCommon::waitForIrq(LockMode& guard,
                                                IrqFlags set_irq,
                                                IrqFlags reset_irq,
                                                bool unlock)
{
    IrqFlags ret_irq = 0;

    auto oldPriority = miosix::Thread::getCurrentThread()->getPriority();

    if (oldPriority < MIN_IRQ_PRIORITY)
        miosix::Thread::getCurrentThread()->setPriority(MIN_IRQ_PRIORITY);

    do
    {
        {
            miosix::FastInterruptDisableLock lock;
            state.irq_wait_thread = miosix::Thread::IRQgetCurrentThread();
        }

        if ((ret_irq = checkForIrqAndReset(set_irq, reset_irq)) != 0)
            break;

        (void)waitForIrqInner(guard, unlock, IRQ_TIMEOUT);

    } while ((ret_irq = checkForIrqAndReset(set_irq, reset_irq)) == 0);

    if (oldPriority < MIN_IRQ_PRIORITY)
        miosix::Thread::getCurrentThread()->setPriority(oldPriority);

    return ret_irq;
}

SX127XCommon::IrqFlags SX127XCommon::waitForIrqBusy(LockMode& guard,
                                                    IrqFlags set_irq,
                                                    IrqFlags reset_irq,
                                                    int timeout)
{
    (void)guard;

    long long start  = Kernel::getOldTick();
    IrqFlags ret_irq = 0;

    while ((Kernel::getOldTick() - start) < timeout)
    {
        const unsigned int delayUs = 100;

        for (unsigned int i = 0; i < 1000 / delayUs; i++)
        {
            if ((ret_irq = checkForIrqAndReset(set_irq, reset_irq)) != 0)
                return ret_irq;

            miosix::delayUs(delayUs);
        }
    }

    return 0;
}

bool SX127XCommon::waitForIrqInner(LockMode& guard, bool unlock,
                                   int timeout_ms)
{
    (void)guard;

    if (unlock)
        mutex.unlock();

    miosix::TimedWaitResult result = miosix::TimedWaitResult::NoTimeout;
    const long long deadline       = Kernel::getOldTick() + timeout_ms;

    {
        miosix::FastInterruptDisableLock lock;
        while (state.irq_wait_thread &&
               result == miosix::TimedWaitResult::NoTimeout)
        {
            result =
                Kernel::Thread::IRQenableIrqAndTimedWaitMs(lock, deadline);
        }

        if (result == miosix::TimedWaitResult::Timeout)
            state.irq_wait_thread = nullptr;
    }

    if (unlock)
        mutex.lock();

    return result == miosix::TimedWaitResult::NoTimeout;
}

SX127XCommon::IrqFlags SX127XCommon::waitForIrqTimeout(LockMode& guard,
                                                       IrqFlags set_irq,
                                                       IrqFlags reset_irq,
                                                       int timeout_ms,
                                                       bool unlock)
{
    if (timeout_ms <= 0)
        return checkForIrqAndReset(set_irq, reset_irq);

    IrqFlags ret_irq = 0;
    const auto oldPriority =
        miosix::Thread::getCurrentThread()->getPriority();

    if (oldPriority < MIN_IRQ_PRIORITY)
        miosix::Thread::getCurrentThread()->setPriority(MIN_IRQ_PRIORITY);

    const long long deadline = Kernel::getOldTick() + timeout_ms;

    do
    {
        {
            miosix::FastInterruptDisableLock lock;
            state.irq_wait_thread = miosix::Thread::IRQgetCurrentThread();
        }

        if ((ret_irq = checkForIrqAndReset(set_irq, reset_irq)) != 0)
            break;

        const long long remaining = deadline - Kernel::getOldTick();
        if (remaining <= 0)
            break;

        const int wait_slice =
            remaining > IRQ_TIMEOUT ? IRQ_TIMEOUT : static_cast<int>(remaining);
        if (!waitForIrqInner(guard, unlock, wait_slice))
            break;

    } while ((ret_irq = checkForIrqAndReset(set_irq, reset_irq)) == 0);

    if (oldPriority < MIN_IRQ_PRIORITY)
        miosix::Thread::getCurrentThread()->setPriority(oldPriority);

    return ret_irq;
}

SX127XCommon::IrqFlags SX127XCommon::checkForIrqAndReset(IrqFlags set_irq,
                                                         IrqFlags reset_irq)
{
    IrqFlags cur_irq = getIrqFlags();
    if (cur_irq & set_irq)
    {
        resetIrqFlags(cur_irq & set_irq);
    }

    return (cur_irq & set_irq) | (~cur_irq & reset_irq);
}

ISX127XFrontend& SX127XCommon::getFrontend() { return *frontend; }

SPISlave& SX127XCommon::getSpiSlave() { return slave; }

SX127XCommon::DeviceState SX127XCommon::lockMode(Mode mode, DioMapping mapping,
                                                 InterruptTrigger dio1_trigger,
                                                 bool tx_frontend,
                                                 bool rx_frontend)
{
    DeviceState old_state = state;

    enterMode(mode, mapping, dio1_trigger, tx_frontend, rx_frontend);
    state.irq_wait_thread = nullptr;

    return old_state;
}

void SX127XCommon::unlockMode(DeviceState old_state)
{
    state.irq_wait_thread = old_state.irq_wait_thread;
    enterMode(old_state.mode, old_state.mapping, old_state.dio1_trigger,
              old_state.is_tx_frontend_on, old_state.is_rx_frontend_on);
}

void SX127XCommon::lock() { mutex.lock(); }

void SX127XCommon::unlock() { mutex.unlock(); }

void SX127XCommon::enterMode(Mode mode, DioMapping mapping,
                             InterruptTrigger dio1_trigger,
                             bool set_tx_frontend_on, bool set_rx_frontend_on)
{
    if (set_tx_frontend_on != state.is_tx_frontend_on && !set_tx_frontend_on)
        getFrontend().disableTx();

    if (set_rx_frontend_on != state.is_rx_frontend_on && !set_rx_frontend_on)
        getFrontend().disableRx();

    if (set_tx_frontend_on != state.is_tx_frontend_on && set_tx_frontend_on)
        getFrontend().enableTx();

    if (set_rx_frontend_on != state.is_rx_frontend_on && set_rx_frontend_on)
        getFrontend().enableRx();

    state.is_tx_frontend_on = set_tx_frontend_on;
    state.is_rx_frontend_on = set_rx_frontend_on;

    if (mode != state.mode)
    {
        setMode(mode);
        state.mode = mode;
    }

    if (dio1_trigger != state.dio1_trigger)
    {
        changeInterruptTrigger(dio1, dio1_trigger);
        state.dio1_trigger = dio1_trigger;
    }

    if (mapping != state.mapping)
    {
        setMapping(mapping);
        state.mapping = mapping;
    }
}

bool SX127XCommon::setFrequency(uint32_t freqHz)
{
    if (freqHz < MIN_FREQ_RF || freqHz > MAX_FREQ_RF)
        return false;

    constexpr double fstepHz = FXOSC / static_cast<double>(1u << 19);
    uint32_t frf = static_cast<uint32_t>(static_cast<double>(freqHz) / fstepHz);

    Lock guard(*this);
    LockMode guard_mode(*this, guard, MODE_STDBY, state.mapping,
                        state.dio1_trigger, state.is_tx_frontend_on,
                        state.is_rx_frontend_on);

    SPITransaction spi(getSpiSlave());
    spi.writeRegister(Fsk::REG_FRF_MSB, (frf >> 16) & 0xFF);
    spi.writeRegister(Fsk::REG_FRF_MID, (frf >> 8) & 0xFF);
    spi.writeRegister(Fsk::REG_FRF_LSB, frf & 0xFF);

    return true;
}

}  // namespace SX127x

}  // namespace Boardcore
