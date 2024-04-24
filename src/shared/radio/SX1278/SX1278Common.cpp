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

#include "SX1278Common.h"

#include <kernel/scheduler/scheduler.h>
#include <utils/KernelTime.h>

namespace Boardcore
{

namespace SX1278
{

void SX1278Common::handleDioIRQ()
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

void SX1278Common::enableIrqs()
{
    enableExternalInterrupt(dio0, InterruptTrigger::RISING_EDGE);
    enableExternalInterrupt(dio1, InterruptTrigger::RISING_EDGE);
    enableExternalInterrupt(dio3, InterruptTrigger::RISING_EDGE);
}

void SX1278Common::disableIrqs()
{
    disableExternalInterrupt(dio0);
    disableExternalInterrupt(dio1);
    disableExternalInterrupt(dio3);
}

void SX1278Common::setDefaultMode(Mode mode, DioMapping mapping,
                                  InterruptTrigger dio1_trigger,
                                  bool tx_frontend, bool rx_frontend)
{
    miosix::Lock<miosix::FastMutex> lock(mutex);
    enterMode(mode, mapping, dio1_trigger, tx_frontend, rx_frontend);
}

ISX1278::IrqFlags SX1278Common::waitForIrq(LockMode &guard, IrqFlags set_irq,
                                           IrqFlags reset_irq, bool unlock)
{
    IrqFlags ret_irq = 0;

    do
    {
        // An interrupt could occur and read from this variables
        {
            miosix::FastInterruptDisableLock lock;
            state.irq_wait_thread = miosix::Thread::IRQgetCurrentThread();
        }

        // Check that this hasn't already happened
        if ((ret_irq = checkForIrqAndReset(set_irq, reset_irq)) != 0)
        {
            break;
        }

        if (!waitForIrqInner(guard, unlock))
        {
            // TODO: Something bad happened, do something!
        }

        // TODO: Check state of the device, and reset if needed!

        // Protect against sporadic IRQs
    } while ((ret_irq = checkForIrqAndReset(set_irq, reset_irq)) == 0);

    return ret_irq;
}

ISX1278::IrqFlags SX1278Common::waitForIrqBusy(LockMode &_guard,
                                               IrqFlags set_irq,
                                               IrqFlags reset_irq, int timeout)
{
    // Take a reference to a _guard to MAKE SURE that the mutex is locked, but
    // otherwise don't do anything with it
    (void)_guard;

    long long start  = Kernel::getOldTick();
    IrqFlags ret_irq = 0;

    while ((Kernel::getOldTick() - start) < timeout)
    {
        // Delay between polls
        const unsigned int DELAY = 100;

        // Tight loop on IRQ register
        for (unsigned int i = 0; i < 1000 / DELAY; i++)
        {
            // Check if some of the interrupts triggered
            if ((ret_irq = checkForIrqAndReset(set_irq, reset_irq)) != 0)
            {
                return ret_irq;
            }

            miosix::delayUs(DELAY);
        }
    }

    return 0;
}

bool SX1278Common::waitForIrqInner(LockMode &_guard, bool unlock)
{
    // Take a reference to a _guard to MAKE SURE that the mutex is locked, but
    // otherwise don't do anything with it
    (void)_guard;

    // Release the lock for others to take
    if (unlock)
    {
        mutex.unlock();
    }

    int start                      = Kernel::getOldTick();
    miosix::TimedWaitResult result = miosix::TimedWaitResult::NoTimeout;

    {
        miosix::FastInterruptDisableLock lock;
        while (state.irq_wait_thread &&
               result == miosix::TimedWaitResult::NoTimeout)
        {
            result = Kernel::Thread::IRQenableIrqAndTimedWaitMs(
                lock, start + IRQ_TIMEOUT);
        }
    }

    // Regain ownership of the lock
    if (unlock)
    {
        mutex.lock();
    }

    // Check that we didn't have a timeout
    return result == miosix::TimedWaitResult::NoTimeout;
}

ISX1278::IrqFlags SX1278Common::checkForIrqAndReset(IrqFlags set_irq,
                                                    IrqFlags reset_irq)
{
    IrqFlags cur_irq = getIrqFlags();
    if (cur_irq & set_irq)
    {
        // Reset all of the interrupts we have detected
        resetIrqFlags(cur_irq & set_irq);
    }

    return (cur_irq & set_irq) | (~cur_irq & reset_irq);
}

ISX1278Frontend &SX1278Common::getFrontend() { return *frontend; }

SPISlave &SX1278Common::getSpiSlave() { return slave; }

SX1278Common::DeviceState SX1278Common::lockMode(Mode mode, DioMapping mapping,
                                                 InterruptTrigger dio1_trigger,
                                                 bool tx_frontend,
                                                 bool rx_frontend)
{
    // Store previous state
    DeviceState old_state = state;

    enterMode(mode, mapping, dio1_trigger, tx_frontend, rx_frontend);
    state.irq_wait_thread = nullptr;

    return old_state;
}

void SX1278Common::unlockMode(DeviceState old_state)
{
    // Do this copy manually, we want stuff to be copied in a specific order
    state.irq_wait_thread = old_state.irq_wait_thread;
    enterMode(old_state.mode, old_state.mapping, old_state.dio1_trigger,
              old_state.is_tx_frontend_on, old_state.is_rx_frontend_on);
}

void SX1278Common::lock() { mutex.lock(); }

void SX1278Common::unlock() { mutex.unlock(); }

void SX1278Common::enterMode(Mode mode, DioMapping mapping,
                             InterruptTrigger dio1_trigger,
                             bool set_tx_frontend_on, bool set_rx_frontend_on)
{
    // disable - enable in order to avoid having both RX/TX frontends active at
    // the same time

    // First disable all of the frontend if necessary
    if (set_tx_frontend_on != state.is_tx_frontend_on && !set_tx_frontend_on)
    {
        getFrontend().disableTx();
    }

    if (set_rx_frontend_on != state.is_rx_frontend_on && !set_rx_frontend_on)
    {
        getFrontend().disableRx();
    }

    // Then enable the newly requested ones
    if (set_tx_frontend_on != state.is_tx_frontend_on && set_tx_frontend_on)
    {
        getFrontend().enableTx();
    }

    if (set_rx_frontend_on != state.is_rx_frontend_on && set_rx_frontend_on)
    {
        getFrontend().enableRx();
    }

    state.is_tx_frontend_on = set_tx_frontend_on;
    state.is_rx_frontend_on = set_rx_frontend_on;

    if (mode != state.mode)
    {
        setMode(mode);
        state.mode = mode;
    }

    // Change DIO1 interrupt kind
    if (dio1_trigger != state.dio1_trigger)
    {
        changeInterruptTrigger(dio1, dio1_trigger);
        state.dio1_trigger = dio1_trigger;
    }

    // Finally setup DIO mapping
    if (mapping != state.mapping)
    {
        setMapping(mapping);
        state.mapping = mapping;
    }
}

}  // namespace SX1278

}  // namespace Boardcore
