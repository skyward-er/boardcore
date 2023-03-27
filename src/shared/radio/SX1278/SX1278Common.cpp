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

namespace Boardcore
{

namespace SX1278
{

void SX1278Common::handleDioIRQ(Dio dio)
{
    if (state.waiting_dio_mask.test(dio) && state.irq_wait_thread)
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

void SX1278Common::setDefaultMode(Mode mode, DioMapping mapping,
                                  bool tx_frontend, bool rx_frontend)
{
    mutex.lock();
    enterMode(mode, mapping, tx_frontend, rx_frontend);
    mutex.unlock();
}

void SX1278Common::waitForIrq(LockMode &_guard, IrqFlags irq, bool unlock)
{
    // Take a reference to a _guard to MAKE SURE that the mutex is locked, but
    // otherwise don't do anything with it
    (void)_guard;

    // Convert the IRQ mask into a DIO mask
    DioMask waiting_dio_mask =
        getDioMaskFromIrqFlags(irq, state.mode, state.mapping);

    do
    {
        // An interrupt could occur and read from this variables
        {
            miosix::FastInterruptDisableLock dLock;
            state.waiting_dio_mask = waiting_dio_mask;
            state.irq_wait_thread  = miosix::Thread::IRQgetCurrentThread();
        }

        // Check that this hasn't already happened
        if (checkForIrqAndReset(irq))
        {
            return;
        }

        if (unlock)
        {
            mutex.unlock();
        }

        {
            miosix::FastInterruptDisableLock dLock;
            while (state.irq_wait_thread)
            {
                miosix::Thread::IRQwait();
                {
                    miosix::FastInterruptEnableLock eLock(dLock);
                    miosix::Thread::yield();
                }
            }
        }

        // Regain ownership of the lock
        if (unlock)
        {
            mutex.lock();
        }

        // Protect against sporadic IRQs
    } while (!checkForIrqAndReset(irq));
}

bool SX1278Common::waitForIrqBusy(LockMode &_guard, IrqFlags irq, int timeout)
{
    // Take a reference to a _guard to MAKE SURE that the mutex is locked, but
    // otherwise don't do anything with it
    (void)_guard;

    long long start = miosix::IRQgetTime() / 1e6;

    while ((miosix::IRQgetTime() / 1e6 - start) < timeout)
    {
        // Delay between polls
        const unsigned int DELAY = 100;

        // Tight loop on IRQ register
        for (unsigned int i = 0; i < 1000 / DELAY; i++)
        {
            if (checkForIrqAndReset(irq))
            {
                return true;
            }

            miosix::delayUs(DELAY);
        }
    }

    return false;
}

bool SX1278Common::checkForIrqAndReset(IrqFlags irq)
{
    IrqFlags cur_irq = getIrqFlags();
    if (cur_irq & irq)
    {
        // Reset all of the interrupts we have detected
        resetIrqFlags(cur_irq & irq);

        return true;
    }
    else
    {
        return false;
    }
}

SX1278Common::DeviceState SX1278Common::lockMode(Mode mode, DioMapping mapping,
                                                 bool tx_frontend,
                                                 bool rx_frontend)
{
    // Store previous state
    DeviceState old_state = state;

    enterMode(mode, mapping, tx_frontend, rx_frontend);
    state.irq_wait_thread  = nullptr;
    state.waiting_dio_mask = DioMask();

    return old_state;
}

void SX1278Common::unlockMode(DeviceState old_state)
{
    // Do this copy manually, we want stuff to be copied in a specific order
    state.irq_wait_thread  = old_state.irq_wait_thread;
    state.waiting_dio_mask = old_state.waiting_dio_mask;
    enterMode(old_state.mode, old_state.mapping, old_state.is_tx_frontend_on,
              old_state.is_rx_frontend_on);
}

void SX1278Common::lock() { mutex.lock(); }

void SX1278Common::unlock() { mutex.unlock(); }

void SX1278Common::enterMode(Mode mode, DioMapping mapping,
                             bool set_tx_frontend_on, bool set_rx_frontend_on)
{
    // disable - enable in order to avoid having both RX/TX frontends active at
    // the same time

    // First disable all of the frontend if necessary
    if (set_tx_frontend_on != state.is_tx_frontend_on && !set_tx_frontend_on)
    {
        disableTxFrontend();
    }

    if (set_rx_frontend_on != state.is_rx_frontend_on && !set_rx_frontend_on)
    {
        disableRxFrontend();
    }

    // Then enable the newly requested ones
    if (set_tx_frontend_on != state.is_tx_frontend_on && set_tx_frontend_on)
    {
        enableTxFrontend();
    }

    if (set_rx_frontend_on != state.is_rx_frontend_on && set_rx_frontend_on)
    {
        enableRxFrontend();
    }

    state.is_tx_frontend_on = set_tx_frontend_on;
    state.is_rx_frontend_on = set_rx_frontend_on;

    // Check if necessary
    if (mode != state.mode)
    {
        setMode(mode);
        state.mode = mode;
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
