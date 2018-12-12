/*
 * Copyright (c) 2018 Skyward Experimental Rocketry
 * Authors: Luca Erbetta
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
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.  IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 */

#ifndef SRC_SHARED_BOARDS_HELITEST_THERMALCUTTER_PWMCUTTER_H
#define SRC_SHARED_BOARDS_HELITEST_THERMALCUTTER_PWMCUTTER_H

#include <miosix.h>

#include "../Events.h"
#include "CutterStatus.h"
#include "debug.h"
#include "drivers/pwm/pwm.h"
#include "events/FSM.h"
#include "logger/Logger.h"
#include "boards/HeliTest/config.h"

using miosix::Gpio;
using miosix::Mode;

template <class INH, class IN>
class PWMThermalCutter : public FSM<PWMThermalCutter<INH, IN>>
{
    using ThermalCutterType = PWMThermalCutter<INH, IN>;

public:
    PWMThermalCutter(PWM* pwm, int channel, unsigned int frequency,
                     float duty_cycle)
        : FSM<ThermalCutterType>(&ThermalCutterType::stateIdle), pwm(pwm),
          frequency(frequency), channel(channel), duty_cycle(duty_cycle)
    {
        INH::mode(Mode::OUTPUT);

        {
            FastInterruptDisableLock dLock;

            IN::alternateFunction(2);
            IN::mode(Mode::ALTERNATE);
        }

        INH::low();

        sEventBroker->subscribe(this, TOPIC_COORDINATION);
    }

protected:
    void handleEvent(const Event& ev) override
    {
        FSM<ThermalCutterType>::handleEvent(ev);

        sUpdateDiagnostics(
            StackData{miosix::getTick(), THREAD_CUTTER,
                      miosix::MemoryProfiling::getCurrentFreeStack(),
                      miosix::MemoryProfiling::getAbsoluteFreeStack()});
    }

private:
    void cutterOn()
    {
        TRACE("Cutter ON\n");
        pwm->start();

        INH::high();
    }

    void cutterOff()
    {
        TRACE("Cutter OFF\n");
        pwm->stop();
        miosix::Thread::sleep(BRIDGE_DISCHARGE_TIME_MS);
        INH::low();
    }

    void stateIdle(const Event& ev)
    {
        switch (ev.sig)
        {
            case EV_ENTRY:
                status.state = CUTTER_IDLE;
                logStatus();
                TRACE("[CUT] Entering idle state.\n");

                pwm->setDutyCycle(duty_cycle);
                pwm->enable(channel, frequency);
                break;
            case EV_START_CUTTING:
                this->transition(&ThermalCutterType::stateCutting);
                break;

            case EV_EXIT:
                break;
        }
    }

    void stateCutting(const Event& ev)
    {
        switch (ev.sig)
        {
            case EV_ENTRY:
                status.state = CUTTER_CUTTING;
                logStatus();
                TRACE("[CUT] Entering cutting state.\n");

                cutterOn();

                break;
            case EV_STOP_CUTTING:
                this->transition(&ThermalCutterType::stateEnded);
                break;
            case EV_EXIT:
                cutterOff();
                break;
        }
    }

    void stateEnded(const Event& ev)
    {
        switch (ev.sig)
        {
            case EV_ENTRY:
                status.state = CUTTER_DONE;
                logStatus();
                TRACE("[CUT] Entering done state.\n");

                pwm->disable();

                break;
            case EV_EXIT:
                break;
        }
    }

    void logStatus()
    {
        status.timestamp = miosix::getTick();
        Logger::instance().log(status);
    }

    Logger& logger = Logger::instance();
    CutterStatus status;
    PWM* pwm;

    unsigned int frequency;
    int channel;
    float duty_cycle;
};

#endif