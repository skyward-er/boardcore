/*
 * Copyright (c) 2018 Skyward Experimental Rocketry
 * Authors: Alvise de'Faveri Tron
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
#pragma once

#include <Common.h>

#include "MotorConfig.h"
#include "CurrentSensor/MotorSensor.h"
#include "MotorStatus.h"

#include <PinObserver.h>
#include <boards/Nosecone/LogProxy/LogProxy.h>

namespace NoseconeBoard
{

/**
 * This class gives access to the H-Bridge that controls the DC motor of the nosecone.
 */
class MotorDriver
{

public:
    /**
     * @brief Class constructor.
     * 
     * @param pinObs    needed to observe the motor limit pins (finecorsa) 
     */
    MotorDriver(PinObserver* pinObs);

    /**
     * @brief Class destructor.
     */
    ~MotorDriver();

    /**
     * @brief Activates the H-Bridge to start the motor. The motor will send an
     *        event when the limit is reached.
     *
     * @param direction     direction of activation (normal or reverse)
     * @param dutyCycle     duty cycle of the PWM sent to the motor
     * @return              false if the motor is already started
     */
    bool start(MotorDirection direction, float dutyCycle);

    /**
     * @brief Stop the motor
     */
    void stop();

    /**
     * @brief Save the status.
     */
    inline void log()
    {
        Singleton<LoggerProxy>::getInstance()->log(status);
    }

private:
    PWM pwm;
    MotorSensor currentSensor;

    MotorStatus status;
};

} /* namespace  */