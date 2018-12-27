/* Copyright (c) 2015-2018 Skyward Experimental Rocketry
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

#include "LogProxy.h"

#include <boards/Nosecone/FSM/NoseconeManagerStatus.h>
#include <boards/Nosecone/Motor/MotorStatus.h>
#include <boards/Nosecone/Motor/CurrentSensor/CurrentStatus.h>
#include <boards/Nosecone/Canbus/CanStatus.h>

template <>
LogResult LoggerProxy::log<NscMgrStatus>(const NscMgrStatus& t)
{
    miosix::PauseKernelLock kLock;

    status.close_received = t.close_received;
    status.close_timeout  = t.close_timeout;
    status.close_stop     = t.close_stop;
    status.close_limit    = t.close_limit;
    status.open_received  = t.open_received;
    status.open_timeout   = t.open_timeout;
    status.open_stop      = t.open_stop;
    status.open_limit     = t.open_limit;

    return logger.log(t);
}

template <>
LogResult LoggerProxy::log<MotorStatus>(const MotorStatus& t)
{
    miosix::PauseKernelLock kLock;

    status.motor_active          = t.motor_active;        
    status.motor_last_direction  = t.motor_last_direction;

    return logger.log(t);
}

template <>
LogResult LoggerProxy::log<CurrentStatus>(const CurrentStatus& t)
{
    miosix::PauseKernelLock kLock;

    status.max_current_sensed  = t.max_current_sensed;
    status.min_current_sensed  = t.min_current_sensed;
    status.last_current_sensed = t.last_current_sensed;

    return logger.log(t);
}

template <>
LogResult LoggerProxy::log<CanStatus>(const CanStatus& t)
{
    miosix::PauseKernelLock kLock;

    status.homeone_not_connected = t.homeone_not_connected;

    return logger.log(t);
}