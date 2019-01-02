/* Copyright (c) 2015-2018 Skyward Experimental Rocketry
 * Authors: Benedetta Margrethe Cattani, Alvise de' Faveri Tron
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

#include <events/FSM.h>

#include <boards/Nosecone/Motor/MotorDriver.h>
#include <boards/Nosecone/LogProxy/LogProxy.h>

#include "NoseconeManagerStatus.h"

namespace NoseconeBoard
{
/**
 * Implementation of the Nosecone Manager Finite State Machine
 */
class NoseconeManager : public FSM<NoseconeManager>
{

public:
    NoseconeManager(MotorDriver& motor);
    ~NoseconeManager() {}

private:
    MotorDriver& motor;
    uint16_t delayedId;

    NscMgrStatus status;

    static const unsigned int OPEN_TIMEOUT = 15000;
    static const unsigned int CLOSE_TIMEOUT = 10000;

    /* States declarations */
    void state_idle(const Event& e);
    void state_opening(const Event& e);
    void state_closing(const Event& e);

    inline void log()
    {
        Singleton<LoggerProxy>::getInstance()->log(status);
    }
};

}
