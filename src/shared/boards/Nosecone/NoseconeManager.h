/* Copyright (c) 2015-2019 Skyward Experimental Rocketry
 * Authors: Benedetta Margrethe Cattani
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
#ifndef SRC_SHARED_BOARDS_HOMEONE_FLIGHTMODEMANAGER_FSM_H
#define SRC_SHARED_BOARDS_HOMEONE_FLIGHTMODEMANAGER_FSM_H

#include "Singleton.h"
#include "MotorDriver.h"

#include "events/Event.h"
#include "Events.h"
#include "events/FSM.h"

namespace NoseconeBoard
{
namespace FMM  // Flight Mode Manager
{
/**
 * Implementation of the Nosecone Manager Finite State Machine
 */
class NoseconeManager : public FSM<NoseconeManager>,
                          public Singleton<NoseconeManager>
{
    friend class Singleton<NoseconeManager>;

private:
    NoseconeManager();
    ~NoseconeManager() {}

    // States declarations

    void state_close(const Event& e);
    void state_opening(const Event& e);
    void state_open(const Event& e);
    void state_closing(const Event& e);

    MotorDriver driver;

};
}
}

#define sNoseconeManager NoseconeManager::getInstance()

#endif /* SRC_SHARED_BOARDS_HOMEONE_FLIGHTMODEMANAGER_FSM_H */
