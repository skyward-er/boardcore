/* Copyright (c) 2015-2018 Skyward Experimental Rocketry
 * Authors: Alvise de' Faveri Tron
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
#ifndef SRC_SHARED_BOARDS_HOMEONE_DPLCONTROLLER_FSM_H
#define SRC_SHARED_BOARDS_HOMEONE_DPLCONTROLLER_FSM_H

#include "Singleton.h"

#include "events/Event.h"
#include "events/FSM.h"

namespace HomeoneBoard
{
namespace DPL  // DeploymentController
{
/**
 * Implementation of the DeploymentController Finite State Machine
 */
class DeploymentController : public FSM<DeploymentController>,
                          public Singleton<DeploymentController>
{
    friend class Singleton<DeploymentController>;

private:
    DeploymentController();
    ~DeploymentController() {}

    void state_idle(const Event& e);

    void updateInternalState(uint8_t *can_msg);
    // State variables
    const uint8_t MAX_RETRY = 5;
};
}
}

#define sDeploymentController HomeoneBoard::DPL::DeploymentController::getInstance()

#endif /* SRC_SHARED_BOARDS_HOMEONE_DPLCONTROLLER_FSM_H */
