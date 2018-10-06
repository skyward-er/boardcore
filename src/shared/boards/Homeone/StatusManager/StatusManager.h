/* Copyright (c) 2018 Skyward Experimental Rocketry
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

#ifndef SRC_SHARED_BOARDS_HOMEONE_STATUSMANAGER_STATUSMANAGER_H_
#define SRC_SHARED_BOARDS_HOMEONE_STATUSMANAGER_STATUSMANAGER_H_

#include <Common.h>
#include <events/Scheduler.h>
#include <events/EventBroker.h>
#include "Status.h"
#include "boards/Homeone/TMTCManager/TMTCManager.h"
#include <diagnostic/CpuMeter.h>

#define HR_TM_RATE 100
#define LR_TM_RATE 1000
#define AUTOTM_TIMEOUT 1000000

namespace HomeoneBoard
{
namespace Status
{

/**
 * This class is in charge of collecting Status information from the board's
 * SW Modules and sending them through the TMTC whenever a status request event
 * is received.
 */
class StatusManager: public EventHandler, public Singleton<StatusManager>
{
    friend class Singleton<StatusManager> ;

public:
    ~StatusManager(){};

protected:
    /* EventHandler implementation */
    void handleEvent(const Event& e) override;

private:
    /* Automatic Telemetries flag */
    bool autoTmEnable = false;

    /* Constructor*/
    StatusManager();

};

}
}

#ifndef sStatusManager
#define sStatusManager StatusManager::getInstance()
#else
#error STATUS MANAGER ALREADY DEFINED
#endif /* sStatusManager */

#endif /* SRC_SHARED_BOARDS_HOMEONE_STATUSMANAGER_STATUSMANAGER_H_ */

