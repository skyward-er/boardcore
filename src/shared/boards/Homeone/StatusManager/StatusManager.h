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

#include "Status.h"

namespace HomeoneBoard
{
namespace Status
{

struct HomeoneStatus
{
	FMMStatus fmm;
	TMTCStatus tmtc;
	SensorManagerStatus sensors;
	LoggerStatus logger;
	IgnitionControllerStatus ignition;
	DeploymentControllerStatus deployment;
};

class StatusManager: public EventHandler, Singleton<StatusManager>
{
	friend class Singleton<StatusManager>;

public:
    ~StatusManager() {}

    HomeoneStatus status;

    void notifyUpdate(componentId_t compId)
    {
    	// TODO: disable interrupts, update timestamp, log status
    	switch(compId)
    	{
    		case FMM_COMP_ID:
    			// log(boardStatus.fmm)
    			printf("FMM status updated!\n");
    			break;
    		case TMTC_COMP_ID:
    			// log(boardStatus.tmtc)
    	    	printf("TMTC status updated!\n");
    			break;
    		case SENSOR_MANAGER_COMP_ID:
    	    	printf("SensorManager status updated!\n");
    			break;
    		case LOGGER_COMP_ID:
				printf("Logger status updated!\n");
    			break;
    		case IGNITION_CONTROLLER_COMP_ID:
				printf("IgnitionController status updated!\n");
    			break;
    		case DEPLOYMENT_CONTROLLER_COMP_ID:
				printf("DeploymentController status updated!\n");
    			break;
    	}
    }

protected:
    void handleEvent(const Event& e) override
    {
    	// TODO: pack mavlink messages and send
    	switch (e.sig)
		{
			case EV_NOSECONE_STATUS_REQUEST:
				break;
			case EV_IGNITION_STATUS_REQUEST:
				break;
			case EV_HOMEONE_STATUS_REQUEST:
				break;
			/* TODO:
			case EV_DEBUG_INFO_REQUEST:
				break;
			case EV_LOW_RATE_TM:
				break;
			case EV_HIGH_RATE_TM:
				break;
		     */
		}
    }

private:

	StatusManager()
	{
		// TODO: add high rate and low rate sampling events
	}
};

}
}

#ifndef sStatusManager
#define sStatusManager StatusManager::getInstance()
#else
#error STATUS MANAGER ALREADY DEFINED
#endif /* sStatusManager */

#endif /* SRC_SHARED_BOARDS_HOMEONE_STATUSMANAGER_STATUSMANAGER_H_ */
