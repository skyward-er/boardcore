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
#include <boards/Homeone/FlightModeManager/FlightModeManager.h>
#include <boards/Homeone/TMTCManager/TMTCManager.h>
#include <logger/Logger.h>


namespace HomeoneBoard
{
namespace Status
{

class StatusManager: public EventHandler, Singleton<StatusManager>
{
	friend class Singleton<StatusManager>;

public:
    ~StatusManager() {}

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
			case EV_DEBUG_INFO_REQUEST:
				break;
			case EV_LOW_RATE_TM:
				// char telemetryMsg[sizeof(LRTelemetry)];
				// LRTelemetry.build(&buffer, sensors->getStatus().pressure1);
				// TODO: TMTCManager.send(telemetryMsg);
				break;
			case EV_HIGH_RATE_TM:
				// char telemetryMsg[sizeof(LRTelemetry)];
				// LRTelemetry.build(&buffer, sensors->getStatus().pressure1);
				// TODO: TMTCManager.send(telemetryMsg);
				break;

			/* TODO:
			case EV_ENABLE_LOW_RATE_TM:
				break;
			case EV_DISABLE_LOW_RATE_TM:
				break;
			case EV_ENABLE_HIGH_RATE_TM:
				break;
			case EV_DISABLE_HIGH_RATE_TM:
				break; */

		}
    }

private:

	StatusManager()
	{
		// TODO: add high rate and low rate sampling events
	}

	FlightModeManager* fmm;
	TMTCManager* tmtc;
	Logger* logger;
	/*
	 * TODO:
	 * 	SensorManager* sensors;
	 *  IgnitionController* ignition;
	 *  DeploymentController* deployment;
	*/

};

}
}

#ifndef sStatusManager
#define sStatusManager StatusManager::getInstance()
#else
#error STATUS MANAGER ALREADY DEFINED
#endif /* sStatusManager */

#endif /* SRC_SHARED_BOARDS_HOMEONE_STATUSMANAGER_STATUSMANAGER_H_ */
