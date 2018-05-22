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

#ifndef SRC_SHARED_BOARDS_HOMEONE_STATUSMANAGER_STATUS_H_
#define SRC_SHARED_BOARDS_HOMEONE_STATUSMANAGER_STATUS_H_

#include "States.h"

namespace HomeoneBoard
{
namespace Status
{

/*************** Base struct *********************/
enum componentId_t
{
	FMM_COMP_ID,
	TMTC_COMP_ID,
	SENSOR_MANAGER_COMP_ID,
	LOGGER_COMP_ID,
	IGNITION_CONTROLLER_COMP_ID,
	DEPLOYMENT_CONTROLLER_COMP_ID
};

enum healthStatus_t
{
	UNINIT,
	OK,
	ERROR,  // Error flag, still alive
	FAILED	// Dead
};

struct ComponentStatus
{
	componentId_t compId;
	healthStatus_t healthStatus;
	uint64_t lastModified;
};



/************* Implementations ***************/
/* FlightModeManager */
struct FMMStatus : ComponentStatus
{
	fmmState_t currentState;
	fmmState_t previousState;
	// TODO: other stuff?
};

/* TMTCManager */
struct TMTCStatus : ComponentStatus
{
	// TODO: mavlink_status_t
	uint32_t maxOccupiedBuffer;
};

/* SensorManager*/
struct SensorManagerStatus : ComponentStatus
{
	// TODO: put last sensors values here
};

/* SDLogger */
struct LoggerStatus : ComponentStatus
{
	uint32_t maxOccupiedBuffer;
};

/* IgnitionController */
struct IgnitionControllerStatus : ComponentStatus
{
	ignitionState_t state;
	bool ignitionAttached;
};

/* DeploymentController */
struct DeploymentControllerStatus : ComponentStatus
{
	deploymentState_t state;
	bool noseconeAttached;
};

// TODO: scheduler?
// TODO: canbus?
// TODO: CPU and Memory usage

}
}

#endif /* SRC_SHARED_BOARDS_HOMEONE_STATUSMANAGER_STATUS_H_ */
