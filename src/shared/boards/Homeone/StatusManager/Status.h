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

namespace HomeoneBoard
{

/*************** Base struct *********************/
enum componentId_t
{
	FMM_COMP_ID,
	TMTC_COMP_ID,
	SENSOR_MANAGER_COMP_ID,
	LOGGER_COMP_ID,
	IGNITION_CONTROLLER_COMP_ID,
	DEPLOYMENT_CONTROLLER_COMP_ID,
	SCHEDULER_COMP_ID,
	EVENT_BROKER_COMP_ID,
	CANBUS_MANAGER_COMP_ID
};

enum healthStatus_t
{
	COMP_UNINIT,
	COMP_OK,
	COMP_ERROR,  // Error flag, still alive
	COMP_FAILED	 // Dead
};

struct ComponentStatus
{
	componentId_t compId;
	healthStatus_t healthStatus;
};

#endif /* SRC_SHARED_BOARDS_HOMEONE_STATUSMANAGER_STATUS_H_ */
