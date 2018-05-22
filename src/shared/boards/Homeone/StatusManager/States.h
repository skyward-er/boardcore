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

#ifndef SRC_SHARED_BOARDS_HOMEONE_STATUSMANAGER_STATES_H_
#define SRC_SHARED_BOARDS_HOMEONE_STATUSMANAGER_STATES_H_

namespace HomeoneBoard
{

enum fmmState_t
{
	DISARMED,
	ARMED,
	TESTING,
	ABORTED,
	ASCENDING,
	APOGEE_DETECTION,
	DESCENDING_PHASE_1,
	DESCENDING_PHASE_2,
	LANDED
};

enum sensorManagerState_t
{
	SAMPLING,
	NOT_SAMPLING
};

enum ignitionState_t
{
	IGNITION_ARMED,
	IGNITION_STARTED,
	IGNITION_ABORTED
};

enum deploymentState_t
{
	NOSECONE_CLOSED,
	NOSECONE_OPEN,
	FIRST_DROGUE_DEPLOYED,
	SECOND_DROGUE_DEPLOYED
};

}
#endif /* SRC_SHARED_BOARDS_HOMEONE_STATUSMANAGER_STATES_H_ */
