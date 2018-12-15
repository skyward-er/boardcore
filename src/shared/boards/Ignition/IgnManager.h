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
#pragma once

#include <Singleton.h>

namespace IgnBoard
{


/**
 * Implementation of the Ignition Board logic.
 */
class IgnitionManager : public Singleton<IgnitionManager>
{
    friend class Singleton<IgnitionManager>;

public:
    IgnitionManager()
    {
    	// Initialize canbus
    	// Assign canReceiver function
    	// Communication with board 2?
    	// Set state
    	// Send state
    }

    ~IgnitionManager() {}

    void abort() 
    {
    	// Set abort pin to 1
    	// wait
    	// set internal state to abort
    	// send status
    }

    void getStatus()
    {
    	// getStatus from other board
    	// refresh myStatus
    	// send status
    }

    void launch(uint64_t launch_code)
    {
    	// if not aborted
    	// check launch code
    	// send launch code to other board
    	// poll for response
    	// if nCycle > 1000
    	// abort()
    }

private:
	bool isAborted = false;
	IgnitionBoardStatus myStatus;

	CanManager c(CAN1);

};

}
