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

#include <Common.h>
#include <drivers/canbus/CanManager.h>
#include <drivers/canbus/CanUtils.h>
#include <interfaces-impl/hwmapping.h>

#include <boards/CanInterfaces.h>

// TODO SPI

namespace IgnBoard
{


/**
 * Implementation of the Ignition Board logic.
 */
class IgnitionManager
{

public:
    IgnitionManager();
    ~IgnitionManager() {}

    void abort();
    void getStatus();
    void launch(uint64_t launch_code);

    void sendStatus();
    bool checkLaunchCode(uint64_t launch_code);

private:
    bool isAborted = false;
    CanInterfaces::IgnitionBoardStatus myStatus;

    CanManager c;

    static const uint64_t EXPECTED_LAUNCH_CODE = 0xAABB;
    static const uint32_t ABORT_DURATION       = 10000;
    static const uint32_t LAUNCH_DURATION      = 10000;

    /**
     * @brief Initialise CAN, set hardware filters and receiver function.
     *
     * @param c CanManager to which the bus has to be added.
     */
    void initCanbus(CanManager& c);

};

/**
 * @brief Canbus receiving function.
 *
 * @param message   each message received on the CANBUS (with the HW filters)
 * @param mngr      IgnitionManager that implements the ignition functions
 */
void canRcv(CanMsg message, IgnitionManager* mngr);

}
