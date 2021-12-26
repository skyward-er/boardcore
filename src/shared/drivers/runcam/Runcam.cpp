/* Copyright (c) 2021 Skyward Experimental Rocketry
 * Author: Federico Mandelli
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
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 */

#include "Runcam.h"

#include <miosix.h>

namespace Boardcore
{

Runcam::Runcam(unsigned int portNumber) : portNumber(portNumber) {}

bool Runcam::init()
{
    // If already initialized
    if (isInit)
    {
        LOG_ERR(logger, "Connection with the camera already initialized");
        return true;
    }

    if (!configureSerialCommunication())
    {
        LOG_ERR(logger, "Unable to config camera port");
        return false;
    }

    isInit = true;

    Runcam::moveDown();
    Runcam::openMenu();

    return true;
}

bool Runcam::close()
{
    // Sensor not init
    if (!isInit)
    {
        LOG_ERR(logger,
                "Runcam not initilized while attempting to close the serial "
                "connection");
        return true;
    }

    // Close the serial
    if (!serialInterface->closeSerial())
    {
        LOG_ERR(logger, "Unable to close serial communication");
        return false;
    }

    isInit = false;

    // Free the serialInterface memory
    delete serialInterface;

    return true;
}

void Runcam::openMenu() { serialInterface->send(&OPEN_MENU, 4); }

void Runcam::selectSetting() { serialInterface->send(&SELECT_SETTING, 4); }

void Runcam::moveDown() { serialInterface->send(&MOVE_DOWN, 4); }

bool Runcam::configureSerialCommunication()
{
    serialInterface = new RuncamSerial(portNumber, defaultBaudRate);

    // Check correct serial init
    if (!serialInterface->init())
    {
        LOG_ERR(logger, "Unable to config the default serial port");
        return false;
    }

    return true;
}

}  // namespace Boardcore
