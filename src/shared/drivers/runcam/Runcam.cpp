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

Runcam::Runcam(USARTInterface &usart) : usart(usart) {}

bool Runcam::init()
{
    // If already initialized
    if (isInit)
    {
        LOG_ERR(logger, "Connection with the camera already initialized");
        return true;
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

    isInit = false;

    return true;
}

void Runcam::openMenu() { usart.write(&OPEN_MENU, sizeof(OPEN_MENU)); }

void Runcam::selectSetting()
{
    usart.write(&SELECT_SETTING, sizeof(SELECT_SETTING));
}

void Runcam::moveDown() { usart.write(&MOVE_DOWN, sizeof(MOVE_DOWN)); }

}  // namespace Boardcore
