/* Copyright (c) 2019-2021 Skyward Experimental Rocketry
 * Author: Luca Erbetta
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

#pragma once

#include <ActiveObject.h>
#include <events/EventBroker.h>
#include <utils/Debug.h>

#include <iostream>
#include <sstream>
#include <string>

using std::string;
using std::stringstream;

namespace Boardcore
{

/**
 * @brief Utility class to manually post events to specific topics.
 */
class EventInjector : public ActiveObject
{
public:
protected:
    void run() override
    {
        using namespace std;

        int ev, topic;

        for (;;)
        {
            TRACE("[EventInjector] Insert Event & Topic:\n");

            string temp;
            getline(cin, temp);
            stringstream(temp) >> ev >> topic;

            EventBroker::getInstance().post({(uint8_t)ev}, topic);
        }
    }
};

}  // namespace Boardcore
