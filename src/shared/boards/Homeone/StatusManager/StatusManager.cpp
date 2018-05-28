/* Copyright (c) 2018 Skyward Experimental Rocketry
 * Authors: Elvis
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

#include <boards/Homeone/StatusManager/StatusManager.h>

namespace HomeoneBoard
{
namespace Status
{

void StatusManager::handleEvent(const Event& e)
{
    switch (e.sig)
    {
        case EV_LOW_RATE_TM:
            TMTC::sTMTCManager->enqueueMsg(lr_tm.serialize(), lr_tm.getSize());
            break;
        case EV_HIGH_RATE_TM:
            TMTC::sTMTCManager->enqueueMsg(hr_tm.serialize(), hr_tm.getSize());
            break;
        case EV_NOSECONE_STATUS_REQUEST:
            TMTC::sTMTCManager->enqueueMsg(nos_tm.serialize(), nos_tm.getSize());
            break;
        case EV_IGNITION_STATUS_REQUEST:
            TMTC::sTMTCManager->enqueueMsg(ign_tm.serialize(), ign_tm.getSize());
            break;
        case EV_HOMEONE_STATUS_REQUEST:
            TMTC::sTMTCManager->enqueueMsg(home_tm.serialize(), home_tm.getSize());
            break;
        case EV_DEBUG_INFO_REQUEST:
            TMTC::sTMTCManager->enqueueMsg(debug_tm.serialize(), debug_tm.getSize());
            break;

            /* TODO:
             case EV_ENABLE_TM:
             break;
             case EV_DISABLE_TM:
             break; */
        }
    }

}
/* namespace Status */
} /* namespace HomeoneBoard */
