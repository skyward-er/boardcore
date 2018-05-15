/* Copyright (c) 2015-2018 Skyward Experimental Rocketry
 * Authors: Luca Erbetta
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

#include "ADA.h"
#include "boards/Homeone/Events.h"
#include "boards/Homeone/Topics.h"
#include "events/EventBroker.h"

namespace HomeoneBoard
{
namespace FMM  // Flight Mode Manager
{

ADA::ADA() : FSM(&ADA::idle)
{
    sEventBroker->subscribe(this, TOPIC_FLIGHT_EVENTS);
    sEventBroker->subscribe(this, TOPIC_SENSORS);
}

ADA::~ADA() {}

void ADA::idle(const Event& e)
{
    switch (e.sig)
    {
        case EV_ENTRY:
            break;
        case EV_EXIT:
            break;
        case EV_ADA_SHADOW_MODE:
            transition(&ADA::shadow_mode);
            break;
        default:
            break;
    }
}

void ADA::shadow_mode(const Event& e)
{
    switch (e.sig)
    {
        case EV_ENTRY:
            break;
        case EV_EXIT:
            break;
        case EV_ADA_ACTIVE_MODE:
            transition(&ADA::active_mode);
            break;
        case EV_PRESSURE_SAMPLE:
            PressureSampleEvent ev_ps = static_cast<PressureSampleEvent>(e);
            onNewSample(ADASample{ev_ps.pressure}, false);
            break;
        default:
            break;
    }
}

void ADA::active_mode(const Event& e)
{
    switch (e.sig)
    {
        case EV_ENTRY:
            break;
        case EV_EXIT:
            break;
        case EV_ADA_STOP:
            transition(&ADA::stopped);
            break;
        case EV_PRESSURE_SAMPLE:
            PressureSampleEvent ev_ps = static_cast<PressureSampleEvent>(e);
            onNewSample(ADASample{ev_ps.pressure}, true);
            break;
        default:
            break;
    }
}

void ADA::stopped(const Event& e)
{
    switch (e.sig)
    {
        case EV_ENTRY:
            // TODO: Stop the thread somehow?
            break;
        case EV_EXIT:
            break;
    }
}

void ADA::onNewSample(ADASample sample, bool send_apogee_event)
{
    // TODO: ADA Implementation
}
}
}
