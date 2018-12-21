/* Copyright (c) 2015-2019 Skyward Experimental Rocketry
 * Authors: Benedetta Margrethe Cattani
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

#include "NoseconeManager.h"
#include "Events.h"
#include "Topics.h"
#include <events/EventBroker.h>

using rena  = miosix::Gpio<GPIOG_BASE, 2>;
using namespace miosix;
using namespace interfaces;
using namespace actuators;

namespace NoseconeBoard
{

NoseconeManager::NoseconeManager() : FSM(&NoseconeManager::state_idle)
{ }


void NoseconeManager::state_idle(const Event& e)
{
    switch (e.sig)
    {
        case EV_ENTRY:
            printf("IDLE state entry\n");
            driver.disable();
            printf("Disabled\n");
            break;

        case EV_EXIT:
            printf("IDLE state exit\n");
            break;

        case EV_OPEN:
            printf("IDLE state received EV_OPEN\n");
            transition(&NoseconeManager::state_opening);
            break;

        case EV_CLOSE:
            printf("IDLE state received EV_CLOSE\n");
            transition(&NoseconeManager::state_closing);
            break;

        case EV_GET_STATUS:
            printf("IDLE state received EV_STATUS\n");
            break;

        default:
            printf("Unknown event received.\n");
            break;
    }
}


void NoseconeManager::state_opening(const Event& e)
{
    switch (e.sig)
    {
        case EV_ENTRY:
            printf("OPENING\n");
            driver.enable_direct();
            sEventBroker->postDelayed(Event{EV_TIMER_EXPIRED}, TOPIC_NOSECONE, 10000);
            break;

        case EV_EXIT:
            printf("[OPENING] exiting\n");
            break;

        case EV_TIMER_EXPIRED:
            printf("[OPEN] received EV_TIMER_EXPIRED\n");
            transition(&NoseconeManager::state_idle);
            break;

        case EV_NC_STOP:
            printf("[OPEN] received EV_NC_STOP\n");
            transition(&NoseconeManager::state_idle);
            break;

        case EV_MOTOR_LIMIT:
            printf("[OPEN] received EV_MOTOR_LIMIT\n");
            transition(&NoseconeManager::state_idle);
            break;

        default:
            printf("Unknown event received.\n");
            break;
    }
}


void NoseconeManager::state_closing(const Event& e)
{
  switch (e.sig)
  {
      case EV_ENTRY:
          printf("[CLOSING] state entry\n");
          driver.enable_reverse();
          sEventBroker->postDelayed(Event{EV_TIMER_EXPIRED}, TOPIC_NOSECONE, 10000);
          break;

      case EV_EXIT:
          printf("[CLOSING] exiting\n");
          break;

      case EV_TIMER_EXPIRED:
          printf("[CLOSING] received EV_TIMER_EXPIRED\n");
          transition(&NoseconeManager::state_idle);
          break;

      case EV_NC_STOP:
          printf("[CLOSING] received EV_NC_STOP\n");
          transition(&NoseconeManager::state_idle);
          break;

      case EV_MOTOR_LIMIT:
          printf("[CLOSING] received EV_MOTOR_LIMIT\n");
          transition(&NoseconeManager::state_idle);
          break;

      default:
          printf("Unknown event received.\n");
          break;
  }
}

} // namespace NoseconeBoard
