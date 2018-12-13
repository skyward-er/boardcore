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

#include <boards/Nosecone/NoseconeManager.h>

#include <events/EventBroker.h>

#include <boards/Nosecone/Events.h>
#include <boards/Nosecone/Topics.h>

using rena  = miosix::Gpio<GPIOG_BASE, 2>;
using namespace miosix;
using namespace interfaces;
using namespace actuators;

namespace NoseconeBoard
{
namespace FMM
{

NoseconeManager::NoseconeManager() : FSM(&NoseconeManager::state_close)
{
    sEventBroker->subscribe(this, TOPIC_NOSECONE);
}

/**
 *
 */
void NoseconeManager::state_close(const Event& e)
{
    switch (e.sig)
    {
        case EV_ENTRY:
            printf("CLOSE state entry\n");
            driver.disable();
            printf("Disabled\n");
            break;
        case EV_EXIT:
            printf("CLOSE state exit\n");
            break;

        case EV_OPEN:
            printf("CLOSE state received EV_OPEN\n");
            transition(&NoseconeManager::state_opening);
            break;
        case EV_STATUS:
            printf("CLOSE state received EV_STATUS\n");
            // TODO: send back status
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
          printf("OPENING state entry\n");
          driver.enable_direct();
          printf("Direct\n");
          sEventBroker->postDelayed(Event{EV_TIMER_EXPIRED}, TOPIC_NOSECONE, 5000);
          break;

        case EV_EXIT:
            printf("OPENING state exit\n");
            break;

        case EV_TIMER_EXPIRED:
            printf("OPEN received EV_TIMER_EXPIRED\n");
            transition(&NoseconeManager::state_open);
            break;

        case EV_ABORT:
            printf("OPEN received EV_ABORT\n");
            //transition(&NoseconeManager::state_error);
            break;

        default:
            printf("Unknown event received.\n");
            break;
    }
}

void NoseconeManager::state_open(const Event& e)
{
  switch (e.sig)
  {
      case EV_ENTRY:
          printf("OPEN state entry\n");
          driver.disable();
          printf("Disabled\n");
          break;
      case EV_EXIT:
          printf("OPEN state exit\n");
          break;

      case EV_CLOSE:
        printf("OPEN state received EV_CLOSE\n");
        transition(&NoseconeManager::state_closing);
      break;

      case EV_STATUS:
        printf("OPEN state received EV_STATUS\n");
      // TODO: send back status
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
          printf("CLOSING state entry\n");
          driver.enable_reverse();
          printf("Reverse\n");
          sEventBroker->postDelayed(Event{EV_TIMER_EXPIRED}, TOPIC_NOSECONE, 5000);
          break;

      case EV_EXIT:
          printf("CLOSING state exit\n");
          break;

      case EV_TIMER_EXPIRED:
          printf("OPEN received EV_TIMER_EXPIRED\n");
          transition(&NoseconeManager::state_close);
      break;

      default:
          printf("Unknown event received.\n");
          break;
  }
}

} // namespace NoseconeBoard
} // namespace FMM
