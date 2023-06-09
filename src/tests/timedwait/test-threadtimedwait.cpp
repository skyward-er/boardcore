/* Copyright (c) 2023 Skyward Experimental Rocketry
 * Author: Niccolò Betto
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

#include <miosix.h>

#include <functional>

using namespace miosix;

namespace
{
GpioPin button =
    GpioPin(GPIOA_BASE, 0);  // PA0 for the stm32f407 discovery board

Thread *mainThread = nullptr;

void blinkLed(int times)
{
    for (int i = 0; i < times; i++)
    {
        ledOn();
        Thread::sleep(100);
        ledOff();
        Thread::sleep(75);
    }
}

bool wasPressed = false;

void buttonPollingThread(void *argv)
{
    while (true)
    {
        printf("[%lld] buttonPollingThread: (%p) Polling button...\n",
               getTick(), (void *)Thread::getCurrentThread());
        bool isNowPressed = !button.value();

        if (isNowPressed && !wasPressed)
        {
            printf("[%lld] buttonPollingThread: (%p) Button pressed\n",
                   getTick(), (void *)Thread::getCurrentThread());
            mainThread->wakeup();
        }

        wasPressed = isNowPressed;

        // 10Hz polling
        Thread::sleep(100);
    }
}
}  // namespace

int main()
{
    printf("TEST: Thread::timedWait\n");

    button.mode(Mode::INPUT);

    Thread *poller =
        Thread::create(buttonPollingThread, 1024, 0, nullptr, Thread::JOINABLE);
    mainThread = Thread::getCurrentThread();
    printf(
        "SUMMARY:\n"
        "* mainThread: thread ID (%p)\n"
        "* buttonPollingThread: thread ID (%p)\n",
        (void *)mainThread, (void *)poller);

    while (true)
    {
        printf("[%lld] main: (%p) Waiting for button press...\n", getTick(),
               (void *)Thread::getCurrentThread());
        if (Thread::timedWaitMs(3000) == TimedWaitResult::Timeout)
        {
            printf("[%lld] main: (%p) Timed out, 3 seconds elapsed\n",
                   getTick(), (void *)Thread::getCurrentThread());
            blinkLed(1);
        }
        else
        {
            printf("[%lld] main: (%p) Waken up, user button pressed\n",
                   getTick(), (void *)Thread::getCurrentThread());
            blinkLed(2);
        }
    }
}
