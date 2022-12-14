/* Copyright (c) 2022 Skyward Experimental Rocketry
 * Author: Alberto Nidasio
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

#include <actuators/Buzzer.h>
#include <miosix.h>

using namespace miosix;
using namespace Boardcore;

int main()
{
    Buzzer buzzer(TIM4, TimerUtils::Channel::CHANNEL_2);

    printf("Now beeping for 2 seconds\n");
    buzzer.oneTimeToggle(2 * 1000);
    Thread::sleep(3 * 1000);

    printf("Now continuosly toggle the buzzer every 500ms for 5 seconds");
    buzzer.continuouslyToggle(500, 500);
    Thread::sleep(6 * 1000);

    printf("Now continuosly toggle the buzzer every 200ms for 5 seconds");
    buzzer.continuouslyToggle(200, 200);
    Thread::sleep(6 * 1000);

    printf("Now manually turning on the buzzer for 1 second\n");
    buzzer.on();
    Thread::sleep(1000);
    buzzer.off();

    while (true)
        Thread::sleep(1000);
}
