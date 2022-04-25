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

#include <miosix.h>
#include <utils/ButtonHandler/ButtonHandler.h>

#include <functional>

using namespace miosix;
using namespace Boardcore;
using namespace std;
using namespace placeholders;

GpioPin button1 = GpioPin(GPIOG_BASE, 10);
GpioPin button2 = GpioPin(GPIOG_BASE, 9);

void buttonCallback(ButtonEvent event, int buttonId);

int main()
{
    button1.mode(Mode::INPUT);
    button2.mode(Mode::INPUT);

    ButtonHandler::getInstance().registerButtonCallback(
        button1, bind(buttonCallback, _1, 1));
    ButtonHandler::getInstance().registerButtonCallback(
        button2, bind(buttonCallback, _1, 2));

    while (true)
        Thread::sleep(1000);
}

void buttonCallback(ButtonEvent event, int buttonId)
{
    switch (event)
    {
        case ButtonEvent::PRESSED:
            printf("Button %d pressed\n", buttonId);
            break;
        case ButtonEvent::SHORT_PRESS:
            printf("Button %d released, it was a short press\n", buttonId);
            break;
        case ButtonEvent::LONG_PRESS:
            printf("Button %d released, it was a long press\n", buttonId);
            break;
        case ButtonEvent::VERY_LONG_PRESS:
            printf("Button %d released, it was a very long press\n", buttonId);
            break;

        default:
            break;
    }
}