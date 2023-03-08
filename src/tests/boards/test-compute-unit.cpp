/* Copyright (c) 2015-2017 Skyward Experimental Rocketry
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

#include <miosix.h>
#include <utils/TestUtils/MockGpioPin.h>

using namespace miosix;
using namespace Boardcore;

GpioPin m2_pins[58] = {
    {GPIOA_BASE, 0},  // 58
    MockGpioPin(),    // NRST
    {GPIOA_BASE, 2},  {GPIOC_BASE, 1},  {GPIOA_BASE, 4},  {GPIOC_BASE, 3},
    {GPIOA_BASE, 6},  {GPIOA_BASE, 1},  {GPIOC_BASE, 4},  {GPIOA_BASE, 3},
    {GPIOB_BASE, 0},  {GPIOA_BASE, 5},  {GPIOD_BASE, 12}, {GPIOA_BASE, 7},
    {GPIOB_BASE, 12}, {GPIOC_BASE, 5},  {GPIOB_BASE, 14}, {GPIOB_BASE, 1},
    {GPIOG_BASE, 7},  {GPIOB_BASE, 11}, {GPIOC_BASE, 6},  {GPIOB_BASE, 13},
    {GPIOC_BASE, 8},  {GPIOB_BASE, 15}, {GPIOA_BASE, 8},  {GPIOD_BASE, 13},
    MockGpioPin(),  // USART1_RX - {GPIOA_BASE, 10},
    {GPIOG_BASE, 6},  {GPIOA_BASE, 12}, {GPIOC_BASE, 7},  {GPIOC_BASE, 10},
    {GPIOC_BASE, 9},  {GPIOC_BASE, 12},
    MockGpioPin(),  // USART1_TX - {GPIOA_BASE, 9},
    {GPIOD_BASE, 2},  {GPIOA_BASE, 11}, {GPIOD_BASE, 4},  {GPIOA_BASE, 15},
    {GPIOD_BASE, 6},  {GPIOC_BASE, 11}, {GPIOG_BASE, 10}, {GPIOD_BASE, 3},
    {GPIOG_BASE, 12}, {GPIOD_BASE, 5},  {GPIOG_BASE, 14}, {GPIOD_BASE, 7},
    {GPIOB_BASE, 4},  {GPIOG_BASE, 11}, {GPIOE_BASE, 2},  {GPIOG_BASE, 13},
    {GPIOE_BASE, 4},  {GPIOB_BASE, 3},  {GPIOB_BASE, 8},  {GPIOB_BASE, 9},
    MockGpioPin(),                    // SWCLK - {GPIOA_BASE, 14},
    {GPIOE_BASE, 5},  MockGpioPin(),  // SWDIO - {GPIOA_BASE, 13},
    {GPIOE_BASE, 6},
};

int main()
{
    printf("Testing gpios...\n");

    while (true)
    {
        for (int i = 0; i < 58; i++)
        {
            auto pin = m2_pins[57 - i];
            pin.mode(Mode::OUTPUT);

            pin.high();
            delayUs(500);
            pin.low();
            delayUs(500);
        }
    }
}