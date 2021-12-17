/* Copyright (c) 2021 Skyward Experimental Rocketry
 * Author: Matteo Pignataro
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

#include <drivers/runcam/Runcam.h>

#include "miosix.h"

using namespace miosix;
using namespace Boardcore;

int main()
{
    printf(R"(
            ___                                                      ___             _       _        _              __ _
    o O O  | _ \   _  _    _ _      __     __ _    _ __      o O O  / __|    ___    | |_    | |_     (_)    _ _     / _` |   ___
   o       |   /  | +| |  | ' \    / _|   / _` |  | '  \    o       \__ \   / -_)   |  _|   |  _|    | |   | ' \    \__, |  (_-<
  TS__[O]  |_|_\   \_,_|  |_||_|   \__|_  \__,_|  |_|_|_|  TS__[O]  |___/   \___|   _\__|   _\__|   _|_|_  |_||_|   |___/   /__/_
 {======|_|"""""|_|"""""|_|"""""|_|"""""|_|"""""|_|"""""| {======|_|"""""|_|"""""|_|"""""|_|"""""|_|"""""|_|"""""|_|"""""|_|"""""|
./o--000'"`-0-0-'"`-0-0-'"`-0-0-'"`-0-0-'"`-0-0-'"`-0-0-'./o--000'"`-0-0-'"`-0-0-'"`-0-0-'"`-0-0-'"`-0-0-'"`-0-0-'"`-0-0-'"`-0-0-'
)");

    Thread::sleep(100);

    printf(
        "Press d to move down, c to confirm the action and o to open the "
        "menu\n");
    Thread::sleep(100);
    GpioPin tx(GPIOB_BASE, 6);
    GpioPin rx(GPIOB_BASE, 7);

    tx.mode(Mode::ALTERNATE);
    rx.mode(Mode::ALTERNATE);

    tx.alternateFunction(7);
    rx.alternateFunction(7);
    Runcam test(1);
    if (!test.init())
    {
        return -1;
    }
    char c;

    while (true)

    {
        scanf("%c", &c);
        c = 'c';

        test.moveDown();
        test.selectSetting();
        test.openMenu();
        if (c == 'd')
        {
            test.moveDown();
        }
        else if (c == 'c')
        {
            test.selectSetting();
        }
        else if (c == 'o')
        {
            test.openMenu();
        }
    }

    test.close();

    return 0;
}
