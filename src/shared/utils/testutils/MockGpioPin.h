/* Copyright (c) 2021 Skyward Experimental Rocketry
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

#include <miosix.h>

#include <memory>

namespace Boardcore
{

class MockGpioPin
{
public:
    MockGpioPin()
        : val(new int(0)),
          gpio_mode(new miosix::Mode::Mode_(miosix::Mode::OUTPUT))
    {
    }

    void mode(miosix::Mode::Mode_ m) { *gpio_mode = m; }

    void high() { *val = 1; }

    void low() { *val = 0; }

    int value() { return *val; }

    // shared_ptr: make all copies of this instance share the same val /
    // gpio_mode
    std::shared_ptr<int> val;
    std::shared_ptr<miosix::Mode::Mode_> gpio_mode;
};

}  // namespace Boardcore
