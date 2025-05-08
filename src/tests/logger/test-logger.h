/* Copyright (c) 2025 Skyward Experimental Rocketry
 * Author: Pietro Bortolus
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

#include <cstring>
#include <ostream>
#include <reflect.hpp>

#ifdef _MIOSIX
#include <miosix.h>
#include <utils/KernelTime.h>
#endif  //_MIOSIX

namespace Boardcore
{

struct Dummy
{
    int64_t timestamp;
    int x;

    Dummy() : x(42)
    {
#ifdef _MIOSIX
        timestamp = Kernel::getOldTick();
#else   //_MIOSIX
        timestamp = 0;
#endif  //_MIOSIX
    };

    void print(std::ostream& os) const
    {
        os << "timestamp=" << timestamp << " ";
        if (x == 42)
            os << "unserialized incorrectly, x =" << x;
        return;
    }

    static constexpr auto reflect()
    {
        return STRUCT_DEF(Dummy, FIELD_DEF(timestamp) FIELD_DEF(x));
    }
};

}  // namespace Boardcore
