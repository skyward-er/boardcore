/* Copyright (c) 2015-2018 Skyward Experimental Rocketry
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

#ifdef DEBUG
#include <miosix.h>
#include <utils/KernelTime.h>

#include <cstdarg>
#include <cstdio>
#include <string>

// linter off

extern miosix::FastMutex traceMutex;

#define D(x) x

inline void TRACE(const char* format, ...)
{
    miosix::Lock<miosix::FastMutex> lock(traceMutex);  // Synchronize TRACEs

    va_list argptr;
    va_start(argptr, format);

    printf("%.2f> ", Boardcore::Kernel::getOldTick() / 1000.0f);
    vprintf(format, argptr);

    va_end(argptr);
}

// #define TRACE(...) printf("%.2f> ", Boardcore::Kernel::getOldTick()/1000.0f);
// printf(__VA_ARGS__)

#else

#define D(x)
#define TRACE(...) (void)0

#endif  // DEBUG

#ifdef LOG_THREAD_STACK

namespace Boardcore
{

inline void LOG_STACK(std::string threadName)
{
    using namespace miosix;
    printf("[STACK %s] Abs: %d, Curr: %d, Size: %d\n", threadName.c_str(),
           (int)MemoryProfiling::getAbsoluteFreeStack(),
           (int)MemoryProfiling::getCurrentFreeStack(),
           (int)MemoryProfiling::getStackSize());
}

}  // namespace Boardcore

#else

#define LOG_STACK(...) (void)0

#endif  // LOG_THREAD_STACK

// clang-format on
