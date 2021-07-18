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
#ifndef SRC_SHARED_DEBUG_H
#define SRC_SHARED_DEBUG_H

#include <miosix.h>
#include <string>
#include <cstdio>

// clang-format off
#ifdef DEBUG

    #include <cstdarg>
    extern miosix::FastMutex m;

    #define D(x) x

    inline void TRACE(const char* format, ...)
    {
        miosix::Lock<miosix::FastMutex> lock(m);
        // Sincronize TRACEs
        
        va_list argptr;
        va_start(argptr, format);

        
        printf("%.2f> ", miosix::getTick()/1000.0f);
        vprintf(format, argptr);

        va_end(argptr);
    }

    // #define TRACE(...) printf("%.2f> ", miosix::getTick()/1000.0f); printf(__VA_ARGS__)

#else

    #define D(x)
    #define TRACE(...) (void)0

#endif //DEBUG

#ifdef LOG_THREAD_STACK
    #define LOG_STACK(...) logStack(__VA_ARGS__)

    static inline void logStack(std::string thread_name)
    {
        using namespace miosix;
        printf("[STACK %s] Abs: %d, Curr: %d, Size: %d\n", thread_name.c_str(), 
                    (int)MemoryProfiling::getAbsoluteFreeStack(), 
                    (int)MemoryProfiling::getCurrentFreeStack(), 
                    (int)MemoryProfiling::getStackSize());
    }
#else
    #define LOG_STACK(...) (void)0
#endif
// clang-format on

#endif /* SRC_SHARED_DEBUG_H */
