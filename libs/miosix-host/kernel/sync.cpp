/* Copyright (c) 2021 Skyward Experimental Rocketry
 * Author: Damiano Amatruda
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

#include "sync.h"

namespace miosix
{

FastMutex::FastMutex(Options opt) {}

void FastMutex::lock() {}

bool FastMutex::tryLock() { return true; }

void FastMutex::unlock() {}

FastMutex::~FastMutex() {}

void Mutex::lock() {}

bool Mutex::tryLock() { return true; }

void Mutex::unlock() {}

void ConditionVariable::wait(Mutex& m) {}

void ConditionVariable::wait(FastMutex& m) {}

void ConditionVariable::signal() {}

void ConditionVariable::broadcast() {}

void Timer::start() { running = true; }

void Timer::stop() { running = false; }

bool Timer::isRunning() const { return running; }

int Timer::interval() const { return 100; }

void Timer::clear() { running = false; }

}  // namespace miosix
