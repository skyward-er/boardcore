/* Copyright (c) 2015-2016 Skyward Experimental Rocketry
 * Authors: Matteo Piazzolla, Alain Carlucci
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

#include <list>

namespace Boardcore
{

template <typename T>
class SynchronizedQueue
{
public:
    SynchronizedQueue() {}
    void put(const T& data);
    T get();
    int len();

private:
    SynchronizedQueue(const SynchronizedQueue&)            = delete;
    SynchronizedQueue& operator=(const SynchronizedQueue&) = delete;
    std::list<T> queue;
    miosix::Mutex mMutex;
    miosix::ConditionVariable mCv;
};

template <typename T>
void SynchronizedQueue<T>::put(const T& data)
{
    mMutex.lock();
    queue.push_back(data);
    mCv.signal();
    mMutex.unlock();
}

template <typename T>
T SynchronizedQueue<T>::get()
{
    mMutex.lock();
    while (queue.empty())
        mCv.wait(mMutex);
    T result = queue.front();
    queue.pop_front();
    mMutex.unlock();

    return result;
}

template <typename T>
int SynchronizedQueue<T>::len()
{
    return queue.size();
}

}  // namespace Boardcore
