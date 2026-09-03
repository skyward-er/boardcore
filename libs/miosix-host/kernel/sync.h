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

#pragma once

#ifndef COMPILE_FOR_HOST
#error "Unsupported"
#endif

namespace miosix
{

class FastMutex
{
public:
    enum Options
    {
        DEFAULT,
        RECURSIVE
    };

    FastMutex(Options opt = DEFAULT);

    void lock();

    bool tryLock();

    void unlock();

    ~FastMutex();

private:
    FastMutex(const FastMutex&);
    FastMutex& operator=(const FastMutex&);
};

class Mutex
{
public:
    void lock();

    bool tryLock();

    void unlock();
};

template <typename T>
class Lock
{
public:
    explicit Lock(T& m);

    ~Lock();

    T& get() { return mutex; }

private:
    Lock(const Lock& l);
    Lock& operator=(const Lock& l);
    T& mutex;
};

template <typename T>
Lock<T>::Lock(T& m) : mutex(m)
{
}

template <typename T>
Lock<T>::~Lock()
{
}

template <typename T>
class Unlock
{
public:
    explicit Unlock(Lock<T>& l);
    Unlock(T& m);

    ~Unlock();

    T& get() { return mutex; }

private:
    Unlock(const Unlock& l);
    Unlock& operator=(const Unlock& l);
    T& mutex;
};

template <typename T>
Unlock<T>::Unlock(Lock<T>& l) : mutex(l.get())
{
}

template <typename T>
Unlock<T>::Unlock(T& m) : mutex(m)
{
}

template <typename T>
Unlock<T>::~Unlock()
{
}

class ConditionVariable
{
public:
    template <typename T>
    void wait(Lock<T>& l);

    void wait(Mutex& m);

    void wait(FastMutex& m);

    void signal();

    void broadcast();
};

template <typename T>
void ConditionVariable::wait(Lock<T>& l)
{
}

class Timer
{
public:
    void start();

    void stop();

    bool isRunning() const;

    int interval() const;

    void clear();

private:
    bool running = false;
};

}  // namespace miosix
