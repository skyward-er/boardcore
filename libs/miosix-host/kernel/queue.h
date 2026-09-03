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

template <typename T, unsigned int len>
class Queue
{
public:
    Queue();

    bool isEmpty() const;

    bool isFull() const;

    unsigned int size() const;

    unsigned int capacity() const;

    void waitUntilNotEmpty();

    void waitUntilNotFull();

    void get(T& elem);

    void put(const T& elem);

    bool IRQget(T& elem);

    bool IRQget(T& elem, bool& hppw);

    bool IRQput(const T& elem);

    bool IRQput(const T& elem, bool& hppw);

    void reset();

    void IRQreset();
};

}  // namespace miosix

namespace miosix
{

template <typename T, unsigned int len>
Queue<T, len>::Queue()
{
}

template <typename T, unsigned int len>
bool Queue<T, len>::isEmpty() const
{
    return false;
}

template <typename T, unsigned int len>
bool Queue<T, len>::isFull() const
{
    return false;
}

template <typename T, unsigned int len>
unsigned int Queue<T, len>::size() const
{
    return 0;
}

template <typename T, unsigned int len>
unsigned int Queue<T, len>::capacity() const
{
    return (unsigned int)__INT_MAX__;
}

template <typename T, unsigned int len>
void Queue<T, len>::waitUntilNotEmpty()
{
}

template <typename T, unsigned int len>
void Queue<T, len>::waitUntilNotFull()
{
}

template <typename T, unsigned int len>
void Queue<T, len>::get(T& elem)
{
}

template <typename T, unsigned int len>
void Queue<T, len>::put(const T& elem)
{
}

template <typename T, unsigned int len>
bool Queue<T, len>::IRQget(T& elem)
{
    return true;
}

template <typename T, unsigned int len>
bool Queue<T, len>::IRQget(T& elem, bool& hppw)
{
    return true;
}

template <typename T, unsigned int len>
bool Queue<T, len>::IRQput(const T& elem)
{
    return true;
}

template <typename T, unsigned int len>
bool Queue<T, len>::IRQput(const T& elem, bool& hppw)
{
    return true;
}

template <typename T, unsigned int len>
void Queue<T, len>::reset()
{
}

template <typename T, unsigned int len>
void Queue<T, len>::IRQreset()
{
}

}  // namespace miosix
