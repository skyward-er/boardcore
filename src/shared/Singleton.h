/* Copyright (c) 2015-2016 Skyward Experimental Rocketry
 * Authors: Illya Dudchenko, Matteo Piazzolla, Federico Terraneo
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

namespace Boardcore
{

/**
 * WARNING: Deriving from this class is not enough to make a singleton, you also
 * have to declare the constructor of your class private!
 *
 * \code
 * class Foo : public Singleton<Foo>
 * {
 *     friend Singleton<Foo>;
 *
 * private:
 *     Foo() {} // Ok, private constructor
 * };
 * \endcode
 *
 * Look here for more info on Singletons:
 * https://stackoverflow.com/questions/1008019/c-singleton-design-pattern
 */
template <typename T>
class Singleton
{
public:
    /**
     * \return A reference to the only instance of the class T.
     */
    inline static T& getInstance()
    {
        static T instance;
        return instance;
    }

protected:
    Singleton() {}

public:
    Singleton(const Singleton&)            = delete;
    Singleton& operator=(const Singleton&) = delete;
};

}  // namespace Boardcore
