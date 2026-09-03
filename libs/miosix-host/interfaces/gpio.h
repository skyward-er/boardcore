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

class Mode
{
public:
    enum Mode_
    {
        INPUT           = 0,
        INPUT_PULL_UP   = 1,
        INPUT_PULL_DOWN = 2,
        INPUT_ANALOG    = 24,
        OUTPUT          = 8,
        OPEN_DRAIN      = 12,
        ALTERNATE       = 16,
        ALTERNATE_OD    = 20,
    };

private:
    Mode();
};

class Speed
{
public:
    enum Speed_
    {
        _2MHz   = 0x0,
        _25MHz  = 0x1,
        _50MHz  = 0x2,
        _100MHz = 0x3
    };

private:
    Speed();
};

class GpioBase
{
};

class GpioPin : private GpioBase
{
public:
    GpioPin(unsigned int p, unsigned char n);

    void mode(Mode::Mode_ m);

    void speed(Speed::Speed_ s);

    void alternateFunction(unsigned char af);

    void high();

    void low();

    int value();

    unsigned int getPort() const;

    unsigned char getNumber() const;
};

template <unsigned int P, unsigned char N>
class Gpio : private GpioBase
{
public:
    static void mode(Mode::Mode_ m);

    static void speed(Speed::Speed_ s);

    static void alternateFunction(unsigned char af);

    static void high();

    static void low();

    static int value();

    static GpioPin getPin();

    unsigned int getPort() const;

    unsigned char getNumber() const;

private:
    Gpio();
};

template <unsigned int P, unsigned char N>
void Gpio<P, N>::mode(Mode::Mode_ m)
{
}

template <unsigned int P, unsigned char N>
void Gpio<P, N>::speed(Speed::Speed_ s)
{
}

template <unsigned int P, unsigned char N>
void Gpio<P, N>::alternateFunction(unsigned char af)
{
}

template <unsigned int P, unsigned char N>
void Gpio<P, N>::high()
{
}

template <unsigned int P, unsigned char N>
void Gpio<P, N>::low()
{
}

template <unsigned int P, unsigned char N>
int Gpio<P, N>::value()
{
    return 0;
}

template <unsigned int P, unsigned char N>
GpioPin Gpio<P, N>::getPin()
{
    return GpioPin{0, 0};
}

template <unsigned int P, unsigned char N>
unsigned int Gpio<P, N>::getPort() const
{
    return 0;
}

template <unsigned int P, unsigned char N>
unsigned char Gpio<P, N>::getNumber() const
{
    return 0;
}

}  // namespace miosix
