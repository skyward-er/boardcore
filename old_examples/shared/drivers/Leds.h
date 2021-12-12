/* Copyright (c) 2016 Skyward Experimental Rocketry
 * Authors: Alain Carlucci, Matteo Piazzolla, Federico Terraneo
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

#include <Common.h>

#include <array>

class Leds
{
public:
    /**
     * Turn on/off all leds using a bitmask
     * \param leds bitmask specifing which led has to be turned on
     */
    static void set(uint16_t leds)
    {
        for (int i = 0; i < 10; i++)
            set(i, (leds & (1 << i)) != 0);
    }

    /**
     * \param id should be in range [0,10).
     */
    static void set(uint8_t id, bool enable)
    {
        if (id >= pins.size())
            return;

        if (enable)
            pins[id].high();
        else
            pins[id].low();
    }

private:
    static const int numLeds = 10;
    static std::array<miosix::GpioPin, numLeds> pins;

    Leds()            = delete;
    Leds(const Leds&) = delete;
    Leds& operator=(const Leds&) = delete;
};
