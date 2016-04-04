/* Copyright (c) 2016 Skyward Experimental Rocketry
 * Authors: Alain Carlucci, Matteo Piazzolla
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
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.  IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 */

#ifndef LEDS_H
#define LEDS_H

#include <Common.h>

class Leds {
public:

    /** Every bit of the param leds is the value for the n-th led.
     * For example leds = 0x201 means (only) first and last led on.
     */
    static void set(uint16_t leds) {
        for(int i=0;i<10;i++)
            set(i, (byte & (1 << i)) != 0);
    }

    /** Param id should be in range [0,10). */
    static void set(uint8_t id,  bool enable) {
        if(id >= pins.size())
            return;

        if(enable)
            pins[id].high();
        else
            pins[id].low();
    }

    /** Call this function on board startup, before the first set() call. */
    inline static void init() {
        #define PIN(x,y) Gpio<GPIO##x##_BASE, y>::getPin()
        pins = { 
            PIN(B, 0), PIN(A,15), PIN(B, 4), PIN(C, 4), PIN(C, 5), 
            PIN(F, 6), PIN(D, 3), PIN(D, 4), PIN(G, 2), PIN(G, 3)
        };
        #undef PIN
        for(auto& pin : pins)
            pin.mode(Mode::OUTPUT);
    }

private:
    static vector<GpioPin> pins;
    Leds() {}
};

vector<GpioPin> Leds::pins;
#endif
