/* CAN-Bus Driver
 *
 * Copyright (c) 2015 Skyward Experimental Rocketry
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
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.  IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 */

#include "CanManager.h"


void CanManager::addBus(initializer_list<canbus_init_t> init_list) {
    for(const auto& i : init_list) {
        typedef Gpio<i.gpio, rx> port;
        port::mode(i.mode);

        if(af >= 0) 
            port::alternateFunction(i.af);

        bus = new CanBus(i.can);
    }
}

CanBus *CanManager::getBus(uint32_t id) {
    if(id >= bus.size())
        return NULL;

    return &bus[id];
}

bool addHWFilter(uint8_t id, unsigned can_id) {
    if(enabled_filters >= max_filters)
        return false;

    if(filters[id] == 0) {
        //enable filter 
    }

    ++filters[id];
    ++enabled_filters;

    return true;
}

void delHWFilter(uint8_t id, unsigned can_id) {
    if(filters[id] == 0)
        return;

    if(filters[id] == 1) {
        // remove filter
    }

    --filters[id];
    --enabled_filters;
}
