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

bool addHWFilter(uint16_t id, unsigned can_id) {
    uint32_t position = max_chan_filters * can_id;
    uint32_t pos1 = 0, pos2 = 0;
    uint32_t *reg;

    /** Invalid id */
    if(id >= filter_max_id)
        return false;

    if(enabled_filters[can_id] >= max_chan_filters)
        return false;

    if(filters[id] == 0) {

        // find first empty position
        while(position < max_chan_filters * (can_id + 1)) {
            pos1 = position / filters_per_bank;
            pos2 = (position % filters_per_bank) >> shift_reg;

            // pos2 == 0,1 -> 0; pos2 == 2,3 -> 1
            reg = &CAN1->sFilterRegister[pos1].FR1 + pos2;

            pos2 *= filter_size_bit;
            if((CAN1->sFilterRegister[pos1] >> (1 << pos2)) & 0xffff == filter_null)
                break;

            ++position; 
        }
        
        // enter filter initialization mode
        CAN1->FMR |= CAN_FMR_FINIT;

        // 16-bit scale for the filter
        CAN1->FS1R &= ~(1 << pos1);

        // set the filter
        reg &= ~(0xffff << (1 << pos2));                 // clear 
        reg &= ((id << filter_id_shift) << (1 << pos2)); // set
        
        // disable masking
        CAN1->FM1R |= 1 << pos1;
        
        // bind to FIFO0
        CAN1->FFA1R &= ~(1 << pos1);

        // enable filter bank
        CAN1->FA1R |= 1 << pos1;
        
        // exit filter initialization mode
        CAN1->FMR &= ~CAN_FMR_FINIT;
    }

    ++filters[id];
    ++enabled_filters[can_id];

    return true;
}

void delHWFilter(uint16_t id, unsigned can_id) {
    /** Invalid id */
    if(id >= filter_max_id)
        return false;

    if(filters[id] == 0)
        return;

    if(filters[id] == 1) {
        // remove filter
    }

    --filters[id];
    --enabled_filters[can_id];
}
