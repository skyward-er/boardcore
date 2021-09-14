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

#include "CanManager.h"

using namespace miosix;
using namespace std;

CanBus *CanManager::getBus(uint32_t id)
{
    if (id >= bus.size())
        return NULL;

    return bus[id];
}

bool CanManager::addHWFilter(uint16_t id, uint32_t can_id)
{
    uint32_t position  = max_chan_filters * can_id;
    __IO uint32_t *reg = NULL;

    /** Invalid id */
    if (id >= filter_max_id || can_id >= bus.size())
        return false;

    if (enabled_filters[can_id] >= max_chan_filters)
        return false;

    auto filter_pos = filters[can_id].find(id);

    if (filter_pos == filters[can_id].end() || filter_pos->second == 0)
    {
        // enter filter initialization mode
        Config->FMR |= CAN_FMR_FINIT;

        // find first empty position
        uint32_t pos1 = 0, pos2 = 0;
        while (position < max_chan_filters * (can_id + 1))
        {
            pos1 = position / filters_per_row;
            pos2 = position % filters_per_row;

            if (!(Config->FA1R & (1 << pos1)))
            {
                uint32_t erase = filter_null | (filter_null << filter_size_bit);
                Config->sFilterRegister[pos1].FR1 = erase;
                Config->sFilterRegister[pos1].FR2 = erase;
            }

            reg =
                &(Config->sFilterRegister[pos1].FR1) + (pos2 >> separation_bit);

            pos2 &= separation_bit;
            pos2 *= filter_size_bit;

            if ((((*reg) >> pos2) & 0xffff) == filter_null)
                break;

            ++position;
        }

        if (reg == NULL)
            return false;

        // disable this filter
        Config->FA1R &= ~(1 << pos1);

        // 16-bit scale for the filter
        Config->FS1R &= ~(1 << pos1);

        // set the filter
        *reg &= ~(0xffff << pos2);                  // clear
        *reg |= ((id << filter_id_shift) << pos2);  // set

        // disable masking
        Config->FM1R |= 1 << pos1;

        // bind to FIFO0
        Config->FFA1R &= ~(1 << pos1);

        // enable filter bank
        Config->FA1R |= 1 << pos1;

        // exit filter initialization mode
        Config->FMR &= ~CAN_FMR_FINIT;

        filters[can_id][id] = 1;
    }
    else
        ++filters[can_id][id];

    ++enabled_filters[can_id];

    return true;
}

bool CanManager::delHWFilter(uint16_t id, uint32_t can_id)
{
    uint32_t position  = max_chan_filters * can_id;
    __IO uint32_t *reg = NULL;

    /** Invalid id */
    if (id >= filter_max_id || can_id >= bus.size())
        return false;

    auto filter_pos = filters[can_id].find(id);
    if (filter_pos == filters[can_id].end() || filter_pos->second <= 0)
        return false;

    if (filters[can_id][id] == 1)
    {
        // enter filter initialization mode
        Config->FMR |= CAN_FMR_FINIT;

        uint32_t pos2 = 0;
        while (position < max_chan_filters * (can_id + 1))
        {
            uint32_t pos1 = 0;
            pos1          = position / filters_per_row;
            pos2          = position % filters_per_row;

            // pos2 == 0,1 -> 0; pos2 == 2,3 -> 1
            reg =
                &(Config->sFilterRegister[pos1].FR1) + (pos2 >> separation_bit);

            pos2 &= separation_bit;
            pos2 *= filter_size_bit;

            if ((((*reg) >> pos2) & 0xffff) ==
                (uint32_t)(id << filter_id_shift))
                break;

            ++position;
        }

        if (reg == NULL)
            return false;

        *reg |= (0xffff << pos2);  // clear

        // exit filter initialization mode
        Config->FMR &= ~CAN_FMR_FINIT;
    }

    if (--(filter_pos->second) <= 0)
        filters[can_id].erase(filter_pos);

    --enabled_filters[can_id];

    return true;
}