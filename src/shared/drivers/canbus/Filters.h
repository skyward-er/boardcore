/**
 * Copyright (c) 2021 Skyward Experimental Rocketry
 * Authors: Luca Erbetta (luca.erbetta@skywarder.eu)
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
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 */

#pragma once

#include <cstdint>

namespace Boardcore
{
namespace Canbus
{
enum class FilterMode
{
    ID,
    MASK
};

enum class FilterScale
{
    SINGLE32,
    DUAL16
};

struct Filter
{
    uint8_t fifo;
    FilterScale scale;
    FilterMode mode;

    uint32_t FR1 = 0xFFFFFFFF;
    uint32_t FR2 = 0xFFFFFFFF;

    Filter(FilterScale scale, FilterMode mode, uint8_t fifo)
        : fifo(fifo), scale(scale), mode(mode)
    {
    }
protected:
    static uint32_t packRegister32(uint32_t id, uint8_t ext, uint8_t rtr, bool is_ext)
    {
        uint32_t reg = 0;
        if (is_ext)
        {
            reg = ((id & 0x1FFFFFFF) << 3);
        }
        else
        {
            reg = ((id & 0x7FF) << 21);
        }

        reg |= ((ext & 0x1) << 2) | ((rtr & 0x1) << 1);
        return reg;
    }

    static uint16_t packRegister16(uint32_t id, uint8_t ext, uint8_t rtr, bool is_ext)
    {
        uint16_t reg = 0;
        if (is_ext)
        {
            reg = ((id >> 18) & 0x7FF) << 5;
            reg |= ((id >> 15) & 0x07);
        }
        else
        {
            reg = ((id & 0x7FF) << 5);
        }

        reg |= ((ext & 0x1) << 3) | ((rtr & 0x1) << 4);
        return reg;
    }
};

struct Mask32Filter : public Filter
{
    Mask32Filter(uint32_t id, uint32_t id_mask, uint8_t ext, uint8_t ext_mask,
                 uint8_t rtr, uint8_t rtr_mask, uint8_t fifo)
        : Filter(FilterScale::SINGLE32, FilterMode::MASK, fifo)
    {
        scale = FilterScale::SINGLE32;
        mode  = FilterMode::MASK;

        FR1 = packRegister32(id, ext, rtr, ext);
        FR2 = packRegister32(id_mask, ext_mask, rtr_mask, ext);
    }
};

struct ID32Filter : public Filter
{
    ID32Filter(uint8_t fifo)
        : Filter(FilterScale::SINGLE32, FilterMode::ID, fifo)
    {
    }

    bool addID(uint32_t id, bool ext, uint8_t rtr)
    {
        if (id_cnt == 2)
            return false;

        uint32_t* reg = id_cnt < 1 ? &FR1 : &FR2;

        *reg = packRegister32(id, ext, rtr, ext);
        ++id_cnt;
        return true;
    }

private:
    uint8_t id_cnt = 0;
};

struct Mask16Filter : public Filter
{
    Mask16Filter(uint8_t fifo)
        : Filter(FilterScale::DUAL16, FilterMode::MASK, fifo)
    {
    }

    bool addID(uint32_t id, uint32_t id_mask, uint8_t ext, uint8_t ext_mask,
               uint8_t rtr, uint8_t rtr_mask)
    {
        if (id_cnt == 2)
            return false;

        uint32_t* reg = id_cnt < 1 ? &FR1 : &FR2;

        *reg = packRegister16(id, ext, rtr, ext) | (packRegister16(id_mask, ext_mask, rtr_mask, ext) << 16);
        ++id_cnt;
        return true;
    }

private:
    uint8_t id_cnt = 0;
};

struct ID16Filter : public Filter
{
    ID16Filter(uint8_t fifo) : Filter(FilterScale::DUAL16, FilterMode::ID, fifo)
    {
    }

    bool addID(uint32_t id, uint8_t ext, uint8_t rtr)
    {
        if (id_cnt == 4)
            return false;

        uint32_t* reg = id_cnt < 2 ? &FR1 : &FR2;

        const uint32_t bitmask = 0xFFFF;

        *reg &= ~(bitmask << (16 * (id_cnt % 2)));
        *reg |= packRegister16(id, ext, rtr, ext) << (16 * (id_cnt % 2));
        ++id_cnt;
        return true;
    }

private:
    uint8_t id_cnt = 0;
};

}  // namespace Canbus
}  // namespace Boardcore