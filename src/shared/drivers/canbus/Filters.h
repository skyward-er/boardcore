/* Copyright (c) 2018 Skyward Experimental Rocketry
 * Author: Luca Erbetta
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

/**
 * @brief Base class for a Canbus filter bank.
 */
struct FilterBank
{
    uint8_t fifo;
    FilterScale scale;
    FilterMode mode;

    uint32_t FR1 = 0xFFFFFFFF;
    uint32_t FR2 = 0xFFFFFFFF;

    FilterBank(FilterScale scale, FilterMode mode, uint8_t fifo)
        : fifo(fifo), scale(scale), mode(mode)
    {
    }

protected:
    static uint32_t packRegister32(uint32_t id, uint8_t ide, uint8_t rtr,
                                   bool isExt)
    {
        uint32_t reg = 0;
        if (isExt)
        {
            reg = ((id & 0x1FFFFFFF) << 3);
        }
        else
        {
            reg = ((id & 0x7FF) << 21);
        }

        reg |= ((ide & 0x1) << 2) | ((rtr & 0x1) << 1);
        return reg;
    }

    static uint16_t packRegister16(uint32_t id, uint8_t ide, uint8_t rtr,
                                   bool isExt)
    {
        uint16_t reg = 0;
        if (isExt)
        {
            reg = ((id >> 18) & 0x7FF) << 5;
            reg |= ((id >> 15) & 0x07);
        }
        else
        {
            reg = ((id & 0x7FF) << 5);
        }

        reg |= ((ide & 0x1) << 3) | ((rtr & 0x1) << 4);
        return reg;
    }
};

/**
 * @brief 32 Bit mask filter bank.
 */
struct Mask32FilterBank : public FilterBank
{
    /**
     * @brief Construct a new 32 bit mask filter.
     *
     * @param id 29 bit frame identifier.
     * @param idMask 29 bit frame identifier mask.
     * @param ide Value for the IDE bit in the canbus frame.
     * @param ideMask Mask the ide bit.
     * @param rtr Value for the RTR bit in the canbus frame.
     * @param rtrMask Mask for RTR bit.
     * @param fifo Where to store the filtered messages (fifo 0 or fifo 1).
     */
    Mask32FilterBank(uint32_t id, uint32_t idMask, uint8_t ide, uint8_t ideMask,
                     uint8_t rtr, uint8_t rtrMask, uint8_t fifo)
        : FilterBank(FilterScale::SINGLE32, FilterMode::MASK, fifo)
    {
        scale = FilterScale::SINGLE32;
        mode  = FilterMode::MASK;

        FR1 = packRegister32(id, ide, rtr, true);
        FR2 = packRegister32(idMask, ideMask, rtrMask, true);
    }
};

struct ID32FilterBank : public FilterBank
{
    /**
     * @brief Construct a new filter matching exactly 32 bit identifiers.
     *
     * @param fifo Where to store the filtered messages (fifo 0 or fifo 1).
     */
    explicit ID32FilterBank(uint8_t fifo)
        : FilterBank(FilterScale::SINGLE32, FilterMode::ID, fifo)
    {
    }

    /**
     * @brief Adds a new ID to the filter. Returns false if there is no room for
     * the ID.
     *
     * @param id 29 bit message identifier.
     * @param ide Value for the IDE bit in the canbus frame.
     * @param rtr Value for the RTR bit in the canbus frame.
     */
    bool addID(uint32_t id, bool ide, uint8_t rtr)
    {
        if (idCount == 2)
            return false;

        uint32_t* reg = idCount < 1 ? &FR1 : &FR2;

        *reg = packRegister32(id, ide, rtr, true);
        ++idCount;
        return true;
    }

private:
    uint8_t idCount = 0;
};

struct Mask16FilterBank : public FilterBank
{
    /**
     * @brief Construct a new filter matching exactly 16 bit identifiers.
     *
     * @param fifo Where to store the filtered messages (fifo 0 or fifo 1).
     */
    explicit Mask16FilterBank(uint8_t fifo)
        : FilterBank(FilterScale::DUAL16, FilterMode::MASK, fifo)
    {
    }

    /**
     * @brief Add a 16 bit identifier mask, where the provided identifier is an
     * extended id (29 bit).
     * @warning only the first 3 bits of the extended frame identifier are
     * checked by the filter. See datasheet!
     *
     * @param id 29 bit frame identifier.
     * @param idMask 29 bit frame identifier mask.
     * @param ide Value for the IDE bit in the canbus frame.
     * @param ideMask Mask the ide bit.
     * @param rtr Set to one if filtering for Remote Transmission Requests.
     * @param rtrMask Mask for RTR bit.
     */
    bool addIDExt(uint32_t id, uint32_t idMask, uint8_t ide, uint8_t ideMask,
                  uint8_t rtr, uint8_t rtrMask)
    {
        return addID(id, idMask, ide, ideMask, rtr, rtrMask, true);
    }

    /**
     * @brief Add a 16 bit identifier mask, where the provided identifier is a
     * standard id (11 bit).
     *
     * @param id 11 bit frame identifier.
     * @param idMask 11 bit frame identifier mask.
     * @param ide Value for the IDE bit in the canbus frame.
     * @param ideMask Mask the ide bit.
     * @param rtr Set to one if filtering for Remote Transmission Requests.
     * @param rtrMask Mask for RTR bit.
     */
    bool addIDStd(uint32_t id, uint32_t idMask, uint8_t ide, uint8_t ideMask,
                  uint8_t rtr, uint8_t rtrMask)
    {
        return addID(id, idMask, ide, ideMask, rtr, rtrMask, false);
    }

private:
    bool addID(uint32_t id, uint32_t idMask, uint8_t ide, uint8_t ideMask,
               uint8_t rtr, uint8_t rtrMask, bool extendedId)
    {
        if (idCount == 2)
            return false;

        uint32_t* reg = idCount < 1 ? &FR1 : &FR2;

        *reg = packRegister16(id, ide, rtr, ide) |
               (packRegister16(idMask, ideMask, rtrMask, extendedId) << 16);
        ++idCount;
        return true;
    }

    uint8_t idCount = 0;
};

struct ID16FilterBank : public FilterBank
{
    /**
     * @brief Construct a new filter matching exactly 16 bits identifiers.
     *
     * @param fifo Where to store the filtered messages (fifo 0 or fifo 1).
     */
    explicit ID16FilterBank(uint8_t fifo)
        : FilterBank(FilterScale::DUAL16, FilterMode::ID, fifo)
    {
    }

    /**
     * @brief Add a 16 bit identifier match filter, where the provided
     * identifier is an extended id (29 bit).
     *
     * @param id 29 bit Identifier to be filterd.
     * @param ide Value of the IDE field in the canbus frame.
     * @param rtr Value of the RTR field in the canbus frame.
     * @return True if the identifier was added, false if no more space
     * available in the current bank.
     */
    bool addID32(uint32_t id, uint8_t ide, uint8_t rtr)
    {
        return addID(id, ide, rtr, true);
    }

    /**
     * @brief Add a 16 bit identifier match filter, where the provided
     * identifier is a standard id (11 bit).
     *
     * @param id 11 bit Identifier to be filterd.
     * @param ide Value of the IDE field in the canbus frame.
     * @param rtr Value of the RTR field in the canbus frame.
     * @return true if the identifier was added, false if no more space.
     * available in the current bank.
     */
    bool addID16(uint32_t id, uint8_t ide, uint8_t rtr)
    {
        return addID(id, ide, rtr, false);
    }

private:
    bool addID(uint32_t id, uint8_t ide, uint8_t rtr, bool extendedId)
    {
        if (idCount == 4)
            return false;

        uint32_t* reg = idCount < 2 ? &FR1 : &FR2;

        const uint32_t bitmask = 0xFFFF;

        *reg &= ~(bitmask << (16 * (idCount % 2)));
        *reg |= packRegister16(id, ide, rtr, extendedId)
                << (16 * (idCount % 2));
        ++idCount;
        return true;
    }
    uint8_t idCount = 0;
};

}  // namespace Canbus

}  // namespace Boardcore
