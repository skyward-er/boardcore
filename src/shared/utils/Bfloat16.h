/* Copyright (c) 2021 Skyward Experimental Rocketry
 * Author: Niccolò Betto
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

#include <utils/Numeric.h>

#include <cmath>
#include <cstdint>

namespace Boardcore
{
/**
 * @brief Google's bfloat16 type.
 *
 * This type is used to represent floating-point numbers with reduced
 * precision, but with the same exponent range as float.
 *
 * It's a 16-bit representation where the first 8 bits are the exponent and the
 * last 7 bits are the mantissa, with the least significant bits being dropped.
 *
 * Casting to and from float is fast since it only involves shifting bits.
 */
struct bfloat16
{
    bfloat16(float value) : storage{cast_to_storage(value)} {}
    explicit operator float() { return cast_from_storage(storage); }

    // Utility casts for debugging purposes of the underlying storage
    explicit operator uint16_t() { return storage; }
    explicit operator uint32_t() { return storage << 16; }

private:
    uint16_t storage;

    static uint16_t cast_to_storage(float value)
    {
        // NaNs need special handling, since some of the NaN bits may be beyond
        // the truncation point
        if (std::isnan(value))
            return NAN_VALUE;

        uint32_t bits = std::bit_cast<uint32_t>(value);
        return bits >> 16;
    }

    static float cast_from_storage(uint16_t value)
    {
        uint32_t bits = static_cast<uint32_t>(value);
        return std::bit_cast<float>(bits << 16);
    }

    // A NaN value suitable for bfloat16, NaN bits need to appear before
    // truncation point
    static constexpr uint16_t NAN_VALUE = 0b0'11111111'1000000;
};
}  // namespace Boardcore
