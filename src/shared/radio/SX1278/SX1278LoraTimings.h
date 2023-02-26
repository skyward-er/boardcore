/* Copyright (c) 2022 Skyward Experimental Rocketry
 * Author: Davide Mor
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

namespace SX1278
{

namespace Lora
{

/**
 * @brief Computes the symbol duration in microseconds
 */
inline constexpr uint32_t symbolDuration(uint32_t spreading_factor,
                                         uint32_t bandwidth)
{
    // The number of chips contained in each symbol is 2^spreadingFactor
    uint32_t chip_count = (1 << spreading_factor);

    // The number of chips per seconds is equal to the bandwidth
    uint32_t chips_per_second = bandwidth;

    // With an SR of 12 (4096) this _barely_ fits in an uint32_t
    return (chip_count * 1000000) / chips_per_second;
}

/**
 * @brief Computes the nominal (without error correction) bitrate in b/s
 */
inline constexpr uint32_t nominalBitrate(uint32_t spreading_factor,
                                         uint32_t bandwidth)
{
    // The number of bits per symbol is equal to the spreading factor
    uint32_t bits_per_symbol = spreading_factor;

    // Calculate the number of symbols per seconds based on the symbol duration
    uint32_t symbols_per_second =
        (1000000 / symbolDuration(spreading_factor, bandwidth));

    return symbols_per_second * bits_per_symbol;
}

/**
 * @brief Computes the actual usable bitrate in b/s
 */
inline constexpr uint32_t effectiveBitrate(uint32_t spreading_factor,
                                           uint32_t bandwidth,
                                           uint32_t coding_rate)
{
    // Input user bits
    uint32_t input_bits = 4;
    // Output transmitted bits
    uint32_t output_bits = 4 + coding_rate;

    return (nominalBitrate(spreading_factor, bandwidth) / output_bits) *
           input_bits;
}

}  // namespace Lora

}  // namespace SX1278

}  // namespace Boardcore
