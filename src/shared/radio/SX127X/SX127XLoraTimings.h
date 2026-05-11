
#pragma once

#include <cstdint>

namespace Boardcore
{

namespace SX127x
{

namespace Lora
{


inline constexpr uint32_t symbolDuration(uint32_t spreading_factor,
                                         uint32_t bandwidth)
{
    uint32_t chip_count = (1 << spreading_factor);

    uint32_t chips_per_second = bandwidth;

    return (chip_count * 1000000) / chips_per_second;
}


inline constexpr uint32_t nominalBitrate(uint32_t spreading_factor,
                                         uint32_t bandwidth)
{
    uint32_t bits_per_symbol = spreading_factor;

    uint32_t symbols_per_second =
        (1000000 / symbolDuration(spreading_factor, bandwidth));

    return symbols_per_second * bits_per_symbol;
}

inline constexpr uint32_t effectiveBitrate(uint32_t spreading_factor,
                                           uint32_t bandwidth,
                                           uint32_t coding_rate)
{
    uint32_t input_bits = 4;
    uint32_t output_bits = 4 + coding_rate;

    return (nominalBitrate(spreading_factor, bandwidth) / output_bits) *
           input_bits;
}

}  // namespace Lora

}  // namespace SX127x

}  // namespace Boardcore
