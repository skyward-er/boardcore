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

#include <drivers/spi/SPIBusInterface.h>
#include <cstdint>

namespace Boardcore
{

namespace SX1278
{

/**
 * @brief Main oscillator frequency (Hz)
 */
constexpr int FXOSC = 32000000;

/**
 * @brief Frequency step (Hz) used in some calculations.
 */
constexpr float FSTEP = 61.03515625;

inline SPIBusConfig getSpiBusConfig(SPI::ClockDivider clock_divider)
{
    SPIBusConfig bus_config = {};
    bus_config.clockDivider = clock_divider;
    bus_config.mode         = SPI::Mode::MODE_0;
    bus_config.bitOrder     = SPI::Order::MSB_FIRST;
    bus_config.byteOrder    = SPI::Order::MSB_FIRST;
    bus_config.writeBit     = SPI::WriteBit::INVERTED;

    return bus_config;
}

/**
 * @brief Represents a DIO..
 */
enum class Dio
{
    DIO0 = 0,
    DIO1 = 1,
    DIO2 = 2,
    DIO3 = 3,
    DIO4 = 4,
    DIO5 = 5
};

namespace RegDioMapping
{

inline constexpr uint16_t make(int dio0, int dio1, int dio2, int dio3, int dio4,
                               int dio5, bool map_preamble_detect)
{
    return (((dio0 & 0b11) << 14) | ((dio1 & 0b11) << 12) |
            ((dio2 & 0b11) << 10) | ((dio3 & 0b11) << 8) |
            ((dio4 & 0b11) << 6) | ((dio5 & 0b11) << 4) |
            (map_preamble_detect ? 1 : 0));
}

/**
 * @brief Represents an actual Dio mapping..
 */
struct Mapping
{
    constexpr Mapping() : raw(0) {}
    constexpr Mapping(int dio0, int dio1, int dio2, int dio3, int dio4,
                      int dio5, bool map_preamble_detect = false)
        : raw(make(dio0, dio1, dio2, dio3, dio4, dio5, map_preamble_detect))
    {
    }

    int getMapping(Dio dio) const
    {
        switch (dio)
        {
            case Dio::DIO0:
                return (raw >> 14) & 0b11;
            case Dio::DIO1:
                return (raw >> 12) & 0b11;
            case Dio::DIO2:
                return (raw >> 10) & 0b11;
            case Dio::DIO3:
                return (raw >> 8) & 0b11;
            case Dio::DIO4:
                return (raw >> 6) & 0b11;
            case Dio::DIO5:
                return (raw >> 4) & 0b11;
            default:
                return 0;
        }
    }

    bool operator==(const Mapping &other) const { return raw == other.raw; }

    bool operator!=(const Mapping &other) const { return raw != other.raw; }

    uint16_t raw;
};

}  // namespace RegDioMapping

/**
 * @brief Definitions only valid for Fsk
 */
namespace Fsk
{

/**
 * @brief Length of the internal FIFO
 */
constexpr int FIFO_LEN = 64;

namespace RegOpMode
{

enum ModulationType
{
    MODULATION_TYPE_FSK = 0 << 6,
    MODULATION_TYPE_OOK = 1 << 6,
};

enum Mode
{
    MODE_SLEEP = 0b000,
    MODE_STDBY = 0b001,
    MODE_FSTX  = 0b010,
    MODE_TX    = 0b011,
    MODE_FSRX  = 0b100,
    MODE_RX    = 0b101,
};

inline constexpr uint8_t make(Mode mode, bool low_frequency_mode_on,
                              ModulationType modulation_type)
{
    return mode | (low_frequency_mode_on ? (1 << 3) : 0) |
           (modulation_type << 5);
}

}  // namespace RegOpMode

namespace RegPaConfig
{

inline constexpr uint8_t make(uint8_t output_power, uint8_t max_power,
                              bool pa_select)
{
    return (output_power & 0b1111) | ((max_power & 0b111) << 4) |
           (pa_select ? 1 << 7 : 0);
}

}

namespace RegPaRamp
{
enum ModulationShaping
{
    MODULATION_SHAPING_NONE            = 0b00,
    MODULATION_SHAPING_GAUSSIAN_BT_1_0 = 0b01,
    MODULATION_SHAPING_GAUSSIAN_BT_0_5 = 0b10,
    MODULATION_SHAPING_GAUSSIAN_BT_0_3 = 0b11,
};

enum PaRamp
{
    PA_RAMP_MS_3_4 = 0b0000,
    PA_RAMP_MS_2   = 0b0001,
    PA_RAMP_MS_1   = 0b0010,
    PA_RAMP_US_500 = 0b0011,
    PA_RAMP_US_250 = 0b0100,
    PA_RAMP_US_125 = 0b0101,
    PA_RAMP_US_100 = 0b0110,
    PA_RAMP_US_62  = 0b0111,
    PA_RAMP_US_50  = 0b1000,
    PA_RAMP_US_40  = 0b1001,
    PA_RAMP_US_31  = 0b1010,
    PA_RAMP_US_25  = 0b1011,
    PA_RAMP_US_20  = 0b1100,
    PA_RAMP_US_15  = 0b1101,
    PA_RAMP_US_12  = 0b1110,
    PA_RAMP_US_10  = 0b1111,
};

inline constexpr uint8_t make(PaRamp pa_ramp,
                              ModulationShaping modulation_shaping)
{
    return pa_ramp | (modulation_shaping << 5);
}

}

namespace RegOcp
{

inline constexpr uint8_t make(uint8_t ocp_trim, bool ocp_on)
{
    return (ocp_trim & 0b11111) | (ocp_on ? 1 << 5 : 0);
}

}

namespace RegRxConfig
{

inline constexpr uint8_t make(bool rx_trigger_rssi_interrupt,
                              bool rx_trigger_preable_detect, bool agc_auto_on,
                              bool afc_auto_on, bool restart_rx_with_pll_lock,
                              bool restart_rx_without_pll_lock,
                              bool restart_rx_on_collision)
{
    return (rx_trigger_rssi_interrupt ? 0b001 : 0) |
           (rx_trigger_preable_detect ? 0b110 << 1 : 0) |
           (agc_auto_on ? 1 << 3 : 0) | (afc_auto_on ? 1 << 4 : 0) |
           (restart_rx_with_pll_lock ? 1 << 5 : 0) |
           (restart_rx_without_pll_lock ? 1 << 6 : 0) |
           (restart_rx_on_collision ? 1 << 7 : 0);
}

}  // namespace RegRxConfig

namespace RegRxBw
{

enum RxBw
{
    HZ_2600   = 0b10111,
    HZ_3100   = 0b01111,
    HZ_3900   = 0b00111,
    HZ_5200   = 0b10110,
    HZ_6300   = 0b01110,
    HZ_7800   = 0b00110,
    HZ_10400  = 0b10101,
    HZ_12500  = 0b01101,
    HZ_15600  = 0b00101,
    HZ_20800  = 0b10100,
    HZ_25000  = 0b01100,
    HZ_31300  = 0b00100,
    HZ_41700  = 0b10011,
    HZ_50000  = 0b01011,
    HZ_62500  = 0b00011,
    HZ_83300  = 0b10010,
    HZ_100000 = 0b01010,
    HZ_125000 = 0b00010,
    HZ_166700 = 0b10001,
    HZ_200000 = 0b01001,
    HZ_250000 = 0b00001,
};

inline constexpr uint8_t make(RxBw rx_bw) { return rx_bw; }

}

namespace RegAfcBw
{

using RxBwAfc = Boardcore::SX1278::Fsk::RegRxBw::RxBw;

inline constexpr uint8_t make(RxBwAfc rx_bw_afc) { return rx_bw_afc; }

}

namespace RegPreambleDetector
{

enum Size
{
    PREAMBLE_DETECTOR_SIZE_1_BYTE  = 0b00,
    PREAMBLE_DETECTOR_SIZE_2_BYTES = 0b01,
    PREAMBLE_DETECTOR_SIZE_3_BYTES = 0b10,
};

inline constexpr uint8_t make(int tol, Size size, bool on)
{
    return (tol & 0b11111) | (size << 5) | (on ? 1 << 7 : 0);
}

}  // namespace RegPreambleDetector

namespace RegSyncConfig
{
enum AutoRestartRxMode
{
    AUTO_RESTART_RX_MODE_OFF                  = 0b00,
    AUTO_RESTART_RX_MODE_ON_WITHOUT_PILL_LOCK = 0b01,
    AUTO_RESTART_RX_MODE_ON_WITH_PILL_LOCK    = 0b10,
};

enum PreamblePolarity
{
    PREAMBLE_POLARITY_AA = 0,
    PREAMBLE_POLARITY_55 = 1,
};

inline constexpr uint8_t make(int size, bool on,
                              PreamblePolarity preamble_polarity,
                              AutoRestartRxMode auto_restart_rx_mode)
{
    return ((size - 1) & 0b111) | (on ? 1 << 4 : 0) | (preamble_polarity << 5) |
           (auto_restart_rx_mode << 6);
}

}  // namespace RegSyncConfig

namespace RegPacketConfig1
{
enum PacketFormat
{
    PACKET_FORMAT_FIXED_LENGTH    = 0,
    PACKET_FORMAT_VARIABLE_LENGTH = 1,
};

enum DcFree
{
    DC_FREE_NONE       = 0b00,
    DC_FREE_MANCHESTER = 0b01,
    DC_FREE_WHITENING  = 0b10,
};

enum AddressFiltering
{
    ADDRESS_FILTERING_NONE                    = 0b00,
    ADDRESS_FILTERING_MATCH_NODE              = 0b01,
    ADDRESS_FILTERING_MATCH_NODE_OR_BROADCAST = 0b10,
};

enum CrcWhiteningType
{
    CRC_WHITENING_TYPE_CCITT_CRC = 0,
    CRC_WHITENING_TYPE_IBM_CRC   = 1,
};

inline constexpr uint8_t make(CrcWhiteningType crc_whitening_type,
                              AddressFiltering address_filtering,
                              bool crc_auto_clear_off, bool crc_on,
                              DcFree dc_free, PacketFormat packet_format)
{
    return crc_whitening_type | (address_filtering << 1) |
           (crc_auto_clear_off ? 1 << 3 : 0) | (crc_on ? 1 << 4 : 0) |
           (dc_free << 5) | (packet_format << 7);
}

}  // namespace RegPacketConfig1

namespace RegPacketConfig2
{
enum DataMode
{
    DATA_MODE_CONTINUOS = 0,
    DATA_MODE_PACKET    = 1
};

inline constexpr uint8_t make(bool beacon_on, bool io_home_power_frame,
                              bool io_home_on, DataMode data_mode)
{
    return (beacon_on ? 1 << 3 : 0) | (io_home_power_frame ? 1 << 4 : 0) |
           (io_home_on ? 1 << 5 : 0) | (data_mode << 6);
}

}  // namespace RegPacketConfig2

namespace RegFifoThresh
{
enum TxStartCondition
{
    TX_START_CONDITION_FIFO_LEVEL     = 0,
    TX_START_CONDITION_FIFO_NOT_EMPTY = 1,
};

inline constexpr uint8_t make(int fifo_threshold,
                              TxStartCondition tx_start_condition)
{
    return (fifo_threshold & 0b111111) | (tx_start_condition << 7);
}

}

namespace RegIrqFlags
{

enum IrqFlags
{
    MODE_READY         = 1 << 15,
    RX_READY           = 1 << 14,
    TX_READY           = 1 << 13,
    PILL_LOCK          = 1 << 12,
    RSSI               = 1 << 11,
    TIMEOUT            = 1 << 10,
    PREAMBLE_DETECT    = 1 << 9,
    SYNC_ADDRESS_MATCH = 1 << 8,
    FIFO_FULL          = 1 << 7,
    FIFO_EMPTY         = 1 << 6,
    FIFO_LEVEL         = 1 << 5,
    FIFO_OVERRUN       = 1 << 4,
    PACKET_SENT        = 1 << 3,
    PAYLOAD_READY      = 1 << 2,
    CRC_OK             = 1 << 1,
    LOW_BAT            = 1 << 0,
};

}  // namespace RegIrqFlags

namespace RegPaDac
{
enum PaDac
{
    PA_DAC_DEFAULT_VALUE = 0x04,
    PA_DAC_PA_BOOST      = 0x07
};

inline constexpr uint8_t make(PaDac pa_dac) { return pa_dac | (0x10 << 3); }

}

enum Registers
{
    REG_FIFO = 0x00,

    // Registers for common settings
    REG_OP_MODE     = 0x01,
    REG_BITRATE_MSB = 0x02,
    REG_BITRATE_LSB = 0x03,
    REG_FDEV_MSB    = 0x04,
    REG_FDEV_LSB    = 0x05,
    REG_FRF_MSB     = 0x06,
    REG_FRF_MID     = 0x07,
    REG_FRF_LSB     = 0x08,

    // Registers for the transmitter
    REG_PA_CONFIG = 0x09,
    REG_PA_RAMP   = 0x0a,
    REG_OCP       = 0x0b,

    // Registers for the receiver
    REG_LNA            = 0x0c,
    REG_RX_CONFIG      = 0x0d,
    REG_RSSI_CONFIG    = 0x0e,
    REG_RSSI_COLLISION = 0x0f,
    REG_RSSI_THRESH    = 0x10,
    REG_RSSI_VALUE     = 0x11,
    REG_RX_BW          = 0x12,
    REG_AFC_BW         = 0x13,
    REG_OOK_PEAK       = 0x14,
    REG_OOK_FIX        = 0x15,
    REG_OOK_AVG        = 0x16,
    // Reserved 17 to 19
    REG_AFC_FEI         = 0x1a,
    REG_AFC_MSB         = 0x1b,
    REG_AFC_LSB         = 0x1c,
    REG_FEI_MSB         = 0x1d,
    REG_FEI_LSB         = 0x1e,
    REG_PREAMBLE_DETECT = 0x1f,
    REG_RX_TIMEOUT_1    = 0x20,
    REG_RX_TIMEOUT_2    = 0x21,
    REG_RX_TIMEOUT_3    = 0x22,
    REG_RX_DELAY        = 0x23,

    // RC Oscillator registers
    REG_OSC = 0x24,

    // Packet handling registers
    REG_PREAMBLE_MSB          = 0x25,
    REG_PREAMBLE_LSB          = 0x26,
    REG_SYNC_CONFIG           = 0x27,
    REG_SYNC_VALUE_1          = 0x28,
    REG_SYNC_VALUE_2          = 0x29,
    REG_SYNC_VALUE_3          = 0x2a,
    REG_SYNC_VALUE_4          = 0x2b,
    REG_SYNC_VALUE_5          = 0x2c,
    REG_SYNC_VALUE_6          = 0x2d,
    REG_SYNC_VALUE_7          = 0x2e,
    REG_SYNC_VALUE_8          = 0x2f,
    REG_PACKET_CONFIG_1       = 0x30,
    REG_PACKET_CONFIG_2       = 0x31,
    REG_PACKET_PAYLOAD_LENGTH = 0x32,
    REG_NODE_ADRS             = 0x33,
    REG_BROADCAST_ADRS        = 0x34,
    REG_FIFO_THRESH           = 0x35,

    // Sequencer registers
    REG_SEQ_CONFIG_1 = 0x36,
    REG_SEQ_CONFIG_2 = 0x37,
    REG_TIMER_RESOL  = 0x38,
    REG_TIMER_1_COEF = 0x39,
    REG_TIMER_2_COEF = 0x3a,

    // Service registers
    REG_IMAGE_CAL = 0x3b,
    REG_TEMP      = 0x3c,
    REG_LOW_BAT   = 0x3d,

    // Status registers
    REG_IRQ_FLAGS_1 = 0x3e,
    REG_IRQ_FLAGS_2 = 0x3f,

    // IO Control registers
    REG_DIO_MAPPING_1 = 0x40,
    REG_DIO_MAPPING_2 = 0x41,

    // Version register
    REG_VERSION = 0x42,

    // Additional registers
    REG_PILL_HOP     = 0x44,
    REG_TCXO         = 0x4b,
    REG_PA_DAC       = 0x4d,
    REG_FORMER_TEMP  = 0x5b,
    REG_BITRATE_FRAC = 0x5d,
    REG_AGC_REF      = 0x61,
    REG_AGC_THRESH_1 = 0x62,
    REG_AGC_THRESH_2 = 0x63,
    REG_AGC_THRESH_3 = 0x64,
    REG_AGC_PILL     = 0x70,
};

static constexpr int DIO_MAPPINGS[6][8][4] =
    {[0] =
         {
             [RegOpMode::MODE_SLEEP] = {0, 0, 0, 0},
             [RegOpMode::MODE_STDBY] = {0, 0, 0, RegIrqFlags::LOW_BAT},
             [RegOpMode::MODE_FSTX]  = {0, 0, 0, RegIrqFlags::LOW_BAT},
             [RegOpMode::MODE_TX]    = {RegIrqFlags::PACKET_SENT, 0, 0,
                                     RegIrqFlags::LOW_BAT},
             [RegOpMode::MODE_FSRX]  = {0, 0, 0, RegIrqFlags::LOW_BAT},
             [RegOpMode::MODE_RX] =
                 {RegIrqFlags::PAYLOAD_READY, RegIrqFlags::CRC_OK, 0,
                  RegIrqFlags::LOW_BAT},
         },
     [1] =
         {
             [RegOpMode::MODE_SLEEP] = {RegIrqFlags::FIFO_LEVEL,
                                        RegIrqFlags::FIFO_EMPTY,
                                        RegIrqFlags::FIFO_FULL, 0},
             [RegOpMode::MODE_STDBY] = {RegIrqFlags::FIFO_LEVEL,
                                        RegIrqFlags::FIFO_EMPTY,
                                        RegIrqFlags::FIFO_FULL, 0},
             [RegOpMode::MODE_FSTX] =
                 {RegIrqFlags::FIFO_LEVEL, RegIrqFlags::FIFO_EMPTY,
                  RegIrqFlags::FIFO_FULL, 0},
             [RegOpMode::MODE_TX] = {RegIrqFlags::FIFO_LEVEL,
                                     RegIrqFlags::FIFO_EMPTY,
                                     RegIrqFlags::FIFO_FULL, 0},
             [RegOpMode::MODE_FSRX] =
                 {RegIrqFlags::FIFO_LEVEL, RegIrqFlags::FIFO_EMPTY,
                  RegIrqFlags::FIFO_FULL, 0},
             [RegOpMode::MODE_RX] = {RegIrqFlags::FIFO_LEVEL,
                                     RegIrqFlags::FIFO_EMPTY,
                                     RegIrqFlags::FIFO_FULL, 0},
         },
     [2] =
         {
             [RegOpMode::MODE_SLEEP] =
                 {RegIrqFlags::FIFO_FULL, 0, RegIrqFlags::FIFO_FULL,
                  RegIrqFlags::FIFO_FULL},
             [RegOpMode::MODE_STDBY] =
                 {RegIrqFlags::FIFO_FULL, 0, RegIrqFlags::FIFO_FULL,
                  RegIrqFlags::FIFO_FULL},
             [RegOpMode::MODE_FSTX] = {RegIrqFlags::FIFO_FULL, 0,
                                       RegIrqFlags::FIFO_FULL,
                                       RegIrqFlags::FIFO_FULL},
             [RegOpMode::MODE_TX] =
                 {RegIrqFlags::FIFO_FULL, 0, RegIrqFlags::FIFO_FULL,
                  RegIrqFlags::FIFO_FULL},
             [RegOpMode::MODE_FSRX] = {RegIrqFlags::FIFO_FULL, 0,
                                       RegIrqFlags::FIFO_FULL,
                                       RegIrqFlags::FIFO_FULL},
             [RegOpMode::MODE_RX] =
                 {RegIrqFlags::FIFO_FULL, RegIrqFlags::RX_READY,
                  RegIrqFlags::TIMEOUT, RegIrqFlags::SYNC_ADDRESS_MATCH},
         },
     [3] =
         {
             [RegOpMode::MODE_SLEEP] =
                 {
                     RegIrqFlags::FIFO_EMPTY,
                     0,
                     RegIrqFlags::FIFO_EMPTY,
                     RegIrqFlags::FIFO_EMPTY,
                 },
             [RegOpMode::MODE_STDBY] =
                 {
                     RegIrqFlags::FIFO_EMPTY,
                     0,
                     RegIrqFlags::FIFO_EMPTY,
                     RegIrqFlags::FIFO_EMPTY,
                 },
             [RegOpMode::MODE_FSTX] =
                 {
                     RegIrqFlags::FIFO_EMPTY,
                     0,
                     RegIrqFlags::FIFO_EMPTY,
                     RegIrqFlags::FIFO_EMPTY,
                 },
             [RegOpMode::MODE_TX] =
                 {
                     RegIrqFlags::FIFO_EMPTY,
                     RegIrqFlags::TX_READY,
                     RegIrqFlags::FIFO_EMPTY,
                     RegIrqFlags::FIFO_EMPTY,
                 },
             [RegOpMode::MODE_FSRX] =
                 {
                     RegIrqFlags::FIFO_EMPTY,
                     0,
                     RegIrqFlags::FIFO_EMPTY,
                     RegIrqFlags::FIFO_EMPTY,
                 },
             [RegOpMode::MODE_RX] =
                 {
                     RegIrqFlags::FIFO_EMPTY,
                     0,
                     RegIrqFlags::FIFO_EMPTY,
                     RegIrqFlags::FIFO_EMPTY,
                 },
         },
     [4] =
         {
             [RegOpMode::MODE_SLEEP] = {0, 0, 0, 0},
             [RegOpMode::MODE_STDBY] = {RegIrqFlags::LOW_BAT, 0, 0, 0},
             [RegOpMode::MODE_FSTX]  = {RegIrqFlags::LOW_BAT,
                                       RegIrqFlags::PILL_LOCK, 0, 0},
             [RegOpMode::MODE_TX]    = {RegIrqFlags::LOW_BAT,
                                     RegIrqFlags::PILL_LOCK, 0, 0},
             [RegOpMode::MODE_FSRX]  = {RegIrqFlags::LOW_BAT,
                                       RegIrqFlags::PILL_LOCK, 0, 0},
             [RegOpMode::MODE_RX] =
                 {RegIrqFlags::LOW_BAT, RegIrqFlags::PILL_LOCK,
                  RegIrqFlags::TIMEOUT,
                  RegIrqFlags::RSSI | RegIrqFlags::PREAMBLE_DETECT},
         },
     [5] = {
         [RegOpMode::MODE_SLEEP] = {0, 0, 0, 0},
         [RegOpMode::MODE_STDBY] = {0, 0, 0, RegIrqFlags::MODE_READY},
         [RegOpMode::MODE_FSTX]  = {0, RegIrqFlags::PILL_LOCK, 0,
                                   RegIrqFlags::MODE_READY},

         [RegOpMode::MODE_TX]   = {0, RegIrqFlags::PILL_LOCK, 0,
                                 RegIrqFlags::MODE_READY},
         [RegOpMode::MODE_FSRX] = {0, RegIrqFlags::PILL_LOCK, 0,
                                   RegIrqFlags::MODE_READY},
         [RegOpMode::MODE_RX]   = {0, RegIrqFlags::PILL_LOCK, 0,
                                 RegIrqFlags::MODE_READY},
     }};
}  // namespace Fsk

/**
 * @brief Definitions only valid for LoRa
 */
namespace Lora
{

namespace RegOpMode
{

enum Mode
{
    MODE_SLEEP        = 0b000,
    MODE_STDBY        = 0b001,
    MODE_FSTX         = 0b010,
    MODE_TX           = 0b011,
    MODE_FSRX         = 0b100,
    MODE_RXCONTINUOUS = 0b101,
    MODE_RXSINGLE     = 0b110,
    MODE_CAD          = 0b111
};

inline constexpr uint8_t make(Mode mode, bool low_frequency_mode_on,
                              bool access_shared_reg)
{
    return mode | (low_frequency_mode_on ? (1 << 3) : 0) |
           (access_shared_reg ? (1 << 6) : 0) | (1 << 7);
}

}  // namespace RegOpMode

namespace RegPaConfig
{

inline constexpr uint8_t make(uint8_t output_power, uint8_t max_power,
                              bool pa_select)
{
    return (output_power & 0b1111) | ((max_power & 0b111) << 4) |
           (pa_select ? 1 << 7 : 0);
}

}  // namespace RegPaConfig

namespace RegOcp
{

inline constexpr uint8_t make(uint8_t ocp_trim, bool ocp_on)
{
    return (ocp_trim & 0b11111) | (ocp_on ? 1 << 5 : 0);
}

}  // namespace RegOcp

namespace RegIrqFlags
{

enum IrqFlags
{
    RX_TIMEOUT          = 1 << 7,
    RX_DONE             = 1 << 6,
    PAYLOAD_CRC_ERROR   = 1 << 5,
    VALID_HEADER        = 1 << 4,
    TX_DONE             = 1 << 3,
    CAD_DONE            = 1 << 2,
    FHSS_CHANGE_CHANNEL = 1 << 1,
    CAD_DETECTED        = 1 << 0,
};

}

namespace RegModemConfig1
{

enum Bw
{
    BW_HZ_7800   = 0b0000,
    BW_HZ_10400  = 0b0001,
    BW_HZ_15600  = 0b0010,
    BW_HZ_20800  = 0b0011,
    BW_HZ_31250  = 0b0100,
    BW_HZ_41700  = 0b0101,
    BW_HZ_62500  = 0b0110,
    BW_HZ_125000 = 0b0111,
    BW_HZ_250000 = 0b1000,
    BW_HZ_500000 = 0b1001,
};

inline constexpr uint32_t bandwidthToInt(Bw bw)
{
    switch (bw)
    {
        case RegModemConfig1::BW_HZ_7800:
            return 7800;
        case RegModemConfig1::BW_HZ_10400:
            return 10400;
        case RegModemConfig1::BW_HZ_15600:
            return 15600;
        case RegModemConfig1::BW_HZ_20800:
            return 20800;
        case RegModemConfig1::BW_HZ_31250:
            return 31250;
        case RegModemConfig1::BW_HZ_41700:
            return 41700;
        case RegModemConfig1::BW_HZ_62500:
            return 62500;
        case RegModemConfig1::BW_HZ_125000:
            return 125000;
        case RegModemConfig1::BW_HZ_250000:
            return 250000;
        case RegModemConfig1::BW_HZ_500000:
            return 500000;
    }

    // Gcc complains...
    return 0;
}

enum Cr
{
    CR_1 = 0b001,
    CR_2 = 0b010,
    CR_3 = 0b011,
    CR_4 = 0b100
};

inline constexpr uint8_t make(bool implicit_mode_on, Cr coding_rate, Bw bw)
{
    return (implicit_mode_on ? 1 : 0) | (coding_rate << 1) | (bw << 4);
}

}  // namespace RegModemConfig1

namespace RegModemConfig2
{

enum Sf
{
    SF_6  = 6,
    SF_7  = 7,
    SF_8  = 8,
    SF_9  = 9,
    SF_10 = 10,
    SF_11 = 11,
    SF_12 = 12,
};

inline constexpr uint8_t make(bool rx_payload_crc_on, bool tx_continuous_mode,
                              Sf spreading_factor)
{
    return (rx_payload_crc_on ? 1 << 2 : 0) |
           (tx_continuous_mode ? 1 << 3 : 0) | (spreading_factor << 4);
}

}  // namespace RegModemConfig2

namespace RegModemConfig3
{

inline constexpr uint8_t make(bool agc_auto_on, bool low_data_rate_optimize)
{
    return (agc_auto_on ? 1 << 2 : 0) | (low_data_rate_optimize ? 1 << 3 : 0);
}

}  // namespace RegModemConfig3

namespace RegDetectOptimize
{

inline constexpr uint8_t make(uint8_t detection_optimize, bool automatic_if_on)
{
    return (detection_optimize & 0b11) | (automatic_if_on ? 1 << 7 : 0);
}

}  // namespace RegDetectOptimize

namespace RegPaDac
{

inline constexpr uint8_t make(bool pa_boost)
{
    return (pa_boost ? 0x07 : 0x04) | (0x10 << 3);
}

}  // namespace RegPaDac

enum Registers
{
    REG_FIFO = 0x00,

    // Registers for common settings
    REG_OP_MODE = 0x01,
    REG_FRF_MSB = 0x06,
    REG_FRF_MID = 0x07,
    REG_FRF_LSB = 0x08,

    // Registers for RF blocks
    REG_PA_CONFIG = 0x09,
    REG_PA_RAMP   = 0x0a,
    REG_OCP       = 0x0b,
    REG_LNA       = 0x0c,

    // Lora page registers
    REG_FIFO_ADDR_PTR           = 0x0d,
    REG_FIFO_TX_BASE_ADDR       = 0x0e,
    REG_FIFO_RX_BASE_ADDR       = 0x0f,
    REG_FIFO_RX_CURRENT_ADDR    = 0x10,
    REG_IRQ_FLAGS_MASK          = 0x11,
    REG_IRQ_FLAGS               = 0x12,
    REG_RX_NB_BYTES             = 0x13,
    REG_RX_HEADER_CNT_VALUE_MSB = 0x14,
    REG_RX_HEADER_CNT_VALUE_LSB = 0x15,
    REG_RX_PACKET_CNT_VALUE_MSB = 0x16,
    REG_RX_PACKET_CNT_VALUE_LSB = 0x17,
    REG_MODEM_STAT              = 0x18,
    REG_PKT_SNR_VALUE           = 0x19,
    REG_PKT_RSSI_VALUE          = 0x1a,
    REG_RSSI_VALUE              = 0x1b,
    REG_HOP_CHANNEL             = 0x1c,
    REG_MODEM_CONFIG_1          = 0x1d,
    REG_MODEM_CONFIG_2          = 0x1e,
    REG_SYMB_TIMEOUT_LSB        = 0x1f,
    REG_PREAMBLE_MSB            = 0x20,
    REG_PREAMBLE_LSB            = 0x21,
    REG_PAYLOAD_LENGTH          = 0x22,
    REG_MAX_PAYLOAD_LENGTH      = 0x23,
    REG_HOP_PERIOD              = 0x24,
    REG_FIFO_RX_BYTE_ADDR       = 0x25,
    REG_MODEM_CONFIG_3          = 0x26,
    REG_PPM_CORRECTION          = 0x27,
    REG_FEI_MSB                 = 0x28,
    REG_FEI_MID                 = 0x29,
    REG_FEI_LSB                 = 0x2a,
    REG_RSSI_WIDEBAND           = 0x2c,
    REG_IF_FREQ_2               = 0x2f,
    REG_IF_FREQ_1               = 0x30,
    REG_DETECT_OPTIMIZE         = 0x31,
    REG_INVERT_IQ               = 0x33,
    REG_HIGH_BW_OPTIMIZE_1      = 0x36,
    REG_DETECTION_THRESHOLD     = 0x37,
    REG_SYNC_WORD               = 0x39,
    REG_HIGH_BW_OPTIMIZE_2      = 0x3a,
    REG_INVERT_IQ_2             = 0x3b,

    // IO Control registers
    REG_DIO_MAPPING_1 = 0x40,
    REG_DIO_MAPPING_2 = 0x41,

    // Version register
    REG_VERSION = 0x42,

    // Additional registers
    REG_TCXO         = 0x4b,
    REG_PA_DAC       = 0x4d,
    REG_FORMER_TEMP  = 0x5b,
    REG_AGC_REF      = 0x61,
    REG_AGC_THRESH_1 = 0x62,
    REG_AGC_THRESH_2 = 0x63,
    REG_AGC_THRESH_3 = 0x64,
    REG_AGC_PILL     = 0x70,
};

static constexpr int DIO_MAPPINGS[6][4] = {
    [0] = {RegIrqFlags::RX_DONE, RegIrqFlags::TX_DONE, RegIrqFlags::CAD_DONE,
           0},
    [1] = {RegIrqFlags::RX_TIMEOUT, RegIrqFlags::FHSS_CHANGE_CHANNEL,
           RegIrqFlags::CAD_DETECTED, 0},
    [2] = {RegIrqFlags::FHSS_CHANGE_CHANNEL, RegIrqFlags::FHSS_CHANGE_CHANNEL,
           RegIrqFlags::FHSS_CHANGE_CHANNEL, 0},
    [3] = {RegIrqFlags::CAD_DONE, RegIrqFlags::VALID_HEADER,
           RegIrqFlags::PAYLOAD_CRC_ERROR, 0},
    [4] = {RegIrqFlags::CAD_DETECTED, 0, 0, 0},
    [5] = {0, 0, 0, 0}};

}  // namespace Lora

}  // namespace SX1278

}  // namespace Boardcore
