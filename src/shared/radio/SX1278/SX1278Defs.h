/* Copyright (c) 2021 Skyward Experimental Rocketry
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

#include <drivers/spi/SPIDriver.h>

namespace Boardcore
{

/**
 * @brief Various SX1278 register/enums definitions.
 */
namespace SX1278Defs
{

/**
 * @brief Length of the internal FIFO
 */
constexpr int FIFO_LEN = 64;

/**
 * @brief Main oscillator frequency (Hz)
 */
constexpr int FXOSC = 32000000;

/**
 * @brief Frequency step (Hz) used in some calculations.
 */
constexpr float FSTEP = 61.03515625;

constexpr int TS_OSC = 250;
constexpr int TS_FS  = 60;

/**
 * @brief Get required spi config
 */
inline SPIBusConfig spiConfig()
{
    SPIBusConfig config = {};

    // FIXME(davide.mor): This depends on the device
    config.clockDivider = SPI::ClockDivider::DIV_64;
    config.mode         = SPI::Mode::MODE_0;
    config.bitOrder     = SPI::BitOrder::MSB_FIRST;
    // config.cs_setup_time_us = 30;
    // config.cs_hold_time_us  = 100;

    return config;
}

namespace RegOpMode
{

enum LongRangeMode
{
    LONG_RANGE_MODE_FSK  = 0 << 7,
    LONG_RANGE_MODE_LORA = 1 << 7
};

enum ModulationType
{
    MODULATION_TYPE_FSK = 0 << 6,
    MODULATION_TYPE_OOK = 1 << 6,
};

constexpr uint8_t LOW_FREQUENCY_MODE_ON = 1 << 3;

enum Mode
{
    MODE_SLEEP = 0b000,
    MODE_STDBY = 0b001,
    MODE_FSTX  = 0b010,
    MODE_TX    = 0b011,
    MODE_FSRX  = 0b100,
    MODE_RX    = 0b101,
};
}  // namespace RegOpMode

namespace RegPaConfig
{
constexpr uint8_t PA_SELECT_BOOST = 1 << 7;
}

namespace RegPaRamp
{
enum ModulationShaping
{
    MODULATION_SHAPING_NONE            = 0b00 << 5,
    MODULATION_SHAPING_GAUSSIAN_BT_1_0 = 0b01 << 5,
    MODULATION_SHAPING_GAUSSIAN_BT_0_5 = 0b10 << 5,
    MODULATION_SHAPING_GAUSSIAN_BT_0_3 = 0b11 << 5,
};
}

namespace RegOcp
{
constexpr uint8_t OCP_ON = 1 << 5;
}

namespace RegRxConfig
{
constexpr uint8_t RESTART_RX_ON_COLLISION      = 1 << 7;
constexpr uint8_t RESTART_RX_WITHOUT_PILL_LOCK = 1 << 6;
constexpr uint8_t RESTART_RX_WITH_PILL_LOCK    = 1 << 5;
constexpr uint8_t AFC_AUTO_ON                  = 1 << 4;
constexpr uint8_t AGC_AUTO_ON                  = 1 << 3;

constexpr uint8_t RX_TRIGGER_RSSI_INTERRUPT  = 0b001;
constexpr uint8_t RX_TRIGGER_PREAMBLE_DETECT = 0b110;
}  // namespace RegRxConfig

namespace RegPreambleDetector
{
constexpr uint8_t PREAMBLE_DETECTOR_ON = 1 << 7;

enum PreambleDetectorSize
{
    PREAMBLE_DETECTOR_SIZE_1_BYTE  = 0b00 << 5,
    PREAMBLE_DETECTOR_SIZE_2_BYTES = 0b01 << 5,
    PREAMBLE_DETECTOR_SIZE_3_BYTES = 0b10 << 5,
};
}  // namespace RegPreambleDetector

namespace RegSyncConfig
{
enum AutoRestartRxMode
{
    AUTO_RESTART_RX_MODE_OFF                  = 0b00 << 6,
    AUTO_RESTART_RX_MODE_ON_WITHOUT_PILL_LOCK = 0b01 << 6,
    AUTO_RESTART_RX_MODE_ON_WITH_PILL_LOCK    = 0b10 << 6,
};

enum PreamblePolarity
{
    PREAMBLE_POLARITY_AA = 0 << 5,
    PREAMBLE_POLARITY_55 = 1 << 5,
};

constexpr uint8_t SYNC_ON = 1 << 4;
}  // namespace RegSyncConfig

namespace RegPacketConfig1
{
enum PacketFormat
{
    PACKET_FORMAT_FIXED_LENGTH    = 0 << 7,
    PACKET_FORMAT_VARIABLE_LENGTH = 1 << 7,
};

enum DcFree
{
    DC_FREE_NONE       = 0b00 << 5,
    DC_FREE_MANCHESTER = 0b01 << 5,
    DC_FREE_WHITENING  = 0b10 << 5,
};

constexpr uint8_t CRC_ON             = 1 << 4;
constexpr uint8_t CRC_AUTO_CLEAR_OFF = 1 << 3;

enum AddressFiltering
{
    ADDRESS_FILTERING_NONE                    = 0b00 << 1,
    ADDRESS_FILTERING_MATCH_NODE              = 0b01 << 1,
    ADDRESS_FILTERING_MATCH_NODE_OR_BROADCAST = 0b10 << 1,
};

enum CrcWhiteningType
{
    CRC_WHITENING_TYPE_CCITT_CRC = 0,
    CRC_WHITENING_TYPE_IBM_CRC   = 1,
};
}  // namespace RegPacketConfig1

namespace RegPacketConfig2
{
enum DataMode
{
    DATA_MODE_CONTINUOS = 0 << 6,
    DATA_MODE_PACKET    = 1 << 6
};

constexpr uint8_t IO_HOME_ON = 1 << 5;
constexpr uint8_t BEACON_ON  = 1 << 4;
}  // namespace RegPacketConfig2

namespace RegFifoThresh
{
enum TxStartCondition
{
    TX_START_CONDITION_FIFO_LEVEL     = 0 << 7,
    TX_START_CONDITION_FIFO_NOT_EMPTY = 1 << 7,
};
}

namespace RegIrqFlags
{
constexpr uint16_t MODE_READY         = 1 << 7;
constexpr uint16_t RX_READY           = 1 << 6;
constexpr uint16_t TX_READY           = 1 << 5;
constexpr uint16_t PILL_LOCK          = 1 << 4;
constexpr uint16_t RSSI               = 1 << 3;
constexpr uint16_t TIMEOUT            = 1 << 2;
constexpr uint16_t PREAMBLE_DETECT    = 1 << 1;
constexpr uint16_t SYNC_ADDRESS_MATCH = 1 << 0;
constexpr uint16_t FIFO_FULL          = 1 << 15;
constexpr uint16_t FIFO_EMPTY         = 1 << 14;
constexpr uint16_t FIFO_LEVEL         = 1 << 13;
constexpr uint16_t FIFO_OVERRUN       = 1 << 12;
constexpr uint16_t PACKET_SENT        = 1 << 11;
constexpr uint16_t PAYLOAD_READY      = 1 << 10;
constexpr uint16_t CRC_OK             = 1 << 9;
constexpr uint16_t LOW_BAT            = 1 << 8;

}  // namespace RegIrqFlags

namespace RegPaDac
{
enum PaDac
{
    PA_DAC_DEFAULT_VALUE = 0x04,
    PA_DAC_PA_BOOST      = 0x07
};
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

    // Low frequency additional registers
    REG_AGC_REF_LF      = 0x61,
    REG_AGC_THRESH_1_LF = 0x62,
    REG_AGC_THRESH_2_LF = 0x63,
    REG_AGC_THRESH_3_LF = 0x61,
    REG_AGC_PILL_LF     = 0x70,

    // High frequency additional registers
    REG_AGC_REF_HF      = 0x61,
    REG_AGC_THRESH_1_HF = 0x62,
    REG_AGC_THRESH_2_HF = 0x63,
    REG_AGC_THRESH_3_HF = 0x61,
    REG_AGC_PILL_HF     = 0x70,
};

}  // namespace SX1278Defs

}  // namespace Boardcore
