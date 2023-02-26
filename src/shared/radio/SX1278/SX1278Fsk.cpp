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

#include "SX1278Fsk.h"

#include <kernel/scheduler/scheduler.h>
#include <utils/Debug.h>

#include <cmath>

namespace Boardcore
{

using namespace SX1278;
using namespace SX1278::Fsk;

long long now() { return miosix::getTick() * 1000 / miosix::TICK_FREQ; }

// Default values for registers
constexpr uint8_t REG_OP_MODE_DEFAULT = RegOpMode::LONG_RANGE_MODE_FSK |
                                        RegOpMode::MODULATION_TYPE_FSK |
                                        RegOpMode::LOW_FREQUENCY_MODE_ON;

constexpr uint8_t REG_SYNC_CONFIG_DEFAULT =
    RegSyncConfig::AUTO_RESTART_RX_MODE_OFF |
    RegSyncConfig::PREAMBLE_POLARITY_55 | RegSyncConfig::SYNC_ON;

constexpr uint8_t REG_IMAGE_CAL_DEFAULT = RegImageCal::TEMP_TRESHOLD_10DEG;

// Enable:
// - PayloadReady, PacketSent on DIO0 (mode 00)
// - FifoLevel on DIO1 (mode 00)
// - TxReady on DIO3 (mode 01)
constexpr DioMapping DEFAULT_MAPPING = DioMapping(0, 0, 0, 1, 0, 0, false);

SX1278Fsk::Error SX1278Fsk::init(const Config &config)
{
    // First probe for the device
    if (!checkVersion())
    {
        TRACE("[sx1278] Wrong chipid\n");
        return Error::BAD_VALUE;
    }

    Error err;
    if ((err = configure(config)) != Error::NONE)
        return err;

    return Error::NONE;
}

bool SX1278Fsk::checkVersion()
{
    Lock guard(*this);
    SPITransaction spi(slave, SPITransaction::WriteBit::INVERTED);

    return spi.readRegister(REG_VERSION) == 0x12;
}

SX1278Fsk::Error SX1278Fsk::configure(const Config &config)
{
    // First cycle the device to bring it into Fsk mode (can only be changed in
    // sleep)
    setDefaultMode(RegOpMode::MODE_SLEEP, DEFAULT_MAPPING, false, false);
    // Make sure the device remains in standby and not in sleep
    setDefaultMode(RegOpMode::MODE_STDBY, DEFAULT_MAPPING, false, false);

    // Lock the bus
    Lock guard(*this);
    LockMode guard_mode(*this, guard, RegOpMode::MODE_STDBY, DEFAULT_MAPPING);

    if (!waitForIrqBusy(guard_mode, RegIrqFlags::MODE_READY, 1000))
        return Error::IRQ_TIMEOUT;

    setBitrate(config.bitrate);
    setFreqDev(config.freq_dev);
    setFreqRF(config.freq_rf);

    setRxBw(config.rx_bw);
    setAfcBw(config.afc_bw);

    setOcp(config.ocp);

    uint8_t sync_word[2] = {0x12, 0xad};
    setSyncWord(sync_word, 2);
    setPreambleLen(2);
    setPa(config.power, config.pa_boost);
    setShaping(config.shaping);

    // Setup generic parameters
    {
        SPITransaction spi(slave, SPITransaction::WriteBit::INVERTED);

        spi.writeRegister(REG_RX_CONFIG,
                          RegRxConfig::AFC_AUTO_ON | RegRxConfig::AGC_AUTO_ON |
                              RegRxConfig::RX_TRIGGER_PREAMBLE_DETECT |
                              RegRxConfig::RX_TRIGGER_RSSI_INTERRUPT);
        spi.writeRegister(REG_RSSI_THRESH, 0xff);
        spi.writeRegister(
            REG_PREAMBLE_DETECT,
            RegPreambleDetector::PREAMBLE_DETECTOR_ON |
                RegPreambleDetector::PREAMBLE_DETECTOR_SIZE_2_BYTES | 0x0a);
        spi.writeRegister(REG_RX_TIMEOUT_1, 0x00);
        spi.writeRegister(REG_RX_TIMEOUT_2, 0x00);
        spi.writeRegister(REG_RX_TIMEOUT_3, 0x00);
        spi.writeRegister(REG_PACKET_CONFIG_1,
                          RegPacketConfig1::PACKET_FORMAT_VARIABLE_LENGTH |
                              static_cast<uint8_t>(config.dc_free) |
                              RegPacketConfig1::CRC_ON |
                              RegPacketConfig1::ADDRESS_FILTERING_NONE |
                              RegPacketConfig1::CRC_WHITENING_TYPE_CCITT_CRC);
        spi.writeRegister(REG_PACKET_CONFIG_2,
                          RegPacketConfig2::DATA_MODE_PACKET);
        spi.writeRegister(
            REG_FIFO_THRESH,
            RegFifoThresh::TX_START_CONDITION_FIFO_NOT_EMPTY | 0x0f);
        spi.writeRegister(REG_NODE_ADRS, 0x00);
        spi.writeRegister(REG_BROADCAST_ADRS, 0x00);
    }

    // imageCalibrate();
    return Error::NONE;
}

ssize_t SX1278Fsk::receive(uint8_t *pkt, size_t max_len)
{
    Lock guard(*this);
    LockMode guard_mode(*this, guard, RegOpMode::MODE_RX, DEFAULT_MAPPING,
                        false, true);

    uint8_t len = 0;
    do
    {
        // Special wait for payload ready
        waitForIrq(guard_mode, RegIrqFlags::PAYLOAD_READY, true);
        last_rx_rssi = getRssi();

        {
            SPITransaction spi(slave, SPITransaction::WriteBit::INVERTED);
            len = spi.readRegister(REG_FIFO);
            if (len > max_len)
                return -1;

            spi.readRegisters(REG_FIFO, pkt, len);
        }

        // For some reason this sometimes happen?
    } while (len == 0);

    return len;
}

bool SX1278Fsk::send(uint8_t *pkt, size_t len)
{
    // Packets longer than FIFO_LEN (-1 for the len byte) are not supported
    if (len > MTU)
        return false;

    // This shouldn't be needed, but for some reason the device "lies" about
    // being ready, so lock up if we are going too fast
    rateLimitTx();

    Lock guard(*this);
    LockMode guard_mode(*this, guard, RegOpMode::MODE_TX, DEFAULT_MAPPING, true,
                        false);

    waitForIrq(guard_mode, RegIrqFlags::TX_READY);

    {
        SPITransaction spi(slave, SPITransaction::WriteBit::INVERTED);

        spi.writeRegister(REG_FIFO, static_cast<uint8_t>(len));
        spi.writeRegisters(REG_FIFO, pkt, len);
    }

    // Wait for packet sent
    waitForIrq(guard_mode, RegIrqFlags::PACKET_SENT);

    last_tx = now();
    return true;
}

float SX1278Fsk::getLastRxFei()
{
    Lock guard(*this);
    return getFei();
}

float SX1278Fsk::getLastRxRssi() { return last_rx_rssi; }

float SX1278Fsk::getCurRssi()
{
    Lock guard(*this);
    return getRssi();
}

void SX1278Fsk::rateLimitTx()
{
    const long long RATE_LIMIT = 2;

    long long delta = now() - last_tx;
    if (delta <= RATE_LIMIT)
    {
        miosix::Thread::sleep(RATE_LIMIT - delta);
    }
}

void SX1278Fsk::imageCalibrate()
{
    SPITransaction spi(slave, SPITransaction::WriteBit::INVERTED);
    spi.writeRegister(REG_IMAGE_CAL, REG_IMAGE_CAL_DEFAULT | (1 << 6));

    // Wait for calibration complete by polling on running register
    while (spi.readRegister(REG_IMAGE_CAL) & (1 << 5))
        miosix::delayUs(10);
}

DioMask SX1278Fsk::getDioMaskFromIrqFlags(IrqFlags flags, Mode mode,
                                          DioMapping mapping)
{
    DioMask dio_mask;

    if (DIO_MAPPINGS[0][mode][mapping.getMapping(Dio::DIO0)] & flags)
        dio_mask.set(Dio::DIO0);

    if (DIO_MAPPINGS[1][mode][mapping.getMapping(Dio::DIO1)] & flags)
        dio_mask.set(Dio::DIO1);

    if (DIO_MAPPINGS[2][mode][mapping.getMapping(Dio::DIO2)] & flags)
        dio_mask.set(Dio::DIO2);

    if (DIO_MAPPINGS[3][mode][mapping.getMapping(Dio::DIO3)] & flags)
        dio_mask.set(Dio::DIO3);

    if (DIO_MAPPINGS[4][mode][mapping.getMapping(Dio::DIO4)] & flags)
        dio_mask.set(Dio::DIO4);

    if (DIO_MAPPINGS[5][mode][mapping.getMapping(Dio::DIO5)] & flags)
        dio_mask.set(Dio::DIO5);

    return dio_mask;
}

ISX1278::IrqFlags SX1278Fsk::getIrqFlags()
{
    SPITransaction spi(slave, SPITransaction::WriteBit::INVERTED);

    uint8_t flags_msb = spi.readRegister(REG_IRQ_FLAGS_1);
    uint8_t flags_lsb = spi.readRegister(REG_IRQ_FLAGS_2);

    return (flags_msb << 8 | flags_lsb);
}

void SX1278Fsk::resetIrqFlags(IrqFlags flags)
{
    // Mask only resettable flags
    flags &= RegIrqFlags::RSSI | RegIrqFlags::PREAMBLE_DETECT |
             RegIrqFlags::SYNC_ADDRESS_MATCH | RegIrqFlags::FIFO_OVERRUN |
             RegIrqFlags::LOW_BAT;

    if (flags != 0)
    {
        SPITransaction spi(slave, SPITransaction::WriteBit::INVERTED);
        spi.writeRegister(REG_IRQ_FLAGS_1, flags >> 8);
        spi.writeRegister(REG_IRQ_FLAGS_2, flags);
    }
}

float SX1278Fsk::getRssi()
{
    SPITransaction spi(slave, SPITransaction::WriteBit::INVERTED);

    return static_cast<float>(spi.readRegister(REG_RSSI_VALUE)) * -0.5f;
}

float SX1278Fsk::getFei()
{
    SPITransaction spi(slave, SPITransaction::WriteBit::INVERTED);

    // Order of read is important!!
    uint8_t fei_msb = spi.readRegister(REG_FEI_MSB);
    uint8_t fei_lsb = spi.readRegister(REG_FEI_LSB);

    uint16_t fei = (static_cast<uint16_t>(fei_msb) << 8) |
                   (static_cast<uint16_t>(fei_lsb));

    return static_cast<float>(fei) * FSTEP;
}

void SX1278Fsk::setMode(Mode mode)
{
    SPITransaction spi(slave, SPITransaction::WriteBit::INVERTED);
    spi.writeRegister(REG_OP_MODE, REG_OP_MODE_DEFAULT | mode);
}

void SX1278Fsk::setMapping(SX1278::DioMapping mapping)
{
    SPITransaction spi(slave, SPITransaction::WriteBit::INVERTED);
    spi.writeRegister(REG_DIO_MAPPING_1, mapping.raw >> 8);
    spi.writeRegister(REG_DIO_MAPPING_2, mapping.raw);
}

void SX1278Fsk::setBitrate(int bitrate)
{
    uint16_t val = FXOSC / bitrate;

    // Update values
    SPITransaction spi(slave, SPITransaction::WriteBit::INVERTED);
    spi.writeRegister(REG_BITRATE_MSB, val >> 8);
    spi.writeRegister(REG_BITRATE_LSB, val);
}

void SX1278Fsk::setFreqDev(int freq_dev)
{
    uint16_t val = freq_dev / FSTEP;
    SPITransaction spi(slave, SPITransaction::WriteBit::INVERTED);
    spi.writeRegister(REG_FDEV_MSB, (val >> 8) & 0x3f);
    spi.writeRegister(REG_FDEV_LSB, val);
}

void SX1278Fsk::setFreqRF(int freq_rf)
{
    uint32_t val = freq_rf / FSTEP;

    // Update values
    SPITransaction spi(slave, SPITransaction::WriteBit::INVERTED);
    spi.writeRegister(REG_FRF_MSB, val >> 16);
    spi.writeRegister(REG_FRF_MID, val >> 8);
    spi.writeRegister(REG_FRF_LSB, val);
}

void SX1278Fsk::setOcp(int ocp)
{
    SPITransaction spi(slave, SPITransaction::WriteBit::INVERTED);
    if (ocp == 0)
    {
        spi.writeRegister(REG_OCP, 0);
    }
    else if (ocp <= 120)
    {
        uint8_t raw = (ocp - 45) / 5;
        spi.writeRegister(REG_OCP, RegOcp::OCP_ON | raw);
    }
    else
    {
        uint8_t raw = (ocp + 30) / 10;
        spi.writeRegister(REG_OCP, RegOcp::OCP_ON | raw);
    }
}

void SX1278Fsk::setSyncWord(uint8_t value[], int size)
{
    SPITransaction spi(slave, SPITransaction::WriteBit::INVERTED);
    spi.writeRegister(REG_SYNC_CONFIG, REG_SYNC_CONFIG_DEFAULT | size);

    for (int i = 0; i < size; i++)
    {
        spi.writeRegister(REG_SYNC_VALUE_1 + i, value[i]);
    }
}

void SX1278Fsk::setRxBw(RxBw rx_bw)
{
    SPITransaction spi(slave, SPITransaction::WriteBit::INVERTED);
    spi.writeRegister(REG_RX_BW, static_cast<uint8_t>(rx_bw));
}

void SX1278Fsk::setAfcBw(RxBw afc_bw)
{
    SPITransaction spi(slave, SPITransaction::WriteBit::INVERTED);
    spi.writeRegister(REG_AFC_BW, static_cast<uint8_t>(afc_bw));
}

void SX1278Fsk::setPreambleLen(int len)
{
    SPITransaction spi(slave, SPITransaction::WriteBit::INVERTED);
    spi.writeRegister(REG_PREAMBLE_MSB, len >> 8);
    spi.writeRegister(REG_PREAMBLE_LSB, len);
}

void SX1278Fsk::setPa(int power, bool pa_boost)
{
    // [2, 17] or 20 if PA_BOOST
    // [0, 15] if !PA_BOOST

    const uint8_t MAX_POWER = 0b111;

    SPITransaction spi(slave, SPITransaction::WriteBit::INVERTED);

    if (!pa_boost)
    {
        // Don't use power amplifier boost
        power = power - MAX_POWER + 15;
        spi.writeRegister(REG_PA_CONFIG, MAX_POWER << 4 | power);
        spi.writeRegister(REG_PA_DAC,
                          RegPaDac::PA_DAC_DEFAULT_VALUE | 0x10 << 3);
    }
    else if (power != 20)
    {
        // Run power amplifier boost but not at full power
        power = power - 2;
        spi.writeRegister(REG_PA_CONFIG, MAX_POWER << 4 | power |
                                             RegPaConfig::PA_SELECT_BOOST);
        spi.writeRegister(REG_PA_DAC, RegPaDac::PA_DAC_PA_BOOST | 0x10 << 3);
    }
    // RA01 modules aren't capable of this, disable it to avoid damaging them
    // else
    // {
    //     // Run power amplifier boost at full power
    //     power = 15;
    //     spi.writeRegister(REG_PA_CONFIG, MAX_POWER << 4 | power |
    //                                          RegPaConfig::PA_SELECT_BOOST);
    //     spi.writeRegister(REG_PA_DAC, RegPaDac::PA_DAC_PA_BOOST | 0x10 << 3);
    // }
}

void SX1278Fsk::setShaping(Shaping shaping)
{
    SPITransaction spi(slave, SPITransaction::WriteBit::INVERTED);
    spi.writeRegister(REG_PA_RAMP, static_cast<uint8_t>(shaping) | 0x09);
}

void SX1278Fsk::debugDumpRegisters()
{
    Lock guard(*this);
    struct RegDef
    {
        const char *name;
        int addr;
    };

    const RegDef defs[] = {
        RegDef{"REG_OP_MODE", REG_OP_MODE},
        RegDef{"REG_BITRATE_MSB", REG_BITRATE_MSB},
        RegDef{"REG_BITRATE_LSB", REG_BITRATE_LSB},
        RegDef{"REG_FDEV_MSB", REG_FDEV_MSB},
        RegDef{"REG_FDEV_LSB", REG_FDEV_LSB},
        RegDef{"REG_FRF_MSB", REG_FRF_MSB}, RegDef{"REG_FRF_MID", REG_FRF_MID},
        RegDef{"REG_FRF_LSB", REG_FRF_LSB},
        RegDef{"REG_PA_CONFIG", REG_PA_CONFIG},
        RegDef{"REG_PA_RAMP", REG_PA_RAMP}, RegDef{"REG_OCP", REG_OCP},
        RegDef{"REG_LNA", REG_LNA}, RegDef{"REG_RX_CONFIG", REG_RX_CONFIG},
        RegDef{"REG_RSSI_CONFIG", REG_RSSI_CONFIG},
        RegDef{"REG_RSSI_COLLISION", REG_RSSI_COLLISION},
        RegDef{"REG_RSSI_THRESH", REG_RSSI_THRESH},
        // RegDef { "REG_RSSI_VALUE", REG_RSSI_VALUE },
        RegDef{"REG_RX_BW", REG_RX_BW}, RegDef{"REG_AFC_BW", REG_AFC_BW},
        RegDef{"REG_OOK_PEAK", REG_OOK_PEAK},
        RegDef{"REG_OOK_FIX", REG_OOK_FIX}, RegDef{"REG_OOK_AVG", REG_OOK_AVG},
        RegDef{"REG_AFC_FEI", REG_AFC_FEI}, RegDef{"REG_AFC_MSB", REG_AFC_MSB},
        RegDef{"REG_AFC_LSB", REG_AFC_LSB}, RegDef{"REG_FEI_MSB", REG_FEI_MSB},
        RegDef{"REG_FEI_LSB", REG_FEI_LSB},
        RegDef{"REG_PREAMBLE_DETECT", REG_PREAMBLE_DETECT},
        RegDef{"REG_RX_TIMEOUT_1", REG_RX_TIMEOUT_1},
        RegDef{"REG_RX_TIMEOUT_2", REG_RX_TIMEOUT_2},
        RegDef{"REG_RX_TIMEOUT_3", REG_RX_TIMEOUT_3},
        RegDef{"REG_RX_DELAY", REG_RX_DELAY}, RegDef{"REG_OSC", REG_OSC},
        RegDef{"REG_PREAMBLE_MSB", REG_PREAMBLE_MSB},
        RegDef{"REG_PREAMBLE_LSB", REG_PREAMBLE_LSB},
        RegDef{"REG_SYNC_CONFIG", REG_SYNC_CONFIG},
        RegDef{"REG_SYNC_VALUE_1", REG_SYNC_VALUE_1},
        RegDef{"REG_SYNC_VALUE_2", REG_SYNC_VALUE_2},
        RegDef{"REG_SYNC_VALUE_3", REG_SYNC_VALUE_3},
        RegDef{"REG_SYNC_VALUE_4", REG_SYNC_VALUE_4},
        RegDef{"REG_SYNC_VALUE_5", REG_SYNC_VALUE_5},
        RegDef{"REG_SYNC_VALUE_6", REG_SYNC_VALUE_6},
        RegDef{"REG_SYNC_VALUE_7", REG_SYNC_VALUE_7},
        RegDef{"REG_SYNC_VALUE_8", REG_SYNC_VALUE_8},
        RegDef{"REG_PACKET_CONFIG_1", REG_PACKET_CONFIG_1},
        RegDef{"REG_PACKET_CONFIG_2", REG_PACKET_CONFIG_2},
        RegDef{"REG_PACKET_PAYLOAD_LENGTH", REG_PACKET_PAYLOAD_LENGTH},
        RegDef{"REG_NODE_ADRS", REG_NODE_ADRS},
        RegDef{"REG_BROADCAST_ADRS", REG_BROADCAST_ADRS},
        RegDef{"REG_FIFO_THRESH", REG_FIFO_THRESH},
        RegDef{"REG_SEQ_CONFIG_1", REG_SEQ_CONFIG_1},
        RegDef{"REG_SEQ_CONFIG_2", REG_SEQ_CONFIG_2},
        RegDef{"REG_TIMER_RESOL", REG_TIMER_RESOL},
        RegDef{"REG_TIMER_1_COEF", REG_TIMER_1_COEF},
        RegDef{"REG_TIMER_2_COEF", REG_TIMER_2_COEF},
        RegDef{"REG_IMAGE_CAL", REG_IMAGE_CAL},
        // RegDef { "REG_TEMP", REG_TEMP },
        RegDef{"REG_LOW_BAT", REG_LOW_BAT},
        // RegDef { "REG_IRQ_FLAGS_1", REG_IRQ_FLAGS_1 },
        // RegDef { "REG_IRQ_FLAGS_2", REG_IRQ_FLAGS_2 },
        RegDef{"REG_DIO_MAPPING_1", REG_DIO_MAPPING_1},
        RegDef{"REG_DIO_MAPPING_2", REG_DIO_MAPPING_2},
        RegDef{"REG_VERSION", REG_VERSION}, RegDef{NULL, 0}};

    SPITransaction spi(slave, SPITransaction::WriteBit::INVERTED);

    int i = 0;
    while (defs[i].name)
    {
        auto name = defs[i].name;
        auto addr = defs[i].addr;

        LOG_DEBUG(logger, "%s: 0x%x\n", name, spi.readRegister(addr));

        i++;
    }
}

}  // namespace Boardcore
