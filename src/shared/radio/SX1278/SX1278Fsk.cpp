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

constexpr uint8_t REG_IMAGE_CAL_DEFAULT = RegImageCal::TEMP_THRESHOLD_10DEG;

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
    SPITransaction spi(getSpiSlave());

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
    setShaping(config.shaping);

    // Make sure the PA matches settings with the frontend
    int power     = std::min(config.power, getFrontend().maxInPower());
    bool pa_boost = getFrontend().isOnPaBoost();
    setPa(power, pa_boost);

    // Setup generic parameters
    {
        SPITransaction spi(getSpiSlave());

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
            SPITransaction spi(getSpiSlave());
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
        SPITransaction spi(getSpiSlave());

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
    SPITransaction spi(getSpiSlave());
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
    SPITransaction spi(getSpiSlave());
    return spi.readRegister16(REG_IRQ_FLAGS_1);
}

void SX1278Fsk::resetIrqFlags(IrqFlags flags)
{
    // Mask only resettable flags
    flags &= RegIrqFlags::RSSI | RegIrqFlags::PREAMBLE_DETECT |
             RegIrqFlags::SYNC_ADDRESS_MATCH | RegIrqFlags::FIFO_OVERRUN |
             RegIrqFlags::LOW_BAT;

    if (flags != 0)
    {
        SPITransaction spi(getSpiSlave());
        spi.writeRegister16(REG_IRQ_FLAGS_1, flags);
    }
}

float SX1278Fsk::getRssi()
{
    SPITransaction spi(getSpiSlave());

    uint8_t rssi_raw = spi.readRegister(REG_RSSI_VALUE);

    return static_cast<float>(rssi_raw) * -0.5f;
}

float SX1278Fsk::getFei()
{
    SPITransaction spi(getSpiSlave());

    uint16_t fei_raw = spi.readRegister16(REG_FEI_MSB);

    return static_cast<float>(fei_raw) * FSTEP;
}

void SX1278Fsk::setMode(Mode mode)
{
    SPITransaction spi(getSpiSlave());
    spi.writeRegister(REG_OP_MODE, REG_OP_MODE_DEFAULT | mode);
}

void SX1278Fsk::setMapping(SX1278::DioMapping mapping)
{
    SPITransaction spi(getSpiSlave());
    spi.writeRegister16(REG_DIO_MAPPING_1, mapping.raw);
}

void SX1278Fsk::setBitrate(int bitrate)
{
    uint16_t val = FXOSC / bitrate;

    // Update values
    SPITransaction spi(getSpiSlave());
    spi.writeRegister16(REG_BITRATE_MSB, val);
}

void SX1278Fsk::setFreqDev(int freq_dev)
{
    uint16_t val = freq_dev / FSTEP;
    SPITransaction spi(getSpiSlave());
    spi.writeRegister16(REG_FDEV_MSB, val & 0x3fff);
}

void SX1278Fsk::setFreqRF(int freq_rf)
{
    uint32_t val = freq_rf / FSTEP;

    // Update values
    SPITransaction spi(getSpiSlave());
    spi.writeRegister24(REG_FRF_MSB, val);
}

void SX1278Fsk::setOcp(int ocp)
{
    SPITransaction spi(getSpiSlave());
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
    SPITransaction spi(getSpiSlave());
    spi.writeRegister(REG_SYNC_CONFIG, REG_SYNC_CONFIG_DEFAULT | size);

    for (int i = 0; i < size; i++)
    {
        spi.writeRegister(REG_SYNC_VALUE_1 + i, value[i]);
    }
}

void SX1278Fsk::setRxBw(RxBw rx_bw)
{
    SPITransaction spi(getSpiSlave());
    spi.writeRegister(REG_RX_BW, static_cast<uint8_t>(rx_bw));
}

void SX1278Fsk::setAfcBw(RxBw afc_bw)
{
    SPITransaction spi(getSpiSlave());
    spi.writeRegister(REG_AFC_BW, static_cast<uint8_t>(afc_bw));
}

void SX1278Fsk::setPreambleLen(int len)
{
    SPITransaction spi(getSpiSlave());
    spi.writeRegister16(REG_PREAMBLE_MSB, len);
}

void SX1278Fsk::setPa(int power, bool pa_boost)
{
    // [2, 17] or 20 if PA_BOOST
    // [0, 15] if !PA_BOOST

    const uint8_t MAX_POWER = 0b111;

    SPITransaction spi(getSpiSlave());

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
    else
    {
        // Run power amplifier boost at full power
        power = 15;
        spi.writeRegister(REG_PA_CONFIG, MAX_POWER << 4 | power |
                                             RegPaConfig::PA_SELECT_BOOST);
        spi.writeRegister(REG_PA_DAC, RegPaDac::PA_DAC_PA_BOOST | 0x10 << 3);
    }
}

void SX1278Fsk::setShaping(Shaping shaping)
{
    SPITransaction spi(getSpiSlave());
    spi.writeRegister(REG_PA_RAMP, static_cast<uint8_t>(shaping) | 0x09);
}

}  // namespace Boardcore
