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

    int bitrate              = config.bitrate;
    int freq_dev             = config.freq_dev;
    int freq_rf              = config.freq_rf;
    RegRxBw::RxBw rx_bw      = static_cast<RegRxBw::RxBw>(config.rx_bw);
    RegAfcBw::RxBwAfc afc_bw = static_cast<RegAfcBw::RxBwAfc>(config.afc_bw);
    int ocp                  = config.ocp;
    int power     = std::min(config.power, getFrontend().maxInPower());
    bool pa_boost = getFrontend().isOnPaBoost();
    RegPaRamp::ModulationShaping shaping =
        static_cast<RegPaRamp::ModulationShaping>(config.shaping);
    RegPacketConfig1::DcFree dc_free =
        static_cast<RegPacketConfig1::DcFree>(config.dc_free);

    {
        SPITransaction spi(getSpiSlave());

        // Setup bitrate
        uint16_t bitrate_val = FXOSC / bitrate;
        spi.writeRegister16(REG_BITRATE_MSB, bitrate_val);

        // Setup frequency deviation
        uint16_t freq_dev_raw = freq_dev / FSTEP;
        spi.writeRegister16(REG_FDEV_MSB, freq_dev_raw & 0x3fff);

        // Setup base frequency
        uint32_t freq_rf_raw = freq_rf / FSTEP;
        spi.writeRegister24(REG_FRF_MSB, freq_rf_raw);

        // Setup RX bandwidth
        spi.writeRegister(REG_RX_BW, RegRxBw::make(rx_bw));

        // Setup afc bandwidth
        spi.writeRegister(REG_AFC_BW, RegAfcBw::make(afc_bw));

        // Setup reg over-current protection
        if (config.ocp == 0)
        {
            spi.writeRegister(REG_OCP, RegOcp::make(0, false));
        }
        else if (ocp <= 120)
        {
            spi.writeRegister(REG_OCP, RegOcp::make((ocp - 45) / 5, true));
        }
        else
        {
            spi.writeRegister(REG_OCP, RegOcp::make((ocp + 30) / 10, true));
        }

        // Setup sync word
        spi.writeRegister(
            REG_SYNC_CONFIG,
            RegSyncConfig::make(2, true, RegSyncConfig::PREAMBLE_POLARITY_55,
                                RegSyncConfig::AUTO_RESTART_RX_MODE_OFF));
        spi.writeRegister(REG_SYNC_VALUE_1, 0x12);
        spi.writeRegister(REG_SYNC_VALUE_2, 0xad);

        // Setup shaping
        spi.writeRegister(REG_PA_RAMP,
                          RegPaRamp::make(RegPaRamp::PA_RAMP_US_40, shaping));

        // Setup power amplifier
        // [2, 17] or 20 if PA_BOOST
        // [0, 15] if !PA_BOOST
        const int MAX_POWER = 0b111;
        if (!pa_boost)
        {
            // Don't use power amplifier boost
            spi.writeRegister(REG_PA_CONFIG,
                              RegPaConfig::make(power, MAX_POWER, false));
            spi.writeRegister(REG_PA_DAC,
                              RegPaDac::make(RegPaDac::PA_DAC_DEFAULT_VALUE));
        }
        else if (power != 20)
        {
            // Run power amplifier boost but not at full power
            spi.writeRegister(REG_PA_CONFIG,
                              RegPaConfig::make(power - 2, MAX_POWER, true));
            spi.writeRegister(REG_PA_DAC,
                              RegPaDac::make(RegPaDac::PA_DAC_PA_BOOST));
        }
        else
        {
            // Run power amplifier boost at full power
            spi.writeRegister(REG_PA_CONFIG,
                              RegPaConfig::make(0b1111, MAX_POWER, true));
            spi.writeRegister(REG_PA_DAC,
                              RegPaDac::make(RegPaDac::PA_DAC_PA_BOOST));
        }

        // Setup other registers
        spi.writeRegister16(REG_PREAMBLE_MSB, 2);

        spi.writeRegister(
            REG_RX_CONFIG,
            RegRxConfig::make(true, true, true, true, false, false, false));

        spi.writeRegister(REG_RSSI_THRESH, 0xff);

        spi.writeRegister(
            REG_PREAMBLE_DETECT,
            RegPreambleDetector::make(
                0x0a, RegPreambleDetector::PREAMBLE_DETECTOR_SIZE_2_BYTES,
                true));

        spi.writeRegister(REG_RX_TIMEOUT_1, 0x00);
        spi.writeRegister(REG_RX_TIMEOUT_2, 0x00);
        spi.writeRegister(REG_RX_TIMEOUT_3, 0x00);

        spi.writeRegister(
            REG_PACKET_CONFIG_1,
            RegPacketConfig1::make(
                RegPacketConfig1::CRC_WHITENING_TYPE_CCITT_CRC,
                RegPacketConfig1::ADDRESS_FILTERING_NONE, false, true, dc_free,
                RegPacketConfig1::PACKET_FORMAT_VARIABLE_LENGTH));

        spi.writeRegister(
            REG_PACKET_CONFIG_2,
            RegPacketConfig2::make(false, false, false,
                                   RegPacketConfig2::DATA_MODE_PACKET));

        spi.writeRegister(
            REG_FIFO_THRESH,
            RegFifoThresh::make(
                0x0f, RegFifoThresh::TX_START_CONDITION_FIFO_NOT_EMPTY));

        spi.writeRegister(REG_NODE_ADRS, 0x00);
        spi.writeRegister(REG_BROADCAST_ADRS, 0x00);
    }

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
    spi.writeRegister(REG_OP_MODE,
                      RegOpMode::make(static_cast<RegOpMode::Mode>(mode), true,
                                      RegOpMode::MODULATION_TYPE_FSK));
}

void SX1278Fsk::setMapping(SX1278::DioMapping mapping)
{
    SPITransaction spi(getSpiSlave());
    spi.writeRegister16(REG_DIO_MAPPING_1, mapping.raw);
}

}  // namespace Boardcore
