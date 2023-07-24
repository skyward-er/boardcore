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

#include "SX1278Lora.h"

#include <utils/Debug.h>

#include "SX1278LoraTimings.h"

namespace Boardcore
{

using namespace SX1278;
using namespace SX1278::Lora;

static constexpr uint8_t MAX_PAYLOAD_LENGTH = 0xff;
static constexpr uint8_t FIFO_TX_BASE_ADDR  = 0x00;
static constexpr uint8_t FIFO_RX_BASE_ADDR  = 0x00;

// This doesn't really matter, as we are going to change it anyway
static constexpr DioMapping DEFAULT_MAPPING =
    DioMapping(0, 0, 0, 0, 0, 0, false);

// This registers allow for optmized rx spurious response
struct ErrataRegistersValues
{
    int freq_rf                = 0;
    int reg_if_freq_1          = 0;
    int reg_if_freq_2          = 0;
    bool automatic_if_on       = false;
    int reg_high_bw_optimize_1 = 0;
    int reg_high_bw_optimize_2 = 0;

    static ErrataRegistersValues calculate(RegModemConfig1::Bw bw, int freq_rf)
    {
        ErrataRegistersValues values = {};

        switch (bw)
        {
            case RegModemConfig1::BW_HZ_7800:
                freq_rf += 7810;
                break;
            case RegModemConfig1::BW_HZ_10400:
                freq_rf += 10420;
                break;
            case RegModemConfig1::BW_HZ_15600:
                freq_rf += 15620;
                break;
            case RegModemConfig1::BW_HZ_20800:
                freq_rf += 20830;
                break;
            case RegModemConfig1::BW_HZ_41700:
                freq_rf += 41670;
                break;
            default:
                break;
        }

        switch (bw)
        {
            case RegModemConfig1::BW_HZ_7800:
                values.reg_if_freq_2 = 0x48;
                values.reg_if_freq_1 = 0x00;
                break;
            case RegModemConfig1::BW_HZ_10400:
            case RegModemConfig1::BW_HZ_15600:
            case RegModemConfig1::BW_HZ_20800:
            case RegModemConfig1::BW_HZ_31250:
            case RegModemConfig1::BW_HZ_41700:
                values.reg_if_freq_2 = 0x44;
                values.reg_if_freq_1 = 0x00;
                break;
            case RegModemConfig1::BW_HZ_62500:
            case RegModemConfig1::BW_HZ_125000:
            case RegModemConfig1::BW_HZ_250000:
                values.reg_if_freq_2 = 0x40;
                values.reg_if_freq_1 = 0x00;
                break;
            case RegModemConfig1::BW_HZ_500000:
                values.reg_if_freq_2 = -1;
                values.reg_if_freq_1 = -1;
                break;
            default:
                break;
        }

        values.automatic_if_on = bw == RegModemConfig1::BW_HZ_500000;

        bool freq_rf_low_range  = 410000000 <= freq_rf && freq_rf <= 525000000;
        bool freq_rf_high_range = 862000000 <= freq_rf && freq_rf <= 1020000000;

        if (bw == RegModemConfig1::BW_HZ_500000 && freq_rf_low_range)
        {
            values.reg_high_bw_optimize_1 = 0x02;
            values.reg_high_bw_optimize_2 = 0x64;
        }
        else if (bw == RegModemConfig1::BW_HZ_500000 && freq_rf_high_range)
        {
            values.reg_high_bw_optimize_1 = 0x02;
            values.reg_high_bw_optimize_2 = 0x7f;
        }
        else
        {
            values.reg_high_bw_optimize_1 = 0x03;
            values.reg_high_bw_optimize_2 = -1;
        }

        values.freq_rf = freq_rf;

        return values;
    }
};

SX1278Lora::Error SX1278Lora::init(const Config &config)
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

bool SX1278Lora::checkVersion()
{
    Lock guard(*this);
    SPITransaction spi(getSpiSlave());

    uint8_t version = spi.readRegister(REG_VERSION);
    TRACE("[sx1278] Chip id: %d\n", version);

    return version == 0x12;
}

SX1278Lora::Error SX1278Lora::configure(const Config &config)
{
    // First make sure the device is in lora mode and in standby
    enterLoraMode();

    // Then make sure the device remains in standby and not in sleep
    setDefaultMode(RegOpMode::MODE_STDBY, DEFAULT_MAPPING,
                   InterruptTrigger::RISING_EDGE, false, false);

    // Lock the bus
    Lock guard(*this);
    LockMode mode_guard(*this, guard, RegOpMode::MODE_STDBY, DEFAULT_MAPPING,
                        InterruptTrigger::RISING_EDGE);

    RegModemConfig1::Bw bw = static_cast<RegModemConfig1::Bw>(config.bandwidth);
    RegModemConfig1::Cr cr =
        static_cast<RegModemConfig1::Cr>(config.coding_rate);
    RegModemConfig2::Sf sf =
        static_cast<RegModemConfig2::Sf>(config.spreading_factor);

    int freq_rf   = config.freq_rf;
    int ocp       = config.ocp;
    int power     = std::min(config.power, getFrontend().maxInPower());
    bool pa_boost = getFrontend().isOnPaBoost();

    bool low_data_rate_optimize = config.low_data_rate_optimize;

    // Mandated for when the symbol length exceeds 16ms
    if (Lora::symbolDuration(sf, RegModemConfig1::bandwidthToInt(bw)) > 16000)
        low_data_rate_optimize = true;

    // Setup high BW errata values
    ErrataRegistersValues errata_values =
        ErrataRegistersValues::calculate(bw, freq_rf);

    crc_enabled = config.enable_crc;

    {
        SPITransaction spi(getSpiSlave());

        // Setup FIFO sections
        spi.writeRegister(REG_FIFO_TX_BASE_ADDR, FIFO_TX_BASE_ADDR);
        spi.writeRegister(REG_FIFO_RX_BASE_ADDR, FIFO_RX_BASE_ADDR);
        spi.writeRegister(REG_MAX_PAYLOAD_LENGTH, MAX_PAYLOAD_LENGTH);

        // Setup frequency
        uint32_t freq_rf_raw = errata_values.freq_rf / FSTEP;
        spi.writeRegister24(REG_FRF_MSB, freq_rf_raw);

        // Setup reg power amplifier
        const int MAX_POWER = 0b111;
        if (!pa_boost)
        {
            // No power amplifier boost
            spi.writeRegister(REG_PA_CONFIG,
                              RegPaConfig::make(power, MAX_POWER, false));
            spi.writeRegister(REG_PA_DAC, RegPaDac::make(false));
        }
        else if (power != 20)
        {
            // Run power amplifier boost but not at full power
            spi.writeRegister(REG_PA_CONFIG,
                              RegPaConfig::make(power - 2, MAX_POWER, true));
            spi.writeRegister(REG_PA_DAC, RegPaDac::make(false));
        }
        else
        {
            // Run power amplifier at full power
            spi.writeRegister(REG_PA_CONFIG,
                              RegPaConfig::make(0b1111, MAX_POWER, true));
            spi.writeRegister(REG_PA_DAC, RegPaDac::make(true));
        }

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

        // Setup generic configuration registers
        spi.writeRegister(REG_MODEM_CONFIG_1,
                          RegModemConfig1::make(false, cr, bw));
        spi.writeRegister(REG_MODEM_CONFIG_2,
                          RegModemConfig2::make(crc_enabled, false, sf));
        spi.writeRegister(REG_MODEM_CONFIG_3,
                          RegModemConfig3::make(true, low_data_rate_optimize));

        // Setup detect optimize (lots of magic values here)
        spi.writeRegister(
            REG_DETECT_OPTIMIZE,
            RegDetectOptimize::make(0x03, errata_values.automatic_if_on));
        spi.writeRegister(REG_DETECTION_THRESHOLD, 0x0a);

        // (just change it to something that isn't the default)
        spi.writeRegister(REG_SYNC_WORD, 0x69);

        // Setup weird errata registers (see errata note)
        if (errata_values.reg_high_bw_optimize_1 != -1)
            spi.writeRegister(REG_HIGH_BW_OPTIMIZE_1,
                              errata_values.reg_high_bw_optimize_1);
        if (errata_values.reg_high_bw_optimize_2 != -1)
            spi.writeRegister(REG_HIGH_BW_OPTIMIZE_2,
                              errata_values.reg_high_bw_optimize_2);
        if (errata_values.reg_if_freq_1 != -1)
            spi.writeRegister(REG_IF_FREQ_1, errata_values.reg_if_freq_1);
        if (errata_values.reg_if_freq_2 != -1)
            spi.writeRegister(REG_IF_FREQ_2, errata_values.reg_if_freq_2);
    }

    return Error::NONE;
}

ssize_t SX1278Lora::receive(uint8_t *pkt, size_t max_len)
{
    Lock guard(*this);

    // Use continuous because it doesn't go into timeout (and you cannot
    // disable it...)
    LockMode mode_guard(*this, guard, RegOpMode::MODE_RXCONTINUOUS,
                        DioMapping(0, 0, 0, 0, 0, 0),
                        InterruptTrigger::RISING_EDGE, false, true);

    waitForIrq(mode_guard, RegIrqFlags::RX_DONE, 0, true);

    uint8_t len;
    {
        SPITransaction spi(getSpiSlave());
        len = spi.readRegister(REG_RX_NB_BYTES);
    }

    if (len > max_len ||
        (crc_enabled &&
         checkForIrqAndReset(RegIrqFlags::PAYLOAD_CRC_ERROR, 0) != 0))
        return -1;

    // Finally read the contents of the fifo
    readFifo(FIFO_RX_BASE_ADDR, pkt, len);
    return len;
}

bool SX1278Lora::send(uint8_t *pkt, size_t len)
{
    if (len > MTU)
        return false;

    Lock guard(*this);

    SPITransaction spi(getSpiSlave());

    spi.writeRegister(REG_PAYLOAD_LENGTH, len);
    writeFifo(FIFO_TX_BASE_ADDR, pkt, len);

    {
        // Now enter in mode TX to send the packet
        LockMode mode_guard(*this, guard, RegOpMode::MODE_TX,
                            DioMapping(1, 0, 0, 0, 0, 0),
                            InterruptTrigger::RISING_EDGE, true, false);

        // Wait for the transmission to end
        waitForIrq(mode_guard, RegIrqFlags::TX_DONE, 0);
    }

    return true;
}

float SX1278Lora::getLastRxRssi()
{
    float rssi;
    {
        Lock guard(*this);
        SPITransaction spi(getSpiSlave());
        rssi =
            static_cast<float>(spi.readRegister(REG_PKT_RSSI_VALUE)) - 164.0f;
    }

    float snr = getLastRxSnr();
    // Handle packets below the noise floor
    if (snr < 0.0f)
        rssi += snr * 0.25f;

    return rssi;
}

float SX1278Lora::getLastRxSnr()
{
    Lock guard(*this);
    SPITransaction spi(getSpiSlave());
    return static_cast<float>(
               static_cast<int8_t>(spi.readRegister(REG_PKT_SNR_VALUE))) /
           4.0f;
}

void SX1278Lora::enterLoraMode()
{
    Lock guard(*this);
    SPITransaction spi(getSpiSlave());

    // First enter LoRa sleep
    spi.writeRegister(REG_OP_MODE,
                      RegOpMode::make(RegOpMode::MODE_SLEEP, true, false));
    miosix::Thread::sleep(1);

    // Then transition to standby
    spi.writeRegister(REG_OP_MODE,
                      RegOpMode::make(RegOpMode::MODE_STDBY, true, false));
    miosix::Thread::sleep(1);
}

void SX1278Lora::readFifo(uint8_t addr, uint8_t *dst, uint8_t size)
{
    SPITransaction spi(getSpiSlave());
    spi.writeRegister(REG_FIFO_ADDR_PTR, addr);
    spi.readRegisters(REG_FIFO, dst, size);
}

void SX1278Lora::writeFifo(uint8_t addr, uint8_t *src, uint8_t size)
{
    SPITransaction spi(getSpiSlave());
    spi.writeRegister(REG_FIFO_ADDR_PTR, addr);
    spi.writeRegisters(REG_FIFO, src, size);
}

ISX1278::IrqFlags SX1278Lora::getIrqFlags()
{
    SPITransaction spi(getSpiSlave());
    return spi.readRegister(REG_IRQ_FLAGS);
}

void SX1278Lora::resetIrqFlags(IrqFlags flags)
{
    SPITransaction spi(getSpiSlave());
    // Register is write clear
    spi.writeRegister(REG_IRQ_FLAGS, flags);
}

void SX1278Lora::setMode(ISX1278::Mode mode)
{
    SPITransaction spi(getSpiSlave());

    spi.writeRegister(
        REG_OP_MODE,
        RegOpMode::make(static_cast<RegOpMode::Mode>(mode), true, false));
}

void SX1278Lora::setMapping(SX1278::DioMapping mapping)
{
    SPITransaction spi(getSpiSlave());
    spi.writeRegister16(REG_DIO_MAPPING_1, mapping.raw);
}

}  // namespace Boardcore
