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

#include "SX1278.h"

#include <Debug.h>
#include <kernel/scheduler/scheduler.h>

#include <cmath>

using namespace SX1278Defs;

// Default values for registers
constexpr uint8_t REG_OP_MODE_DEFAULT = RegOpMode::LONG_RANGE_MODE_FSK |
                                        RegOpMode::MODULATION_TYPE_FSK |
                                        RegOpMode::LOW_FREQUENCY_MODE_ON;

constexpr uint8_t REG_SYNC_CONFIG_DEFAULT =
    RegSyncConfig::AUTO_RESTART_RX_MODE_OFF |
    RegSyncConfig::PREAMBLE_POLARITY_55 | RegSyncConfig::SYNC_ON;

SX1278::SX1278(SPIBusInterface &bus, GpioPin cs, GpioPin dio)
    : slave(bus, cs, spiConfig()), mode(SX1278::Mode::MODE_SLEEP), dio(dio)
{
}

SX1278::Error SX1278::init(Config config)
{
    if (getVersion() != 0x12)
    {
        return Error::BAD_VERSION;
    }

    // Enter standby mode
    enterMode(Mode::MODE_STDBY);

    setBitrate(config.bitrate);
    setFreqDev(config.freq_dev);
    setFreqRF(config.freq_rf);

    setRxBw(config.rx_bw);
    setAfcBw(config.afc_bw);

    setOcp(config.ocp);

    uint8_t sync_word[2] = {0x12, 0xad};
    setSyncWord(sync_word, 2);
    setPreableLen(2);
    setPa(config.power, true);

    enable_int = config.enable_int;

    // Setup generic parameters
    {
        SPITransaction spi(slave, SPIWriteBit::INVERTED);

        spi.write(REG_PA_RAMP, RegPaRamp::MODULATION_SHAPING_NONE | 0x09);
        spi.write(REG_RX_CONFIG, RegRxConfig::AFC_AUTO_ON |
                                     RegRxConfig::AGC_AUTO_ON |
                                     RegRxConfig::RX_TRIGGER_PREAMBLE_DETECT |
                                     RegRxConfig::RX_TRIGGER_RSSI_INTERRUPT);
        spi.write(REG_RSSI_THRESH, 0xff);
        spi.write(REG_PREAMBLE_DETECT,
                  RegPreambleDetector::PREAMBLE_DETECTOR_ON |
                      RegPreambleDetector::PREAMBLE_DETECTOR_SIZE_2_BYTES |
                      0x0a);
        spi.write(REG_RX_TIMEOUT_1, 0x00);
        spi.write(REG_RX_TIMEOUT_2, 0x00);
        spi.write(REG_RX_TIMEOUT_3, 0x00);
        spi.write(REG_PACKET_CONFIG_1,
                  RegPacketConfig1::PACKET_FORMAT_VARIABLE_LENGTH |
                      RegPacketConfig1::DC_FREE_NONE |
                      RegPacketConfig1::CRC_ON |
                      RegPacketConfig1::ADDRESS_FILTERING_NONE |
                      RegPacketConfig1::CRC_WHITENING_TYPE_CCITT_CRC);
        spi.write(REG_PACKET_CONFIG_2, RegPacketConfig2::DATA_MODE_PACKET);
        spi.write(REG_FIFO_THRESH,
                  RegFifoThresh::TX_START_CONDITION_FIFO_NOT_EMPTY | 0x0f);
        spi.write(REG_NODE_ADRS, 0x00);
        spi.write(REG_BROADCAST_ADRS, 0x00);

        // Enable PayloadReady, PacketSent on DIO0
        spi.write(REG_DIO_MAPPING_1, 0x00);
    }

    return Error::NONE;
}

uint8_t SX1278::recv(uint8_t *buf)
{
    // Enter RX mode
    if (mode == Mode::MODE_STDBY)
        enterMode(Mode::MODE_FSRX);

    enterMode(Mode::MODE_RX);
    waitForIrq2(RegIrqFlags2::PAYLOAD_READY);

    uint8_t len = 0;
    {
        SPITransaction spi(slave, SPIWriteBit::INVERTED);
        len = spi.read(REG_FIFO);
        spi.read(REG_FIFO, buf, len);
    }

    return len;
}

void SX1278::send(const uint8_t *buf, uint8_t len)
{
    // Enter TX mode
    if (mode == Mode::MODE_STDBY)
        enterMode(Mode::MODE_FSTX);

    enterMode(Mode::MODE_TX);

    // Wait for TX ready
    waitForIrq1(RegIrqFlags1::TX_READY);

    {
        SPITransaction spi(slave, SPIWriteBit::INVERTED);

        spi.write(REG_FIFO, len);
        // FIXME(Davide Mor): This needs to be fixed!!
        spi.write(REG_FIFO, const_cast<uint8_t *>(buf), len);
    }

    // Wait for packet sent
    waitForIrq2(RegIrqFlags2::PACKET_SENT);
}

void SX1278::handleDioIRQ()
{
    if (irq_wait_thread)
    {
        irq_wait_thread->IRQwakeup();
        if (irq_wait_thread->IRQgetPriority() >
            miosix::Thread::IRQgetCurrentThread()->IRQgetPriority())
        {
            miosix::Scheduler::IRQfindNextThread();
        }
        irq_wait_thread = nullptr;
    }
}

uint8_t SX1278::getVersion() const
{
    SPITransaction spi(slave, SPIWriteBit::INVERTED);

    return spi.read(REG_VERSION);
}

void SX1278::setBitrate(int bitrate)
{
    uint16_t val = SX1278Defs::FXOSC / bitrate;

    // Update values
    SPITransaction spi(slave, SPIWriteBit::INVERTED);
    spi.write(REG_BITRATE_MSB, val >> 8);
    spi.write(REG_BITRATE_LSB, val);
}

void SX1278::setFreqDev(int freq_dev)
{
    uint16_t val = freq_dev / FSTEP;

    // Update values
    SPITransaction spi(slave, SPIWriteBit::INVERTED);
    spi.write(REG_FDEV_MSB, (val >> 8) & 0x3f);
    spi.write(REG_FDEV_LSB, val);
}

void SX1278::setFreqRF(int freq_rf)
{
    uint32_t val = freq_rf / FSTEP;

    // Update values
    SPITransaction spi(slave, SPIWriteBit::INVERTED);
    spi.write(REG_FRF_MSB, val >> 16);
    spi.write(REG_FRF_MID, val >> 8);
    spi.write(REG_FRF_LSB, val);
}

void SX1278::setOcp(int ocp)
{
    SPITransaction spi(slave, SPIWriteBit::INVERTED);
    if (ocp == 0)
    {
        spi.write(REG_OCP, 0);
    }
    else if (ocp <= 120)
    {
        uint8_t raw = (ocp - 45) / 5;
        spi.write(REG_OCP, RegOcp::OCP_ON | raw);
    }
    else
    {
        uint8_t raw = (ocp + 30) / 10;
        spi.write(REG_OCP, RegOcp::OCP_ON | raw);
    }
}

void SX1278::setSyncWord(uint8_t value[], int size)
{
    SPITransaction spi(slave, SPIWriteBit::INVERTED);
    spi.write(REG_SYNC_CONFIG, REG_SYNC_CONFIG_DEFAULT | size);

    for (int i = 0; i < size; i++)
    {
        spi.write(REG_SYNC_VALUE_1 + i, value[i]);
    }
}

void SX1278::setRxBw(RxBw rx_bw)
{
    SPITransaction spi(slave, SPIWriteBit::INVERTED);
    spi.write(REG_RX_BW, static_cast<uint8_t>(rx_bw));
}

void SX1278::setAfcBw(RxBw afc_bw)
{
    SPITransaction spi(slave, SPIWriteBit::INVERTED);
    spi.write(REG_AFC_BW, static_cast<uint8_t>(afc_bw));
}

void SX1278::setPreableLen(int len)
{
    SPITransaction spi(slave, SPIWriteBit::INVERTED);
    spi.write(REG_PREAMBLE_MSB, len >> 8);
    spi.write(REG_PREAMBLE_LSB, len);
}

void SX1278::setPa(int power, bool pa_boost)
{
    // [2, 17] or 20 if PA_BOOST
    // [0, 15] if !PA_BOOST

    const uint8_t MAX_POWER = 0b111;

    SPITransaction spi(slave, SPIWriteBit::INVERTED);

    if (!pa_boost)
    {
        // Don't use power amplifier boost
        power = power - MAX_POWER + 15;
        spi.write(REG_PA_CONFIG, MAX_POWER << 4 | power);
        spi.write(REG_PA_DAC, RegPaDac::PA_DAC_DEFAULT_VALUE | 0x10 << 3);
    }
    else if (power != 20)
    {
        // Run power amplifier boost but not at full power
        power = power - 2;
        spi.write(REG_PA_CONFIG,
                  MAX_POWER << 4 | power | RegPaConfig::PA_SELECT_BOOST);
        spi.write(REG_PA_DAC, RegPaDac::PA_DAC_PA_BOOST | 0x10 << 3);
    }
    else
    {
        // Run power amplifier boost at full power
        power = 15;
        spi.write(REG_PA_CONFIG,
                  MAX_POWER << 4 | power | RegPaConfig::PA_SELECT_BOOST);
        spi.write(REG_PA_DAC, RegPaDac::PA_DAC_PA_BOOST | 0x10 << 3);
    }
}

void SX1278::enterMode(Mode mode)
{
    // Only do this if we need to
    if (mode == this->mode)
        return;

    uint8_t value = REG_OP_MODE_DEFAULT | mode;
    {
        SPITransaction spi(slave, SPIWriteBit::INVERTED);
        spi.write(REG_OP_MODE, value);
    }

    // Wait for mode ready
    waitForIrq1(RegIrqFlags1::MODE_READY);

    this->mode = mode;
}

void SX1278::waitForIrq(uint8_t reg, uint8_t mask)
{
    SPITransaction spi(slave, SPIWriteBit::INVERTED);

    if ((mask == RegIrqFlags2::PACKET_SENT ||
         mask == RegIrqFlags2::PAYLOAD_READY) &&
        enable_int)
    {
        // // Optimized handling using interrupts
        // // TODO(Davide Mor): transform this in a CondVar
        // while (!dio.value())
        // {
        //     miosix::delayUs(10);
        // }

        // TRACE("Wait\n");
        // TRACE("Wait\n");

        miosix::FastInterruptDisableLock dLock;
        irq_wait_thread = miosix::Thread::getCurrentThread();
        // Avoid spurious wakeups
        while (irq_wait_thread != 0)
        {
            irq_wait_thread->IRQwait();
            {
                miosix::FastInterruptEnableLock eLock(dLock);
                miosix::Thread::yield();
            }
        }

        // TRACE("Wait finished\n");
    }
    else
    {
        // Tight loop on IRQ register
        while (!(spi.read(reg) & mask))
        {
            miosix::delayUs(10);
        }
    }

    // if(mask == RegIrqFlags2::PAYLOAD_READY)
    //     TRACE("Wait finished\n");
}

void SX1278::waitForIrq1(uint8_t mask) { waitForIrq(REG_IRQ_FLAGS_1, mask); }

void SX1278::waitForIrq2(uint8_t mask) { waitForIrq(REG_IRQ_FLAGS_2, mask); }

void SX1278::debugDumpRegisters()
{
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

    SPITransaction spi(slave, SPIWriteBit::INVERTED);

    int i = 0;
    while (defs[i].name)
    {
        auto name = defs[i].name;
        auto addr = defs[i].addr;
        TRACE("%s: 0x%x\n", name, spi.read(addr));

        i++;
    }
}