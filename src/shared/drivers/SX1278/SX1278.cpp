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

#include <cmath>

using namespace SX1278Defs;

SX1278::SX1278(SPIBusInterface& bus, GpioPin cs, SX1278::Config config)
    : slave(bus, cs, spiConfig()), config(config), mode(SX1278::Mode::MODE_SLEEP)
{
}

void SX1278::init()
{
    {
        SPITransaction spi(slave, SPIWriteBit::INVERTED);

        // Setup transmitter registers

        // Setup receiver registers
        spi.write(REG_RX_CONFIG, RegRxConfig::RESTART_RX_ON_COLLISION |
                                    RegRxConfig::AGC_AUTO_ON |
                                    RegRxConfig::RX_TRIGGER_PREAMBLE_DETECT);

        // Setup packet registers
        spi.write(REG_SYNC_CONFIG,
                RegSyncConfig::AUTO_RESTART_RX_MODE_ON_WITHOUT_PILL_LOCK |
                    RegSyncConfig::PREAMBLE_POLARITY_AA |
                    RegSyncConfig::SYNC_ON | 0x03);

        spi.write(REG_PACKET_CONFIG_1,
                RegPacketConfig1::PACKET_FORMAT_VARIABLE_LENGTH |
                    RegPacketConfig1::DC_FREE_NONE |
                    RegPacketConfig1::CRC_ON |
                    RegPacketConfig1::ADDRESS_FILTERING_NONE |
                    RegPacketConfig1::CRC_WHITENING_TYPE_CCITT_CRC);

        spi.write(REG_PACKET_CONFIG_2, RegPacketConfig2::DATA_MODE_PACKET);

        spi.write(REG_FIFO_THRESH, RegFifoTresh::TX_START_CONDITION_FIFO_EMPTY | 0x0f);
    }

    // Enter standby mode
    enterMode(Mode::MODE_STDBY);
}

uint8_t SX1278::recv(uint8_t *buf) {
    // Enter RX mode
    if(mode == Mode::MODE_STDBY)
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

void SX1278::send(const uint8_t *buf, uint8_t len) {
    // Enter TX mode
    if(mode == Mode::MODE_STDBY)
        enterMode(Mode::MODE_FSTX);

    enterMode(Mode::MODE_TX);

    // Wait for TX ready
    waitForIrq1(RegIrqFlags1::TX_READY);

    {
        SPITransaction spi(slave, SPIWriteBit::INVERTED);
        spi.write(REG_FIFO, len);
        // FIXME: This needs to be fixed!!
        spi.write(REG_FIFO, const_cast<uint8_t*>(buf), len);
    }

    // Wait for packet sent
    waitForIrq2(RegIrqFlags2::PACKET_SENT);
}

uint8_t SX1278::getVersion() const
{
    SPITransaction spi(slave, SPIWriteBit::INVERTED);

    return spi.read(REG_VERSION);
}

void SX1278::setBitrate(float bitrate)
{
    float val = SX1278Defs::FXOSC / bitrate;

    // Split value in integer and fractional
    float rate = 0.0f;
    float frac = modff(val, &rate);

    uint16_t ratei = static_cast<uint16_t>(rate);
    uint8_t fraci  = static_cast<uint8_t>(frac * 16.0f);

    // Update values
    SPITransaction spi(slave, SPIWriteBit::INVERTED);
    spi.write(REG_BITRATE_MSB, ratei >> 8);
    spi.write(REG_BITRATE_LSB, ratei);
    spi.write(REG_BITRATE_FRAC, fraci & 0x0f);
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

void SX1278::debugDumpRegisters()
{
    const char* names[] = {
        "REG_OP_MODE",  "REG_BITRATE_MSB", "REG_BITRATE_LSB", "REG_FDEV_MSB",
        "REG_FDEV_LSB", "REG_FRF_MSB",     "REG_FRF_MID",     "REG_FRF_LSB",
    };

    SPITransaction spi(slave, SPIWriteBit::INVERTED);
    for (int i = 0x01; i <= 0x08; i++)
    {
        TRACE("%s: 0x%x\n", names[i - 1], spi.read(i));
    }
}

void SX1278::enterMode(Mode mode) {
    // Only do this if we need to
    if(mode == this->mode)
        return;

    {
        SPITransaction spi(slave, SPIWriteBit::INVERTED);
        spi.write(REG_OP_MODE, REG_OP_MODE_DEFAULT | mode);
    }

    // Wait for mode ready
    waitForIrq1(RegIrqFlags1::MODE_READY);

    this->mode = mode;
}

inline void waitForIrq(SPISlave &slave, uint8_t reg, uint8_t mask) {
    SPITransaction spi(slave, SPIWriteBit::INVERTED);

    // Tight loop to wait on device
    while((spi.read(reg) & mask) == 0)
        miosix::delayUs(10);
}

void SX1278::waitForIrq1(uint8_t mask) {
    waitForIrq(slave, REG_IRQ_FLAGS_1, mask);
}

void SX1278::waitForIrq2(uint8_t mask) {
    waitForIrq(slave, REG_IRQ_FLAGS_2, mask);
}