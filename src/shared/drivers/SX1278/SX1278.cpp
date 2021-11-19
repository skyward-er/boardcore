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

SX1278::SX1278(SPIBusInterface& bus, GpioPin cs)
    : slave(bus, cs, spiConfig()), mode(SX1278::Mode::MODE_SLEEP)
{
}

SX1278::Error SX1278::init(Config config)
{
    if(getVersion() != 0x12) {
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

    uint8_t sync_word[3] = { 0x12, 0xad };
    setSyncWord(sync_word, sizeof(sync_word));
    setPreableLen(3);

    // Setup generic parameters
    {
        SPITransaction spi(slave, SPIWriteBit::INVERTED);

        // Setup transmitter registers

        // Setup receiver registers
        spi.write(REG_RX_CONFIG, REG_RX_CONFIG_DEFAULT);
        
        // spi.write(REG_RSSI_CONFIG, 2);

        // Setup packet registers

        spi.write(REG_PREAMBLE_DETECT, 0x01 << 7 | 0x01 << 5 | 0x0a << 4);

        spi.write(REG_PACKET_CONFIG_1,
                RegPacketConfig1::PACKET_FORMAT_VARIABLE_LENGTH |
                    RegPacketConfig1::DC_FREE_NONE |
                    RegPacketConfig1::CRC_ON |
                    RegPacketConfig1::ADDRESS_FILTERING_NONE |
                    RegPacketConfig1::CRC_WHITENING_TYPE_CCITT_CRC);

        spi.write(REG_PACKET_CONFIG_2, RegPacketConfig2::DATA_MODE_PACKET);

        spi.write(REG_FIFO_THRESH, RegFifoTresh::TX_START_CONDITION_FIFO_EMPTY | 0x0f);
    }

    return Error::NONE;
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

void SX1278::setOcp(int ocp) {
    SPITransaction spi(slave, SPIWriteBit::INVERTED);
    if(ocp == 0) {
        spi.write(REG_OCP, 0);
    } else if(ocp <= 120) {
        uint8_t raw = (ocp - 45) / 5;
        spi.write(REG_OCP, RegOcp::REG_OCP_ON | raw);
    } else {
        uint8_t raw = (ocp + 30) / 10;
        spi.write(REG_OCP, RegOcp::REG_OCP_ON | raw);
    }
}

void SX1278::setSyncWord(uint8_t value[], int size) {
    SPITransaction spi(slave, SPIWriteBit::INVERTED);
    spi.write(REG_SYNC_CONFIG, REG_SYNC_CONFIG_DEFAULT | size);

    for(int i = 0; i < size; i++) {
        spi.write(REG_SYNC_VALUE_1 + i, value[i]);
    }
}

void SX1278::setRxBw(RxBw rx_bw) {
    SPITransaction spi(slave, SPIWriteBit::INVERTED);
    spi.write(REG_RX_BW, static_cast<uint8_t>(rx_bw));
}

void SX1278::setAfcBw(RxBw afc_bw) {
    SPITransaction spi(slave, SPIWriteBit::INVERTED);
    spi.write(REG_AFC_BW, static_cast<uint8_t>(afc_bw));
}

void SX1278::setPreableLen(int len) {
    SPITransaction spi(slave, SPIWriteBit::INVERTED);
    spi.write(REG_PREAMBLE_MSB, len >> 8);
    spi.write(REG_PREAMBLE_LSB, len);
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