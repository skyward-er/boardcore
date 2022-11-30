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

#include <kernel/scheduler/scheduler.h>
#include <utils/Debug.h>

#include <cmath>

namespace Boardcore
{

using namespace SX1278Defs;

long long now() { return miosix::getTick() * 1000 / miosix::TICK_FREQ; }

// Default values for registers
constexpr uint8_t REG_OP_MODE_DEFAULT = RegOpMode::LONG_RANGE_MODE_FSK |
                                        RegOpMode::MODULATION_TYPE_FSK |
                                        RegOpMode::LOW_FREQUENCY_MODE_ON;

constexpr uint8_t REG_SYNC_CONFIG_DEFAULT =
    RegSyncConfig::AUTO_RESTART_RX_MODE_OFF |
    RegSyncConfig::PREAMBLE_POLARITY_55 | RegSyncConfig::SYNC_ON;

constexpr uint8_t REG_IMAGE_CAL_DEFAULT = RegImageCal::TEMP_TRESHOLD_10DEG;

SX1278BusManager::SX1278BusManager(SPIBusInterface &bus, miosix::GpioPin cs)
    : slave(bus, cs, spiConfig())
{
}

void SX1278BusManager::lock() { mutex.lock(); }

void SX1278BusManager::unlock() { mutex.unlock(); }

SX1278BusManager::DeviceState SX1278BusManager::lockMode(Mode mode)
{
    mutex.lock();
    // Store previous state
    DeviceState old_state = state;

    enterMode(mode);
    state.irq_wait_thread = nullptr;
    state.waiting_on_dio  = static_cast<Dio>(0);

    return old_state;
}

void SX1278BusManager::unlockMode(DeviceState old_state)
{
    // Do this copy manually, we want stuff to be copied in a specific order
    state.irq_wait_thread = old_state.irq_wait_thread;
    state.waiting_on_dio  = old_state.waiting_on_dio;
    enterMode(old_state.mode);

    mutex.unlock();
}

void SX1278BusManager::handleDioIRQ(Dio dio)
{
    if ((state.waiting_on_dio & dio) && state.irq_wait_thread)
    {
        state.irq_wait_thread->IRQwakeup();
        if (state.irq_wait_thread->IRQgetPriority() >
            miosix::Thread::IRQgetCurrentThread()->IRQgetPriority())
        {
            miosix::Scheduler::IRQfindNextThread();
        }

        state.irq_wait_thread = nullptr;
    }
}

void SX1278BusManager::waitForIrq(const LockMode &_guard, Irq irq, bool unlock)
{
    // Unused, just for API purposes
    (void)_guard;

    // Convert the IRQ mask into a DIO mask
    state.waiting_on_dio  = irq_to_dio(irq);
    state.irq_wait_thread = miosix::Thread::getCurrentThread();

    // Check that this hasn't already happened
    if (getIrqFlags() & irq)
        return;

    if (unlock)
        mutex.unlock();

    {
        miosix::FastInterruptDisableLock dLock;
        while (state.irq_wait_thread)
        {
            state.irq_wait_thread->IRQwait();
            {
                miosix::FastInterruptEnableLock eLock(dLock);
                miosix::Thread::yield();
            }
        }
    }

    // Regain ownership of the lock
    if (unlock)
        mutex.lock();
}

bool SX1278BusManager::waitForIrqBusy(const LockMode &_guard, uint16_t mask,
                                      int timeout)
{
    // Unused, just for API purposes
    (void)_guard;

    long long start = miosix::getTick();
    while ((miosix::getTick() - start) < timeout)
    {
        // Delay between polls
        const unsigned int DELAY = 100;

        // Tight loop on IRQ register
        for (unsigned int i = 0; i < 1000 / DELAY; i++)
        {
            if (getIrqFlags() & mask)
                return true;

            miosix::delayUs(DELAY);
        }
    }

    return false;
}

void SX1278BusManager::enterMode(Mode mode)
{
    // Check if necessary
    if (mode == state.mode)
        return;

    setMode(mode);

    state.mode = mode;
}

uint16_t SX1278BusManager::getIrqFlags()
{
    SPITransaction spi(slave, SPITransaction::WriteBit::INVERTED);

    uint8_t flags[2] = {0, 0};
    spi.readRegisters(REG_IRQ_FLAGS_1, flags, 2);

    return (flags[0] | flags[1] << 8);
}

void SX1278BusManager::setMode(Mode mode)
{
    SPITransaction spi(slave, SPITransaction::WriteBit::INVERTED);
    spi.writeRegister(REG_OP_MODE, REG_OP_MODE_DEFAULT | mode);
}

SX1278::SX1278(SPIBusInterface &bus, miosix::GpioPin cs) : bus_mgr(bus, cs) {}

SX1278::Error SX1278::init(const Config &config)
{
    // First probe for the device
    uint8_t version = getVersion();
    if (version != 0x12)
    {
        TRACE("[sx1278] Wrong chipid: %d\n", version);
        return Error::BAD_VALUE;
    }

    if (!configure(config))
        return Error::CONFIGURE_FAILED;

    return Error::NONE;
}

bool SX1278::probe() { return getVersion() == 0x12; }

bool SX1278::configure(const Config &config)
{
    // Lock the bus
    SX1278BusManager::LockMode guard(bus_mgr, Mode::MODE_STDBY);
    if (!bus_mgr.waitForIrqBusy(guard, RegIrqFlags::MODE_READY, 5))
        return false;

    SPISlave &slave = bus_mgr.getBus(guard);

    setBitrate(slave, config.bitrate);
    setFreqDev(slave, config.freq_dev);
    setFreqRF(slave, config.freq_rf);

    setRxBw(slave, config.rx_bw);
    setAfcBw(slave, config.afc_bw);

    setOcp(slave, config.ocp);

    uint8_t sync_word[2] = {0x12, 0xad};
    setSyncWord(slave, sync_word, 2);
    setPreableLen(slave, 2);
    setPa(slave, config.power, true);
    setShaping(slave, config.shaping);

    // Setup generic parameters
    {
        SPITransaction spi(bus_mgr.getBus(guard),
                           SPITransaction::WriteBit::INVERTED);

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

        // Enable:
        // - PayloadReady, PacketSent on DIO0 (mode 00)
        // - FifoLevel on DIO1 (mode 00)
        // - TxReady on DIO3 (mode 01)
        spi.writeRegister(REG_DIO_MAPPING_1, 0b01);
    }

    // imageCalibrate();
    return true;
}

ssize_t SX1278::receive(uint8_t *pkt, size_t max_len)
{
    SX1278BusManager::LockMode guard(bus_mgr, Mode::MODE_RX);

    uint8_t len = 0;
    do
    {
        // Special wait for payload ready
        bus_mgr.waitForIrq(guard, RegIrqFlags::PAYLOAD_READY, true);
        last_rx_rssi = getRssi(bus_mgr.getBus(guard));

        {
            SPITransaction spi(bus_mgr.getBus(guard),
                               SPITransaction::WriteBit::INVERTED);
            len = spi.readRegister(REG_FIFO);
            if (len > max_len)
                return -1;

            spi.readRegisters(REG_FIFO, pkt, len);
        }

        // For some reason this sometimes happen?
    } while (len == 0);

    return len;
}

bool SX1278::send(uint8_t *pkt, size_t len)
{
    // Packets longer than FIFO_LEN (-1 for the len byte) are not supported
    if (len > SX1278Defs::FIFO_LEN - 1)
        return false;

    // This shouldn't be needed, but for some reason the device "lies" about
    // being ready, so lock up if we are going too fast
    rateLimitTx();

    SX1278BusManager::LockMode guard(bus_mgr, Mode::MODE_TX);
    bus_mgr.waitForIrq(guard, RegIrqFlags::TX_READY);

    {
        SPITransaction spi(bus_mgr.getBus(guard),
                           SPITransaction::WriteBit::INVERTED);

        spi.writeRegister(REG_FIFO, static_cast<uint8_t>(len));
        spi.writeRegisters(REG_FIFO, pkt, len);
    }

    // Wait for packet sent
    bus_mgr.waitForIrq(guard, RegIrqFlags::PACKET_SENT);

    last_tx = now();
    return true;
}

void SX1278::handleDioIRQ(Dio dio) { bus_mgr.handleDioIRQ(dio); }

float SX1278::getLastRxFei()
{
    SX1278BusManager::Lock guard(bus_mgr);
    return getFei(bus_mgr.getBus(guard));
}

float SX1278::getLastRxRssi() { return last_rx_rssi; }

float SX1278::getCurRssi()
{
    SX1278BusManager::Lock guard(bus_mgr);
    return getRssi(bus_mgr.getBus(guard));
}

uint8_t SX1278::getVersion()
{
    SX1278BusManager::Lock guard(bus_mgr);
    SPITransaction spi(bus_mgr.getBus(guard),
                       SPITransaction::WriteBit::INVERTED);

    return spi.readRegister(REG_VERSION);
}

void SX1278::rateLimitTx()
{
    const long long RATE_LIMIT = 2;

    long long delta = now() - last_tx;
    if (delta <= RATE_LIMIT)
    {
        miosix::Thread::sleep(RATE_LIMIT - delta);
    }
}

void SX1278::imageCalibrate(SPISlave &slave)
{
    SPITransaction spi(slave, SPITransaction::WriteBit::INVERTED);
    spi.writeRegister(REG_IMAGE_CAL, REG_IMAGE_CAL_DEFAULT | (1 << 6));

    // Wait for calibration complete by polling on running register
    while (spi.readRegister(REG_IMAGE_CAL) & (1 << 5))
        miosix::delayUs(10);
}

float SX1278::getRssi(SPISlave &slave)
{
    SPITransaction spi(slave, SPITransaction::WriteBit::INVERTED);

    return static_cast<float>(spi.readRegister(REG_RSSI_VALUE)) * -0.5f;
}

float SX1278::getFei(SPISlave &slave)
{
    SPITransaction spi(slave, SPITransaction::WriteBit::INVERTED);

    // Order of read is important!!
    uint8_t fei_msb = spi.readRegister(REG_FEI_MSB);
    uint8_t fei_lsb = spi.readRegister(REG_FEI_LSB);

    uint16_t fei = (static_cast<uint16_t>(fei_msb) << 8) |
                   (static_cast<uint16_t>(fei_lsb));

    return static_cast<float>(fei) * FSTEP;
}

void SX1278::setBitrate(SPISlave &slave, int bitrate)
{
    uint16_t val = SX1278Defs::FXOSC / bitrate;

    // Update values
    SPITransaction spi(slave, SPITransaction::WriteBit::INVERTED);
    spi.writeRegister(REG_BITRATE_MSB, val >> 8);
    spi.writeRegister(REG_BITRATE_LSB, val);
}

void SX1278::setFreqDev(SPISlave &slave, int freq_dev)
{
    uint16_t val = freq_dev / FSTEP;
    SPITransaction spi(slave, SPITransaction::WriteBit::INVERTED);
    spi.writeRegister(REG_FDEV_MSB, (val >> 8) & 0x3f);
    spi.writeRegister(REG_FDEV_LSB, val);
}

void SX1278::setFreqRF(SPISlave &slave, int freq_rf)
{
    uint32_t val = freq_rf / FSTEP;

    // Update values
    SPITransaction spi(slave, SPITransaction::WriteBit::INVERTED);
    spi.writeRegister(REG_FRF_MSB, val >> 16);
    spi.writeRegister(REG_FRF_MID, val >> 8);
    spi.writeRegister(REG_FRF_LSB, val);
}

void SX1278::setOcp(SPISlave &slave, int ocp)
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

void SX1278::setSyncWord(SPISlave &slave, uint8_t value[], int size)
{
    SPITransaction spi(slave, SPITransaction::WriteBit::INVERTED);
    spi.writeRegister(REG_SYNC_CONFIG, REG_SYNC_CONFIG_DEFAULT | size);

    for (int i = 0; i < size; i++)
    {
        spi.writeRegister(REG_SYNC_VALUE_1 + i, value[i]);
    }
}

void SX1278::setRxBw(SPISlave &slave, RxBw rx_bw)
{
    SPITransaction spi(slave, SPITransaction::WriteBit::INVERTED);
    spi.writeRegister(REG_RX_BW, static_cast<uint8_t>(rx_bw));
}

void SX1278::setAfcBw(SPISlave &slave, RxBw afc_bw)
{
    SPITransaction spi(slave, SPITransaction::WriteBit::INVERTED);
    spi.writeRegister(REG_AFC_BW, static_cast<uint8_t>(afc_bw));
}

void SX1278::setPreableLen(SPISlave &slave, int len)
{
    SPITransaction spi(slave, SPITransaction::WriteBit::INVERTED);
    spi.writeRegister(REG_PREAMBLE_MSB, len >> 8);
    spi.writeRegister(REG_PREAMBLE_LSB, len);
}

void SX1278::setPa(SPISlave &slave, int power, bool pa_boost)
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

void SX1278::setShaping(SPISlave &slave, Shaping shaping)
{
    SPITransaction spi(slave, SPITransaction::WriteBit::INVERTED);
    spi.writeRegister(REG_PA_RAMP, static_cast<uint8_t>(shaping) | 0x09);
}

void SX1278::debugDumpRegisters()
{
    SX1278BusManager::Lock guard(bus_mgr);
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

    SPITransaction spi(bus_mgr.getBus(guard),
                       SPITransaction::WriteBit::INVERTED);

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
