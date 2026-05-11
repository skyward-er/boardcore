#include "SX127XFsk.h"

#include <kernel/scheduler/scheduler.h>
#include <utils/Debug.h>
#include <utils/KernelTime.h>

#include <algorithm>
#include <cassert>
#include <cmath>
#include <cstring>

namespace Boardcore
{

using namespace SX127x;
using namespace SX127x::Fsk;

namespace
{

long long now() { return Kernel::getOldTick(); }

template <typename T>
T clampValue(T value, T low, T high)
{
    return std::max(low, std::min(value, high));
}

constexpr DioMapping DEFAULT_MAPPING = DioMapping(0, 0, 0, 1, 0, 0, false);

}  // namespace

SX127XFsk::Error SX127XFsk::init(const Config& config)
{
    if (!checkVersion())
        return Error::BAD_VERSION;

    return configure(config);
}

bool SX127XFsk::checkVersion()
{
    Lock guard(*this);
    SPITransaction spi(getSpiSlave());

    uint8_t version = spi.readRegister(REG_VERSION);
    if (version == 0x12)
        return true;

    LOG_ERR(logger, "Wrong chip id: {}", version);
    return false;
}

SX127XFsk::Error SX127XFsk::configure(const Config& config)
{
    bool pa_boost = getFrontend().isOnPaBoost();
    int min_power = pa_boost ? 2 : 0;
    int max_power = getFrontend().maxInPower();

    bool validOcp = ((config.ocp >= 0 && config.ocp <= 120) ||
                     (config.ocp >= 130 && config.ocp <= 240));

    if (config.power < min_power || config.power > max_power || !validOcp ||
        config.freq_dev < MIN_FREQ_DEV || config.freq_dev > MAX_FREQ_DEV ||
        config.freq_rf < MIN_FREQ_RF || config.freq_rf > MAX_FREQ_RF ||
        config.bitrate <= 0 || config.sync_word_len < 1 ||
        config.sync_word_len > 8 || config.preamble_len < 1)
    {
        return Error::BAD_VALUE;
    }

    enterFskMode();

    setDefaultMode(RegOpMode::MODE_STDBY, DEFAULT_MAPPING,
                   InterruptTrigger::RISING_EDGE, false, false);
    miosix::Thread::sleep(1);

    Lock guard(*this);
    LockMode guard_mode(*this, guard, RegOpMode::MODE_STDBY, DEFAULT_MAPPING,
                        InterruptTrigger::RISING_EDGE);

    int bitrate = config.bitrate;
    int freq_dev = clampValue(config.freq_dev, MIN_FREQ_DEV, MAX_FREQ_DEV);
    int freq_rf  = clampValue(config.freq_rf, MIN_FREQ_RF, MAX_FREQ_RF);
    RegRxBw::RxBw rx_bw      = static_cast<RegRxBw::RxBw>(config.rx_bw);
    RegAfcBw::RxBwAfc afc_bw = static_cast<RegAfcBw::RxBwAfc>(config.afc_bw);
    int ocp   = config.ocp <= 120 ? clampValue(config.ocp, 0, 120)
                                  : clampValue(config.ocp, 130, 240);
    int power = clampValue(config.power, min_power, max_power);
    RegPaRamp::ModulationShaping shaping =
        static_cast<RegPaRamp::ModulationShaping>(config.shaping);
    RegPacketConfig1::DcFree dc_free =
        static_cast<RegPacketConfig1::DcFree>(config.dc_free);

    crc_enabled = config.enable_crc;

    SPITransaction spi(getSpiSlave());

    uint16_t bitrate_raw = FXOSC / bitrate;
    spi.writeRegister16(REG_BITRATE_MSB, bitrate_raw);

    uint16_t freq_dev_raw = freq_dev / FSTEP;
    spi.writeRegister16(REG_FDEV_MSB, freq_dev_raw & 0x3fff);

    uint32_t freq_rf_raw = freq_rf / FSTEP;
    spi.writeRegister24(REG_FRF_MSB, freq_rf_raw);

    spi.writeRegister(REG_RX_BW, RegRxBw::make(rx_bw));
    spi.writeRegister(REG_AFC_BW, RegAfcBw::make(afc_bw));

    if (config.ocp == 0)
        spi.writeRegister(REG_OCP, RegOcp::make(0, false));
    else if (ocp <= 120)
        spi.writeRegister(REG_OCP, RegOcp::make((ocp - 45) / 5, true));
    else
        spi.writeRegister(REG_OCP, RegOcp::make((ocp + 30) / 10, true));

    spi.writeRegister(
        REG_SYNC_CONFIG,
        RegSyncConfig::make(
            config.sync_word_len, true,
            static_cast<RegSyncConfig::PreamblePolarity>(
                config.preamble_polarity),
            RegSyncConfig::AUTO_RESTART_RX_MODE_OFF));

    for (int i = 0; i < config.sync_word_len; i++)
        spi.writeRegister(REG_SYNC_VALUE_1 + i, config.sync_word[i]);

    spi.writeRegister16(REG_PREAMBLE_MSB, config.preamble_len);

    spi.writeRegister(
        REG_PA_RAMP,
        RegPaRamp::make(static_cast<RegPaRamp::PaRamp>(config.pa_ramp),
                        shaping));

    const int MAX_POWER = 0b111;
    if (!pa_boost)
    {
        spi.writeRegister(REG_PA_CONFIG,
                          RegPaConfig::make(power, MAX_POWER, false));
        spi.writeRegister(REG_PA_DAC,
                          RegPaDac::make(RegPaDac::PA_DAC_DEFAULT_VALUE));
    }
    else if (power != 20)
    {
        spi.writeRegister(REG_PA_CONFIG,
                          RegPaConfig::make(power - 2, MAX_POWER, true));
        spi.writeRegister(REG_PA_DAC,
                          RegPaDac::make(RegPaDac::PA_DAC_PA_BOOST));
    }
    else
    {
        spi.writeRegister(REG_PA_CONFIG,
                          RegPaConfig::make(0b1111, MAX_POWER, true));
        spi.writeRegister(REG_PA_DAC,
                          RegPaDac::make(RegPaDac::PA_DAC_PA_BOOST));
    }

    spi.writeRegister(
        REG_RX_CONFIG,
        RegRxConfig::make(true, true, true, true, false, false, false));

    spi.writeRegister(REG_RSSI_THRESH, 0xff);

    spi.writeRegister(
        REG_PREAMBLE_DETECT,
        RegPreambleDetector::make(
            config.preamble_detector_tol,
            static_cast<RegPreambleDetector::Size>(
                config.preamble_detector_size),
            config.preamble_detector_enable));

    spi.writeRegister(REG_RX_TIMEOUT_1, 0x00);
    spi.writeRegister(REG_RX_TIMEOUT_2, 0x00);
    spi.writeRegister(REG_RX_TIMEOUT_3, 0x00);

    spi.writeRegister(
        REG_PACKET_CONFIG_1,
        RegPacketConfig1::make(
            static_cast<RegPacketConfig1::CrcWhiteningType>(config.whitening),
            RegPacketConfig1::ADDRESS_FILTERING_NONE, true, crc_enabled,
            dc_free, RegPacketConfig1::PACKET_FORMAT_VARIABLE_LENGTH));

    spi.writeRegister(
        REG_PACKET_CONFIG_2,
        RegPacketConfig2::make(false, false, false,
                               RegPacketConfig2::DATA_MODE_PACKET));

    spi.writeRegister(REG_PACKET_PAYLOAD_LENGTH, MTU);

    spi.writeRegister(
        REG_FIFO_THRESH,
        RegFifoThresh::make(
            FIFO_LEN / 2, RegFifoThresh::TX_START_CONDITION_FIFO_NOT_EMPTY));

    spi.writeRegister(REG_NODE_ADRS, 0x00);
    spi.writeRegister(REG_BROADCAST_ADRS, 0x00);

    return Error::NONE;
}

ssize_t SX127XFsk::receive(uint8_t* pkt, size_t max_len)
{
    Lock guard(*this);
    LockMode guard_mode(*this, guard, RegOpMode::MODE_RX, DEFAULT_MAPPING,
                        InterruptTrigger::RISING_EDGE, false, true);

    uint8_t tmp_pkt[MTU];

    uint8_t len = 0;
    bool crc_ok;
    IrqFlags flags = 0;

    do
    {
        crc_ok = false;

        uint8_t cur_len = 0;

        flags = waitForIrq(guard_mode,
                           RegIrqFlags::FIFO_LEVEL | RegIrqFlags::PAYLOAD_READY,
                           0, true);
        if ((flags & RegIrqFlags::PAYLOAD_READY) != 0 && crc_enabled)
            crc_ok = checkForIrqAndReset(RegIrqFlags::CRC_OK, 0) != 0;

        last_rx_rssi = getRssi();

        {
            SPITransaction spi(getSpiSlave());
            len = spi.readRegister(REG_FIFO);

            int read_size = std::min(static_cast<int>(len), FIFO_LEN / 2);
            if (read_size != 0)
                spi.readRegisters(REG_FIFO, &tmp_pkt[cur_len], read_size);

            cur_len += read_size;
        }

        while (cur_len < len)
        {
            flags = waitForIrq(
                guard_mode,
                RegIrqFlags::FIFO_LEVEL | RegIrqFlags::PAYLOAD_READY, 0);
            if ((flags & RegIrqFlags::PAYLOAD_READY) != 0 && crc_enabled)
                crc_ok = checkForIrqAndReset(RegIrqFlags::CRC_OK, 0) != 0;

            SPITransaction spi(getSpiSlave());

            int read_size =
                std::min(static_cast<int>(len - cur_len), FIFO_LEN / 2);
            spi.readRegisters(REG_FIFO, &tmp_pkt[cur_len], read_size);

            cur_len += read_size;
        }

    } while (len == 0);

    if (len > max_len || (!crc_ok && crc_enabled))
        return -1;

    memcpy(pkt, tmp_pkt, len);
    return len;
}

bool SX127XFsk::send(uint8_t* pkt, size_t len)
{
    if (len > MTU)
        return false;

    rateLimitTx();

    Lock guard(*this);
    LockMode guard_mode(*this, guard, RegOpMode::MODE_TX, DEFAULT_MAPPING,
                        InterruptTrigger::FALLING_EDGE, true, false);

    waitForIrq(guard_mode, RegIrqFlags::TX_READY, 0);

    {
        SPITransaction spi(getSpiSlave());

        spi.writeRegister(REG_FIFO, static_cast<uint8_t>(len));

        int write_size = std::min(static_cast<int>(len), FIFO_LEN - 1);
        spi.writeRegisters(REG_FIFO, pkt, write_size);

        pkt += write_size;
        len -= write_size;
    }

    while (len > 0)
    {
        waitForIrq(guard_mode, 0, RegIrqFlags::FIFO_LEVEL);

        SPITransaction spi(getSpiSlave());

        int write_size = std::min(static_cast<int>(len), FIFO_LEN / 2);
        spi.writeRegisters(REG_FIFO, pkt, write_size);

        pkt += write_size;
        len -= write_size;
    }

    waitForIrq(guard_mode, RegIrqFlags::PACKET_SENT, 0);

    last_tx = now();
    return true;
}

bool SX127XFsk::sendTimeout(uint8_t* pkt, size_t len, int timeoutMs)
{
    if (len > MTU)
        return false;

    rateLimitTx();

    Lock guard(*this);
    LockMode guard_mode(*this, guard, RegOpMode::MODE_TX, DEFAULT_MAPPING,
                        InterruptTrigger::FALLING_EDGE, true, false);

    if (waitForIrqTimeout(guard_mode, RegIrqFlags::TX_READY, 0, timeoutMs) == 0)
    {
        TRACE("[sx127x-fsk] tx timeout waiting TX_READY len=%u irq=0x%04x\n",
              static_cast<unsigned int>(len),
              static_cast<unsigned int>(getIrqFlags()));
        return false;
    }

    {
        SPITransaction spi(getSpiSlave());

        int first_payload = std::min(static_cast<int>(len), FIFO_LEN - 1);
        uint8_t tmp[FIFO_LEN];
        tmp[0] = static_cast<uint8_t>(len);
        if (first_payload > 0)
            memcpy(&tmp[1], pkt, first_payload);

        spi.writeRegisters(REG_FIFO, tmp, 1 + first_payload);

        pkt += first_payload;
        len -= first_payload;
    }

    while (len > 0)
    {
        if (waitForIrqTimeout(guard_mode, 0, RegIrqFlags::FIFO_LEVEL,
                              timeoutMs) == 0)
        {
            TRACE(
                "[sx127x-fsk] tx timeout waiting FIFO_LEVEL len=%u irq=0x%04x\n",
                static_cast<unsigned int>(len),
                static_cast<unsigned int>(getIrqFlags()));
            return false;
        }

        SPITransaction spi(getSpiSlave());

        int write_size = std::min(static_cast<int>(len), FIFO_LEN / 2);
        spi.writeRegisters(REG_FIFO, pkt, write_size);

        pkt += write_size;
        len -= write_size;
    }

    if (waitForIrqTimeout(guard_mode, RegIrqFlags::PACKET_SENT, 0,
                          timeoutMs) == 0)
    {
        TRACE("[sx127x-fsk] tx timeout waiting PACKET_SENT irq=0x%04x\n",
              static_cast<unsigned int>(getIrqFlags()));
        return false;
    }

    last_tx = now();
    return true;
}

ssize_t SX127XFsk::receiveTimeout(uint8_t* pkt, size_t max_len, int timeoutMs)
{
    Lock guard(*this);
    LockMode guard_mode(*this, guard, RegOpMode::MODE_RX, DEFAULT_MAPPING,
                        InterruptTrigger::RISING_EDGE, false, true);

    uint8_t tmp_pkt[MTU];
    uint8_t len = 0;
    bool crc_ok;
    IrqFlags flags = 0;

    do
    {
        crc_ok = false;

        uint8_t cur_len = 0;

        flags = waitForIrqTimeout(
            guard_mode,
            RegIrqFlags::FIFO_LEVEL | RegIrqFlags::PAYLOAD_READY, 0, timeoutMs,
            true);
        if (flags == 0)
            return -2;

        if ((flags & RegIrqFlags::PAYLOAD_READY) != 0 && crc_enabled)
            crc_ok = checkForIrqAndReset(RegIrqFlags::CRC_OK, 0) != 0;

        last_rx_rssi = getRssi();

        {
            SPITransaction spi(getSpiSlave());
            uint8_t first[FIFO_LEN / 2 + 1];

            size_t first_payload = FIFO_LEN / 2;
            spi.readRegisters(REG_FIFO, first, 1 + first_payload);

            len = first[0];
            int copy_len = std::min(static_cast<int>(len),
                                    static_cast<int>(first_payload));
            if (copy_len > 0)
                memcpy(&tmp_pkt[cur_len], &first[1], copy_len);
            cur_len += copy_len;
        }

        while (cur_len < len)
        {
            flags = waitForIrqTimeout(
                guard_mode,
                RegIrqFlags::FIFO_LEVEL | RegIrqFlags::PAYLOAD_READY, 0,
                timeoutMs);
            if (flags == 0)
                return -2;

            if ((flags & RegIrqFlags::PAYLOAD_READY) != 0 && crc_enabled)
                crc_ok = checkForIrqAndReset(RegIrqFlags::CRC_OK, 0) != 0;

            SPITransaction spi(getSpiSlave());

            int read_size =
                std::min(static_cast<int>(len - cur_len), FIFO_LEN / 2);
            spi.readRegisters(REG_FIFO, &tmp_pkt[cur_len], read_size);

            cur_len += read_size;
        }

    } while (len == 0);

    if (len > max_len || (!crc_ok && crc_enabled))
    {
        TRACE(
            "[sx127x-fsk] rx packet rejected len=%u max=%u crc_ok=%d crc_en=%d\n",
            static_cast<unsigned int>(len), static_cast<unsigned int>(max_len),
            crc_ok ? 1 : 0, crc_enabled ? 1 : 0);
        return -1;
    }

    memcpy(pkt, tmp_pkt, len);
    return len;
}

float SX127XFsk::getLastRxFei()
{
    Lock guard(*this);
    return getFei();
}

float SX127XFsk::getLastRxRssi() { return last_rx_rssi; }

float SX127XFsk::getCurRssi()
{
    Lock guard(*this);
    return getRssi();
}

void SX127XFsk::enterFskMode()
{
    Lock guard(*this);
    SPITransaction spi(getSpiSlave());

    spi.writeRegister(REG_OP_MODE,
                      RegOpMode::make(RegOpMode::MODE_SLEEP, true,
                                      RegOpMode::MODULATION_TYPE_FSK));
    miosix::Thread::sleep(1);

    spi.writeRegister(REG_OP_MODE,
                      RegOpMode::make(RegOpMode::MODE_STDBY, true,
                                      RegOpMode::MODULATION_TYPE_FSK));
    miosix::Thread::sleep(1);
}

void SX127XFsk::rateLimitTx()
{
    const long long RATE_LIMIT = 2;

    long long delta = now() - last_tx;
    if (delta <= RATE_LIMIT)
        miosix::Thread::sleep(RATE_LIMIT - delta);
}

SX127XFsk::IrqFlags SX127XFsk::getIrqFlags()
{
    SPITransaction spi(getSpiSlave());
    return spi.readRegister16(REG_IRQ_FLAGS_1);
}

void SX127XFsk::resetIrqFlags(IrqFlags flags)
{
    flags &= RegIrqFlags::RSSI | RegIrqFlags::PREAMBLE_DETECT |
             RegIrqFlags::SYNC_ADDRESS_MATCH | RegIrqFlags::FIFO_OVERRUN |
             RegIrqFlags::LOW_BAT;

    if (flags != 0)
    {
        SPITransaction spi(getSpiSlave());
        spi.writeRegister16(REG_IRQ_FLAGS_1, flags);
    }
}

float SX127XFsk::getRssi()
{
    SPITransaction spi(getSpiSlave());

    uint8_t rssi_raw = spi.readRegister(REG_RSSI_VALUE);
    return static_cast<float>(rssi_raw) * -0.5f;
}

float SX127XFsk::getFei()
{
    SPITransaction spi(getSpiSlave());

    uint16_t fei_raw = spi.readRegister16(REG_FEI_MSB);
    return static_cast<float>(fei_raw) * FSTEP;
}

void SX127XFsk::setMode(Mode mode)
{
    SPITransaction spi(getSpiSlave());
    spi.writeRegister(REG_OP_MODE,
                      RegOpMode::make(static_cast<RegOpMode::Mode>(mode), true,
                                      RegOpMode::MODULATION_TYPE_FSK));
}

void SX127XFsk::setMapping(SX127x::DioMapping mapping)
{
    SPITransaction spi(getSpiSlave());
    spi.writeRegister16(REG_DIO_MAPPING_1, mapping.raw);
}

}  // namespace Boardcore
