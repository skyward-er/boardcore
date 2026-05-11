#include "SX127XLora.h"

#include <utils/Debug.h>

#include <algorithm>

namespace Boardcore
{

using namespace SX127x;
using namespace SX127x::Lora;

static constexpr uint8_t MAX_PAYLOAD_LENGTH = 0xff;
static constexpr uint8_t FIFO_TX_BASE_ADDR  = 0x00;
static constexpr uint8_t FIFO_RX_BASE_ADDR  = 0x00;
static constexpr float RSSI_OFFSET_LF_DBM   = 164.0f;
static constexpr float RSSI_OFFSET_HF_DBM   = 157.0f;

template <typename T>
T clampValue(T value, T low, T high)
{
    return std::max(low, std::min(value, high));
}

bool isLowFrequencyMode(int freq_rf) { return freq_rf <= 525000000; }

static constexpr DioMapping DEFAULT_MAPPING =
    DioMapping(0, 0, 0, 0, 0, 0, false);

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

SX127XLora::Error SX127XLora::init(const Config& config)
{
    if (!checkVersion())
        return Error::BAD_VERSION;

    return configure(config);
}

bool SX127XLora::checkVersion()
{
    Lock guard(*this);
    SPITransaction spi(getSpiSlave());

    uint8_t version = spi.readRegister(REG_VERSION);
    if (version == 0x12)
        return true;

    LOG_ERR(logger, "Wrong chip id: {}", version);
    return false;
}

SX127XLora::Error SX127XLora::configure(const Config& config)
{
    bool pa_boost = getFrontend().isOnPaBoost();
    int min_power = pa_boost ? 2 : 0;
    int max_power = getFrontend().maxInPower();

    bool validOcp = ((config.ocp >= 0 && config.ocp <= 120) ||
                     (config.ocp >= 130 && config.ocp <= 240));

    if (config.power < min_power || config.power > max_power || !validOcp ||
        config.freq_rf < MIN_FREQ_RF || config.freq_rf > MAX_FREQ_RF)
    {
        return Error::BAD_VALUE;
    }

    RegModemConfig1::Bw bw = static_cast<RegModemConfig1::Bw>(config.bandwidth);
    RegModemConfig1::Cr cr =
        static_cast<RegModemConfig1::Cr>(config.coding_rate);
    RegModemConfig2::Sf sf =
        static_cast<RegModemConfig2::Sf>(config.spreading_factor);

    int freq_rf = clampValue(config.freq_rf, MIN_FREQ_RF, MAX_FREQ_RF);
    int ocp     = config.ocp <= 120 ? clampValue(config.ocp, 0, 120)
                                    : clampValue(config.ocp, 130, 240);
    int power   = clampValue(config.power, min_power, max_power);

    low_frequency_mode_on = isLowFrequencyMode(freq_rf);

    enterLoraMode();

    setDefaultMode(RegOpMode::MODE_STDBY, DEFAULT_MAPPING,
                   InterruptTrigger::RISING_EDGE, false, false);

    Lock guard(*this);
    LockMode mode_guard(*this, guard, RegOpMode::MODE_STDBY, DEFAULT_MAPPING,
                        InterruptTrigger::RISING_EDGE);

    bool low_data_rate_optimize = config.low_data_rate_optimize;

    if (Lora::symbolDuration(sf, RegModemConfig1::bandwidthToInt(bw)) > 16000)
        low_data_rate_optimize = true;

    ErrataRegistersValues errata_values =
        ErrataRegistersValues::calculate(bw, freq_rf);

    crc_enabled = config.enable_crc;

    SPITransaction spi(getSpiSlave());

    spi.writeRegister(REG_FIFO_TX_BASE_ADDR, FIFO_TX_BASE_ADDR);
    spi.writeRegister(REG_FIFO_RX_BASE_ADDR, FIFO_RX_BASE_ADDR);
    spi.writeRegister(REG_MAX_PAYLOAD_LENGTH, MAX_PAYLOAD_LENGTH);

    uint32_t freq_rf_raw = errata_values.freq_rf / FSTEP;
    spi.writeRegister24(REG_FRF_MSB, freq_rf_raw);

    const int MAX_POWER = 0b111;
    if (!pa_boost)
    {
        spi.writeRegister(REG_PA_CONFIG,
                          RegPaConfig::make(power, MAX_POWER, false));
        spi.writeRegister(REG_PA_DAC, RegPaDac::make(false));
    }
    else if (power != 20)
    {
        spi.writeRegister(REG_PA_CONFIG,
                          RegPaConfig::make(power - 2, MAX_POWER, true));
        spi.writeRegister(REG_PA_DAC, RegPaDac::make(false));
    }
    else
    {
        spi.writeRegister(REG_PA_CONFIG,
                          RegPaConfig::make(0b1111, MAX_POWER, true));
        spi.writeRegister(REG_PA_DAC, RegPaDac::make(true));
    }

    if (config.ocp == 0)
        spi.writeRegister(REG_OCP, RegOcp::make(0, false));
    else if (ocp <= 120)
        spi.writeRegister(REG_OCP, RegOcp::make((ocp - 45) / 5, true));
    else
        spi.writeRegister(REG_OCP, RegOcp::make((ocp + 30) / 10, true));

    spi.writeRegister(REG_MODEM_CONFIG_1, RegModemConfig1::make(false, cr, bw));
    spi.writeRegister(REG_MODEM_CONFIG_2,
                      RegModemConfig2::make(crc_enabled, false, sf));
    spi.writeRegister(REG_MODEM_CONFIG_3,
                      RegModemConfig3::make(true, low_data_rate_optimize));

    spi.writeRegister(
        REG_DETECT_OPTIMIZE,
        RegDetectOptimize::make(0x03, errata_values.automatic_if_on));
    spi.writeRegister(REG_DETECTION_THRESHOLD, 0x0a);

    spi.writeRegister(REG_SYNC_WORD, 0x69);

    if (errata_values.reg_high_bw_optimize_1 != -1)
    {
        spi.writeRegister(REG_HIGH_BW_OPTIMIZE_1,
                          errata_values.reg_high_bw_optimize_1);
    }
    if (errata_values.reg_high_bw_optimize_2 != -1)
    {
        spi.writeRegister(REG_HIGH_BW_OPTIMIZE_2,
                          errata_values.reg_high_bw_optimize_2);
    }
    if (errata_values.reg_if_freq_1 != -1)
        spi.writeRegister(REG_IF_FREQ_1, errata_values.reg_if_freq_1);
    if (errata_values.reg_if_freq_2 != -1)
        spi.writeRegister(REG_IF_FREQ_2, errata_values.reg_if_freq_2);

    return Error::NONE;
}

ssize_t SX127XLora::receive(uint8_t* pkt, size_t max_len)
{
    Lock guard(*this);
    resetIrqFlags(0xff);

    LockMode mode_guard(*this, guard, RegOpMode::MODE_RXCONTINUOUS,
                        DioMapping(0, 0, 0, 0, 0, 0),
                        InterruptTrigger::RISING_EDGE, false, true);

    waitForIrq(mode_guard, RegIrqFlags::RX_DONE, 0, true);

    uint8_t len;
    uint8_t fifo_addr;
    {
        SPITransaction spi(getSpiSlave());
        fifo_addr = spi.readRegister(REG_FIFO_RX_CURRENT_ADDR);
        len       = spi.readRegister(REG_RX_NB_BYTES);
    }

    if (len > max_len ||
        (crc_enabled &&
         checkForIrqAndReset(RegIrqFlags::PAYLOAD_CRC_ERROR, 0) != 0))
    {
        return -1;
    }

    readFifo(fifo_addr, pkt, len);
    return len;
}

bool SX127XLora::send(uint8_t* pkt, size_t len)
{
    if (len > MTU)
        return false;

    Lock guard(*this);

    {
        SPITransaction spi(getSpiSlave());
        spi.writeRegister(REG_PAYLOAD_LENGTH, len);
        writeFifo(FIFO_TX_BASE_ADDR, pkt, len);
    }

    resetIrqFlags(0xff);

    {
        LockMode mode_guard(*this, guard, RegOpMode::MODE_TX,
                            DioMapping(1, 0, 0, 0, 0, 0),
                            InterruptTrigger::RISING_EDGE, true, false);

        waitForIrq(mode_guard, RegIrqFlags::TX_DONE, 0);
    }

    return true;
}

bool SX127XLora::sendTimeout(uint8_t* pkt, size_t len, int timeoutMs)
{
    if (len > MTU)
        return false;

    Lock guard(*this);

    {
        SPITransaction spi(getSpiSlave());
        spi.writeRegister(REG_PAYLOAD_LENGTH, len);
        writeFifo(FIFO_TX_BASE_ADDR, pkt, len);
    }

    resetIrqFlags(0xff);

    {
        LockMode mode_guard(*this, guard, RegOpMode::MODE_TX,
                            DioMapping(1, 0, 0, 0, 0, 0),
                            InterruptTrigger::RISING_EDGE, true, false);

        if (waitForIrqTimeout(mode_guard, RegIrqFlags::TX_DONE, 0,
                              timeoutMs) == 0)
        {
            TRACE(
                "[sx127x-lora] tx timeout waiting TX_DONE len=%u timeout=%d irq=0x%02x\n",
                static_cast<unsigned int>(len), timeoutMs,
                static_cast<unsigned int>(getIrqFlags()));
            return false;
        }
    }

    return true;
}

ssize_t SX127XLora::receiveTimeout(uint8_t* pkt, size_t max_len, int timeoutMs)
{
    Lock guard(*this);
    resetIrqFlags(0xff);

    LockMode mode_guard(*this, guard, RegOpMode::MODE_RXCONTINUOUS,
                        DioMapping(0, 0, 0, 0, 0, 0),
                        InterruptTrigger::RISING_EDGE, false, true);

    if (waitForIrqTimeout(mode_guard, RegIrqFlags::RX_DONE, 0, timeoutMs,
                          true) == 0)
    {
        return -2;
    }

    uint8_t len;
    uint8_t fifo_addr;
    {
        SPITransaction spi(getSpiSlave());
        fifo_addr = spi.readRegister(REG_FIFO_RX_CURRENT_ADDR);
        len       = spi.readRegister(REG_RX_NB_BYTES);
    }

    if (len > max_len ||
        (crc_enabled &&
         checkForIrqAndReset(RegIrqFlags::PAYLOAD_CRC_ERROR, 0) != 0))
    {
        TRACE(
            "[sx127x-lora] rx packet rejected len=%u max=%u crc_en=%d irq=0x%02x\n",
            static_cast<unsigned int>(len), static_cast<unsigned int>(max_len),
            crc_enabled ? 1 : 0, static_cast<unsigned int>(getIrqFlags()));
        return -1;
    }

    readFifo(fifo_addr, pkt, len);
    return len;
}

float SX127XLora::getLastRxRssi()
{
    float rssi;
    {
        Lock guard(*this);
        SPITransaction spi(getSpiSlave());
        const float rssi_offset =
            low_frequency_mode_on ? RSSI_OFFSET_LF_DBM : RSSI_OFFSET_HF_DBM;
        rssi =
            static_cast<float>(spi.readRegister(REG_PKT_RSSI_VALUE)) -
            rssi_offset;
    }

    float snr = getLastRxSnr();
    if (snr < 0.0f)
        rssi += snr * 0.25f;

    return rssi;
}

float SX127XLora::getLastRxSnr()
{
    Lock guard(*this);
    SPITransaction spi(getSpiSlave());
    return static_cast<float>(
               static_cast<int8_t>(spi.readRegister(REG_PKT_SNR_VALUE))) /
           4.0f;
}

bool SX127XLora::setFrequency(uint32_t freqHz)
{
    if (freqHz < MIN_FREQ_RF || freqHz > MAX_FREQ_RF)
        return false;

    const bool new_low_frequency_mode_on = isLowFrequencyMode(freqHz);
    if (new_low_frequency_mode_on != low_frequency_mode_on)
    {
        low_frequency_mode_on = new_low_frequency_mode_on;
        enterLoraMode();
    }

    return SX127XCommon::setFrequency(freqHz);
}

void SX127XLora::enterLoraMode()
{
    Lock guard(*this);
    SPITransaction spi(getSpiSlave());

    spi.writeRegister(REG_OP_MODE,
                      RegOpMode::make(RegOpMode::MODE_SLEEP,
                                      low_frequency_mode_on, false));
    miosix::Thread::sleep(1);

    spi.writeRegister(REG_OP_MODE,
                      RegOpMode::make(RegOpMode::MODE_STDBY,
                                      low_frequency_mode_on, false));
    miosix::Thread::sleep(1);
}

void SX127XLora::readFifo(uint8_t addr, uint8_t* dst, uint8_t size)
{
    SPITransaction spi(getSpiSlave());
    spi.writeRegister(REG_FIFO_ADDR_PTR, addr);
    spi.readRegisters(REG_FIFO, dst, size);
}

void SX127XLora::writeFifo(uint8_t addr, uint8_t* src, uint8_t size)
{
    SPITransaction spi(getSpiSlave());
    spi.writeRegister(REG_FIFO_ADDR_PTR, addr);
    spi.writeRegisters(REG_FIFO, src, size);
}

SX127XLora::IrqFlags SX127XLora::getIrqFlags()
{
    SPITransaction spi(getSpiSlave());
    return spi.readRegister(REG_IRQ_FLAGS);
}

void SX127XLora::resetIrqFlags(IrqFlags flags)
{
    SPITransaction spi(getSpiSlave());
    spi.writeRegister(REG_IRQ_FLAGS, flags);
}

void SX127XLora::setMode(Mode mode)
{
    SPITransaction spi(getSpiSlave());
    spi.writeRegister(
        REG_OP_MODE, RegOpMode::make(static_cast<RegOpMode::Mode>(mode),
                                     low_frequency_mode_on, false));
}

void SX127XLora::setMapping(SX127x::DioMapping mapping)
{
    SPITransaction spi(getSpiSlave());
    spi.writeRegister16(REG_DIO_MAPPING_1, mapping.raw);
}

}  // namespace Boardcore
