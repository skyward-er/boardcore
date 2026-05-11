#pragma once

#include <diagnostic/PrintLogger.h>

#include "SX127XCommon.h"
#include "SX127XLoraTimings.h"

namespace Boardcore
{

class SX127XLora : public SX127x::SX127XCommon
{
public:
    static constexpr size_t MTU = 255;

    struct Config
    {
        enum class Bw
        {
            HZ_7800   = SX127x::Lora::RegModemConfig1::BW_HZ_7800,
            HZ_10400  = SX127x::Lora::RegModemConfig1::BW_HZ_10400,
            HZ_15600  = SX127x::Lora::RegModemConfig1::BW_HZ_15600,
            HZ_20800  = SX127x::Lora::RegModemConfig1::BW_HZ_20800,
            HZ_31250  = SX127x::Lora::RegModemConfig1::BW_HZ_31250,
            HZ_41700  = SX127x::Lora::RegModemConfig1::BW_HZ_41700,
            HZ_62500  = SX127x::Lora::RegModemConfig1::BW_HZ_62500,
            HZ_125000 = SX127x::Lora::RegModemConfig1::BW_HZ_125000,
            HZ_250000 = SX127x::Lora::RegModemConfig1::BW_HZ_250000,
            HZ_500000 = SX127x::Lora::RegModemConfig1::BW_HZ_500000,
        };

        enum class Cr
        {
            CR_1 = SX127x::Lora::RegModemConfig1::CR_1,
            CR_2 = SX127x::Lora::RegModemConfig1::CR_2,
            CR_3 = SX127x::Lora::RegModemConfig1::CR_3,
            CR_4 = SX127x::Lora::RegModemConfig1::CR_4,
        };

        enum class Sf
        {
            SF_7  = SX127x::Lora::RegModemConfig2::SF_7,
            SF_8  = SX127x::Lora::RegModemConfig2::SF_8,
            SF_9  = SX127x::Lora::RegModemConfig2::SF_9,
            SF_10 = SX127x::Lora::RegModemConfig2::SF_10,
            SF_11 = SX127x::Lora::RegModemConfig2::SF_11,
            SF_12 = SX127x::Lora::RegModemConfig2::SF_12,
        };

        Bw bandwidth        = Bw::HZ_125000;
        Cr coding_rate      = Cr::CR_1;
        Sf spreading_factor = Sf::SF_7;
        bool low_data_rate_optimize = false;
        int freq_rf                  = 434000000;
        int ocp                      = 120;
        int power                    = 13;
        bool enable_crc              = true;

        uint32_t effectiveBitrate() const
        {
            using namespace SX127x::Lora;

            return SX127x::Lora::effectiveBitrate(
                static_cast<RegModemConfig2::Sf>(spreading_factor),
                RegModemConfig1::bandwidthToInt(
                    static_cast<RegModemConfig1::Bw>(bandwidth)),
                static_cast<RegModemConfig1::Cr>(coding_rate));
        }
    };

    enum class Error
    {
        NONE,
        BAD_VALUE,
        BAD_VERSION,
        IRQ_TIMEOUT,
    };

    explicit SX127XLora(SPIBus& bus, miosix::GpioPin cs, miosix::GpioPin dio0,
                        miosix::GpioPin dio1, miosix::GpioPin dio3,
                        SPI::ClockDivider clock_divider,
                        std::unique_ptr<SX127x::ISX127XFrontend> frontend)
        : SX127XCommon(bus, cs, dio0, dio1, dio3, clock_divider,
                       std::move(frontend)),
          crc_enabled(false), low_frequency_mode_on(true)
    {
    }

    [[nodiscard]] Error init(const Config& config);
    bool checkVersion();
    [[nodiscard]] Error configure(const Config& config);

    ssize_t receive(uint8_t* pkt, size_t max_len) override;
    bool send(uint8_t* pkt, size_t len) override;
    bool sendTimeout(uint8_t* pkt, size_t len, int timeoutMs) override;
    ssize_t receiveTimeout(uint8_t* pkt, size_t max_len,
                           int timeoutMs) override;
    bool setFrequency(uint32_t freqHz) override;

    float getLastRxRssi() override;
    float getLastRxSnr() override;

private:
    void enterLoraMode();

    void readFifo(uint8_t addr, uint8_t* dst, uint8_t size);
    void writeFifo(uint8_t addr, uint8_t* src, uint8_t size);

    IrqFlags getIrqFlags() override;
    void resetIrqFlags(IrqFlags flags) override;

    void setMode(Mode mode) override;
    void setMapping(SX127x::DioMapping mapping) override;

    bool crc_enabled;
    bool low_frequency_mode_on;
    PrintLogger logger = Logging::getLogger("sx127x-lora");
};

}  // namespace Boardcore
