#pragma once

#include <diagnostic/PrintLogger.h>
#include <drivers/spi/SPIDriver.h>

#include <array>
#include <cstdint>

#include "SX127XCommon.h"

namespace Boardcore
{

class SX127XFsk : public SX127x::SX127XCommon
{
public:
    static constexpr size_t MTU = 255;

    struct Config
    {
        enum class RxBw
        {
            HZ_2600   = SX127x::Fsk::RegRxBw::HZ_2600,
            HZ_3100   = SX127x::Fsk::RegRxBw::HZ_3100,
            HZ_3900   = SX127x::Fsk::RegRxBw::HZ_3900,
            HZ_5200   = SX127x::Fsk::RegRxBw::HZ_5200,
            HZ_6300   = SX127x::Fsk::RegRxBw::HZ_6300,
            HZ_7800   = SX127x::Fsk::RegRxBw::HZ_7800,
            HZ_10400  = SX127x::Fsk::RegRxBw::HZ_10400,
            HZ_12500  = SX127x::Fsk::RegRxBw::HZ_12500,
            HZ_15600  = SX127x::Fsk::RegRxBw::HZ_15600,
            HZ_20800  = SX127x::Fsk::RegRxBw::HZ_20800,
            HZ_25000  = SX127x::Fsk::RegRxBw::HZ_25000,
            HZ_31300  = SX127x::Fsk::RegRxBw::HZ_31300,
            HZ_41700  = SX127x::Fsk::RegRxBw::HZ_41700,
            HZ_50000  = SX127x::Fsk::RegRxBw::HZ_50000,
            HZ_62500  = SX127x::Fsk::RegRxBw::HZ_62500,
            HZ_83300  = SX127x::Fsk::RegRxBw::HZ_83300,
            HZ_100000 = SX127x::Fsk::RegRxBw::HZ_100000,
            HZ_125000 = SX127x::Fsk::RegRxBw::HZ_125000,
            HZ_166700 = SX127x::Fsk::RegRxBw::HZ_166700,
            HZ_200000 = SX127x::Fsk::RegRxBw::HZ_200000,
            HZ_250000 = SX127x::Fsk::RegRxBw::HZ_250000,
        };

        enum class Shaping
        {
            NONE = SX127x::Fsk::RegPaRamp::MODULATION_SHAPING_NONE,
            GAUSSIAN_BT_1_0 =
                SX127x::Fsk::RegPaRamp::MODULATION_SHAPING_GAUSSIAN_BT_1_0,
            GAUSSIAN_BT_0_5 =
                SX127x::Fsk::RegPaRamp::MODULATION_SHAPING_GAUSSIAN_BT_0_5,
            GAUSSIAN_BT_0_3 =
                SX127x::Fsk::RegPaRamp::MODULATION_SHAPING_GAUSSIAN_BT_0_3,
        };

        enum class DcFree
        {
            NONE       = SX127x::Fsk::RegPacketConfig1::DC_FREE_NONE,
            MANCHESTER = SX127x::Fsk::RegPacketConfig1::DC_FREE_MANCHESTER,
            WHITENING  = SX127x::Fsk::RegPacketConfig1::DC_FREE_WHITENING,
        };

        enum class PaRamp
        {
            MS_3_4 = SX127x::Fsk::RegPaRamp::PA_RAMP_MS_3_4,
            MS_2   = SX127x::Fsk::RegPaRamp::PA_RAMP_MS_2,
            MS_1   = SX127x::Fsk::RegPaRamp::PA_RAMP_MS_1,
            US_500 = SX127x::Fsk::RegPaRamp::PA_RAMP_US_500,
            US_250 = SX127x::Fsk::RegPaRamp::PA_RAMP_US_250,
            US_125 = SX127x::Fsk::RegPaRamp::PA_RAMP_US_125,
            US_100 = SX127x::Fsk::RegPaRamp::PA_RAMP_US_100,
            US_62  = SX127x::Fsk::RegPaRamp::PA_RAMP_US_62,
            US_50  = SX127x::Fsk::RegPaRamp::PA_RAMP_US_50,
            US_40  = SX127x::Fsk::RegPaRamp::PA_RAMP_US_40,
            US_31  = SX127x::Fsk::RegPaRamp::PA_RAMP_US_31,
            US_25  = SX127x::Fsk::RegPaRamp::PA_RAMP_US_25,
            US_20  = SX127x::Fsk::RegPaRamp::PA_RAMP_US_20,
            US_15  = SX127x::Fsk::RegPaRamp::PA_RAMP_US_15,
            US_12  = SX127x::Fsk::RegPaRamp::PA_RAMP_US_12,
            US_10  = SX127x::Fsk::RegPaRamp::PA_RAMP_US_10,
        };

        enum class PreambleDetectorSize
        {
            BYTES_1 = SX127x::Fsk::RegPreambleDetector::
                PREAMBLE_DETECTOR_SIZE_1_BYTE,
            BYTES_2 = SX127x::Fsk::RegPreambleDetector::
                PREAMBLE_DETECTOR_SIZE_2_BYTES,
            BYTES_3 = SX127x::Fsk::RegPreambleDetector::
                PREAMBLE_DETECTOR_SIZE_3_BYTES,
        };

        enum class PreamblePolarity
        {
            AA  = SX127x::Fsk::RegSyncConfig::PREAMBLE_POLARITY_AA,
            P55 = SX127x::Fsk::RegSyncConfig::PREAMBLE_POLARITY_55,
        };

        enum class WhiteningType
        {
            CCITT_CRC =
                SX127x::Fsk::RegPacketConfig1::CRC_WHITENING_TYPE_CCITT_CRC,
            IBM_CRC =
                SX127x::Fsk::RegPacketConfig1::CRC_WHITENING_TYPE_IBM_CRC,
        };

        int freq_rf                    = 434000000;
        int freq_dev                   = 50000;
        int bitrate                    = 48000;
        RxBw rx_bw                     = RxBw::HZ_125000;
        RxBw afc_bw                    = RxBw::HZ_125000;
        int ocp                        = 120;
        int power                      = 13;
        Shaping shaping                = Shaping::GAUSSIAN_BT_1_0;
        PaRamp pa_ramp                 = PaRamp::US_40;
        int preamble_len               = 3;
        bool preamble_detector_enable  = true;
        int preamble_detector_tol      = 0x0a;
        PreambleDetectorSize preamble_detector_size =
            PreambleDetectorSize::BYTES_2;
        PreamblePolarity preamble_polarity = PreamblePolarity::P55;
        int sync_word_len                  = 2;
        std::array<uint8_t, 8> sync_word   = {0x12, 0xad, 0, 0, 0, 0, 0, 0};
        int rssi_smoothing                 = 0;
        WhiteningType whitening            = WhiteningType::CCITT_CRC;
        DcFree dc_free                     = DcFree::WHITENING;
        bool enable_crc                    = true;
    };

    enum class Error
    {
        NONE,
        BAD_VALUE,
        BAD_VERSION,
        IRQ_TIMEOUT,
    };

    explicit SX127XFsk(SPIBus& bus, miosix::GpioPin cs, miosix::GpioPin dio0,
                       miosix::GpioPin dio1, miosix::GpioPin dio3,
                       SPI::ClockDivider clock_divider,
                       std::unique_ptr<SX127x::ISX127XFrontend> frontend)
        : SX127XCommon(bus, cs, dio0, dio1, dio3, clock_divider,
                       std::move(frontend)),
          crc_enabled(false)
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

    float getCurRssi();
    float getLastRxRssi() override;
    float getLastRxFei() override;

private:
    void enterFskMode();
    void rateLimitTx();

    IrqFlags getIrqFlags() override;
    void resetIrqFlags(IrqFlags flags) override;

    float getRssi();
    float getFei();

    void setMode(Mode mode) override;
    void setMapping(SX127x::DioMapping mapping) override;

    bool crc_enabled;
    long long last_tx  = 0;
    float last_rx_rssi = 0.0f;
    PrintLogger logger = Logging::getLogger("sx1278-fsk");
};

}  // namespace Boardcore
