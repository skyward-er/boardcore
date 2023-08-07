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

#pragma once

#include <diagnostic/PrintLogger.h>
#include <drivers/spi/SPIDriver.h>

#include <cstdint>

#include "SX1278Common.h"

namespace Boardcore
{

/**
 * @brief Various SX1278 register/enums definitions.
 */
class SX1278Fsk : public SX1278::SX1278Common
{
public:
    /**
     * @brief Maximum transmittable unit of this driver.
     */
    static constexpr size_t MTU = 255;

    /**
     * @brief Requested SX1278 configuration.
     */
    struct Config
    {
        /**
         * @brief Channel filter bandwidth.
         */
        enum class RxBw
        {
            HZ_2600   = SX1278::Fsk::RegRxBw::HZ_2600,
            HZ_3100   = SX1278::Fsk::RegRxBw::HZ_3100,
            HZ_3900   = SX1278::Fsk::RegRxBw::HZ_3900,
            HZ_5200   = SX1278::Fsk::RegRxBw::HZ_5200,
            HZ_6300   = SX1278::Fsk::RegRxBw::HZ_6300,
            HZ_7800   = SX1278::Fsk::RegRxBw::HZ_7800,
            HZ_10400  = SX1278::Fsk::RegRxBw::HZ_10400,
            HZ_12500  = SX1278::Fsk::RegRxBw::HZ_12500,
            HZ_15600  = SX1278::Fsk::RegRxBw::HZ_15600,
            HZ_20800  = SX1278::Fsk::RegRxBw::HZ_20800,
            HZ_25000  = SX1278::Fsk::RegRxBw::HZ_25000,
            HZ_31300  = SX1278::Fsk::RegRxBw::HZ_31300,
            HZ_41700  = SX1278::Fsk::RegRxBw::HZ_41700,
            HZ_50000  = SX1278::Fsk::RegRxBw::HZ_50000,
            HZ_62500  = SX1278::Fsk::RegRxBw::HZ_62500,
            HZ_83300  = SX1278::Fsk::RegRxBw::HZ_83300,
            HZ_100000 = SX1278::Fsk::RegRxBw::HZ_100000,
            HZ_125000 = SX1278::Fsk::RegRxBw::HZ_125000,
            HZ_166700 = SX1278::Fsk::RegRxBw::HZ_166700,
            HZ_200000 = SX1278::Fsk::RegRxBw::HZ_200000,
            HZ_250000 = SX1278::Fsk::RegRxBw::HZ_250000,
        };

        /**
         * @brief Output modulation shaping.
         */
        enum class Shaping
        {
            NONE = SX1278::Fsk::RegPaRamp::MODULATION_SHAPING_NONE,
            GAUSSIAN_BT_1_0 =
                SX1278::Fsk::RegPaRamp::MODULATION_SHAPING_GAUSSIAN_BT_1_0,
            GAUSSIAN_BT_0_5 =
                SX1278::Fsk::RegPaRamp::MODULATION_SHAPING_GAUSSIAN_BT_0_5,
            GAUSSIAN_BT_0_3 =
                SX1278::Fsk::RegPaRamp::MODULATION_SHAPING_GAUSSIAN_BT_0_3,
        };

        /**
         * @brief Dc free encoding.
         *
         * @warning Setting this to NONE heightens the probability of long
         * sequences of 0s, reducing reliability.
         *
         * @warning MANCHESTER might be slightly more reliable, but halves the
         * bitrate.
         */
        enum class DcFree
        {
            NONE       = SX1278::Fsk::RegPacketConfig1::DC_FREE_NONE,
            MANCHESTER = SX1278::Fsk::RegPacketConfig1::DC_FREE_MANCHESTER,
            WHITENING  = SX1278::Fsk::RegPacketConfig1::DC_FREE_WHITENING
        };

        int freq_rf  = 434000000;        //< RF Frequency in Hz.
        int freq_dev = 50000;            //< Frequency deviation in Hz.
        int bitrate  = 48000;            //< Bitrate in b/s.
        RxBw rx_bw   = RxBw::HZ_125000;  //< Rx filter bandwidth.
        RxBw afc_bw  = RxBw::HZ_125000;  //< Afc filter bandwidth.
        int ocp =
            120;  //< Over current protection limit in mA (0 for no limit).
        int power = 13;  //< Output power in dB (Between +2 and +17, or +20 for
                         // full power).
        Shaping shaping = Shaping::GAUSSIAN_BT_1_0;  //< Shaping applied to
                                                     // the modulation stream.
        DcFree dc_free  = DcFree::WHITENING;  //< Dc free encoding scheme.
        bool enable_crc = true;  //< Enable hardware CRC calculation/checking
    };

    /**
     * @brief Error enum.
     */
    enum class Error
    {
        NONE,         //< No error encountered.
        BAD_VALUE,    //< A requested value was outside the valid range.
        BAD_VERSION,  //< Chip isn't connected.
        IRQ_TIMEOUT,  //< Timeout on IRQ register.
    };

    /**
     * @brief Construct a new SX1278
     */
    explicit SX1278Fsk(SPIBus &bus, miosix::GpioPin cs, miosix::GpioPin dio0,
                       miosix::GpioPin dio1, miosix::GpioPin dio3,
                       SPI::ClockDivider clock_divider,
                       std::unique_ptr<SX1278::ISX1278Frontend> frontend)
        : SX1278Common(bus, cs, dio0, dio1, dio3, clock_divider,
                       std::move(frontend)),
          crc_enabled(false)
    {
    }

    /**
     * @brief Setup the device.
     */
    [[nodiscard]] virtual Error init(const Config &config);

    /*
     * @brief Check if this device is connected.
     */
    bool checkVersion();

    /**
     * @brief Configure this device on the fly.
     */
    [[nodiscard]] virtual Error configure(const Config &config);

    /**
     * @brief Wait until a new packet is received.
     *
     * @param pkt       Buffer to store the received packet into.
     * @param pkt_len   Maximum length of the received data.
     * @return          Size of the data received or -1 on failure
     */
    ssize_t receive(uint8_t *pkt, size_t max_len) override;

    /**
     * @brief Send a packet.
     * The function must block until the packet is sent (successfully or not)
     *
     * @param pkt       Pointer to the packet (needs to be at least pkt_len
     * bytes).
     * @param pkt_len   Length of the packet to be sent.
     * @return          True if the message was sent correctly.
     */
    bool send(uint8_t *pkt, size_t len) override;

    /**
     * @brief Get the current perceived RSSI in dBm.
     */
    float getCurRssi();

    /**
     * @brief Get the RSSI in dBm, during last packet receive.
     */
    float getLastRxRssi() override;

    /**
     * @brief Get the frequency error index in Hz, during last packet receive.
     */
    float getLastRxFei() override;

private:
    void enterFskMode();

    void rateLimitTx();

    IrqFlags getIrqFlags() override;
    void resetIrqFlags(IrqFlags flags) override;

    float getRssi();
    float getFei();

    void setMode(Mode mode) override;
    void setMapping(SX1278::DioMapping mapping) override;

    bool crc_enabled;
    long long last_tx  = 0;
    float last_rx_rssi = 0.0f;
    PrintLogger logger = Logging::getLogger("sx1278");
};

}  // namespace Boardcore
