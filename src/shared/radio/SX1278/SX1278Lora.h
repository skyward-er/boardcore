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

#pragma once

#include "SX1278Common.h"
#include "SX1278LoraTimings.h"

namespace Boardcore
{

class SX1278Lora : public SX1278::SX1278Common
{
public:
    static constexpr size_t MTU = 255;

    /**
     * @brief Requested SX1278 configuration.
     */
    struct Config
    {
        /**
         * @brief Bandwidth of the device
         */
        enum class Bw
        {
            HZ_7800   = SX1278::Lora::RegModemConfig1::BW_HZ_7800,
            HZ_10400  = SX1278::Lora::RegModemConfig1::BW_HZ_10400,
            HZ_15600  = SX1278::Lora::RegModemConfig1::BW_HZ_15600,
            HZ_20800  = SX1278::Lora::RegModemConfig1::BW_HZ_20800,
            HZ_31250  = SX1278::Lora::RegModemConfig1::BW_HZ_31250,
            HZ_41700  = SX1278::Lora::RegModemConfig1::BW_HZ_41700,
            HZ_62500  = SX1278::Lora::RegModemConfig1::BW_HZ_62500,
            HZ_125000 = SX1278::Lora::RegModemConfig1::BW_HZ_125000,
            HZ_250000 = SX1278::Lora::RegModemConfig1::BW_HZ_250000,
            HZ_500000 = SX1278::Lora::RegModemConfig1::BW_HZ_500000,
        };

        /**
         * @brief Coding rate of the device.
         *
         * This defines how many bits in the datastream are error correction
         * bits.
         */
        enum class Cr
        {
            CR_1 = SX1278::Lora::RegModemConfig1::CR_1,  //< +25% error
                                                         // correction overhead.
            CR_2 = SX1278::Lora::RegModemConfig1::CR_2,  //< +50% error
                                                         // correction overhead.
            CR_3 = SX1278::Lora::RegModemConfig1::CR_3,  //< +75% error
                                                         // correction overhead.
            CR_4 = SX1278::Lora::RegModemConfig1::CR_4,  //< +100% error
                                                         // correction overhead.
        };

        /**
         * @brief Spreading factor of the devide.
         *
         * This defines how long a chirp really is, every increment doubles the
         * duration of the chirp, while adding one extra bit.
         */
        enum class Sf
        {
            // Spreading factor 6 is special, and not supported by this driver
            // SF_6  = SX1278::Lora::RegModemConfig2::SF_6,
            SF_7  = SX1278::Lora::RegModemConfig2::SF_7,
            SF_8  = SX1278::Lora::RegModemConfig2::SF_8,
            SF_9  = SX1278::Lora::RegModemConfig2::SF_9,
            SF_10 = SX1278::Lora::RegModemConfig2::SF_10,
            SF_11 = SX1278::Lora::RegModemConfig2::SF_11,
            SF_12 = SX1278::Lora::RegModemConfig2::SF_12,
        };

        Bw bandwidth        = Bw::HZ_125000;
        Cr coding_rate      = Cr::CR_1;
        Sf spreading_factor = Sf::SF_7;

        bool low_data_rate_optimize =
            false;  //< Optimize the transmission at high spreading factors.

        int freq_rf = 434000000;  //< RF frequency in Hz.
        int ocp =
            120;  //< Over current protection limit in mA (0 for no limit).
        int power =
            13;  //< Output power in dB (between +2 and +17 with pa_boost = on,
                 // and between +0 and +14 with pa_boost = off, +20 for +20dBm
                 // max power ignoring pa_boost).

        /**
         * @brief Calculates effective and usable bitrate.
         */
        uint32_t effectiveBitrate() const
        {
            using namespace SX1278::Lora;

            return SX1278::Lora::effectiveBitrate(
                static_cast<RegModemConfig2::Sf>(spreading_factor),
                RegModemConfig1::bandwidthToInt(
                    static_cast<RegModemConfig1::Bw>(bandwidth)),
                static_cast<RegModemConfig1::Cr>(coding_rate));
        }
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
    explicit SX1278Lora(SPIBus &bus, miosix::GpioPin cs,
                        SPI::ClockDivider clock_divider,
                        std::unique_ptr<SX1278::ISX1278Frontend> frontend)
        : SX1278Common(bus, cs, clock_divider, std::move(frontend))
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
     * @return          Size of the data received or -1 if failure
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
     * @brief Get the RSSI in dBm, during last packet receive.
     */
    float getLastRxRssi();

    /**
     * @brief Get the RSSI in dBm, during last packet receive.
     */
    float getLastRxSnr();

private:
    void readFifo(uint8_t addr, uint8_t *dst, uint8_t size);
    void writeFifo(uint8_t addr, uint8_t *src, uint8_t size);

    SX1278::DioMask getDioMaskFromIrqFlags(IrqFlags flags, Mode mode,
                                           SX1278::DioMapping mapping) override;

    IrqFlags getIrqFlags() override;
    void resetIrqFlags(IrqFlags flags) override;

    void setMode(Mode mode) override;
    void setMapping(SX1278::DioMapping mapping) override;

    void setFreqRF(int freq_rf);
};

}  // namespace Boardcore
