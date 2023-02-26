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
    static constexpr size_t MTU = SX1278::Fsk::FIFO_LEN - 1;

    /**
     * @brief Channel filter bandwidth.
     */
    enum class RxBw
    {
        HZ_2600   = (0b10 << 3) | 7,
        HZ_3100   = (0b01 << 3) | 7,
        HZ_3900   = 7,
        HZ_5200   = (0b10 << 3) | 6,
        HZ_6300   = (0b01 << 3) | 6,
        HZ_7800   = 6,
        HZ_10400  = (0b10 << 3) | 5,
        HZ_12500  = (0b01 << 3) | 5,
        HZ_15600  = 5,
        HZ_20800  = (0b10 << 3) | 4,
        HZ_25000  = (0b01 << 3) | 4,
        HZ_31300  = 4,
        HZ_41700  = (0b10 << 3) | 3,
        HZ_50000  = (0b01 << 3) | 3,
        HZ_62500  = 3,
        HZ_83300  = (0b10 << 3) | 2,
        HZ_100000 = (0b01 << 3) | 2,
        HZ_125000 = 2,
        HZ_166700 = (0b10 << 3) | 1,
        HZ_200000 = (0b01 << 3) | 1,
        HZ_250000 = 1,
    };

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

    enum class DcFree
    {
        NONE       = SX1278::Fsk::RegPacketConfig1::DC_FREE_NONE,
        MANCHESTER = SX1278::Fsk::RegPacketConfig1::DC_FREE_MANCHESTER,
        WHITENING  = SX1278::Fsk::RegPacketConfig1::DC_FREE_WHITENING
    };

    /**
     * @brief Requested SX1278 configuration.
     */
    struct Config
    {
        int freq_rf  = 434000000;        //< RF Frequency in Hz.
        int freq_dev = 50000;            //< Frequency deviation in Hz.
        int bitrate  = 48000;            //< Bitrate in b/s.
        RxBw rx_bw   = RxBw::HZ_125000;  //< Rx filter bandwidth.
        RxBw afc_bw  = RxBw::HZ_125000;  //< Afc filter bandwidth.
        int ocp =
            120;  //< Over current protection limit in mA (0 for no limit).
        int power = 10;  //< Output power in dB (Between +2 and +17, or +20 for
                         // full power).
        bool pa_boost   = true;  //< Enable output on PA_BOOST.
        Shaping shaping = Shaping::GAUSSIAN_BT_1_0;  //< Shaping applied to
                                                     // the modulation stream.
        DcFree dc_free = DcFree::WHITENING;  //< Dc free encoding scheme.
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
     *
     * @param bus SPI bus used.
     * @param cs Chip select pin.
     */
    explicit SX1278Fsk(SPISlave slave) : SX1278Common(slave) {}

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
     * @brief Get the current perceived RSSI in dBm.
     */
    float getCurRssi();

    /**
     * @brief Get the RSSI in dBm, during last packet receive.
     */
    float getLastRxRssi();

    /**
     * @brief Get the frequency error index in Hz, during last packet receive.
     */
    float getLastRxFei();

    /**
     * @brief Dump all registers via TRACE.
     */
    void debugDumpRegisters();

protected:
    // Stuff to work with various front-ends
    virtual void enableRxFrontend() override {}
    virtual void disableRxFrontend() override {}
    virtual void enableTxFrontend() override {}
    virtual void disableTxFrontend() override {}

private:
    void rateLimitTx();

    void imageCalibrate();

    SX1278::DioMask getDioMaskFromIrqFlags(IrqFlags flags, Mode mode,
                                           SX1278::DioMapping mapping) override;

    IrqFlags getIrqFlags() override;
    void resetIrqFlags(IrqFlags flags) override;

    float getRssi();
    float getFei();

    void setMode(Mode mode) override;
    void setMapping(SX1278::DioMapping mapping) override;

    void setBitrate(int bitrate);
    void setFreqDev(int freq_dev);
    void setFreqRF(int freq_rf);
    void setOcp(int ocp);
    void setSyncWord(uint8_t value[], int size);
    void setRxBw(RxBw rx_bw);
    void setAfcBw(RxBw afc_bw);
    void setPreambleLen(int len);
    void setPa(int power, bool pa_boost);
    void setShaping(Shaping shaping);

    long long last_tx  = 0;
    float last_rx_rssi = 0.0f;
    PrintLogger logger = Logging::getLogger("sx1278");
};

}  // namespace Boardcore
