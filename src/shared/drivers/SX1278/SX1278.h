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
#include <kernel/kernel.h>

#include <cstdint>

#include "SX1278Defs.h"

namespace Boardcore
{

/**
 * @brief Various SX1278 register/enums definitions.
 */
class SX1278
{
public:
    /**
     * @brief Channel filter bandwidth.
     */
    enum class RxBw
    {
        HZ_2600   = (0b10 << 3) | 7,
        HZ_3100   = (0b01 << 3) | 7,
        HZ_3900   = (0b00 << 3) | 7,
        HZ_5200   = (0b10 << 3) | 6,
        HZ_6300   = (0b01 << 3) | 6,
        HZ_7800   = (0b00 << 3) | 6,
        HZ_10400  = (0b10 << 3) | 5,
        HZ_12500  = (0b01 << 3) | 5,
        HZ_15600  = (0b00 << 3) | 5,
        HZ_20800  = (0b10 << 3) | 4,
        HZ_25000  = (0b01 << 3) | 4,
        HZ_31300  = (0b00 << 3) | 4,
        HZ_41700  = (0b10 << 3) | 3,
        HZ_50000  = (0b01 << 3) | 3,
        HZ_62500  = (0b00 << 3) | 3,
        HZ_83300  = (0b10 << 3) | 2,
        HZ_100000 = (0b01 << 3) | 2,
        HZ_125000 = (0b00 << 3) | 2,
        HZ_166700 = (0b10 << 3) | 1,
        HZ_200000 = (0b01 << 3) | 1,
        HZ_250000 = (0b00 << 3) | 1,
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
        bool enable_int = false;  //< Enable interrupt pin.
        int power = 10;  //< Output power in dB (Between +2 and +17, or +20 for
                         // full power).
    };

    /**
     * @brief Error enum.
     */
    enum class Error
    {
        NONE,         //< No error encountered.
        BAD_VALUE,    //< A requested value was outside the valid range.
        BAD_VERSION,  //< Chip didn't report the correct version.
    };

    /**
     * @brief Construct a new SX1278
     *
     * @param bus SPI bus used.
     * @param cs Chip select pin.
     */
    SX1278(SPIBusInterface &bus, miosix::GpioPin cs);

    /**
     * @brief Setup the device.
     */
    Error init(Config config);

    /**
     * @brief Read the chip revision.
     *
     * @return It should always return 0x12.
     */
    uint8_t getVersion() const;

    /**
     * @brief Receive data.
     *
     * @param buf Buffer to put the received data.
     * @param max_len How big is the supplied buffer.
     * @return How many bytes were received (or -1 if the supplied buffer was
     * too small).
     */
    int recv(uint8_t *buf, size_t max_len);

    /**
     * @brief Send data.
     *
     * @param buf Buffer with the data to send.
     * @param len Length of buffer.
     */
    void send(const uint8_t *buf, uint8_t len);

    /**
     * @brief Handle an incoming interrupt.
     */
    void handleDioIRQ();

    /**
     * @brief Dump all registers via TRACE.
     */
    void debugDumpRegisters();

public:
    using Mode = SX1278Defs::RegOpMode::Mode;

    void setBitrate(int bitrate);
    void setFreqDev(int freq_dev);
    void setFreqRF(int freq_rf);
    void setOcp(int ocp);
    void setSyncWord(uint8_t value[], int size);
    void setRxBw(RxBw rx_bw);
    void setAfcBw(RxBw afc_bw);
    void setPreableLen(int len);
    void setPa(int power, bool pa_boost);

    void enterMode(Mode mode);
    void waitForIrq(uint8_t reg, uint8_t mask);
    void waitForIrq1(uint8_t mask);
    void waitForIrq2(uint8_t mask);

    miosix::Thread *irq_wait_thread = nullptr;
    bool enable_int                 = false;

    SPISlave slave;
    Mode mode;

    PrintLogger logger = Logging::getLogger("bmx160");
};

}  // namespace Boardcore
