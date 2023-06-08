/* Copyright (c) 2023 Skyward Experimental Rocketry
 * Author: Giulia Ghirardini
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

#include <diagnostic/PrintLogger.h>
#include <drivers/spi/SPIDriver.h>
#include <drivers/timer/TimestampTimer.h>
#include <miosix.h>
#include <sensors/Sensor.h>

#include <Eigen/Core>
#include <array>

#include "LPS22DFData.h"

namespace Boardcore
{

/**
 * Driver for LPS22DF, Low-power and high-precision MEMS nano pressure sensor.
 */

static constexpr uint32_t LPS22DF_FIFO_SIZE = 128;

class LPS22DF : public SensorFIFO<LPS22DFData, LPS22DF_FIFO_SIZE>
{
public:
    /**
     * @brief Constants for Output Data Rate Configuration
     *
     * Notes:
     * 1. ODR configuration is valid for both pressure and temperature. When
     *    the ODR bits are set to a value different than '0000' the device is in
     *    continuous mode, otherwise it is set to be in Power-down mode or
     * One-shot mode. The latter mode is only possible if the ONESHOT bit in
     * CTRL-REG2 is set to 1.
     * 2. ODR_200_HZ is set to '1xxx' which means it doesn't matter if the
     * x-bits are setted to 0 or to 1
     */
    enum ODR : uint8_t
    {
        ODR_PWR_DWN = 0x00,  //!<  power-down / one-shot
        ODR_1_HZ    = 0x01,  //!<   1 Hz
        ODR_4_HZ    = 0x02,  //!<   4 Hz
        ODR_10_HZ   = 0x03,  //!<  10 Hz
        ODR_25_HZ   = 0x04,  //!<  25 Hz
        ODR_50_HZ   = 0x05,  //!<  50 Hz
        ODR_75_HZ   = 0x06,  //!<  75 Hz
        ODR_100_HZ  = 0x07,  //!< 100 Hz
        ODR_200_HZ  = 0x08,  //!< 200 Hz
    };

    /**
     * @brief FIFO Modes
     *
     * Note: The TRIG_MODES bit enables the triggered FIFO modes.
     * 0 - 00 - Bypass
     * 0 - 01 - FIFO mode
     * 0 - 1x - Continuous (Dynamic-Stream)
     * 1 - 01 - Bypass-to-FIFO
     * 1 - 01 - Bypass-to-Continuous (Dynamic-Stream)
     * 1 - 01 - Continuous (Dynamic-Stream)-to-FIFO
     */
    enum FIFOMode : uint8_t
    {
        FIFO_MODE0 = 0x0,
        FIFO_MODE1 = 0x1,
        FIFO_MODE2 = 0x2,
        FIFO_MODE3 = 0x3,
    };

    /**
     * @brief Sensor configuration
     *
     * This struct contains all the settings the user
     * is able to modify with the help of the driver.
     * They are applied in the constructor of LPS22DF class
     * and on each call of LPS22DF::applyConfig()
     */
    struct Config
    {
        Config() {}
        /**
         * @brief Data rate configuration
         *
         * Default: Power-down / one-shot
         *
         * @see LPS22DF::ODR
         */
        ODR odr = ODR_PWR_DWN;

        bool enableTemperature = true;

        unsigned temperatureDivider = 1;

        bool enableInterrupt[3] = {false, false, false};

        float threshold = 0;

        bool doBlockDataUpdate = false;
    };

    LPS22DF(SPIBusInterface& bus, miosix::GpioPin pin,
            SPIBusConfig spiConfig = {}, Config config = {});

    bool init() override;

    bool selfTest() override;

    bool applyConfig(Config config);

private:
    LPS22DFData sampleImpl() override;

    SPISlave mSlave;
    Config mConfig;

    unsigned currDiv;
    bool isInitialized;
    float mUnit = 0;

    enum Registers : uint8_t
    {
        WHO_AM_I = 0x0f,

        CTRL_REG1 = 0x20,
        CTRL_REG2 = 0x21,
        CTRL_REG3 = 0x22,
        CTRL_REG4 = 0x23,

        FIFO_CTRL = 0x14,
        FIFO_WTM  = 0x15,

        REF_P_L = 0x16,
        REF_P_H = 0x17,

        FIFO_STATUS1 = 0x25,
        FIFO_STATUS2 = 0x26,
        STATUS       = 0x27,

        PRESSURE_OUT_XL = 0x28,
        PRESSURE_OUT_L  = 0x29,
        PRESSURE_OUT_H  = 0x2a,
        TEMP_OUT_L      = 0x2b,
        TEMP_OUT_H      = 0x2c,

        FIFO_DATA_OUT_PRESS_XL = 0x78,
        FIFO_DATA_OUT_PRESS_L  = 0x79,
        FIFO_DATA_OUT_PRESS_H  = 0x7a,
    };

    enum Constants : unsigned
    {
        WHO_AM_I_VALUE = 0xb4,

        REFERENCE_TEMPERATURE = 25,
        LSB_PER_CELSIUS       = 100,
        LSB_PER_HPA           = 4096,
    };

    PrintLogger logger = Logging::getLogger("lps22df");
};

}  // namespace Boardcore