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
 * @brief Driver for LPS22DF, Low-power and high-precision MEMS pressure sensor.
 */
class LPS22DF : public Sensor<LPS22DFData>
{
public:
    /**
     * @brief Constants for Output Data Rate Configuration
     *
     * ODR configuration is valid for both pressure and temperature.
     */
    enum ODR : uint8_t
    {
        ONE_SHOT = 0b0000 << 3,
        ODR_1    = 0b0001 << 3,
        ODR_4    = 0b0010 << 3,
        ODR_10   = 0b0011 << 3,
        ODR_25   = 0b0100 << 3,
        ODR_50   = 0b0101 << 3,
        ODR_75   = 0b0110 << 3,
        ODR_100  = 0b0111 << 3,
        ODR_200  = 0b1000 << 3
    };

    /**
     * @brief Averaging of pressure and temperature.
     */
    enum AVG : uint8_t
    {
        AVG_4   = 0b000,
        AVG_8   = 0b001,
        AVG_16  = 0b010,
        AVG_32  = 0b011,
        AVG_64  = 0b100,
        AVG_128 = 0b101,
        AVG_512 = 0b111
    };

    typedef struct
    {
        ODR odr = ODR::ODR_25;
        AVG avg = AVG::AVG_512;
    } Config;

    LPS22DF(SPIBusInterface& bus, miosix::GpioPin pin);

    LPS22DF(SPIBusInterface& bus, miosix::GpioPin pin, SPIBusConfig spiConfig,
            Config config);

    static SPIBusConfig getDefaultSPIConfig();

    bool init() override;

    bool selfTest() override;

    void setConfig(const Config& config);

    void setAverage(AVG avg);

    void setOutputDataRate(ODR odr);

private:
    LPS22DFData sampleImpl() override;

    SPISlave slave;
    Config config;

    bool isInitialized = false;

    enum Registers : uint8_t
    {
        IF_CTRL_REG = 0x0e,

        WHO_AM_I_REG = 0x0f,

        CTRL1_REG = 0x10,
        CTRL2_REG = 0x11,
        CTRL3_REG = 0x12,
        CTRL4_REG = 0x13,

        STATUS_REG = 0x27,

        PRESSURE_OUT_XL_REG = 0x28,
        PRESSURE_OUT_L_REG  = 0x29,
        PRESSURE_OUT_H_REG  = 0x2a,
        TEMP_OUT_L_REG      = 0x2b,
        TEMP_OUT_H_REG      = 0x2c,
    };

    static constexpr uint32_t WHO_AM_I_VALUE  = 0xb4;
    static constexpr uint8_t I2C_I3C_DIS      = 1 << 6;
    static constexpr uint8_t ONE_SHOT_TRIGGER = 1 << 0;

    static constexpr float TEMP_SENS = 100;    // LSB / °C
    static constexpr float PRES_SENS = 40.96;  // LSB / Pa

    PrintLogger logger = Logging::getLogger("lps22df");
};

}  // namespace Boardcore