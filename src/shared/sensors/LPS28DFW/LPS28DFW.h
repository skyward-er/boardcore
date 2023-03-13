/* Copyright (c) 2023 Skyward Experimental Rocketry
 * Author: Emilio Corigliano
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

#include <drivers/i2c/I2C.h>
#include <sensors/Sensor.h>

#include "LPS28DFWData.h"

namespace Boardcore
{

class LPS28DFW : public Sensor<LPS28DFWData>
{
public:
    enum FullScaleRange : uint8_t
    {
        FS_1260,
        FS_4060
    };

    enum ODR : uint8_t
    {
        ONE_SHOT = (0b0000 << 3),
        ODR_1    = (0b0001 << 3),
        ODR_4    = (0b0010 << 3),
        ODR_10   = (0b0011 << 3),
        ODR_25   = (0b0100 << 3),
        ODR_50   = (0b0101 << 3),
        ODR_75   = (0b0110 << 3),
        ODR_100  = (0b0111 << 3),
        ODR_200  = (0b1000 << 3)
    };

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

    enum Mode
    {
        ONE_SHOT_MODE,    // BYPASS, ODR = ONE_SHOT
        CONTINUOUS_MODE,  // BYPASS, ODR = odr
        FIFO_MODE         // FIFO, ODR = odr
    };

    typedef struct
    {
        FullScaleRange fsr;
        AVG avg;
        Mode mode;
        ODR odr;
        bool drdy;        // Enable Interrupt for Data Ready
        size_t fifoSize;  // If fifo, length of fifo
    } SensorConfig;

    LPS28DFW(I2C& i2c, bool sa0, SensorConfig sensorConfig);

    bool init() override;

    bool setConfig(const SensorConfig& config);

    bool selfTest() override;

    bool setAverage(AVG avg);

    bool setOutputDataRate(ODR odr);

    bool setFullScaleRange(FullScaleRange fs);

private:
    LPS28DFWData sampleImpl() override;

    I2C& i2c;
    I2CDriver::I2CSlaveConfig i2cConfig;
    SensorConfig sensorConfig;
    bool isInitialized = false;
    uint16_t pressureSensitivity;
    uint16_t temperatureSensitivity = 100;  // [LSB/°C]
    PrintLogger logger              = Logging::getLogger("lps28dfw");
};

}  // namespace Boardcore