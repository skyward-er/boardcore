
/* Copyright (c) 2019 Skyward Experimental Rocketry
 * Author: Luca Erbetta
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

#include <drivers/spi/SPIDriver.h>
#include <drivers/timer/TimestampTimer.h>
#include <miosix.h>
#include <sensors/Sensor.h>

#include <Eigen/Core>
#include <array>

#include "L3GD20Data.h"

namespace Boardcore
{

static constexpr uint32_t L3GD20_FIFO_SIZE = 32;

class L3GD20 : public SensorFIFO<L3GD20Data, L3GD20_FIFO_SIZE>
{
public:
    enum class FullScaleRange
    {
        FS_250  = 250,
        FS_500  = 500,
        FS_2000 = 2000
    };

    enum class OutPutDataRate
    {
        ODR_95  = 95,
        ODR_190 = 190,
        ODR_380 = 380,
        ODR_760 = 760
    };

    /**
     * @brief Creates an instance of an L3GD20 sensor
     *
     * @param    bus SPI bus the sensor is connected to
     * @param    cs Chip Select GPIO
     * @param    range Full Scale Range (See datasheet)
     * @param    odr Output Data Rate (See datasheet)
     * @param    cutoffFreq Low pass filter cutoff frequency (See datasheet)
     */
    L3GD20(SPIBusInterface& bus, miosix::GpioPin cs,
           FullScaleRange range = FullScaleRange::FS_250,
           OutPutDataRate odr   = OutPutDataRate::ODR_95,
           uint8_t cutoffFreq   = 0x03)
        : L3GD20(bus, cs, {}, range, odr, cutoffFreq)
    {
        // Configure SPI
        spislave.config.clockDivider = SPI::ClockDivider::DIV_32;
    }

    /**
     * @brief Creates an instance of an L3GD20 sensor
     *
     * @param    bus SPI bus the sensor is connected to
     * @param    cs Chip Select GPIO
     * @param    cfg Custom SPI bus configuration
     * @param    range Full Scale Range (See datasheet)
     * @param    odr Output Data Rate (See datasheet)
     * @param    cutoffFreq Low pass filter cutoff selector (See datasheet)
     */
    L3GD20(SPIBusInterface& bus, miosix::GpioPin cs, SPIBusConfig cfg,
           FullScaleRange range = FullScaleRange::FS_250,
           OutPutDataRate odr   = OutPutDataRate::ODR_95,
           uint8_t cutoffFreq   = 0x03)
        : spislave(bus, cs, cfg), fs(range), odr(odr), cutoffFreq(cutoffFreq)
    {
        switch (fs)
        {
            case FullScaleRange::FS_250:
                sensitivity = SENSITIVITY_250;
                break;
            case FullScaleRange::FS_500:
                sensitivity = SENSITIVITY_500;
                break;
            case FullScaleRange::FS_2000:
                sensitivity = SENSITIVITY_2000;
                break;
        }
    }

    /**
     * @brief Enables storing samples in a FIFO, must be called before init().
     *
     * @param fifoWatermark How many samples in the FIFO when the fifo
     * watermark input is generated on INT2.
     */
    void enableFifo(unsigned int fifoWatermark)
    {
        fifoEnabled         = true;
        this->fifoWatermark = fifoWatermark;
    }

    bool init()
    {
        SPITransaction spi(spislave);

        uint8_t whoami = spi.readRegister(REG_WHO_AM_I);

        if (whoami != WHO_AM_I_VAL)
        {
            lastError = SensorErrors::INVALID_WHOAMI;
            return false;
        }

        switch (fs)
        {
            case FullScaleRange::FS_250:
                spi.writeRegister(REG_CTRL4, 0);
                break;
            case FullScaleRange::FS_500:
                spi.writeRegister(REG_CTRL4, (uint8_t)(1 << 4));
                break;
            case FullScaleRange::FS_2000:
                spi.writeRegister(REG_CTRL4, (uint8_t)(2 << 4));
                break;
            default:
                break;
        }

        if (fifoEnabled)
        {
            // Enable fifo
            spi.writeRegister(REG_CTRL5, 1 << 6);

            // Set watermark level to fifoWatermark samples
            uint8_t fifoCtrl = fifoWatermark;

            // Set fifo to STREAM mode
            fifoCtrl |= 0x02 << 5;

            spi.writeRegister(REG_FIFO_CTRL, fifoCtrl);

            // Enable FIFO watermark interrupt on INT2
            spi.writeRegister(REG_CTRL3, 0x04);
        }
        else
        {
            // Enable DRDY interrupt on INT2
            spi.writeRegister(REG_CTRL3, 0x08);
        }

        // Enter normal mode, enable output
        uint8_t ctrl1 = 0x0F;

        // Configure cutoff frequency
        ctrl1 |= (cutoffFreq & 0x03) << 4;

        // Configure ODR
        switch (odr)
        {
            case OutPutDataRate::ODR_95:
                break;
            case OutPutDataRate::ODR_190:
                ctrl1 |= 1 << 6;
                break;
            case OutPutDataRate::ODR_380:
                ctrl1 |= 2 << 6;
                break;
            case OutPutDataRate::ODR_760:
                ctrl1 |= 3 << 6;
                break;
        }

        spi.writeRegister(REG_CTRL1, ctrl1);

        return true;
    }

    bool selfTest() { return true; }

    L3GD20Data sampleImpl()
    {
        using namespace Units::Angle;

        if (!fifoEnabled)  // FIFO not enabled
        {
            // Timestamp of the last sample
            uint64_t lastSampleTimestamp;

            // Read output data registers (X, Y, Z)
            {
                SPITransaction spi(spislave);
                spi.readRegisters(REG_OUT_X_L | 0x40, buf, 6);
            }

            {
                // Disable interrupts and copy the last sample locally, as a new
                // interrupt may come just as we are reading it and causing a
                // race condition.
                miosix::FastInterruptDisableLock dLock;
                lastSampleTimestamp = lastInterruptTimestamp;
            }

            int16_t x = buf[0] | buf[1] << 8;
            int16_t y = buf[2] | buf[3] << 8;
            int16_t z = buf[4] | buf[5] << 8;

            Eigen::Vector3f rads = toRadiansPerSecond(x, y, z);
            lastFifo[0]          = {lastSampleTimestamp,
                                    angle_cast<Degree>(Radian(rads(0))),
                                    angle_cast<Degree>(Radian(rads(1))),
                                    angle_cast<Degree>(Radian(rads(2)))};
        }

        else  // FIFO is enabled
        {
            SPITransaction spi(spislave);
            // Read last fifo level
            uint8_t fifoSrc   = spi.readRegister(REG_FIFO_SRC);
            uint8_t ovr       = (fifoSrc & 0x40) >> 7;  // Overrun bit
            uint8_t fifoLevel = (fifoSrc & 0x1F) + ovr;

            // Read fifo
            spi.readRegisters(REG_OUT_X_L | 0x40, buf, fifoLevel * 6);

            uint64_t dt = interruptTimestampDelta / lastFifoLevel;

            uint8_t duplicates = 0;
            for (uint8_t i = 0; i < fifoLevel; ++i)
            {
                bool rmv;
                // Check for duplicates: there seems to be a bug where the
                // sensor occasionally shifts out the same sample two times in a
                // row: discard one
                if (i < fifoLevel - 1)
                {
                    rmv = true;
                    for (uint8_t j = 0; j < 6; ++j)
                    {
                        if (buf[i * 6 + j] != buf[(i + 1) * 6 + j])
                        {
                            rmv = false;
                            break;
                        }
                    }
                }
                else
                {
                    rmv = false;
                }

                if (rmv)
                {
                    // Skip this sample;
                    ++duplicates;
                    continue;
                }

                // Assign a timestamp to each sample in the FIFO based of the
                // timestamp of the FIFO_WATERMARKth sample and the delta
                // between the watermark interrupts (evenly distribute the
                // timestamp between the two times)
                // Ignoring possible duplicates, the timestamp of the ith sample
                // in the fifo is:
                // ts(i) = ts(fifoWatermark) + (i - fifoWatermark)*dt;
                Eigen::Vector3f rads =
                    toRadiansPerSecond(buf[i * 6] | buf[i * 6 + 1] << 8,
                                       buf[i * 6 + 2] | buf[i * 6 + 3] << 8,
                                       buf[i * 6 + 4] | buf[i * 6 + 5] << 8);

                lastFifo[i - duplicates] = L3GD20Data{
                    lastInterruptTimestamp +
                        ((int)i - (int)fifoWatermark - (int)duplicates) * dt,
                    angle_cast<Degree>(Radian(rads(0))),
                    angle_cast<Degree>(Radian(rads(1))),
                    angle_cast<Degree>(Radian(rads(2)))};
            }

            lastFifoLevel = fifoLevel - duplicates;
        }

        return lastFifo[lastFifoLevel - 1];
    }

private:
    Eigen::Vector3f toRadiansPerSecond(int16_t x, int16_t y, int16_t z)
    {
        return Eigen::Vector3f(x * sensitivity, y * sensitivity,
                               z * sensitivity);
    }

    static constexpr float SENSITIVITY_250  = 0.00875f;
    static constexpr float SENSITIVITY_500  = 0.0175f;
    static constexpr float SENSITIVITY_2000 = 0.070f;

    SPISlave spislave;

    FullScaleRange fs  = FullScaleRange::FS_250;
    OutPutDataRate odr = OutPutDataRate::ODR_760;
    uint8_t cutoffFreq = 0x03;

    bool fifoEnabled           = false;
    unsigned int fifoWatermark = 24;

    float sensitivity = SENSITIVITY_250;

    uint8_t buf[192];

    constexpr static uint8_t WHO_AM_I_VAL = 212;

    enum RegMap
    {
        REG_WHO_AM_I = 0x0F,

        REG_CTRL1 = 0x20,
        REG_CTRL2 = 0x21,
        REG_CTRL3 = 0x22,
        REG_CTRL4 = 0x23,
        REG_CTRL5 = 0x24,

        REG_REFERENCE = 0x25,
        REG_OUT_TEMP  = 0x26,
        REG_STATUS    = 0x27,

        REG_OUT_X_L = 0x28,
        REG_OUT_X_H = 0x29,

        REG_OUT_Y_L = 0x2A,
        REG_OUT_Y_H = 0x2B,

        REG_OUT_Z_L = 0x2C,
        REG_OUT_Z_H = 0x2D,

        REG_FIFO_CTRL = 0x2E,
        REG_FIFO_SRC  = 0x2F
    };
};

}  // namespace Boardcore
