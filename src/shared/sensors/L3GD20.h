/**
 * Copyright (c) 2019 Skyward Experimental Rocketry
 * Authors: Luca Erbetta
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
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.  IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 */

#pragma once
#include <miosix.h>

#include "Sensor.h"
#include "drivers/spi/SPIDriver.h"

using miosix::GpioPin;

class L3GD20 : public GyroSensor
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
        ODR_95  = 0x00,
        ODR_190 = 0x01,
        ODR_380 = 0x02,
        ODR_760 = 0x03
    };

    L3GD20(SPIBusInterface& bus, GpioPin cs,
           FullScaleRange range = FullScaleRange::FS_250,
           OutPutDataRate odr   = OutPutDataRate::ODR_95,
           uint8_t cutoff_freq = 0x03, bool fifo_enabled = false)
        : fifo_enabled(fifo_enabled), spislave(bus, cs), fs(range), odr(odr),
          cutoff_freq(cutoff_freq)
    {
        spislave.config.br = SPIBaudRate::DIV_128;
        // memset(last_fifo, 0, sizeof(Vec3) * 32);
    }

    bool init()
    {
        SPITransaction spi(spislave);

        uint8_t whoami = spi.read(REG_WHO_AM_I);

        if (whoami != WHO_AM_I_VAL)
        {
            printf("WAMI: %d\n", whoami);
            last_error = ERR_NOT_ME;
            return false;
        }

        uint8_t ctrl4 = spi.read(REG_CTRL4);

        switch (fs)
        {
            case FullScaleRange::FS_500:
                spi.write(REG_CTRL4, ctrl4 | (uint8_t)(1 << 4));
                break;
            case FullScaleRange::FS_2000:
                spi.write(REG_CTRL4, ctrl4 | (uint8_t)(2 << 4));
                break;
            default:
                break;
        }
        if (fifo_enabled)
        {
            // Enable fifo
            spi.write(REG_CTRL5, 1 << 6);

            // Set watermark level to 24 samples
            uint8_t fifo_ctrl = 24;

            // Set fifo to stream mode
            fifo_ctrl |= 0x02 << 5;

            spi.write(REG_FIFO_CTRL, fifo_ctrl);
        }

        // Enter normal mode, enable output
        uint8_t ctrl1 = 0x0F;

        // // Configure ODR
        ctrl1 |= static_cast<uint8_t>(odr) << 6;

        // Configure cutoff frequency
        ctrl1 |= (cutoff_freq & 0x02) << 4;

        spi.write(REG_CTRL1, ctrl1);

        return true;
    }

    bool selfTest() { return true; }

    bool onSimpleUpdate()
    {
        float scale = static_cast<int>(fs);

        if (!fifo_enabled)
        {
            uint8_t data[6];
            // High level access to the bus
            {
                SPITransaction spi(spislave);
                spi.read(REG_OUT_X_L | 0x40, data, 6);
            }

            int16_t x = data[0] | data[1] << 8;
            int16_t y = data[2] | data[3] << 8;
            int16_t z = data[4] | data[5] << 8;
            // printf("%02X,%02X,%02X\n", x, y, z);

            mLastGyro =
                Vec3(x * scale / 65535, y * scale / 65535, z * scale / 65535);
        }
        else  // FIFO is enabled
        {
            uint8_t buf[192];

            SPITransaction spi(spislave);
            // Read last fifo level
            uint8_t fifo_src = spi.read(REG_FIFO_SRC);
            uint8_t ovr      = (fifo_src & 0x40);
            last_fifo_level  = fifo_src & 0x1F;
            if (ovr > 0)
            {
                ++last_fifo_level;
            }

            // Read last fifo level
            spi.read(REG_OUT_X_L | 0x40, buf, 32 * 6);

            // Read samples from the FIFO
            for (uint8_t i = 0; i < last_fifo_level; i++)
            {
                int16_t x = buf[i * 6] | buf[i * 6 + 1] << 8;
                int16_t y = buf[i * 6 + 2] | buf[i * 6 + 3] << 8;
                int16_t z = buf[i * 6 + 4] | buf[i * 6 + 5] << 8;

                Vec3 t = Vec3(x * scale / 65535, y * scale / 65535,
                              z * scale / 65535);

                last_fifo[i] = t;
            }
        }

        return true;
    }

    Vec3* getLastFifo() { return last_fifo; }
    uint8_t getLastFifoSize() { return last_fifo_level; }

private:
    bool fifo_enabled = false;

    Vec3 last_fifo[32];
    uint8_t last_fifo_level = 0;

    SPISlave spislave;
    FullScaleRange fs;
    OutPutDataRate odr;
    uint8_t cutoff_freq;

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