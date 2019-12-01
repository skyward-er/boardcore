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

    L3GD20(SPIBusInterface& bus, GpioPin cs,
           FullScaleRange range = FullScaleRange::FS_250)
        : spislave(bus, cs), fs(range)
    {
        spislave.config.br = SPIBaudRate::DIV_128;
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

        uint8_t ctrl1 = 0x0F;
        spi.write(REG_CTRL1, ctrl1);

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

        return true;
    }

    bool selfTest() { return true; }

    bool onSimpleUpdate()
    {
        uint8_t data[6];

        SPITransaction spi(spislave);
        spi.read(REG_OUT_X_L | 0x40, data, 6);

        int16_t x = data[0] | data[1] << 8;
        int16_t y = data[2] | data[3] << 8;
        int16_t z = data[4] | data[5] << 8;

        float scale = static_cast<int>(fs);
        mLastGyro =
            Vec3(x * scale / 65535, y * scale / 65535, z * scale / 65535);
        return true;
    }

private:
    SPISlave spislave;
    FullScaleRange fs;
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
        REG_OUT_Z_H = 0x2D
    };
};