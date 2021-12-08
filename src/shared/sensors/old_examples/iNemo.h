/* Copyright (c) 2016-2017 Skyward Experimental Rocketry
 * Authors: Matteo Piazzolla, Alain Carlucci
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

#include <drivers/BusTemplate.h>

#include "Common.h"
#include "Sensor.h"

template <typename BusG, typename BusXM>
class iNEMOLSM9DS0 : public GyroSensor,
                     public AccelSensor,
                     public CompassSensor,
                     public TemperatureSensor
{

public:
    iNEMOLSM9DS0(uint8_t accelFullScale, uint8_t gyroFullScale,
                 uint8_t compassFullScale)
    {
        accelFS   = accelFullScale & 0x07;
        gyroFS    = gyroFullScale & 0x03;
        compassFS = compassFullScale & 0x03;
        mLastTemp = 0.0f;
    }

    bool init()
    {
        uint8_t whoami_g  = BusG::read(RegMap::WHO_AM_I_G);
        uint8_t whoami_xm = BusXM::read(RegMap::WHO_AM_I_XM);

        if ((whoami_g != whoami_g_value) || (whoami_xm != whoami_xm_value))
        {
            last_error = ERR_NOT_ME;
            return false;
        }

        // gyro configuration
        // 95 ODR 25 cutoff normal mode, xyz enabled
        BusG::write(CTRL_REG1_G, 0x0F);
        BusG::write(CTRL_REG2_G, 0x00);

        // TODO: CTRL_REG2_G
        // INT_G pin interrupt enable Fifo overrun interrupt
        BusG::write(CTRL_REG3_G, 0x88);

        //[5:4] scale
        //(00: 245 dps; 01: 500 dps; 10: 2000 dps; 11: 2000 dps)
        // continuous update, 2000dps
        BusG::write(CTRL_REG4_G, 0x00 | (gyroFS << 4));

        // FIFO enabled, get data after the first low pass filter
        BusG::write(CTRL_REG5_G, 0x00);

        // accelerometer configuration
        // reset internal memory
        BusXM::write(CTRL_REG0_XM, 0x80);

        uint8_t timeout = 10;

        while (BusXM::read(CTRL_REG0_XM) != 0x00 && --timeout > 0)
            miosix::Thread::sleep(1);

        if (timeout == 0)
        {
            last_error = ERR_RESET_TIMEOUT;
            return false;
        }

        // 100Hz data rate, continuous update, xyz enabled
        BusXM::write(CTRL_REG1_XM, 0x67);
        // antialias filter 773 Hz, normal mode no test
        BusXM::write(CTRL_REG2_XM, (0x01 << 6) | (accelFS << 3));

        // interrupt not enabled
        BusXM::write(CTRL_REG3_XM, 0x00);
        BusXM::write(CTRL_REG4_XM, 0x00);

        // temperature sensor enabled, 50hz magnetic data rate, 2 gauss
        BusXM::write(CTRL_REG5_XM, 0xF0);

        // 2 gauss
        BusXM::write(CTRL_REG6_XM, 0x00 | (compassFS << 5));
        BusXM::write(CTRL_REG7_XM, 0x80);

        return true;
    }

    bool selfTest() { return false; }

    std::vector<SPIRequest> buildDMARequest() override
    {
        return {
            SPIRequest(DMA_GYRO, BusG::getCSPin(),
                       {OUT_X_L_G | 0xc0, 0, 0, 0, 0, 0, 0}),
            SPIRequest(DMA_ACC, BusXM::getCSPin(),
                       {OUT_X_L_A | 0xc0, 0, 0, 0, 0, 0, 0}),
            SPIRequest(DMA_COMP, BusXM::getCSPin(),
                       {OUT_X_L_M | 0xc0, 0, 0, 0, 0, 0, 0}),
            SPIRequest(DMA_TEMP, BusXM::getCSPin(),
                       {OUT_TEMP_L_XM | 0xc0, 0, 0}),
        };
    }

    void onDMAUpdate(const SPIRequest& req) override
    {
        const auto& r = req.readResponseFromPeripheral();

        int16_t data[3] = {0};
        memcpy(data, &r[1], r.size() - 1);

        switch (req.id())
        {
            case DMA_GYRO:
                mLastGyro.setX(normalizeGyro(data[0]));
                mLastGyro.setY(normalizeGyro(data[1]));
                mLastGyro.setZ(normalizeGyro(data[2]));
                break;
            case DMA_ACC:
                mLastAccel.setX(normalizeAccel(data[0]));
                mLastAccel.setY(normalizeAccel(data[1]));
                mLastAccel.setZ(normalizeAccel(data[2]));
                break;
            case DMA_COMP:
                mLastCompass.setX(normalizeCompass(data[0]));
                mLastCompass.setY(normalizeCompass(data[1]));
                mLastCompass.setZ(normalizeCompass(data[2]));
                break;
            case DMA_TEMP:
                mLastTemp = static_cast<float>(data[0]) / 8.0f + 21.0f;
                break;
        }
    }

    bool onSimpleUpdate() { return false; }

    // clang-format off
    enum DMAType
    {
        DMA_GYRO = 0,
        DMA_ACC  = 1,
        DMA_COMP = 2,
        DMA_TEMP = 3
    };

    enum accelFullScale
    {
        ACC_FS_16G = 4,
        ACC_FS_8G  = 3,
        ACC_FS_6G  = 2,
        ACC_FS_4G  = 1,
        ACC_FS_2G  = 0,
    };

    enum gyroFullScale
    {
        GYRO_FS_245  = 0,
        GYRO_FS_500  = 1,
        GYRO_FS_2000 = 2,
    };

    enum compassFullScale
    {
        COMPASS_FS_12 = 0,
        COMPASS_FS_8  = 1,
        COMPASS_FS_4  = 2,
        COMPASS_FS_2  = 3,
    };
    // clang-format on

private:
    uint8_t accelFS, gyroFS, compassFS;

    constexpr static uint8_t whoami_g_value  = 0xD4;
    constexpr static uint8_t whoami_xm_value = 0x49;

    static constexpr const float accelFSMAP[]   = {2.0, 4.0, 6.0, 8.0, 16.0};
    static constexpr const float compassFSMAP[] = {12.0, 8.0, 4.0, 2.0};
    static constexpr const float gyroFSMAP[]    = {245, 500, 2000};

    inline constexpr float normalizeAccel(int16_t val)
    {
        return static_cast<float>(val) * (accelFSMAP[accelFS] / 32768.0f) *
               EARTH_GRAVITY;
    }

    inline constexpr float normalizeGyro(int16_t val)
    {
        return static_cast<float>(val) * (gyroFSMAP[gyroFS] / 32768.0f) *
               DEGREES_TO_RADIANS;
    }

    inline constexpr float normalizeCompass(int16_t val)
    {
        return static_cast<float>(val) * (compassFSMAP[compassFS] / 32768.0f);
    }

    inline constexpr float normalizeTemp(int16_t val)
    {
        return static_cast<float>(val) / 256.0f;
    }

    // clang-format off
    enum RegMap
    {
        WHO_AM_I_G      = 0x0F,  // default value 0xD4,
        CTRL_REG1_G     = 0x20,
        CTRL_REG2_G     = 0x21,
        CTRL_REG3_G     = 0x22,
        CTRL_REG4_G     = 0x23,
        CTRL_REG5_G     = 0x24,
        STATUS_REG_G    = 0x27,

        // magnetic data registers
        OUT_X_L_M       = 0x08,
        OUT_X_H_M       = 0x09,
        OUT_Y_L_M       = 0x0A,
        OUT_Y_H_M       = 0x0B,
        OUT_Z_L_M       = 0x0C,
        OUT_Z_H_M       = 0x0D,

        WHO_AM_I_XM     = 0x0F,  // default value = 0x49

        OUT_TEMP_L_XM   = 0x05,
        OUT_TEMP_H_XM   = 0x06,

        // gyro data registers
        OUT_X_L_G       = 0x28,
        OUT_X_H_G       = 0x29,
        OUT_Y_L_G       = 0x2A,
        OUT_Y_H_G       = 0x2B,
        OUT_Z_L_G       = 0x2C,
        OUT_Z_H_G       = 0x2D,

        // accelerometer control registers
        CTRL_REG0_XM    = 0x1F,
        CTRL_REG1_XM    = 0x20,
        CTRL_REG2_XM    = 0x21,
        CTRL_REG3_XM    = 0x22,
        CTRL_REG4_XM    = 0x23,
        CTRL_REG5_XM    = 0x24,
        CTRL_REG6_XM    = 0x25,
        CTRL_REG7_XM    = 0x26,

        // accelerometer data registers
        OUT_X_L_A       = 0x28,
        OUT_X_H_A       = 0x29,
        OUT_Y_L_A       = 0x2A,
        OUT_Y_H_A       = 0x2B,
        OUT_Z_L_A       = 0x2C,
        OUT_Z_H_A       = 0x2D
    };
    // clang-format on
};

template <typename BusG, typename BusXM>
constexpr float iNEMOLSM9DS0<BusG, BusXM>::accelFSMAP[];

template <typename BusG, typename BusXM>
constexpr float iNEMOLSM9DS0<BusG, BusXM>::gyroFSMAP[];

template <typename BusG, typename BusXM>
constexpr float iNEMOLSM9DS0<BusG, BusXM>::compassFSMAP[];
