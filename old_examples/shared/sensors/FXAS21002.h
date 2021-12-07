/* Copyright (c) 2016 Skyward Experimental Rocketry
 * Authors: Alain Carlucci, Matteo Piazzolla, Silvano Seva
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

#include "Sensor.h"

template <typename Bus>
class FXAS21002 : public GyroSensor
{
public:
    FXAS21002(uint8_t curScale) : mCurScale(curScale & 0x03) {}

    bool init() override
    {
        uint8_t whoami = Bus::read(REG_WHO_AM_I);

        if (whoami != who_am_i_value)
        {
            last_error = ERR_NOT_ME;
            return false;
        }

        Bus::write(REG_CTRL1, 0b01000000);
        uint8_t timeout = 20;
        while ((Bus::read(REG_CTRL1) & 0b01000000) && --timeout > 0)
        {
            miosix::Thread::sleep(1);
        }

        if (timeout == 0)
        {
            last_error = ERR_RESET_TIMEOUT;
            return false;
        }

        // Set current scale
        Bus::write(REG_CTRL0, mCurScale);

        // Datarate 100HZ + Enable READY bit
        Bus::write(REG_CTRL1, (DR_100HZ << 2) | 0x02);

        // Enable data ready interrupt and set as active high
        Bus::write(REG_CTRL2, 0b00000110);

        // Enable wrap to one function
        // Auto increment rolls back from z_axis_lsb to x_axis_msb
        Bus::write(REG_CTRL3, Bus::read(REG_CTRL3) | 0b00001000);

        // Wait until boot is completed
        timeout = 20;
        while (!(Bus::read(REG_INT_SRC_FLAG) & 0b00001000) && --timeout > 0)
        {
            miosix::Thread::sleep(1);
        }

        if (timeout == 0)
        {
            last_error = ERR_RESET_TIMEOUT;
            return false;
        }

        return true;
    }

    // Performs a device self test, see datasheet for further details
    bool selfTest() override
    {
        // Trigger a self test
        Bus::write(REG_CTRL1, Bus::read(REG_CTRL1) | 0b00010000);

        int16_t xAxis =
            (Bus::read(REG_OUT_X_MSB) << 8) | Bus::read(REG_OUT_X_LSB);
        int16_t yAxis =
            (Bus::read(REG_OUT_Y_MSB) << 8) | Bus::read(REG_OUT_Y_LSB);
        int16_t zAxis =
            (Bus::read(REG_OUT_Z_MSB) << 8) | Bus::read(REG_OUT_Z_LSB);

        if (xAxis < 7000 || xAxis > 25000)
        {
            last_error = ERR_X_SELFTEST_FAIL;
            return false;
        }

        if (yAxis < 7000 || yAxis > 25000)
        {
            last_error = ERR_Y_SELFTEST_FAIL;
            return false;
        }

        if (zAxis < 7000 || zAxis > 25000)
        {
            last_error = ERR_Z_SELFTEST_FAIL;
            return false;
        }

        return true;
    }

    std::vector<SPIRequest> buildDMARequest() override
    {
        std::vector<uint8_t> data = {REG_OUT_X_MSB | 0x80, 0, 0, 0, 0, 0, 0};

        return {SPIRequest(0, Bus::getCSPin(), data)};
    }

    void onDMAUpdate(const SPIRequest& req) override
    {
        const auto& r = req.readResponseFromPeripheral();
        int16_t data[3];
        memcpy(data, &r[1], sizeof(data));

        for (int i = 0; i < 3; i++)
            data[i] = fromBigEndian16(data[i]);

        mLastGyro.setX(normalizeGyro(data[0]));
        mLastGyro.setY(normalizeGyro(data[1]));
        mLastGyro.setZ(normalizeGyro(data[2]));
    }

    bool onSimpleUpdate() override { return false; }

    /* UNUSED CODE

    // Set operating mode. Modes available are: STANDBY, READY, ACTIVE
    void setPowerMode(uint8_t mode)
    {
        uint8_t regCtrl1 = Bus::read(REG_CTRL1);

        regCtrl1 &= ~0x03;      //clear last two bits
        regCtrl1 |= mode;       //set mode

        Bus::write(REG_CTRL1, regCtrl1);
    }

    void setSampleRate(uint8_t rate)
    {
        uint8_t regCtrl1 = Bus::read(REG_CTRL1);

        regCtrl1 &= ~0b00011100;      //clear sample rate bits
        regCtrl1 |= rate;             //set rate

        Bus::write(REG_CTRL1, regCtrl1);
    }

    void setFullScaleRange(uint8_t range)
    {
        uint8_t regCtrl0 = Bus::read(REG_CTRL0);
        range &= 0x03;

        regCtrl0 &= ~(0x03);          //clear full scale range bits
        regCtrl0 |= range;            //set range

        mCurScale = range;
        Bus::write(REG_CTRL0, regCtrl0);
    }

    // Set internal low pass filter bandwidth.
    // See datasheet at page 39 for further details
    void setBandwidth(uint8_t bandwidth)
    {
        uint8_t regCtrl0 = Bus::read(REG_CTRL0);

        regCtrl0 &= ~0b11000000;      //clear bandwidth bits
        regCtrl0 |= bandwidth;        //set range

        Bus::write(REG_CTRL0, regCtrl0);
    }

    // Set internal high pass filter bandwidth.
    // See datasheet at page 39 for further details
    void setHiPassFreq(uint8_t freq)
    {
        uint8_t regCtrl0 = Bus::read(REG_CTRL0);

        regCtrl0 &= ~0b00011000;      //clear full hi pass freq bits
        regCtrl0 |= freq;             //set freq

        Bus::write(REG_CTRL0, regCtrl0);
    }

    void enableHiPassFiter()
    {
        Bus::write(REG_CTRL0, Bus::read(REG_CTRL0) | 0b00000100);
    }

    void disableHiPassFiter()
    {
        Bus::write(REG_CTRL0, Bus::read(REG_CTRL0) & (~0b00000100));
    }
    */

    // clang-format off
    enum dataRates
    {
        DR_800HZ  = 0,
        DR_400HZ  = 1,
        DR_200HZ  = 2,
        DR_100HZ  = 3,
        DR_50HZ   = 4,
        DR_25HZ   = 5,
        DR_12_5HZ = 6
    };

    enum opModes
    {
        STANDBY = 0x00,
        READY   = 0x01,
        ACTIVE  = 0x02
    };

    enum gyroFullScale
    {
        DPS2000 = 0x00,
        DPS1000 = 0x01,
        DPS500  = 0x02,
        DPS250  = 0x03
    };
    // clang-format on

private:
    constexpr static uint8_t who_am_i_value = 0xD7;
    constexpr static float gyroFSMAP[]      = {2000, 1000, 500, 250};
    uint8_t mCurScale;

    // clang-format off
    enum regMap
    {
        REG_STATUS          = 0x00,
        REG_OUT_X_MSB       = 0x01,
        REG_OUT_X_LSB       = 0x02,
        REG_OUT_Y_MSB       = 0x03,
        REG_OUT_Y_LSB       = 0x04,
        REG_OUT_Z_MSB       = 0x05,
        REG_OUT_Z_LSB       = 0x06,
        REG_DR_STATUS       = 0x07,
        REG_FIFO_STATUS     = 0x08,
        REG_FIFO_SETUP      = 0x09,
        REG_FIFO_EVENT      = 0x0A,
        REG_INT_SRC_FLAG    = 0x0B,
        REG_WHO_AM_I        = 0x0C,
        REG_CTRL0           = 0x0D,
        REG_RT_CFG          = 0x0E,
        REG_RT_SRC          = 0x0F,
        REG_RT_THF          = 0x10,
        REG_RT_CNT          = 0x11,
        REG_TEMP            = 0x12,
        REG_CTRL1           = 0x13,
        REG_CTRL2           = 0x14,
        REG_CTRL3           = 0x15
    };
    // clang-format on

    inline constexpr float normalizeGyro(int16_t val)
    {
        return static_cast<float>(val) / 32768.0f * gyroFSMAP[mCurScale] *
               DEGREES_TO_RADIANS;
    }
};

template <typename Bus>
constexpr float FXAS21002<Bus>::gyroFSMAP[];
