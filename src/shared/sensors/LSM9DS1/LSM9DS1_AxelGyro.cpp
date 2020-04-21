/* LSM9DS1 accelerometer + giroscope Driver
 *
 * Copyright (c) 2016,2020 Skyward Experimental Rocketry
 * Authors: Andrea Milluzzo
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

#include "LSM9DS1_AxelGyro.h"

using miosix::GpioPin;
using std::array;

LSM9DS1_XLG::LSM9DS1_XLG(SPIBusInterface& bus, GpioPin cs, AxelFSR axelRange,
                         GyroFSR gyroRange, ODR odr, bool fifo_enabled,
                         unsigned int fifo_watermark)
    : fifo_enabled(fifo_enabled), fifo_watermark(fifo_watermark),
      spislave(bus, cs, {}), axelFSR(axelRange), gyroFSR(gyroRange), odr(odr)
{
    // SPI config
    spislave.config.clock_div = SPIClockDivider::DIV64;
}

LSM9DS1_XLG::LSM9DS1_XLG(SPIBusInterface& bus, GpioPin cs, SPIBusConfig config,
                         AxelFSR axelRange, GyroFSR gyroRange, ODR odr,
                         bool fifo_enabled, unsigned int fifo_watermark)
    : fifo_enabled(fifo_enabled), fifo_watermark(fifo_watermark),
      spislave(bus, cs, config), axelFSR(axelRange), gyroFSR(gyroRange),
      odr(odr)
{
}

bool LSM9DS1_XLG::init()
{
    if (sensor_initialized)
    {
        TRACE("[LSM9DS1 XLG] init() : already initialized\n");
        return false;
    }

    SPITransaction spi(spislave);

    // Who Am I check:
    uint8_t whoami = spi.read(regMapXLG::WHO_AM_I);
    if (whoami != WHO_AM_I_XLG_VAL)
    {
        TRACE("[LSM9DS1 XLG] init() : unexpected WAMI -> %02X\n", whoami);
        last_error = ERR_NOT_ME;
        return false;
    }

    // address auto-increment while reading/writing
    spi.write(regMapXLG::CTRL_REG8, CTRL_REG8_VAL);

    // FIFO setup:
    if (fifo_enabled)
    {
        // FIFO continous mode + fifo watermark threshold setup
        spi.write(regMapXLG::FIFO_CTRL, (FIFO_CTRL_VAL | fifo_watermark));

        // interrupt on FIFO treshold enabled
        spi.write(regMapXLG::INT1_CTRL, INT1_CTRL_VAL);

        // DRDY_mask_bit OFF, I2C OFF, FIFO ON
        spi.write(regMapXLG::CTRL_REG9, (CTRL_REG9_VAL | 0x02));
    }
    else
    {
        // DRDY_mask_bit OFF, I2C OFF, FIFO OFF
        spi.write(regMapXLG::CTRL_REG9, CTRL_REG9_VAL);
    }

    /** Axel Setup:
     * ODR, FSR defined by constructor, auto anti-aliasing BW
     * (max), LPF2/HPF bypassed and disabled, axel output enabled by default
     * @ startup
     */
    uint8_t CTRL_REG6_XL_VAL = (int)odr << 5 | (int)axelFSR << 3;
    spi.write(regMapXLG::CTRL_REG6_XL, CTRL_REG6_XL_VAL);

    /** Gyro Setup : ODR, FSR defined by constructor, LPF2/HPF bypassed and
     * disabled, gyro output enabled by default @ startup
     */
    uint8_t CTRL_REG1_G_VAL = (int)odr << 5 | (int)gyroFSR << 3;
    spi.write(regMapXLG::CTRL_REG1_G, CTRL_REG1_G_VAL);

    // sign and orientation Setup <--- BOARD DEPENDENT
    // spi.write(regMapXLG::ORIENT_CFG_G, ORIENT_CFG_VAL);

    // Check all the registers have been written correctly
    if (spi.read(regMapXLG::CTRL_REG8) != CTRL_REG8_VAL)
    {
        return false;
    }
    if (fifo_enabled)
    {
        if (spi.read(regMapXLG::FIFO_CTRL) != (FIFO_CTRL_VAL | fifo_watermark))
        {
            return false;
        }
        if (spi.read(regMapXLG::INT1_CTRL) != INT1_CTRL_VAL)
        {
            return false;
        }
        if (spi.read(regMapXLG::CTRL_REG9) != (CTRL_REG9_VAL | 0x02))
        {
            return false;
        }
    }
    else
    {
        if (spi.read(regMapXLG::CTRL_REG9) != CTRL_REG9_VAL)
        {
            return false;
        }
    }
    if (spi.read(regMapXLG::CTRL_REG6_XL) != CTRL_REG6_XL_VAL)
    {
        return false;
    }
    if (spi.read(regMapXLG::CTRL_REG1_G) != CTRL_REG1_G_VAL)
    {
        return false;
    }

    // select Sensitivity and ODR
    switch (axelFSR)

    {
        case AxelFSR::FS_2:
            axelSensitivity = 0.598f;
            break;
        case AxelFSR::FS_4:
            axelSensitivity = 1.196f;
            break;
        case AxelFSR::FS_8:
            axelSensitivity = 2.393f;
            break;
        case AxelFSR::FS_16:
            axelSensitivity = 7.178f;
            break;
        default:
            axelSensitivity = 0.598f;
            break;
    }

    switch (gyroFSR)
    {
        case GyroFSR::FS_245:
            gyroSensitivity = 8.75f;
            break;
        case GyroFSR::FS_500:
            gyroSensitivity = 17.50f;
            break;
        case GyroFSR::FS_2000:
            gyroSensitivity = 70.0f;
            break;
        default:
            gyroSensitivity = 8.75f;
            break;
    }

    switch (odr)
    {
        case ODR::PWR_DW:
            odrHz = 0.0f;
            break;
        case ODR::ODR_15:
            odrHz = 14.9f;
            break;
        case ODR::ODR_60:
            odrHz = 59.5f;
            break;
        case ODR::ODR_119:
            odrHz = 119.0f;
            break;
        case ODR::ODR_238:
            odrHz = 238.0f;
            break;
        case ODR::ODR_476:
            odrHz = 476.0f;
            break;
        case ODR::ODR_952:
            odrHz = 952.0f;
            break;
        default:
            odrHz = 14.9f;
            break;
    }

    // discard first samples (see datasheet)
    LSM9DS1_XLG::discardSamples(spi);

    TRACE("[LSM9DS1 XLG] init() : done\n");

    sensor_initialized = true;

    return true;
}

bool LSM9DS1_XLG::selfTest() { return true; }

bool LSM9DS1_XLG::onSimpleUpdate()
{
    // if FIFO disabled
    if (!fifo_enabled)
    {
        uint8_t data[12], tempData[2];

        // Read output axel+gyro raw data X,Y,Z and temp raw data
        {
            SPITransaction spi(spislave);
            spi.read(regMapXLG::OUT_X_L_G, data, 12);
            spi.read(regMapXLG::OUT_TEMP_L, tempData, 2);
        }

        // compose signed 16-bit raw data as 2 bytes from the sensor
        // clang-format off
        int16_t x_gy = data[0]  | data[1]  << 8;
        int16_t y_gy = data[2]  | data[3]  << 8;
        int16_t z_gy = data[4]  | data[5]  << 8;

        int16_t x_xl = data[6]  | data[7]  << 8;
        int16_t y_xl = data[8]  | data[9]  << 8;
        int16_t z_xl = data[10] | data[11] << 8;

        int16_t temp = tempData[0] | tempData[1] << 8;

        //convert raw data
        mLastAccel = Vec3(x_xl * axelSensitivity / 1000, 
                          y_xl * axelSensitivity / 1000,
                          z_xl * axelSensitivity / 1000);

        mLastGyro =  Vec3(x_gy * gyroSensitivity / 1000, 
                          y_gy * gyroSensitivity / 1000,
                          z_gy * gyroSensitivity / 1000);

        mLastTemp = tempZero + (temp / tempSensistivity);

        // clang-format on
    }

    /** if FIFO enabled:
     * dump fifo_watermark samples (axel+gyro only) from the sensor at one
     * time. Temperature data can be read using temperatureUpdate() function
     * at a lower frequency
     */
    else
    {
        // 2 bytes per data * 3 axes per type * 2 types(axel+gyro) *
        // 32(FIFO DEPTH MAX) = 384 samples
        uint8_t buf[384];

        // Read output axel+gyro FIFO raw data X,Y,Z
        {
            SPITransaction spi(spislave);
            spi.read(OUT_X_L_G, buf, fifo_watermark * 12);
        }

        // convert & store
        for (int i = 0; i < fifo_watermark; i++)
        {
            // compose signed 16-bit raw data as 2 bytes from the sensor
            // clang-format off
            int16_t x_gy = buf[i * 12]      | buf[i * 12 + 1]  << 8;
            int16_t y_gy = buf[i * 12 + 2]  | buf[i * 12 + 3]  << 8;
            int16_t z_gy = buf[i * 12 + 4]  | buf[i * 12 + 5]  << 8;

            int16_t x_xl = buf[i * 12 + 6]  | buf[i * 12 + 7]  << 8;
            int16_t y_xl = buf[i * 12 + 8]  | buf[i * 12 + 9]  << 8;
            int16_t z_xl = buf[i * 12 + 10] | buf[i * 12 + 11] << 8;

            //convert raw data
            fifo[i].gyroData = Vec3(x_gy * gyroSensitivity / 1000,
                                    y_gy * gyroSensitivity / 1000,
                                    z_gy * gyroSensitivity / 1000);
            fifo[i].axelData = Vec3(x_xl * axelSensitivity / 1000,
                                    y_xl * axelSensitivity / 1000,
                                    z_xl * axelSensitivity / 1000);
            // clang-format on
        }
    }
    return true;
}

bool LSM9DS1_XLG::temperatureUpdate()
{
    // Read output temp raw data
    uint8_t tempData[2];
    {
        SPITransaction spi(spislave);
        spi.read(regMapXLG::OUT_TEMP_L, tempData, 2);
    }
    // compose signed 16-bit raw data as 2 bytes from the sensor
    int16_t temp = tempData[0] | tempData[1] << 8;
    // convert raw data
    mLastTemp = tempZero + temp / tempSensistivity;
    return true;
}

void LSM9DS1_XLG::clearFIFO()
{
    SPITransaction spi(spislave);
    spi.write(FIFO_CTRL, 0);                               // Bypass Mode
    miosix::Thread::sleep(20);                             // Wait
    spi.write(FIFO_CTRL, FIFO_CTRL_VAL | fifo_watermark);  // re-enable FIFO
    LSM9DS1_XLG::discardSamples(spi);
}

const array<lsm9ds1XLGSample, 32>& LSM9DS1_XLG::getLsm9ds1FIFO() const
{
    return fifo;
}

void LSM9DS1_XLG::discardSamples(SPITransaction& spi)
{
    //@ startup, some samples must be discarded (datasheet)
    if (odr != ODR::PWR_DW)
    {
        // wait samples to be overwritten or stored (FIFO on)
        uint16_t toWait_ms = samplesToDiscard * 1000 / odrHz;
        miosix::Thread::sleep(toWait_ms);

        // if FIFO is enabled, read first "samplesToDiscard" samples and
        // discard them.
        if (fifo_enabled)
        {
            spi.read(OUT_X_L_G, samplesToDiscard * 12);
        }
    }
}