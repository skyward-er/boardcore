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
                         GyroFSR gyroRange, ODR odr, uint8_t temp_div_freq)
    : spislave(bus, cs, {}), axelFSR(axelRange), gyroFSR(gyroRange), odr(odr),
      temp_div_freq(temp_div_freq)
{
    // SPI config
    spislave.config.clock_div = SPIClockDivider::DIV64;
}

LSM9DS1_XLG::LSM9DS1_XLG(SPIBusInterface& bus, GpioPin cs, SPIBusConfig config,
                         AxelFSR axelRange, GyroFSR gyroRange, ODR odr,
                         uint8_t temp_div_freq)
    : spislave(bus, cs, config), axelFSR(axelRange), gyroFSR(gyroRange),
      odr(odr), temp_div_freq(temp_div_freq)
{
}

void LSM9DS1_XLG::enable_fifo(uint8_t watermark)
{
    fifo_enabled   = true;
    fifo_watermark = watermark;
}

bool LSM9DS1_XLG::init()
{
#ifdef DEBUG
    assert(sensor_initialized == false);
#endif

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
        // check fifo-watermark <= 32
        if (fifo_watermark > 32)
        {
            TRACE("[LSM9DS1 XLG] init() : fifo watermark > 32, set to 32\n");
            fifo_watermark = 32;
        }

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
    uint8_t CTRL_REG6_XL_VAL = odr << 5 | axelFSR << 3;
    spi.write(regMapXLG::CTRL_REG6_XL, CTRL_REG6_XL_VAL);

    /** Gyro Setup : ODR, FSR defined by constructor, LPF2/HPF bypassed and
     * disabled, gyro output enabled by default @ startup
     */
    uint8_t CTRL_REG1_G_VAL = odr << 5 | gyroFSR << 3;
    spi.write(regMapXLG::CTRL_REG1_G, CTRL_REG1_G_VAL);

    // sign and orientation Setup <--- BOARD DEPENDENT
    // spi.write(regMapXLG::ORIENT_CFG_G, ORIENT_CFG_VAL);

    // Check all the registers have been written correctly
    if (spi.read(regMapXLG::CTRL_REG8) != CTRL_REG8_VAL)
    {
        TRACE("[LSM9DS1 XLG] init() : CTRL_REG8 readback failed\n");
        return false;
    }
    if (fifo_enabled)
    {
        if (spi.read(regMapXLG::FIFO_CTRL) != (FIFO_CTRL_VAL | fifo_watermark))
        {
            TRACE("[LSM9DS1 XLG] init() : FIFO_CTRL readback failed\n");
            return false;
        }
        if (spi.read(regMapXLG::INT1_CTRL) != INT1_CTRL_VAL)
        {
            TRACE("[LSM9DS1 XLG] init() : INT1_CTRL readback failed\n");
            return false;
        }
        if (spi.read(regMapXLG::CTRL_REG9) != (CTRL_REG9_VAL | 0x02))
        {
            TRACE("[LSM9DS1 XLG] init() : CTRL_REG9 readback failed\n");
            return false;
        }
    }
    else
    {
        if (spi.read(regMapXLG::CTRL_REG9) != CTRL_REG9_VAL)
        {
            TRACE("[LSM9DS1 XLG] init() : CTRL_REG9 readback failed\n");
            return false;
        }
    }
    if (spi.read(regMapXLG::CTRL_REG6_XL) != CTRL_REG6_XL_VAL)
    {
        TRACE("[LSM9DS1 XLG] init() : CTRL_REG6_XL readback failed\n");
        return false;
    }
    if (spi.read(regMapXLG::CTRL_REG1_G) != CTRL_REG1_G_VAL)
    {
        TRACE("[LSM9DS1 XLG] init() : CTRL_REG1_G readback failed\n");
        return false;
    }

    // clear FIFO and discard first samples
    LSM9DS1_XLG::clearFIFO(spi);

    TRACE("[LSM9DS1 XLG] init() : done\n");

    sensor_initialized = true;

    return true;
}

bool LSM9DS1_XLG::selfTest() { return true; }

bool LSM9DS1_XLG::onSimpleUpdate()
{
// you have to call init() before
#ifdef DEBUG
    assert(sensor_initialized == true);
#endif

    // if FIFO disabled
    if (!fifo_enabled)
    {
        uint8_t data[12];

        // Read output axel+gyro raw data X,Y,Z and temp raw data
        {
            SPITransaction spi(spislave);
            spi.read(regMapXLG::OUT_X_L_G, data, 12);
        }

        // compose signed 16-bit raw data as 2 bytes from the sensor
        // clang-format off
        int16_t x_gy = data[0]  | data[1]  << 8;
        int16_t y_gy = data[2]  | data[3]  << 8;
        int16_t z_gy = data[4]  | data[5]  << 8;

        int16_t x_xl = data[6]  | data[7]  << 8;
        int16_t y_xl = data[8]  | data[9]  << 8;
        int16_t z_xl = data[10] | data[11] << 8;

        //convert raw data
        /*fifo[0].axelData = Vec3(x_xl * axelFSR_SMap.at(axelFSR), 
                                y_xl * axelFSR_SMap.at(axelFSR),
                                z_xl * axelFSR_SMap.at(axelFSR));

        fifo[0].gyroData = Vec3(x_gy * gyroFSR_SMap.at(gyroFSR), 
                                y_gy * gyroFSR_SMap.at(gyroFSR),
                                z_gy * gyroFSR_SMap.at(gyroFSR));
        
        fifo[0].timestamp = IRQ_timestamp;*/
        // clang-format on
    }

    /** if FIFO enabled:
     * dump fifo_watermark samples (axel+gyro only) from the sensor at one
     * time.
     */
    else
    {
        // 2 bytes per data * 3 axes per type * 2 types(axel+gyro) *
        // 32(FIFO DEPTH MAX) = 384 samples
        uint8_t buf[384];
        uint8_t overrun            = 0;
        uint32_t delta_transaction = 0;

        // Read output axel+gyro FIFO raw data X,Y,Z
        {
            delta_transaction = miosix::getTick();
            SPITransaction spi(spislave);

            uint8_t fifo_src_reg;
            spi.read(FIFO_SRC, &fifo_src_reg, 1);

            fifo_samples = fifo_src_reg & FIFO_UNREAD_MASK;
            overrun      = (fifo_src_reg & FIFO_OVERRUN_MASK) >> 6;

            spi.read(OUT_X_L_G, buf, fifo_samples * 12);
            delta_transaction = miosix::getTick() - delta_transaction;
        }

        // compute delta time for each sample
        uint64_t dt = delta / last_fifo_level;
        fifo_num++;

        // convert & store
        for (uint8_t i = 0; i < fifo_samples; ++i)
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
            /*fifo[i].gyroData = Vec3(x_gy * gyroFSR_SMap.at(gyroFSR),
                                    y_gy * gyroFSR_SMap.at(gyroFSR),
                                    z_gy * gyroFSR_SMap.at(gyroFSR));
            fifo[i].axelData = Vec3(x_xl * axelFSR_SMap.at(axelFSR),
                                    y_xl * axelFSR_SMap.at(axelFSR),
                                    z_xl * axelFSR_SMap.at(axelFSR));*/
            // clang-format on

            fifo[i].timestamp =
                IRQ_timestamp - ((int)fifo_watermark - (int)i - 1) * dt;
            //fifo[i].fifo_num = fifo_num;
        }

        fifodebug.fifo_num         = fifo_num;
        fifodebug.overrun          = (bool)overrun;
        fifodebug.unread           = fifo_samples;
        fifodebug.transaction_time = delta_transaction;

        last_fifo_level = fifo_samples;  // unused
    }

    // temperature update if temp_count = temp_div_freq
    temp_count++;
    if (temp_count == temp_div_freq)
    {
        temp_count = 0;
        LSM9DS1_XLG::temperatureUpdate();
    }

    return true;
}

void LSM9DS1_XLG::updateTimestamp(uint64_t timestamp)
{
    delta         = timestamp - IRQ_timestamp;
    IRQ_timestamp = timestamp;
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
    lastTemp.tempData  = tempZero + temp / tempSensistivity;
    lastTemp.timestamp = IRQ_timestamp;
    return true;
}

const array<lsm9ds1XLGSample, 32>& LSM9DS1_XLG::getFIFO() const { return fifo; }

const uint8_t& LSM9DS1_XLG::getFIFOdepth() const { return fifo_samples; }

const lsm9ds1XLGSample& LSM9DS1_XLG::getXLGSample() const { return fifo[0]; }

const lsm9ds1TSample& LSM9DS1_XLG::getTSample() const { return lastTemp; }

const lsm9ds1debug& LSM9DS1_XLG::getFIFOStats() const { return fifodebug; }

void LSM9DS1_XLG::clearFIFO(SPITransaction& spi)
{
    spi.write(FIFO_CTRL, 0);                               // Bypass Mode
    spi.write(FIFO_CTRL, FIFO_CTRL_VAL | fifo_watermark);  // re-enable FIFO

    // sleep 20ms for stable output
    miosix::Thread::sleep(20);

    LSM9DS1_XLG::discardSamples(spi);
}

void LSM9DS1_XLG::discardSamples(SPITransaction& spi)
{
    //@ startup, some samples must be discarded (datasheet)
    if (odr != ODR::PWR_DW)
    {
        // wait samples to be overwritten or stored (FIFO on)
        uint16_t toWait_ms = SAMPLES_TO_DISCARD * 1000 / odr_Map.at(odr);
        miosix::Thread::sleep(toWait_ms);

        // if FIFO is enabled, read first "SAMPLES_TO_DISCARD" samples and
        // discard them.
        if (fifo_enabled)
        {
            spi.read(OUT_X_L_G, SAMPLES_TO_DISCARD * 12);
        }
    }
}