/* Copyright (c) 2022 Skyward Experimental Rocketry
 * Author: Fabrizio Monti
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

#include "LSM6DSRXConfig.h"

#include <drivers/spi/SPIDriver.h>
#include <miosix.h>

namespace Boardcore
{

class LSM6DSRX
{
public:

    /**
     * @brief Struct used to store the accelerometer data.
     */
    struct AccData
    {
        float x;
        float y;
        float z;
    };


    /**
     * @brief LSM6DSRX constructor.
     *
     * @param bus SPI bus.
     * @param csPin SPI chip select pin.
     * @param busConfiguration SPI bus configuration.
     * @param config LSM6DSRX configuration.
     */
    LSM6DSRX(SPIBus& bus, miosix::GpioPin csPin, SPIBusConfig busConfiguration, LSM6DSRXConfig& config);


    /**
     * @brief Initialize the sensor.
     */
    bool init();


    /**
     * @brief Retrieves data from the accelerometer.
     */
    void getAccelerometerData(AccData& data);


private:

    bool m_isInit = false;
    SPISlave m_spiSlave;
    LSM6DSRXConfig m_config;
    
    float m_sensitivityAcc;      ///< Sensitivity value for the accelerator.

    const uint8_t WHO_AM_I_VALUE = 0x6B;


    /**
     * @brief Internal registers definitions.
     */
    enum Registers
    {
        REG_WHO_AM_I = 0x0F,  ///< who_am_i register

        REG_CTRL1_XL = 0x10,  ///< accelerometer control register
        REG_CTRL3_C  = 0x12,  ///< set bdu
        REG_CTRL6_C  = 0x15,  ///< enable/disable high performance mode for the
                              ///< accelerometer

        REG_FIFO_CTRL4 = 0x0A,  ///< fifo control register 4 (fifo mode)

        REG_OUTX_L_A =
            0x28,  ///< Low bits output register for the accelerometer (x axis)
        REG_OUTX_H_A =
            0x29,  ///< High bits output register for the accelerometer (x axis)
        REG_OUTY_L_A =
            0x2A,  ///< Low bits output register for the accelerometer (y axis)
        REG_OUTY_H_A =
            0x2B,  ///< High bits output register for the accelerometer (y axis)
        REG_OUTZ_L_A =
            0x2C,  ///< Low bits output register for the accelerometer (z axis)
        REG_OUTZ_H_A =
            0x2D,  ///< High bits output register for the accelerometer (z axis)
    };


    /**
     * @brief Check who_am_i register for validity.
     *
     * @return Returns false if not valid.
     */
    bool checkWhoAmI();


    /**
     * @brief Utility for combining two 8 bits numbers in one 16 bits number.
     * @param low Low bits of the 16 bit number.
     * @param high High bits of the 16 bit number.
     */
    int16_t combineHighLowBits(uint8_t low, uint8_t high);


    /**
     * @brief Reads 16-bits float data from the specified registers.
     * @param lowReg Register containing the low bits of the output
     * @param highReg Register containing the high bits of the output
     */
    float getAxisData(Registers lowReg, Registers highReg);
};

}  // namespace Boardcore