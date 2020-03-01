/* LSM9DS1 magnetometer Driver
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

#pragma once
#include <miosix.h>

#include "drivers/spi/SPIDriver.h"
#include "Sensor.h"

using miosix::GpioPin;


class LSM9DS1_M : public CompassSensor
{
    public:
        
        enum class MagFSR
        {
            FS_4 = 0x00,
            FS_8,
            FS_12,
            FS_16 
        };

        enum class ODR
        {
            ODR_0_625  =   0X00,
            ODR_1_25,
            ODR_2_5,
            ODR_5,
            ODR_10,
            ODR_20,
            ODR_40,
            ODR_80
        };

        /**
         * @brief Creates an instance of an LSM9DS1 accelerometer + gyroscope sensor
         *
         * @param    bus SPI bus the sensor is connected to
         * @param    cs Chip Select GPIO
         * @param    config (OPTIONAL) custom SPIBusConfig 
         * @param    magRange magnetometer Full Scale Range (See datasheet)
         * @param    odr Output Data Rate (See datasheet)
         */

        LSM9DS1_M(
           SPIBusInterface& bus, 
           GpioPin          cs,
           MagFSR           magRange            = MagFSR::FS_8,
           ODR              odr                 = ODR::ODR_0_625
           ): spislave(bus, cs), magFSR(magRange), odr(odr){
            //SPI config
            spislave.config.br = SPIBaudRate::DIV_64; //baud = fclk/64
        }

        LSM9DS1_M(
           SPIBusInterface& bus, 
           GpioPin          cs,
           SPIBusConfig     config,
           MagFSR           magRange            = MagFSR::FS_8,
           ODR              odr                 = ODR::ODR_0_625
           ): spislave(bus, cs, config), magFSR(magRange), odr(odr){

        }

        bool init() override
        {
            //Set FSR
            switch(magFSR)
            {
                case MagFSR::FS_4:
                    magFSRval = 4.0f;
                    break;
                case MagFSR::FS_8:
                    magFSRval = 8.0f;
                    break;
                case MagFSR::FS_12:
                    magFSRval = 12.0f;
                    break;
                case MagFSR::FS_16:
                    magFSRval = 16.0f;
                    break;
                default:
                    magFSRval = 4.0f;
                    break;
            }

            SPITransaction spi(spislave);

            //Who Am I check:
            uint8_t whoami = spi.read(regMapM::WHO_AM_I_M);
            if(whoami != WHO_AM_I_M_VAL){
                printf("LSM9DS1 WAMI: %d\n", whoami);
                last_error = ERR_NOT_ME;
                return false;
            }

            //setup

        }

        bool selfTest() override
        {
            return true; 
        }

        bool onSimpleUpdate() override
        {
            uint8_t magData[6];
            //read output data X,Y,Z
            {
                SPITransaction spi(spislave);
                spi.read(regMapM::OUT_X_L_M, magData, 6);
            }

            int16_t x = magData[0] | magData[1] << 8;
            int16_t y = magData[2] | magData[3] << 8;
            int16_t z = magData[4] | magData[5] << 8;

            mLastCompass =
                    Vec3(x * magFSRval / 65535,
                         y * magFSRval / 65535,
                         z * magFSRval / 65535);
        } 
        
    private:

        SPISlave spislave;

        MagFSR magFSR;
        ODR odr;

        float magFSRval;

        static const uint8_t WHO_AM_I_M_VAL = 0x3D;
        enum regMapM
        {
            OFFSET_X_REG_L_M    =   0x05,
            OFFSET_X_REG_H_M    =   0x06,
            OFFSET_Y_REG_L_M    =   0x07,
            OFFSET_Y_REG_H_M    =   0x08,
            OFFSET_Z_REG_L_M    =   0x09,
            OFFSET_Z_REG_H_M    =   0x0A,
            WHO_AM_I_M          =   0x0F,
            CTRL_REG1_M         =   0x20,
            CTRL_REG2_M         =   0x21,
            CTRL_REG3_M         =   0x22,
            CTRL_REG4_M         =   0x23,
            CTRL_REG5_M         =   0x24,
            STATUS_REG_M        =   0x27,
            OUT_X_L_M           =   0x28,
            OUT_X_H_M           =   0x29,
            OUT_Y_L_M           =   0x2A,
            OUT_Y_H_M           =   0x2B,
            OUT_Z_L_M           =   0x2C,
            OUT_Z_H_M           =   0x2D,
            INT_CFG_M           =   0x30,
            INT_SRC_M           =   0x31,
            INT_THS_L_M         =   0x32,
            INT_THS_H_M         =   0x33
        };
};