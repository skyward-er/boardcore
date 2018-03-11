/* ST iNEMO LSM9DS0 Driver
 *
 * Copyright (c) 2016 Skyward Experimental Rocketry
 * Authors: Matteo Michele Piazzolla
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
#include "Sensor.h"
#include "BusTemplate.h"

 template <typename BusType>
 class iNEMOLSM9DS0 : public GyroSensor, public AccelSensor,
                 public CompassSensor, public TemperatureSensor {

    public:
        iNEMOLSM9DS0() {

        }

        bool Init() {

          uint8_t whoami_g = bus.ReadReg(RegMap::REG_WHO_AM_I);
          uint8_t whoami_xm = bus.ReadReg(RegMap::REG_WHO_AM_I);

            if(whoami_g != whoami_g_value | whoami_xm !=whoami_g_value ) {
                last_error = ERR_NOT_ME;
                return false;
            }
            //gyro initialization
            //760 ODR 100 cutoff normal mode, xyz enabled
            bus.WriteReg(RegMap::CTRL_REG1_G, 0xFF);

            //TODO: CTRL_REG2_G

            //INT_G pin interrupt enable
            bus.WriteReg(RegMap::CTRL_REG3_G, 0x80);

            //[5:4] scale
            //(00: 245 dps; 01: 500 dps; 10: 2000 dps; 11: 2000 dps)
            //continuous update, 2000dps
            bus.WriteReg(RegMap::CTRL_REG3_G, 0x30);


            return true;
        }






        /**
         * Performs a device self test, see datasheet for further details
         */
        bool selfTest() {

            //trigger a self test
            bus.WriteReg(RegMap::REG_CTRL1, bus.ReadReg(RegMap::REG_CTRL1) | 0b00010000);

            int16_t xAxis = (bus.ReadReg(RegMap::REG_OUT_X_MSB) << 8) | bus.ReadReg(RegMap::REG_OUT_X_LSB);
            int16_t yAxis = (bus.ReadReg(RegMap::REG_OUT_Y_MSB) << 8) | bus.ReadReg(RegMap::REG_OUT_Y_LSB);
            int16_t zAxis = (bus.ReadReg(RegMap::REG_OUT_Z_MSB) << 8) | bus.ReadReg(RegMap::REG_OUT_Z_LSB);

            if(xAxis < 7000 || xAxis > 25000){

                last_error = ERR_X_SELFTEST_FAIL;
                return false;
            }

            if(yAxis < 7000 || yAxis > 25000){

                last_error = ERR_Y_SELFTEST_FAIL;
                return false;
            }

            if(zAxis < 7000 || zAxis > 25000){

                last_error = ERR_Z_SELFTEST_FAIL;
                return false;
            }

            return true;
        }


        uint16_t ReadGyroX() { return (bus.ReadReg(RegMap::OUT_X_H_G) << 8) | bus.ReadReg(RegMap::OUT_X_L_G); }
        uint16_t ReadGyroY() { return (bus.ReadReg(RegMap::OUT_Y_H_G) << 8) | bus.ReadReg(RegMap::OUT_Y_L_G); }
        uint16_t ReadGyroZ() { return (bus.ReadReg(RegMap::OUT_Z_H_G) << 8) | bus.ReadReg(RegMap::OUT_Z_L_G); }

        uint16_t ReadTemp() { return (bus.ReadReg(RegMap::OUT_TEMP_L_XM << 8) | bus.ReadReg(RegMap::OUT_TEMP_H_XM);}


        void enableHiPassFiter() { WriteReg(RegMap::REG_CTRL0, ReadReg(RegMap::REG_CTRL0) | 0b00000100); }

        void disableHiPassFiter() { WriteReg(RegMap::REG_CTRL0, ReadReg(RegMap::REG_CTRL0) & (~0b00000100)); }


        enum dataRates {
            DR_800HZ = 0,
            DR_400HZ = 1,
            DR_200HZ = 2,
            DR_100Hz = 3,
            DR_50HZ = 4,
            DR_25HZ = 5,
            DR_12_5HZ = 6
        };

        enum opModes {
            STANDBY = 0x00,
            READY = 0x01,
            ACTIVE = 0x02
        };

        enum fullScaleRanges {
            DPS2000 = 0x00,
            DPS1000 = 0x01,
            DPS500 = 0x02,
            DPS250 = 0x03
        };

    private:
        BusType bus;
        constexpr static uint8_t whoami_g_value = 0xD4;
        constexpr static uint8_t whoami_xm_value = 0xD4;

        enum RegMap {
          WHO_AM_I_G = 0x0F  //default value 0xD4,
          CTRL_REG1_G = 0x20,
          CTRL_REG2_G = 0x21,
          CTRL_REG3_G = 0x22,
          CTRL_REG4_G = 0x23,
          CTRL_REG5_G = 0x24,

          STATUS_REG_G = 0x27,

          OUT_X_L_G = 0x28,
          OUT_X_H_G = 0x29,
          OUT_Y_L_G = 0x2A,
          OUT_Y_H_G = 0x2B,
          OUT_Z_L_G = 0x2C,
          OUT_Z_H_G = 0x2D,




            REG_OUT_X_MSB = 0x01,
            REG_OUT_X_LSB = 0x02,
            REG_OUT_Y_MSB = 0x03,
            REG_OUT_Y_LSB = 0x04,
            REG_OUT_Z_MSB = 0x05,
            REG_OUT_Z_LSB = 0x06,
            REG_DR_STATUS = 0x07,
            REG_FIFO_STATUS = 0x08,
            REG_FIFO_SETUP = 0x09,
            REG_FIFO_EVENT = 0x0A,
            REG_INT_SRC_FLAG = 0x0B,
            REG_WHO_AM_I = 0x0C,
            REG_CTRL0 = 0x0D,
            REG_RT_CFG = 0x0E,
            REG_RT_SRC = 0x0F,
            REG_RT_THF = 0x10,
            REG_RT_CNT = 0x11,
            REG_TEMP = 0x12,
            REG_CTRL1 = 0x13,
            REG_CTRL2 = 0x14,
            REG_CTRL3 = 0x15
        };




};
