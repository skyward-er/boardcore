/* FXAS21002 Driver 
 *
 * Copyright (c) 2016 Skyward Experimental Rocketry
 * Authors: Silvano Seva
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

#ifndef FXAS21002_H
#define FXAS21002_H 
#include "Sensor.h"
#include "BusTemplate.h"

template <typename BusType>
class FXAS21002 : public GyroSensor {

    public:
        FXAS21002() {
            
        }
        
        bool init() {
            
            uint8_t whoami = bus.ReadReg(regMap::REG_WHO_AM_I);
            
            if(whoami != who_am_i_value) {
                last_error = ERR_NOT_ME;
                return false;
            }
            
            uint8_t regCtrl2 = ReadReg(regMap::REG_CTRL2);
            
            //enable data ready interrupt and set as active high
            bus.WriteReg(regMap::REG_CTRL2, regCtrl2 |= 0b00000110);      
            
            //enable wrap to one function, auto increment rolls back from z_axis_lsb to x_axis_msb
            bus.WriteReg(regMap::REG_CTRL3, bus.ReadReg(regMap::REG_CTRL3) | 0b00001000);
            
            uint8_t regCtrl1 = ReadReg(regMap::REG_CTRL1);
                        
            //set data rate to 100Hz
            regCtrl1 |= DR_100Hz << 2;
            
            //set device in ready mode
            regCtrl1 |= 0b00000010;
            
            bus.WriteReg(regMap::REG_CTRL1, regCtrl1);
            
            //wait until boot is completed
            while(bus.ReadReg(regMap::REG_INT_SRC_FLAG) & 0b00001000) ;
            
            return true;            
        }
        
        /**
         * Performs a device self test, see datasheet for further details
         */        
        bool selfTest() {
            
            //trigger a self test
            bus.WriteReg(regMap::REG_CTRL1, bus.ReadReg(regMap::REG_CTRL1) | 0b00010000);
            
            int16_t xAxis = (bus.ReadReg(regMap::REG_OUT_X_MSB) << 8) | bus.ReadReg(regMap::REG_OUT_X_LSB);
            int16_t yAxis = (bus.ReadReg(regMap::REG_OUT_Y_MSB) << 8) | bus.ReadReg(regMap::REG_OUT_Y_LSB);
            int16_t zAxis = (bus.ReadReg(regMap::REG_OUT_Z_MSB) << 8) | bus.ReadReg(regMap::REG_OUT_Z_LSB);
            
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
        
        /**
         * Reset device. Restore all registers to their default values
         */        
        void reset() {
            
            bus.WriteReg(regMap::REG_CTRL1, bus.ReadReg(regMap::REG_CTRL1) | 0b01000000);
            while(bus.ReadReg(regMap::REG_CTRL1) & 0b01000000) ;
        }
        
        /**
         * Set operating mode. Modes available are: STANDBY, READY, ACTIVE
         */       
        void setPowerMode(uint8_t mode) {
            
            uint8_t regCtrl1 = ReadReg(regMap::REG_CTRL1);
                        
            regCtrl1 &= ~0x03;      //clear last two bits
            regCtrl1 |= mode;       //set mode
            
            WriteReg(regMap::REG_CTRL1, regCtrl1);
        }

        void setSampleRate(uint8_t rate) {
            
            uint8_t regCtrl1 = ReadReg(regMap::REG_CTRL1);
                        
            regCtrl1 &= ~0b00011100;      //clear sample rate bits
            regCtrl1 |= rate;             //set rate
            
            WriteReg(regMap::REG_CTRL1, regCtrl1);
        }
        
        void setFullScaleRange(uint8_t range) {
            
            uint8_t regCtrl0 = ReadReg(regMap::REG_CTRL0);
                        
            regCtrl0 &= ~0b00000011;      //clear full scale range bits
            regCtrl0 |= range;            //set range
            
            WriteReg(regMap::REG_CTRL0, regCtrl0);
        }
        
        /**
         * Set internal low pass filter bandwidth. See datasheet at page 39 for further details
         */
        void setBandwidth(uint8_t bandwidth) {
            
            uint8_t regCtrl0 = ReadReg(regMap::REG_CTRL0);
                        
            regCtrl0 &= ~0b11000000;      //clear bandwidth bits
            regCtrl0 |= bandwidth;        //set range
            
            WriteReg(regMap::REG_CTRL0, regCtrl0);
            
        }
        
        /**
         * Set internal high pass filter bandwidth. See datasheet at page 39 for further details
         */
        void setHiPassFreq(uint8_t freq) {
            
            uint8_t regCtrl0 = ReadReg(regMap::REG_CTRL0);
                        
            regCtrl0 &= ~0b00011000;      //clear full hi pass freq bits
            regCtrl0 |= bandwidth;        //set freq
            
            WriteReg(regMap::REG_CTRL0, regCtrl0);
            
        }
        
        void enableHiPassFiter() { WriteReg(regMap::REG_CTRL0, ReadReg(regMap::REG_CTRL0) | 0b00000100); }
        
        void disableHiPassFiter() { WriteReg(regMap::REG_CTRL0, ReadReg(regMap::REG_CTRL0) & (~0b00000100)); }
        
        uint16_t getXaxis() { return (bus.ReadReg(regMap::REG_OUT_X_MSB) << 8) | bus.ReadReg(regMap::REG_OUT_X_LSB); }
        uint16_t getYaxis() { return (bus.ReadReg(regMap::REG_OUT_Y_MSB) << 8) | bus.ReadReg(regMap::REG_OUT_Y_LSB); }
        uint16_t getZaxis() { return (bus.ReadReg(regMap::REG_OUT_Z_MSB) << 8) | bus.ReadReg(regMap::REG_OUT_Z_LSB); }
                        
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
        constexpr static uint8_t whoami_value = 0xD7;
                
        enum regMap { 
            REG_STATUS = 0x00,
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

#endif