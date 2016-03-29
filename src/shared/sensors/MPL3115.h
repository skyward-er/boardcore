/* Freescale MPL3115A barometer driver
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

#ifndef MPL3115_H
#define MPL3115_H

#include "Sensor.h"
#include <BusTemplate.h>

template <typename BusType>
class MPL3115 : public PressureSensor, public TemperatureSensor, public AltitudeSensor {

public:
    MPL3115() { init(); }
    
    bool init() {
        
        lastAlt = 0;
        lastPress = 0;
        lastTemp = 0;
        
        setMode(MPL3115_MODE_BAROMETER);
        
        // raise and event flag on new pressure/altitude data and on new temperature data
        uint8_t value = 0x03;
        BusType::write(PT_DATA_CFG,&value,1);
        
        return true;
    }
    
    bool selfTest() {
        
        uint8_t value;
        BusType::read(WHO_AM_I,&value,1);
        if(value == 0xC4)
            return true;
        
        last_error = ERR_NOT_ME;
        return false;
    }
    
    void updateParams() {
        
        /* To start a new one-shot conversion we have to set OST bit whith
         * SBYB bit cleared; these bit are, respectively, the second and 
         * first bits from right in control register 1.
         * After having done this, we poll data register status register checking
         * if PDR and TDR bits are set, meaning that we have new data both for
         * pressure / altitude and for temperature.
         * PDR bit is the second from right, while TDR is the first one         
         */
        
        uint8_t temp;
                
        BusType::read(CTRL_REG1,&temp,1);
        temp &= ~0x03;      //clear first and second bits
        temp |= 0x02;       //set second bit
        BusType::write(CTRL_REG1,&temp,1);
        
        do{
            
            BusType::read(DR_STATUS,&temp,1);
            Thread::sleep(6);   //minimum sample time is 6 ms
            
        }while(!(temp & 0x03));
        
        /* This device supports register pointer auto-increment when reading
         * registers from 0x00 to 0x05, so with one read of 6 bytes we get 
         * status, pressure and temperature registers values
         */
        
        uint8_t data[5];
        BusType::read(STATUS,data,5);
        
        if(sensorMode == MPL3115_MODE_BAROMETER) {            

            //see datasheet at page 21 for more informations about calculations below
            uint32_t press = (static_cast<uint32_t>(data[0]) << 16) | (static_cast<uint32_t>(data[1]) << 8) | static_cast<uint32_t>(data[2]);
            lastPress = static_cast<float>(press) / 64;                    
        }
        
        if(sensorMode == MPL3115_MODE_ALTIMETER) {
  
            uint32_t altitude = (static_cast<uint32_t>(data[0]) << 24) | (static_cast<uint32_t>(data[1]) << 16) | (static_cast<uint32_t>(data[2]) << 8);
            lastAlt =  static_cast<float>(altitude) / 65536;                    
        }

        uint16_t temp = (static_cast<uint16_t>(data[3]) << 8) | static_cast<uint16_t>(data[4]);
        lastTemp =  static_cast<float>(temp) / 256;
    }
    
    /**
     * @return last pressure measured.
     * NOTE: if sensor is not set in barometer mode and this function
     * is called, ZERO IS RETURNED to signal that a wrong request has
     * been performed!!
     */
    
    float getPressure() { 
        
        if(sensorMode == MPL3115_MODE_BAROMETER)
            return lastPress;
        else
            return 0;
    }
    
    /**
     * @return last altitude measured.
     * NOTE: if sensor is not set in altimeter mode and this function
     * is called, ZERO IS RETURNED to signal that a wrong request has
     * been performed!!
     */
               
    float getAltitude() {
        if(sensorMode == MPL3115_MODE_ALTIMETER)
            return lastAlt;
        else
            return 0;
    }
    
    float getTemperature() { return lastTemp; }

    /**
     * Set sensor mode: altimeter or barometer
     * true is returned in case of success, false if 
     * the value passed in @param mode was not recognized
     */
    bool setMode(uint8_t mode) {
        
        if(sensorMode == mode)
            return true;
        
        uint8_t temp;
        BusType::read(CTRL_REG1,&temp,1);
        
        if(mode == MPL3115_MODE_ALTIMETER)
            temp |= 0x80;
        else if(mode == MPL3115_MODE_BAROMETER)
            temp &= ~0x80;
        else
            return false;
        
        BusType::write(CTRL_REG1,&temp,1);
        sensorMode = mode;
        return true;
    }
    
    /**
     * Correction factor used to trim temperature output value
     * @param offset offset trim value as 8 bit two's complement number.
     * Range is from -8 to +7.9375°C, 0.0625°C per LSB
     */
    void setTempOffset(int8_t offset) { BusType::write(TEMP_OFFSET,&offset,1); }
    
    /**
     * Correction factor used to trim pressure output value
     * @param offset offset trim value as 8 bit two's complement number.
     * Range is from -512 to +508 Pa, 4 Pa per LSB
     */
    void setPressOffset(int8_t offset) { BusType::write(PRESS_OFFSET,&offset,1); }
    
    /**
     * Correction factor used to trim altitude output value
     * @param offset offset trim value as 8 bit two's complement number.
     * The range of values are from -128 to +127 meters
     */
    void setAltOffset(int8_t offset) { BusType::write(ALTIT_OFFSET,&offset,1); }
    
    /**
     * Set oversampling ratio. Allowed values are: 1,2,4,8,16,32,64,128
     * Higher the oversample ratio, lower the sample rate allowed.
     * For further informations about minimum time between samples and its
     * relation with oversampling ratio see datasheet at page 31
     */
    void setOversampleRatio(uint8_t ratio) {
        
        uint8_t temp, osr = 0;
        
        BusType::read(CTRL_REG1,&temp,1);
        temp &= ~0b00111000;    //oversample ratio bits are 4th to 6th
        
        switch(ratio) {
            case 1:
            break;
            
            case 2:
                osr = 0x01;
            break;
            
            case 4:
                osr = 0x02;
            break;
            
            case 8:
                osr = 0x03;
            break;
            
            case 16:
                osr = 0x04;
            break;
            
            case 32:
                osr = 0x05;
            break;
            
            case 64:
                osr = 0x06;
            break;
            
            case 128:
                osr = 0x7;
            break;
            
            default:
            break;
        }
        
        temp |= osr << 3;
        BusType::write(CTRL_REG1,&temp,1);
    }
   enum sensMode {       
       MPL3115_MODE_BAROMETER = 0x01,
       MPL3115_MODE_ALTIMETER = 0x02        
    };
    
private:
    static constexpr uint8_t slaveAddr = 0xC0;    
    
    uint8_t sensorMode;
    float lastTemp;
    float lastPress;
    float lastAlt;
    
    enum registers {
        STATUS          = 0x00,
        OUT_P_MSB       = 0x01,
        OUT_P_CSB       = 0x02,
        OUT_P_LSB       = 0x03,
        OUT_T_MSB       = 0x04,        
        OUT_T_LSB       = 0x05,
        DR_STATUS       = 0x06,
        OUT_P_DELTA_MSB = 0x07,
        OUT_P_DELTA_CSB = 0x08,
        OUT_P_DELTA_LSB = 0x09,
        OUT_T_DELTA_MSB = 0x0A,
        OUT_T_DELTA_LSB = 0x0B,
        WHO_AM_I        = 0x0C,
        FIFO_STATUS     = 0x0D,
        FIFO_DATA       = 0x0E,
        FIFO_SETUP      = 0x0F,
        TIME_DELAY      = 0x10,
        SYSMOD          = 0x11,
        INT_SOURCE      = 0x12,
        PT_DATA_CFG     = 0x13,
        BARO_IN_MSB     = 0x14,
        BARO_IN_LSB     = 0x15,
        P_TGT_MSB       = 0x16,
        P_TGT_LSB       = 0x17,
        T_TGT           = 0x18,
        P_WND_MSB       = 0x19,
        P_WND_LSB       = 0x1A,
        T_WND           = 0x1B,
        P_MIN_MSB       = 0x1C,
        P_MIN_CSB       = 0x1D,
        P_MIN_LSB       = 0x1E,
        T_MIN_MSB       = 0x1F,
        T_MIN_LSB       = 0x20,
        P_MAX_MSB       = 0x21,
        P_MAX_CSB       = 0x22,
        P_MAX_LSB       = 0x23,
        T_MAX_MSB       = 0x24,
        T_MAX_LSB       = 0x25,
        CTRL_REG1       = 0x26,
        CTRL_REG2       = 0x27,
        CTRL_REG3       = 0x28,
        CTRL_REG4       = 0x29,
        CTRL_REG5       = 0x2A,
        PRESS_OFFSET    = 0x2B,
        TEMP_OFFSET     = 0x2C,
        ALTIT_OFFSET    = 0x2D        
    };
};

#endif
