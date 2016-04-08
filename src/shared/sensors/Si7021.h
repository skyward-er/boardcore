/* Si7021 Driver 
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

#ifndef SI7021_H
#define SI7021_H 

#include "Sensor.h"
#include <BusTemplate.h>

template <typename Bus>
class Si7021 : public HumiditySensor, public TemperatureSensor {
    
public:
    Si7021() { }
    
    bool init() { return selfTest(); }
    
    bool selfTest() {
        
        //test the whoami value
        uint8_t buf[6];
        
        Bus::send(CMD_READ_ID1_1,NULL,0);        
        Bus::read(CMD_READ_ID1_2,buf,6);
        
        if(buf[0] != 0x15) {
            last_error = ERR_NOT_ME;
            return false;
        }

        return true;
    }

    bool updateParams() { return true; }
    
    float getTemperature() {
        uint8_t buf[2];
        Bus::read(CMD_MEAS_TEMP,buf,2);
        
        uint16_t retVal = (buf[1] << 8) | buf[0];
        
        float temp = (172.72 * retVal)/65535;
        return temp - 46.85;
    }
    
    float getHumidity() {
        
        uint8_t buf[2];
        Bus::read(CMD_MEAS_HUM,buf,2);
        
        uint16_t retVal = (buf[1] << 8) | buf[0];
        
        float temp = (125 * retVal)/65535;
        return temp - 6;
    }
    
    /**
        * \return temperature measurement made along the previous humidity measurement.
        * \code
        *      getHumidity(); //reads current humidity
        *      getTempRh();   //returns temperature reading made along the humidity reading
        * \endcode
        */
    
    float getTempRh() {
        
        uint8_t buf[2];
        Bus::read(CMD_MEAS_TEMP_PREV_HUM,buf,2);
        
        uint16_t retVal = (buf[1] << 8) | buf[0];
        
        float temp = (172.72 * retVal)/65535;
        return temp - 46.85;    
    }
    
    void heaterOn() {
        
        uint8_t regValue;
        Bus::read(CMD_READ_USR1,&regValue,1);
        
        regValue |= 0b00000100;
        
        Bus::write(CMD_WRITE_USR1,&regValue,1);
    }
    
    void heaterOff() {
        
        uint8_t regValue;
        Bus::read(CMD_READ_USR1,&regValue,1);
        
        regValue &= ~0b00000100;
        
        Bus::write(CMD_WRITE_USR1,&regValue,1);     
    }
    
    /**
     * Set internal heater draw current value, 
     * it also can be used to turn on/off the internal heater
     * @param level current draw level, between 0 and 16: 
     *        0 -> heater off, 16 -> heater at max power
     */
    void setHeaterLevel(uint8_t level) {
        if(level == 0)
            heaterOff();
        else if(level <= 16) {
            heaterOn();
            uint8_t lvl = level - 1;
            
            Bus::write(CMD_WRITE_HEAT_CTL,&lvl,1);
        }
    }
    
private:
    static constexpr uint8_t slaveAddr = 0x40;
    
    enum commands {
        CMD_MEAS_HUM = 0xE5,
        CMD_MEAS_TEMP = 0xE3,
        CMD_MEAS_TEMP_PREV_HUM = 0xE0,  //read temperature value from previous RH measurement
        CMR_RESET = 0xFE,
        CMD_WRITE_USR1 = 0xE6,
        CMD_READ_USR1 = 0xE7,
        CMD_WRITE_HEAT_CTL = 0x51,
        CMD_READ_HEAT_CTL = 0x11,
        CMD_READ_ID1_1 = 0xFA,
        CMD_READ_ID1_2 = 0x0F,
        CMD_READ_ID2_1 = 0xFC,
        CMD_READ_ID2_2 = 0xC9,
        CMD_READ_FW_REV = 0x84,
    };

};
#endif
