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

template <typename BusType>
class Si7021 : public HumiditySensor, public TemperatureSensor {
    
public:
    Si7021() { }
    
    bool init() { return selfTest(); }
    
    bool selfTest() {
        
        //test the whoami value
        uint8_t buf[6];
        
        /* To get the second serial number we have to 
         * send CMD_READ_ID1_2 and CMD_READ_ID2_2, so we
         * set the first as register address and the second
         * as payload data and then we send them */
        
        buf[0] = CMD_READ_ID2_2;        
        
        BusType::write(slaveAddr,CMD_READ_ID2_1,buf,1);        
        BusType::read(slaveAddr,0,buf,6);

//         if(buf[0] != 0x15) {
//             last_error = ERR_NOT_ME;
//             return false;
//         }
        
        return true;           
    }
    
    bool updateParams(){
        
        uint8_t buf[2];                        
        BusType::read(slaveAddr,CMD_MEAS_HUM,buf,2);        
        humidity = ((static_cast<float>((buf[0] << 8) | buf[1])*125)/65536) - 6;
        
        BusType::read(slaveAddr,CMD_MEAS_TEMP_PREV_HUM,buf,2);                
        temperature = (
            (static_cast<float>((buf[0] << 8) | buf[1])*175.72)/65536
        ) - 46.85;
        
        return true;
    }
    
    float getTemperature() { return temperature; }
    
       
    float getHumidity() { return humidity; }
    
    /**
     * \return temperature measurement made along the previous 
     *         humidity measurement.
     * \code
     *      getHumidity(); //reads current humidity
     *      getTempRh();   //returns temperature reading made along 
     *                       the humidity reading
     * \endcode
     */
    float getTempRh() {
        
        uint8_t buf[2];
        BusType::read(CMD_MEAS_TEMP_PREV_HUM,buf,2);
        
        uint16_t retVal = (buf[0] << 8) | buf[1];
        
        float temp = (172.72 * retVal)/65535;
        return temp - 46.85;    
    }    
    
    void heaterOn() {
        
        uint8_t regValue;
        BusType::read(slaveAddr,CMD_READ_USR1,&regValue,1);
        
        regValue |= 0b00000100;
        
        BusType::write(slaveAddr,CMD_WRITE_USR1,&regValue,1);
    }
    
    void heaterOff() {
        
        uint8_t regValue;
        BusType::read(slaveAddr,CMD_READ_USR1,&regValue,1);
        
        regValue &= ~0b00000100;
        
        BusType::write(slaveAddr,CMD_WRITE_USR1,&regValue,1);     
    }
    
    /**
     * Set internal heater draw current value, it also can be used to 
     * turn on/off the internal heater 
     *
     * @param level current draw level, between 0 and 16: 
     *                  0 -> heater off, 16 -> heater at max power
     */
    void setHeaterLevel(uint8_t level) {
        
        if(level == 0)
            heaterOff();
        else if(level <= 16)
        {
            heaterOn();
            uint8_t lvl = level - 1;
            
            BusType::write(slaveAddr,CMD_WRITE_HEAT_CTL,&lvl,1);
        }
    }
    
private:
    static constexpr uint8_t slaveAddr = 0x40 << 1;
    
    float temperature;
    float humidity;
    
    enum commands {
        CMD_MEAS_HUM = 0xE5,
        CMD_MEAS_TEMP = 0xE3,
        CMD_MEAS_TEMP_PREV_HUM = 0xE0,  // Read temperature value from 
                                        // previous RH measurement
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
