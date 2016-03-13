/* SI7021 Driver 
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
#include "BusTemplate.h"


template <typename BusType>
class Si7021 : public HumiditySensor, public TemperatureSensor {
    
    public:
        Si7021() { }
        
        bool init() { return selfTest(); }
        
        bool selfTest() {
            
            //test the whoami value
            uint8_t buf[6] = {CMD_READ_ID2_1, CMD_READ_ID2_2,0,0,0,0};
            
            if(!bus.send(slaveAddr, reinterpret_cast<void*>(buf),2)) {
                last_error = ERR_BUS_FAULT;
                return false;
            }
            
            if(!bus.recv(slaveAddr, reinterpret_cast<void*>(buf),6)) {
                last_error = ERR_BUS_FAULT;
                return false;
            }
            
            if(buf[0] != 0x15) {
                last_error = ERR_NOT_ME;
                return false;
            }
            
            return true;           
        }
        
        float getTemperature() {
            
            
        }
        
        float getHumidity() {
            
        }
        
    private:
        BusType bus;
        static constexpr uint8_t slaveAddr = 0x40;
        
        enum commands {
            CMD_MEAS_HUM = 0xF5,
            CMD_MEAS_TEMP = 0xF3,
            CMD_MEAS_TEMP_PREV_HUM = 0xE0,  //read temperature value from previous RH measurement
            CMR_RESET = 0xFE,
            CMD_WRITE_USR1 = 0xE6,
            CMD_WRITE_USR7 = 0xE7,
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