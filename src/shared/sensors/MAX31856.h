/* MAX31856 Driver
 *
 * Copyright (c) 2015 Skyward Experimental Rocketry
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
#ifndef MAX31856_H
#define MAX31856_H

#include "Sensor.h"
#include <BusTemplate.h>

/** This is a thermocouple */

template <class BusType>
class MAX31856 : public TemperatureSensor {
public:
   
    MAX31856() : lastTemperature(0.0f), cjEnabled(true) { }

    bool init() {
        // TODO: check WHO_AM_I
        uint8_t cr0val = bus.read(REG_CR0); //get CR0 register actual value
        
        cr0val |= 0x01; //select 50Hz noise rejection
        
        bus.write(REG_CR0, cr0val);

        return true;
    }
    
    bool selfTest() {
        return false;
    }

    void updateParams() {
        uint32_t temp = getThermocoupleTemp();

        // TODO convert uint32 temp to a float value
        
        lastTemperature = temp;
    }

    float getTemperature() {
        return lastTemperature; 
    }


    // ----------- BELOW THERMOCOUPLE TUNING FUNCTIONS. ------------
    
    /**
     * @param mode true to set auto conversion mode, that stands
     * for continuous conversion, one every 100ms
     */
    void setConversionMode(bool mode) {
        uint8_t regVal = bus.read(REG_CR0);
        
        if(mode)
            regVal |= 0x80; //conversion mode is the 8-th bit of CR0 register
        else
            regVal &= ~0x80;
        
        bus.write(REG_CR0,regVal);        
    }
    
    /**
     * @param mode true to enable internal cold junction temperature
     * sensor, false to disable it
     */
    void enableCjSensor(bool mode) {
        uint8_t regVal = bus.read(REG_CR0);
        
        if(mode){
            regVal |= 0x08; //CJ temp. sensor is the 4-th bit of CR0 register
            cjEnabled = true;
            
        }else{
            regVal &= ~0x08;
            cjEnabled = false;
        }
        
        bus.write(REG_CR0,regVal);        
    }
    
    /**
     * Trigger a one-shot conversion
     */
    void requestConversion() {
        uint8_t regVal = bus.read(REG_CR0);
        bus.write(REG_CR0, regVal | 0x40);   //conversion trigger is the 7-th bit of CR0 register
    }
    
    /**
     * Set the number of samples that are averaged in order to 
     * achieve one masure
     * @param samplesNum number of samples, that are: 1,2,4,8,16
     */
    void setSamples(uint8_t samplesNum) {
        uint8_t regVal = bus.read(REG_CR1);
        regVal &= 0x8F; //clear bits from 7-th to 5-th
        
        switch(samplesNum){
            case 1:
                //do nothing, since single conversion is set writing 000 to the register
                break;
            
            case 2:
                regVal |= 0b00000001 << 4;
                break;
                
            case 4:
                regVal |= 00000010 << 4;
                break;

            case 8:
                regVal |= 00000011 << 4;
                break;
                
            case 16:
                regVal |= 00000100 << 4;
                break;                  
        }
        bus.write(REG_CR1, regVal);
    }
    
    /**
     * Set Thermocouple type
     */
    void setThermoType(uint8_t type) {
        uint8_t regVal = bus.read(REG_CR1);
        bus.write(REG_CR1, regVal | type);
    }
    
    /**
     * @return cold junction temperature
     */
    uint16_t getColdJunctionTemp() {
        return bus.read(REG_CJTH) << 8 | bus.read(REG_CJTL);
    }
    
    /**
     * Set cold junction temperature, this is possible
     * only if internal cold junction temperature sensor is disabled
     */
    void setColdJunctionTemp(uint16_t temp) {
        if(!cjEnabled){
            bus.write(REG_CJTH, (temp & 0xFF00) >> 16);
            bus.write(REG_CJTL, temp & 0x00FF);
        }
    }
    
    /**
     * @return thermocouple temperature value, 19 bit right-aligned
     */
    uint32_t getThermocoupleTemp() {
        return bus.read(REG_LTCBH) << 16 |
               bus.read(REG_LTCBM) << 8 |
               bus.read(REG_LTCBL);
    }
    
private:
    float lastTemperature;
    BusType bus;

    enum eRegMap{
        REG_CR0     = 0x00,
        REG_CR1     = 0x01,
        REG_MASK    = 0x02,
        REG_CJHF    = 0x03,
        REG_CJLF    = 0x04,
        REG_LTHFTH  = 0x05,
        REG_LTHFTL  = 0x06,
        REG_LTLFTH  = 0x07,
        REG_LTLFTL  = 0x08,
        REG_CJTO    = 0x09,
        REG_CJTH    = 0x0A,
        REG_CJTL    = 0x0B,
        REG_LTCBH   = 0x0C,
        REG_LTCBM   = 0x0D,
        REG_LTCBL   = 0x0E,
        REG_SR      = 0x0F,
    };

    enum eTypes {
        TH_B = 0x00,
        TH_E = 0x01,
        TH_J = 0x02,
        TH_K = 0x03,
        TH_N = 0x04,
        TH_R = 0x05,
        TH_S = 0x06,
        TH_T = 0x07,
    };
    
    //flag to determine if internal cold junction temp sensor is enabled,
    //by default is enabled
    bool cjEnabled;
};

#endif
