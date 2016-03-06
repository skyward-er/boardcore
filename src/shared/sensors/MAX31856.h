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

#include "BusTemplate.h"

class MAX31856_Regmap{
    
public:
    
    static constexpr uint8_t CR0 = 0x00;
    static constexpr uint8_t CR1 = 0x01;
    static constexpr uint8_t MASK = 0x02;
    static constexpr uint8_t CJHF = 0x03;
    static constexpr uint8_t CJLF = 0x04;
    static constexpr uint8_t LTHFTH = 0x05;
    static constexpr uint8_t LTHFTL = 0x06;
    static constexpr uint8_t LTLFTH = 0x07;
    static constexpr uint8_t LTLFTL = 0x08;
    static constexpr uint8_t CJTO = 0x09;
    static constexpr uint8_t CJTH = 0x0A;
    static constexpr uint8_t CJTL = 0x0B;
    static constexpr uint8_t LTCBH = 0x0C;
    static constexpr uint8_t LTCBM = 0x0D;
    static constexpr uint8_t LTCBL = 0x0E;
    static constexpr uint8_t SR = 0x0F;
    
    static constexpr enum thTypes{
        TH_B = 0x00,
        TH_E = 0x01,
        TH_J = 0x02,
        TH_K = 0x03,
        TH_N = 0x04,
        TH_R = 0x05,
        TH_S = 0x06,
        TH_T = 0x07       
    };
    
private:
    //since this is only a wrapper class disallow creating instances
    MAX31856_Regmap();
};

template <class Regmap>
class Thermocouple : public Sensor<>
{
public:
    
    Thermocouple()
    {
        uint8_t cr0val = Sensor::ReadReg(Regmap::CR0); //get CR0 register actual value
        
        cr0val |= 0x01; //select 50Hz noise rejection
        
        Sensor::WriteReg(Regmap::CR0, cr0val);
    }
    
    /**
     * @param mode true to set auto conversion mode, that stands
     * for continuous conversion, one every 100ms
     */
    void setConversionMode(bool mode)
    {
        uint8_t regVal = Sensor::ReadReg(Regmap::CR0);
        
        if(mode)
            regVal |= 0x80; //conversion mode is the 8-th bit of CR0 register
        else
            regVal &= ~0x80;
        
        Sensor::WriteReg(Regmap::CR0,regVal);        
    }
    
    /**
     * @param mode true to enable internal cold junction temperature
     * sensor, false to disable it
     */
    void enableCjSensor(bool mode)
    {
        uint8_t regVal = Sensor::ReadReg(Regmap::CR0);
        
        if(mode){
            regVal |= 0x08; //CJ temp. sensor is the 4-th bit of CR0 register
            cjEnabled = true;
            
        }else{
            regVal &= ~0x08;
            cjEnabled = false;
        }
        
        Sensor::WriteReg(Regmap::CR0,regVal);        
    }
    
    /**
     * Trigger a one-shot conversion
     */
    void requestConversion()
    {
        uint8_t regVal = Sensor::ReadReg(Regmap::CR0);
        Sensor::WriteReg(Regmap::CR0, regVal | 0x40);   //conversion trigger is the 7-th bit of CR0 register
    }
    
    /**
     * Set the number of samples that are averaged in order to 
     * achieve one masure
     * @param samplesNum number of samples, that are: 1,2,4,8,16
     */
    void setSamples(uint8_t samplesNum)
    {
        uint8_t regVal = Sensor::ReadReg(Regmap::CR1);
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
        Sensor::WriteReg(Regmap::CR1, regVal);
    }
    
    /**
     * Set Thermocouple type
     */
    void setThermoType(uint8_t type)
    {
        uint8_t regVal = Sensor::ReadReg(Regmap::CR1);
        Sensor::WriteReg(Regmap::CR1, regVal | type);
    }
    
    /**
     * @return cold junction temperature
     */
    uint16_t getColdJunctionTemp()
    {
        return Sensor::ReadReg(Regmap::CJTH) << 8 | Sensor::ReadReg(Regmap::CJTL);
    }
    
    /**
     * Set cold junction temperature, this is possible
     * only if internal cold junction temperature sensor is disabled
     */
    void setColdJunctionTemp(uint16_t temp)
    {
        if(!cjEnabled){
            Sensor::WriteReg(Regmap::CJTH, (temp & 0xFF00) >> 16);
            Sensor::WriteReg(Regmap::CJTL, temp & 0x00FF);
        }
    }
    
    /**
     * @return thermocouple temperature value, 19 bit right-aligned
     */
    uint32_t getThermocoupleTemp()
    {
        return Sensor::ReadReg(Regmap::LTCBH) << 16 |
               Sensor::ReadReg(Regmap::LTCBM) << 8 |
               Sensor::ReadReg(Regmap::LTCBL);
    }
    
private:
    
    //flag to determine if internal cold junction temp sensor is enabled,
    //by default is enabled
    bool cjEnabled = true;
};

#endif