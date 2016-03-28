/* MS580301BA07 Driver
 *
 * Copyright (c) 2015-2016 Skyward Experimental Rocketry
 * Authors: Davide Benini, Matteo Piazzolla, Alain Carlucci
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

#ifndef MS580301BA07_H
#define MS580301BA07_H

#include "Sensor.h"
#include <BusTemplate.h>

using namespace miosix;

template<class Bus>
class MS580301BA07 : public PressureSensor, public TemperatureSensor {
	
public:
    /* Class constructor. Reset lastPressure and lastTemperature */
    MS580301BA07() : lastPressure(0.0f), lastTemperature(0.0f) {}

    bool selfTest() {
		return false;
    }

    bool init() {
        int timeout = 30;
        do {
            Bus::read(RESET_DEV);
            cd.sens = readReg(PROM_READ_MASK | PROM_SENS_MASK);
            if(cd.sens == 0)
                Thread::sleep(1);
        } while (cd.sens == 0 && --timeout > 0);

        if(timeout <= 0) {
            last_error = ERR_RESET_TIMEOUT;
            return false;
        }

        cd.off  = readReg(PROM_READ_MASK | PROM_OFF_MASK);
        cd.tcs  = readReg(PROM_READ_MASK | PROM_TCS_MASK);
        cd.tco  = readReg(PROM_READ_MASK | PROM_TCO_MASK);
        cd.tref = readReg(PROM_READ_MASK | PROM_TREF_MASK);
        cd.tempsens = readReg(PROM_READ_MASK | PROM_TEMPSENS_MASK);
	
        return true; 
    }

    float getPressure() {
        return lastPressure; 
    }

    float getTemperature() {
        return lastTemperature; 
    }

    void updateParams() {
        uint8_t rcvbuf[3];
        uint32_t pressure = 0;
        uint32_t temperature =0;

        //TODO: dubbio, non devo fare un ADC read sola?
        //FIXME: add a timeout
        do {
            Bus::write(CONVERT_D1_4096);
            Bus::read(ADC_READ,rcvbuf, 3);

            // FIXME: is this an endianness swapping? miosix should have a 
            // function that do this with only one assembly line.
            pressure = rcvbuf[2] 
                        | ((uint32_t)rcvbuf[1] << 8) 
                        | ((uint32_t)rcvbuf[0] << 16);

            if (pressure == 0)
                Thread::sleep(1);
        } while (pressure == 0);

        do{
            Bus::write(CONVERT_D2_4096);
            Bus::read(ADC_READ,rcvbuf, 3);
            temperature = (uint32_t) rcvbuf[2] 
                        | ((uint32_t)rcvbuf[1] << 8) 
                        | ((uint32_t)rcvbuf[0] << 16);
            if(temperature == 0)
                Thread::sleep(1);
        } while(temperature == 0);

        // FIXME: is this code correct? seems doing floating point ops
        // using int variables
        // FIXME: this code is using cd.* variables but cd is always zero.
        int32_t dt = temperature - cd.tref * (1 << 8);
        lastTemperature = 2000 + dt * (cd.tempsens) / (1 << 23);

        int64_t offs =  cd.off * (1 << 16) + (cd.tco * dt) / (1 << 7);
        int64_t senst =  cd.sens * (1 << 15) + (cd.tcs * dt) / (1 << 8);
        lastPressure = (pressure * senst / (1 << 21) - offs) / (1 << 15);
    }

private:
    float lastPressure;
    float lastTemperature;

    typedef struct {
        uint16_t sens;
        uint16_t off;
        uint16_t tcs;
        uint16_t tco;
        uint16_t tref;
        uint16_t tempsens;
    } calibration_data;
    
    calibration_data cd = {0};
    uint32_t ref_pressure = 0;

    uint16_t readReg(uint8_t reg){
        uint8_t rcv[2];
        Bus::read(reg, rcv, 2);
        uint16_t data = (rcv[0] << 8) | rcv[1];
        return data;
    }

    enum eRegisters {
        RESET_DEV                 = 0x1E,

        CONVERT_D1_256            = 0x40,
        CONVERT_D1_512            = 0x42,
        CONVERT_D1_1024           = 0x44,
        CONVERT_D1_2048           = 0x46,
        CONVERT_D1_4096           = 0x48,

        CONVERT_D2_256            = 0x50,
        CONVERT_D2_512            = 0x52,
        CONVERT_D2_1024           = 0x54,
        CONVERT_D2_2048           = 0x56,
        CONVERT_D2_4096           = 0x58,

        ADC_READ                  = 0x00,

        PROM_READ_MASK            = 0xA0,
        PROM_SENS_MASK            = 0x02,
        PROM_OFF_MASK             = 0x04,
        PROM_TCS_MASK             = 0x06,
        PROM_TCO_MASK             = 0x08,
        PROM_TREF_MASK            = 0x0A,
        PROM_TEMPSENS_MASK        = 0x0C,
    };

    // FIXME: is this also an altimeter!? if not plz move this code away
    // uint32_t ReadAltitude() {
    //     ReadPressure();
    //     altitude =(int32_t)(44330769 - 44330769*pow(((double)comp_pressure/ref_pressure),0.1902));
    //     return altitude;
    // }
    // void setReferencePressure(uint32_t ref){
    //     ref_pressure = ref;
    // }
   
};

#endif
