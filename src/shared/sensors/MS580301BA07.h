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

// TODO second order temperature compensation 
template<class Bus>
class MS580301BA07 : public PressureSensor, public TemperatureSensor,
    public AltitudeSensor {
	
public:
    /* Class constructor. Reset lastPressure and lastTemperature */
    MS580301BA07() : 
        lastPressure(0.0f), lastTemperature(0.0f), 
        localPressure(1013.25) {}

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

    bool selfTest() {
		return false;
    }

    float getPressure() {
        return lastPressure; 
    }

    float getTemperature() {
        return lastTemperature; 
    }

    void setLocalPressure(float mBarPressure) {
        localPressure = mBarPressure;
    }

    // TODO move this code away
    float getAltitude() {
        return 44330.769 * 
            (1.0f - pow(lastPressure/localPressure,0.19019f));
    }

    bool updateParams() {
        uint8_t rcvbuf[3];
        uint32_t pressure = 0, temperature = 0, timeout;

        timeout = 3;
        do {
            Bus::write(CONVERT_D1_4096);
            Thread::sleep(10);
            Bus::read_low(ADC_READ,rcvbuf, 3);

            // TODO use swapBytes
            pressure = rcvbuf[2] 
                        | ((uint32_t)rcvbuf[1] << 8) 
                        | ((uint32_t)rcvbuf[0] << 16);
        } while (pressure == 0 && --timeout > 0);

        if(timeout == 0) {
            last_error = ERR_RESET_TIMEOUT;
            return false;
        }

        timeout = 3;
        do {
            Bus::write(CONVERT_D2_4096);
            Thread::sleep(10);
            Bus::read_low(ADC_READ,rcvbuf, 3);

            // TODO use swapBytes
            temperature = (uint32_t) rcvbuf[2] 
                        | ((uint32_t)rcvbuf[1] << 8) 
                        | ((uint32_t)rcvbuf[0] << 16);
        } while(temperature == 0 && --timeout > 0);

        if(timeout == 0) {
            last_error = ERR_RESET_TIMEOUT;
            return false;
        }

        int32_t dt = temperature - (cd.tref << 8);
        int32_t temp = 2000 + ((dt * cd.tempsens) >> 23);
        lastTemperature =  temp / 100.0f;

        int64_t offs = ((int64_t)cd.off << 16)+(((int64_t)cd.tco * dt) >> 7);
        int64_t senst = ((int64_t)cd.sens << 15)+(((int64_t)cd.tcs * dt) >> 8);

        int64_t ttemp = pressure * senst;
        int32_t pres = ((ttemp >> 21) - offs) >> 15;
        lastPressure = pres / 100.0f;
        return true;
    }

private:
    float lastPressure, lastTemperature, localPressure;

    typedef struct {
        uint16_t sens;
        uint16_t off;
        uint16_t tcs;
        uint16_t tco;
        uint16_t tref;
        uint16_t tempsens;
    } calibration_data;
    
    calibration_data cd = {0};

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
};

#endif
