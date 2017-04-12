/* LPS331AP Driver 
 *
 * Copyright (c) 2016 Skyward Experimental Rocketry
 * Authors: Alain Carlucci
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

#ifndef LPS331AP_H
#define LPS331AP_H 
#include "Sensor.h"
#include <BusTemplate.h>

template <typename Bus>
class LPS331AP : public PressureSensor, public TemperatureSensor {

public:

    LPS331AP(uint8_t samplingSpeed) : 
        last_pressure(0.0f), last_temperature(0.0f) { 
        hi_speed = (samplingSpeed == SS_25HZ);
    }

    bool init() {
        uint8_t whoami = Bus::read(REG_WHO_AM_I);

        if(whoami != who_am_i_value) {
            last_error = ERR_NOT_ME;
            return false;
        }

        uint8_t reg1 = hi_speed ? 0xf0 : 0xe0;
        uint8_t reg2 = hi_speed ? 0x69 : 0x7a;
        Bus::write(REG_CTRL1, reg1);    // Power on, 3-wire SPI
        Bus::write(REG_RES_CONF, reg2); // AVG_P: 384, AVG_T: 64
        return true;
    }

    bool selfTest() {
        return false; 
    }

    bool updateParams() {
        #pragma pack(1)
        struct {
            int32_t press;
            int16_t temp;
        } data = {0};
        #pragma pack()
        
        // 0x40: read from STATUS up to sizeof(data) bytes
        Bus::read(REG_STATUS | 0x40, 
                reinterpret_cast<uint8_t *>(&data), sizeof(data));

        data.press >>= 8; // Remove status and realign bytes

        last_pressure = normalizePressure(data.press);
        last_temperature = normalizeTemp(data.temp);

        return true;
    }

    float getPressure() {
        return last_pressure; 
    }
    
    float getTemperature() {
        return last_temperature;
    }

    enum samplingSpeed {
        SS_25HZ     = 0, // 25Hz
        SS_12HZ5    = 1, // 12.5Hz
    };

private:
    constexpr static uint8_t who_am_i_value = 0xbb;
    uint8_t hi_speed;
    float last_pressure, last_temperature;

    inline constexpr float normalizePressure(int32_t val) {
        // Page 28 @ Datasheet
        return static_cast<float>(val) / 4096.0f;
    }

    inline constexpr float normalizeTemp(int16_t val) {
        // Page 29 @ Datasheet
        return static_cast<float>(val) / 480.0f + 42.5f;
    }

    enum regMap {
        REG_WHO_AM_I        = 0x0f, 
        REG_RES_CONF        = 0x10, 

        REG_CTRL1           = 0x20, 
        REG_CTRL2           = 0x21, 
        REG_CTRL3           = 0x22, 
        REG_INT_CFG         = 0x23, 

        REG_STATUS          = 0x27,

        REG_PRESS_OUT       = 0x28,
        REG_TEMP_OUT        = 0x2b,
    };
};

#endif
