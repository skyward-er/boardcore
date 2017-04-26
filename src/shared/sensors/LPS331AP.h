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
class LPS331AP : public PressureSensor, public TemperatureSensor
{
    struct data_t
    {
        int16_t padding;
        int32_t press;
        int16_t temp;
    };
public:

    LPS331AP(uint8_t samplingSpeed)
    { 
        mLastPressure = 0.0f;
        mLastTemp = 0.0f;
        mHighSpeed = (samplingSpeed == SS_25HZ);
    }

    bool init()
    {
        uint8_t whoami = Bus::read(REG_WHO_AM_I);

        if(whoami != who_am_i_value) {
            last_error = ERR_NOT_ME;
            return false;
        }

        uint8_t reg1 = mHighSpeed ? 0xf0 : 0xe0;
        uint8_t reg2 = mHighSpeed ? 0x69 : 0x7a;
        Bus::write(REG_CTRL1, reg1);    // Power on, 3-wire SPI
        Bus::write(REG_RES_CONF, reg2); // AVG_P: 384, AVG_T: 64

        return true;
    }

    bool selfTest() 
    {
        return false; 
    }

    std::vector<SPIRequest> buildDMARequest() override 
    {
        return { 
            SPIRequest(0, Bus::getCSPin(), { 
                REG_STATUS | 0xc0, 0,
                0,0,0,0, // pressure    (int32_t)
                0,0      // temperature (int16_t)
            })
        };
    }

    void onDMAUpdate(const SPIRequest& req) override
    {
        const auto& r = req.readResponseFromPeripheral();
        const data_t *data = (const data_t*) &r[0];
    
        // Remove status and realign bytes
        int32_t pressure = data->press >> 8;

        mLastPressure = normalizePressure(pressure);
        mLastTemp = normalizeTemp(data->temp);
    }

    bool onSimpleUpdate() override
    {
        return false;
    }

    enum samplingSpeed
    {
        SS_25HZ     = 0, // 25Hz
        SS_12HZ5    = 1, // 12.5Hz
    };

private:
    constexpr static uint8_t who_am_i_value = 0xbb;
    uint8_t mHighSpeed;

    inline constexpr float normalizePressure(int32_t val)
    {
        // Page 28 @ Datasheet
        return static_cast<float>(val) / 4096.0f;
    }

    inline constexpr float normalizeTemp(int16_t val)
    {
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
