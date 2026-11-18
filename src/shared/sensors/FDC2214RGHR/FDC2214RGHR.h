/* Copyright (c) 2023 Skyward Experimental Rocketry
 * Author: Alberto Nidasio
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
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 */

#include <drivers/i2c/I2C.h>
#include <sensors/Sensor.h>

namespace Boardcore
{

class FDC2214RGHR : public Sensor<FDC2214RGHR>
{
public:
    bool init() override;
    static constexpr uint16_t DEVICE_ID_VAL = 0x3055;

private:
    bool checkWhoAmI();
    enum Registers : uint8_t
    {
        RCOUNT_CH0         = 0x08,  // 4 registri 0x08-0x0B
        SETTLECOUNT_CH0    = 0x10,  // 4 registri 0x10-0x13
        CLOCK_DIVIDERS_CH0 = 0x14,  // 4 registri 0x14-0x17
        ERROR_CONFIG       = 0x19,
        CONFIG             = 0x1A,
        MUX_CONFIG         = 0x1B,
        RESET_DEV          = 0x1C,
        DRIVE_CURRENT_CH0  = 0x1E,  // 4 registri 0x1E-0x21
        MANUFACTURER_ID    = 0x7E,
        DEVICE_ID          = 0x7F,
    };

    I2C& bus;
    // TODO: cosa figa devo scriverci dentro?
    I2CDriver::I2CSlaveConfig slaveConfig{};
};

}  // namespace Boardcore
