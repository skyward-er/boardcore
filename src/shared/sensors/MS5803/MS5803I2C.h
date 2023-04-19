/* Copyright (c) 2015-2021 Skyward Experimental Rocketry
 * Authors: Davide Benini, Matteo Piazzolla, Alain Carlucci, Alberto Nidasio
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

#pragma once

#include <diagnostic/PrintLogger.h>
#include <drivers/i2c/I2C.h>
#include <sensors/Sensor.h>

#include "MS5803Data.h"

namespace Boardcore
{

class MS5803I2C : public Sensor<MS5803Data>
{
public:
    enum FSMStates
    {
        STATE_INIT = 0,
        STATE_SAMPLED_TEMP,
        STATE_SAMPLED_PRESS
    };

    enum MS5803Registers : uint8_t
    {
        REG_RESET = 0x1E,

        // Conversion commands for pressure and output data resolution
        REG_CONVERT_D1_256  = 0x40,  // Conversion time: 0.48-0.54-0.60ms
        REG_CONVERT_D1_512  = 0x42,  // Conversion time: 0.95-1.06-1.17ms
        REG_CONVERT_D1_1024 = 0x44,  // Conversion time: 1.88-2.08-2.28ms
        REG_CONVERT_D1_2048 = 0x46,  // Conversion time: 3.72-4.13-4-54ms
        REG_CONVERT_D1_4096 = 0x48,  // Conversion time: 7.40-8.22-9.04ms

        // Conversion commands for temperature and output data resolution
        REG_CONVERT_D2_256  = 0x50,  // Conversion time: 0.48-0.54-0.60ms
        REG_CONVERT_D2_512  = 0x52,  // Conversion time: 0.95-1.06-1.17ms
        REG_CONVERT_D2_1024 = 0x54,  // Conversion time: 1.88-2.08-2.28ms
        REG_CONVERT_D2_2048 = 0x56,  // Conversion time: 3.72-4.13-4-54ms
        REG_CONVERT_D2_4096 = 0x58,  // Conversion time: 7.40-8.22-9.04ms

        // Read command for adc output
        REG_ADC_READ = 0x00,

        REG_PROM_SENS_MASK     = 0xA2,
        REG_PROM_OFF_MASK      = 0xA4,
        REG_PROM_TCS_MASK      = 0xA6,
        REG_PROM_TCO_MASK      = 0xA8,
        REG_PROM_TREF_MASK     = 0xAA,
        REG_PROM_TEMPSENS_MASK = 0xAC,
    };

    static constexpr uint8_t TIMEOUT = 5;

    MS5803I2C(I2C& bus, uint16_t temperatureDivider = 1);

    bool init() override;

    bool selfTest() override;

private:
    /**
     * Implements a state machines composed of 3 states:
     * 1. Command pressure sample
     * 2. Read Pressure sample & command temperature sample
     * 3. Read temperature sample & command pressure sample
     *
     * After the first call to sample() (state 1), the machine
     * transitions between states 2 and 3: The effective sampling rate is half
     * the rate at which this function is called.
     * Example: call sample() at 100 Hz -> Pressure & Temperature sample
     * Rate = 50 Hz
     */
    MS5803Data sampleImpl() override;

    MS5803Data updateData();

    uint16_t readReg(uint8_t reg);

    I2C& bus;
    I2CDriver::I2CSlaveConfig slaveConfig{0x77, I2CDriver::Addressing::BIT7,
                                          I2CDriver::Speed::FAST};

    MS5803CalibrationData calibrationData;
    uint16_t temperatureDivider;

    uint8_t deviceState               = STATE_INIT;
    uint32_t rawTemperature           = 0;
    uint32_t rawPressure              = 0;
    uint64_t lastTemperatureTimestamp = 0;
    uint16_t tempCounter              = 0;

    PrintLogger logger = Logging::getLogger("ms5803");
};

}  // namespace Boardcore
