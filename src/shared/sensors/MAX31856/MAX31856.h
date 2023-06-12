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

#pragma once

#include <diagnostic/PrintLogger.h>
#include <drivers/spi/SPIDriver.h>
#include <sensors/Sensor.h>

#include "MAX31856Data.h"

namespace Boardcore
{

/**
 * @brief MAX31855 thermocouple sensor driver.
 *
 * The MAX31856 performs cold-junction compensation and digitizes the signal
 * from any type of thermocouple. This converter resolves temperatures to
 * 0.0078125°C, allows readings as high as +1800°C and as low as -210°C
 * (depending on thermocouple type), and exhibits thermocouple voltage
 * measurement accuracy of ±0.15%. The thermocouple inputs are protected against
 * over voltage conditions up to ±45V. A lookup table (LUT) stores linearity
 * correction data for several types of thermocouples (K, J, N, R, S, T, E, and
 * B). Line frequency filtering of 50Hz and 60Hz is included, as is thermocouple
 * fault detection
 */
class MAX31856 : public Sensor<MAX31856Data>
{
public:
    enum class ThermocoupleType : uint8_t
    {
        B_TYPE  = 0x0,
        E_TYPE  = 0x1,
        J_TYPE  = 0x2,
        K_TYPE  = 0x3,
        N_TYPE  = 0x4,
        R_TYPE  = 0x5,
        S_TYPE  = 0x6,
        T_TYPE  = 0x7,
        GAIN_8  = 0x8,
        GAIN_32 = 0x9,
    };

    MAX31856(SPIBusInterface &bus, miosix::GpioPin cs,
             SPIBusConfig config   = getDefaultSPIConfig(),
             ThermocoupleType type = ThermocoupleType::K_TYPE);

    static SPIBusConfig getDefaultSPIConfig();

    bool init();

    /**
     * @brief Checks whether the thermocouple is connected or not.
     *
     * @return True if the thermocouple is connected.
     */
    bool selfTest();

    void setThermocoupleType(ThermocoupleType type);

    void setColdJunctionOffset(float offset);

private:
    MAX31856Data sampleImpl() override;

    SPISlave slave;
    ThermocoupleType type;

    PrintLogger logger = Logging::getLogger("max31856");

    enum Registers : uint8_t
    {
        CR0    = 0x0,  ///< Configuration 0 Register
        CR1    = 0x1,  ///< Configuration 1 Register
        MASK   = 0x2,  ///< Fault Mask Register
        CJHF   = 0x3,  ///< Cold-Junction High Fault Threshold
        CJLF   = 0x4,  ///< Cold-Junction Low Fault Threshold
        LTHFTH = 0x5,  ///< Linearized Temperature High Fault Threshold MSB
        LTHFTL = 0x6,  ///< Linearized Temperature High Fault Threshold LSB
        LTLFTH = 0x7,  ///< Linearized Temperature Low Fault Threshold MSB
        LTLFTL = 0x8,  ///< Linearized Temperature Low Fault Threshold LSB
        CJTO   = 0x9,  ///< Cold-Junction Temperature Offset Register
        CJTH   = 0xa,  ///< Cold-Junction Temperature Register, MSB
        CJTL   = 0xb,  ///< Cold-Junction Temperature Register, LSB
        LTCBH  = 0xc,  ///< Linearized TC Temperature, Byte 2
        LTCBM  = 0xd,  ///< Linearized TC Temperature, Byte 1
        LTCBL  = 0xe,  ///< Linearized TC Temperature, Byte 0
        SR     = 0xf,  ///< Fault Status Register
    };

    static constexpr uint8_t CR0_CMODE     = 0x1 << 7;
    static constexpr uint8_t CR0_OCFAULT_0 = 0x1 << 4;
    static constexpr uint8_t SR_OPEN       = 0x1 << 0;

    static constexpr float TC_TEMP_LSB_VALUE = 0.007812;  // [°C]
    static constexpr float CJ_TEMP_LSB_VALUE = 0.0625;    // [°C]
};

}  // namespace Boardcore
