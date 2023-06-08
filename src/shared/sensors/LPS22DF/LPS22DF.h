/* Copyright (c) 2023 Skyward Experimental Rocketry
 * Author: Giulia Ghirardini
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

#include <diagnostic/PrintLogger.h>
#include <drivers/spi/SPIDriver.h>
#include <drivers/timer/TimestampTimer.h>
#include <miosix.h>
#include <sensors/Sensor.h>

#include <Eigen/Core>
#include <array>

#include "LPS22DFData.h"

namespace Boardcore
{

/**
 * Driver for LPS22DF, Low-power and high-precision MEMS nano pressure sensor.
 */
class LPS22DF : public Sensor<LPS22DFData>
{
public:
    /**
     * @brief Constants for Output Data Rate Configuration
     *
     * Notes:
     * 1. ODR configuration is valid for both pressure and temperature. When
     *    the ODR bits are set to a value different than '0000' the device is in
     *    continuous mode, otherwise it is set to be in Power-down mode or
     * One-shot mode. The latter mode is only possible if the ONESHOT bit in
     * CTRL-REG2 is set to 1.
     * 2. ODR_200_HZ is set to '1xxx' which means it doesn't matter if the
     * x-bits are setted to 0 or to 1
     */
    enum ODR : uint8_t
    {
        ONE_SHOT = (0b0000 << 3),
        ODR_1    = (0b0001 << 3),
        ODR_4    = (0b0010 << 3),
        ODR_10   = (0b0011 << 3),
        ODR_25   = (0b0100 << 3),
        ODR_50   = (0b0101 << 3),
        ODR_75   = (0b0110 << 3),
        ODR_100  = (0b0111 << 3),
        ODR_200  = (0b1000 << 3)
    };

    enum AVG : uint8_t
    {
        AVG_4   = 0b000,
        AVG_8   = 0b001,
        AVG_16  = 0b010,
        AVG_32  = 0b011,
        AVG_64  = 0b100,
        AVG_128 = 0b101,
        AVG_512 = 0b111
    };

    enum Mode
    {
        ONE_SHOT_MODE,   // ODR = ONE_SHOT
        CONITNUOUS_MODE  // BYPASS, ODR = odr
    };

    /**
     * @brief Sensor configuration
     *
     * This struct contains all the settings the user
     * is able to modify with the help of the driver.
     * They are applied in the constructor of LPS22DF class
     * and on each call of LPS22DF::applyConfig()
     */
    typedef struct
    {
        AVG avg;
        Mode mode;
        ODR odr;

        // bool enableTemperature      = true;
        // unsigned temperatureDivider = 1;
        // bool enableInterrupt        = false;
        // float threshold             = 0;
        // bool doBlockDataUpdate      = false;
    } Config;

    LPS22DF(SPIBusInterface& bus, miosix::GpioPin pin,
            SPIBusConfig spiConfig = {}, Config config = {});

    static SPIBusConfig getDefaultSPIConfig();

    bool init() override;

    bool selfTest() override;

    /**
     * @brief Overwrites the sensor settings.
     *
     * Writes a certain config to the sensor registers. This method is
     * automatically called in LPS22DF::init() using as parameter the
     * configuration given in the constructor.
     *
     * This method checks if the values were actually written in the sensor's
     * registers and returns false if at least one of them was not as expected.
     *
     * @param config The configuration to be applied.
     * @returns True if the configuration was applied successfully,
     * false otherwise.
     */
    bool setConfig(const Config& config);

    bool setAverage(AVG avg);
    bool setOutputDataRate(ODR odr);

private:
    LPS22DFData sampleImpl() override;

    float convertPressure(uint8_t pressXL, uint8_t pressL, uint8_t pressH);
    float convertTemperature(uint8_t tempL, uint8_t tempH);

    SPISlave mSlave;
    Config mConfig;

    unsigned tempCounter = 0;
    bool isInitialized   = false;

    enum Registers : uint8_t
    {
        INTERRUPT_CFG = 0x0b,
        THS_P_L       = 0x0c,
        THS_P_H       = 0x0d,
        IF_CTRL       = 0x0e,

        WHO_AM_I = 0x0f,

        CTRL_REG1 = 0x20,
        CTRL_REG2 = 0x21,
        CTRL_REG3 = 0x22,
        CTRL_REG4 = 0x23,

        FIFO_CTRL = 0x14,
        FIFO_WTM  = 0x15,

        REF_P_L = 0x16,
        REF_P_H = 0x17,

        FIFO_STATUS1 = 0x25,
        FIFO_STATUS2 = 0x26,
        STATUS       = 0x27,

        PRESSURE_OUT_XL = 0x28,
        PRESSURE_OUT_L  = 0x29,
        PRESSURE_OUT_H  = 0x2a,
        TEMP_OUT_L      = 0x2b,
        TEMP_OUT_H      = 0x2c,

        FIFO_DATA_OUT_PRESS_XL = 0x78,
        FIFO_DATA_OUT_PRESS_L  = 0x79,
        FIFO_DATA_OUT_PRESS_H  = 0x7a,
    };

    static constexpr uint16_t temperatureSensitivity = 100;   // LSB/°C
    static constexpr uint16_t pressureSensitivity    = 4096;  // LSB/hPa
    static constexpr float REFERENCE_TEMPERATURE     = 25;    // °C
    static constexpr uint32_t WHO_AM_I_VALUE         = 0xb4;

    enum if_ctrl : uint8_t
    {
        CS_PU_DIS   = (1 << 1),
        INT_PD_DIS  = (1 << 2),
        SDO_PU_EN   = (1 << 3),
        SDA_PU_EN   = (1 << 4),
        SIM         = (1 << 5),
        I2C_I3C_DIS = (1 << 6),  ///< Disable I2C and I3C digital interfaces
        INT_EN_I3C  = (1 << 7)
    };

    enum crtl_reg2 : uint8_t
    {
        ONE_SHOT_START = (1 << 0),  ///< Enable one-shot mode
        SWRESET        = (1 << 2),  ///< Software reset
        BDU            = (1 << 3),  ///< Block data update
        EN_LPFP        = (1 << 4),  ///< Enable low-pass filter on pressure data
        LFPF_CFG       = (1 << 5),  ///< Low-pass filter configuration
        FS_MODE        = (1 << 6),  ///< Full-scale selection
        BOOT           = (1 << 7)   ///< Reboot memory content
    };

    enum ctrl_reg3 : uint8_t
    {
        IF_ADD_INC =
            (0b1 << 0),  ///< Increment register during a multiple byte access
        PP_OD =
            (0b1 << 1),  ///< Push-pull/open-drain selection on interrupt pin
        INT_H_L = (0b1 << 3)  ///< Select interrupt active-high, active-low
    };

    enum ctrl_reg4 : uint8_t
    {
        INT_F_OVR  = (0b1 << 0),  ///< FIFO overrun status on INT_DRDY pin
        INT_F_WTM  = (0b1 << 1),  ///< FIFO threshold status on INT_DRDY pin
        INT_F_FULL = (0b1 << 2),  ///< FIFO full flag on INT_DRDY pin
        INT_EN     = (0b1 << 4),  ///< Interrupt signal on INT_DRDY pin
        DRDY       = (0b1 << 5),  ///< Date-ready signal on INT_DRDY pin
        DRDY_PLS   = (0b1 << 6)   ///< Data-ready pulsed on INT_DRDY pin
    };

    PrintLogger logger = Logging::getLogger("lps22df");
};  // namespace Boardcore

}  // namespace Boardcore