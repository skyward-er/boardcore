/* Copyright (c) 2023 Skyward Experimental Rocketry
 * Author: Radu Raul
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

namespace Boardcore
{

namespace H3LIS331DLDefs
{

/**
 * @brief Constants for the FullScale Range.
 *
 * Note: it also changes the sensitivity [mg/digit] (see datasheet).
 */
enum FullScaleRange
{
    FS_100 = 0 << 4,  ///< +-100 g, sensitivity: 0.049 mg/digit
    FS_200 = 1 << 4,  ///< +-200 g, sensitivity: 0.098 mg/digit
    FS_400 = 3 << 4   ///< +-400 g, sensitivity: 0.195 mg/digit
};

/**
 * @brief Constants for Output Data Rate configuration.
 *
 * Note: The ODR also sets the Sensor's Power Mode as it is strictly
 * dependant on the ODR (See datasheet).
 * As the ODR is set differently based on the Power Mode the ODRs including
 * and after ODR_50 will be offsetted to 0 by subtracting ODR_50.
 */
enum OutputDataRate
{
    ODR_LP_0_5 = 0b010 << 5,              ///<  0.5 Hz low power mode
    ODR_LP_1   = 0b011 << 5,              ///<    1 Hz low power mode
    ODR_LP_2   = 0b100 << 5,              ///<    2 Hz low power mode
    ODR_LP_5   = 0b101 << 5,              ///<    5 Hz low power mode
    ODR_LP_10  = 0b110 << 5,              ///<   10 Hz low power mode
    ODR_50     = 0b001 << 5 | 0b00 << 3,  ///<   50 Hz normal power mode
    ODR_100    = 0b001 << 5 | 0b01 << 3,  ///<  100 Hz normal power mode
    ODR_400    = 0b001 << 5 | 0b10 << 3,  ///<  400 Hz normal power mode
    ODR_1000   = 0b001 << 5 | 0b11 << 3   ///< 1000 Hz normal power mode
};

/**
 * @brief Constants for Block Data Update
 */
enum BlockDataUpdate
{
    BDU_CONTINUOS_UPDATE = 0 << 7,
    BDU_WAIT_UNTIL_READ  = 1 << 7
};

/**
 * @brief Constants for the Registers
 */
enum Registers
{
    REG_WHO_AM_I   = 0x0F,
    REG_CTRL_REG1  = 0x20,
    REG_CTRL_REG2  = 0x21,
    REG_CTRL_REG3  = 0x22,
    REG_CTRL_REG4  = 0x23,
    REG_CTRL_REG5  = 0x24,
    REG_STATUS_REG = 0x27,
    REG_OUT_X_L    = 0x28,
    REG_OUT_X_H    = 0x29,
    REG_OUT_Y_L    = 0x2a,
    REG_OUT_Y_H    = 0x2b,
    REG_OUT_Z_L    = 0x2c,
    REG_OUT_Z_H    = 0x2d
};

/**
 * @brief magic numbers for the Status Register
 */
enum Statuses : uint8_t
{
    STATUS_REG_XDR   = 0b0000'0001,  ///< Data Ready on X-Axis
    STATUS_REG_YDR   = 0b0000'0010,  ///< Data Ready on Y-Axis
    STATUS_REG_ZDR   = 0b0000'0100,  ///< Data Ready on Z-Axis
    STATUS_REG_XYZDR = 0b0000'1000,  ///< Data Ready on All Axis
    STATUS_REG_XOR   = 0b0001'0000,  ///< Data Overrun on X-Axis
    STATUS_REG_YOR   = 0b0010'0000,  ///< Data Overrun on Y-Axis
    STATUS_REG_ZOR   = 0b0100'0000,  ///< Data Overrun on Z-Axis
    STATUS_REG_XYZOR = 0b1000'0000   ///< Data Overrun on All Axis
};

constexpr uint8_t CTRL_REG1_XEN = 0b001;
constexpr uint8_t CTRL_REG1_YEN = 0b010;
constexpr uint8_t CTRL_REG1_ZEN = 0b100;

/**
 * @brief make the driver automatically increase the register address when
 * reading multiple bytes with SPI.
 */
constexpr uint8_t AUTOINC_ADDR = 0b0100'0000;

const uint8_t WHO_AM_I_ID = 0x32;

/**
 * @brief Constants for the sensitivity values based on the Full Scale Range.
 *
 * Note: as there is no 0b10 configuration for the FSR the third value is
 * set to 0.
 */
constexpr float SENSITIVITY_VALUES[] = {0.049, 0.098, 0.0, 0.195};

}  // namespace H3LIS331DLDefs

}  // namespace Boardcore
