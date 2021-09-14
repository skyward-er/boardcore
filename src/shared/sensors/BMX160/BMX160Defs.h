/* Copyright (c) 2020 Skyward Experimental Rocketry
 * Author: Davide Mor
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

#include <cstdint>

/**
 * @brief Various BMX160 register/enums definitions.
 */
namespace BMX160Defs
{

/**
 * @brief Temperature sensor sensibility.
 */
const float TEMP_SENSIBILITY = 64.0f / 32768.0f;

/**
 * @brief Magnetometer fixed sensibility.
 */
const float MAG_SENSIBILITY = 0.3f;

/**
 * @brief BMX160 Chip Id.
 */
const uint8_t CHIPID = 0xD8;

/**
 * @brief Default value for FIFO_CONFIG_1.
 */
const uint8_t FIFO_CONFIG_1_DEFAULT = 0x10;

/**
 * @brief Values for SELF_TEST register.
 */
enum SELF_TEST
{
    SELF_TEST_GYR     = 0x10,  ///< Starts selftest of the gyroscope.
    SELF_TEST_ACC_AMP = 0x08,  ///< Select amplitude of the selftest deflection.
    SELF_TEST_ACC_SIGN   = 0x04,  ///< Select sign of selftest exitation.
    SELF_TEST_ACC_ENABLE = 0x01   ///< Starts selftest of the accelerometer.
};

/**
 * @brief Values for FIFO_CONFIG_1 register.
 */
enum FIFO_CONFIG_1
{
    FIFO_CONFIG_1_ACC_EN    = 0x80,  ///< Store accelerometer data in fifo.
    FIFO_CONFIG_1_GYR_EN    = 0x40,  ///< Store gyroscope data in fifo.
    FIFO_CONFIG_1_MAG_EN    = 0x20,  ///< Store magnetometer data in fifo.
    FIFO_CONFIG_1_HEADER_EN = 0x10,  ///< Stores an header for each frame.
};

/**
 * @brief Values for FIFO_DOWNS register.
 */
enum FIFO_DOWNS
{
    FIFO_DOWNS_ACC_FILT_DATA = 0x80,
    FIFO_DOWNS_GYR_FILT_DATA = 0x08,
};

/**
 * @brief Values for INT_OUT_CTRL register.
 */
enum INT_OUT_CTRL
{
    INT_OUT_CTRL_INT2_OUT_EN = 0x80,  ///< Output enable for INT2 pin.
    INT_OUT_CTRL_INT2_OD     = 0x40,  ///< Open drain enable for INT2 pin.
    INT_OUT_CTRL_INT1_OUT_EN = 0x08,  ///< Output enable for INT1 pin.
    INT_OUT_CTRL_INT1_OD     = 0x04,  ///< Open drain enable for INT1 pin.
};

/**
 * @brief Values for INT_MAP_1 register.
 */
enum INT_MAP_1
{
    INT_MAP_1_INT_1_FIFO_WATERMARK = 0x40,
    INT_MAP_1_INT_1_FIFO_FULL      = 0x20,
    INT_MAP_1_INT_2_FIFO_WATERMARK = 0x04,
    INT_MAP_1_INT_2_FIFO_FULL      = 0x02,
};

/**
 * @brief Values for INT_EN_1 register.
 */
enum INT_EN_1
{
    INT_EN_1_FIFO_WATERMARK = 0x40,  ///< Enables interrupt for FIFO watermark.
    INT_EN_1_FIFO_FULL      = 0x20,  ///< Enables interrupt for FIFO full.
};

/**
 * @brief Values for MAG_IF_0 register.
 */
enum MAG_IF_0
{
    MAG_IF_0_MANUAL  = 0x80,
    MAG_IF_0_BURST_1 = 0x00,  ///< 1 byte of burst operation.
    MAG_IF_0_BURST_2 = 0x01,  ///< 2 byte of burst operation.
    MAG_IF_0_BURST_6 = 0x02,  ///< 6 byte of burst operation.
    MAG_IF_0_BURST_8 = 0x03,  ///< 8 byte of burst operation.
};

/**
 * @brief Mask for PMU_STATUS register (Power Mode Unit).
 */
const uint8_t PMU_STATUS_ALL_MASK = 0x03 | 0x0C | 0x30;

/**
 * @brief Mask for PMU_STATUS register, normal status for all sensors.
 */
const uint8_t PMU_STATUS_ALL_NORMAL = 0x01 | 0x04 | 0x10;

/**
 * @brief This value indicates that the data in the FIFO stops prematurely.
 */
const uint8_t FIFO_STOP_BYTE = 128;

/**
 * @brief Mask for fifo header mode.
 */
const uint8_t FIFO_HEADER_MODE_MASK = 0xC0;

/**
 * @brief Mask for fifo header parm.
 */
const uint8_t FIFO_HEADER_PARM_MASK = 0x3C;

/**
 * @brief Mask for fifo header ext.
 */
const uint8_t FIFO_HEADER_EXT_MASK = 0x03;

/**
 * @brief Values for fifo header byte.
 */
enum FIFO_HEADER
{
    FIFO_HEADER_MODE_REGULAR = 0x80,
    FIFO_HEADER_MODE_CONTROL = 0x40,

    FIFO_HEADER_PARM_ACC_DATA = 0x04,
    FIFO_HEADER_PARM_GYR_DATA = 0x08,
    FIFO_HEADER_PARM_MAG_DATA = 0x10,

    FIFO_HEADER_PARM_SKIP       = 0x00,
    FIFO_HEADER_PARM_SENSORTIME = 0x04,
    FIFO_HEADER_PARM_CONFIG     = 0x08,
};

/**
 * @brief Values for mag RESET.
 */
enum MAG_RESET
{
    MAG_RESET_POWER_CONTROL = 0x01
};

/**
 * @brief Values for mag CONTROL.
 */
enum MAG_CONTROL
{
    MAG_CONTROL_SELF_TEST = 0x01,
    MAG_CONTROL_NORMAL    = 0x00,
    MAG_CONTROL_FORCED    = 0x02,
    MAG_CONTROL_SLEEP     = 0x06,
};

/**
 * @brief Raw struct, read directly from device.
 */
struct MagRaw
{
    int16_t x, y, z;
    uint16_t rhall;
};

/**
 * @brief Raw struct, read directly from device.
 */
struct GyrRaw
{
    int16_t x, y, z;
};

/**
 * @brief Raw struct, read directly from device.
 */
struct AccRaw
{
    int16_t x, y, z;
};

/**
 * @brief Struct holding trim data used for magnetomer compensation.
 */
struct TrimData
{
    int8_t dig_x1;
    int8_t dig_y1;
    int8_t dig_x2;
    int8_t dig_y2;
    uint16_t dig_z1;
    int16_t dig_z2;
    int16_t dig_z3;
    int16_t dig_z4;
    uint8_t dig_xy1;
    int8_t dig_xy2;
    uint16_t dig_xyz1;
};

/**
 * BMX160 Commands.
 */
enum class Cmd
{
    START_FOC = 0x03,  ///< Starts Fast Offset Calibration for acc and gyro.
    ACC_SET_PMU_MODE    = 0x10,  ///< Sets the PMU mode for the accelerometer.
    GYR_SET_PMU_MODE    = 0x14,  ///< Sets the PMU mode for the gyroscope.
    MAG_IF_SET_PMU_MODE = 0x18,  ///< Sets the PMU mode for the magnetometer.
    PROG_NVM            = 0xA0,  ///< Writes the NVM backed registers into NVM.
    FIFO_FLUSH          = 0xB0,  ///< Clears all data in the fifo.
    INT_RESET = 0xB1,  ///< Resets interrupt engine, INT_STATUS reg and int pin.
    SOFTRESET = 0xB6,  ///< Triggers a reset including a reboot.
    STEP_CNT_CLR = 0xB2,  ///< Triggers a reset of the step counter.
};

/**
 * BMX160 power modes.
 */
enum class PowerMode
{
    SUSPEND       = 0x00,
    NORMAL        = 0x01,
    LOW_POWER     = 0x02,
    FAST_START_UP = 0x03
};

/**
 * @brief Internal register definitions.
 */
enum Registers
{
    REG_CHIPID     = 0x00,
    REG_ERR        = 0x02,
    REG_PMU_STATUS = 0x03,

    REG_DATA = 0x04,

    REG_DATA_MAG = 0x04,
    REG_DATA_GYR = 0x0C,
    REG_DATA_ACC = 0x12,

    REG_SENSORTIME_0 = 0x18,
    REG_SENSORTIME_1 = 0x19,
    REG_SENSORTIME_2 = 0x1A,

    REG_STATUS = 0x1B,

    REG_TEMPERATURE_0 = 0x20,
    REG_TEMPERATURE_1 = 0x21,

    REG_FIFO_LENGTH_0 = 0x22,
    REG_FIFO_LENGTH_1 = 0x23,

    REG_FIFO_DATA = 0x24,

    REG_ACC_CONF  = 0x40,
    REG_ACC_RANGE = 0x41,
    REG_GYR_CONF  = 0x42,
    REG_GYR_RANGE = 0x43,
    REG_MAG_CONF  = 0x44,

    REG_FIFO_DOWNS    = 0x45,
    REG_FIFO_CONFIG_0 = 0x46,
    REG_FIFO_CONFIG_1 = 0x47,

    REG_MAG_IF_0 = 0x4C,
    REG_MAG_IF_1 = 0x4D,
    REG_MAG_IF_2 = 0x4E,
    REG_MAG_IF_3 = 0x4F,

    REG_INT_EN_0 = 0x50,
    REG_INT_EN_1 = 0x51,
    REG_INT_EN_2 = 0x52,

    REG_INT_OUT_CTRL = 0x53,
    REG_INT_LATCH    = 0x54,

    REG_INT_MAP_0 = 0x55,
    REG_INT_MAP_1 = 0x56,
    REG_INT_MAP_2 = 0x57,

    // INT_DATA_[0-1] not needed
    // INT_LOWHIGH_[0-5] not needed
    // INT_MOTION_[0-3] not needed
    // INT_TAP_[0-1] not needed
    // INT_ORIENT_[0-1] not needed
    // INT_FLAT_[0-1] not needed

    REG_FOC_CONF = 0x69,
    REG_CONF     = 0x6A,
    REG_IF_CONF  = 0x6B,

    // PMU_TRIGGER not needed

    REG_SELF_TEST = 0x6D,

    // NV_CONF not needed
    // OFFSET_[0-6] not needed
    // STEP_CNT_[0-1] not needed
    // STEP_CONF_[0-1] not needed

    REG_CMD       = 0x7E,
    REG_COMM_TEST = 0x7F,
};

/**
 * @brief Internal magnetometer definitions.
 */
enum MagnetometerRegisters
{
    MAG_REG_DATA    = 0x42,
    MAG_REG_RESET   = 0x4B,
    MAG_REG_CONTROL = 0x4C,

    MAG_REG_REPXY = 0x51,
    MAG_REG_REPZ  = 0x52,

    // Factory calibrated trim registers. This is all undocumented territory
    // (the datasheet mentions this as "reserved").

    MAG_REG_DIG_X1     = 0x5D,
    MAG_REG_DIG_Y1     = 0x5E,
    MAG_REG_DIG_Z4_0   = 0x62,
    MAG_REG_DIG_Z4_1   = 0x63,
    MAG_REG_DIG_X2     = 0x64,
    MAG_REG_DIG_Y2     = 0x65,
    MAG_REG_DIG_Z2_0   = 0x68,
    MAG_REG_DIG_Z2_1   = 0x69,
    MAG_REG_DIG_Z1_0   = 0x6A,
    MAG_REG_DIG_Z1_1   = 0x6B,
    MAG_REG_DIG_XYZ1_0 = 0x6C,
    MAG_REG_DIG_XYZ1_1 = 0x6D,
    MAG_REG_DIG_Z3_0   = 0x6E,
    MAG_REG_DIG_Z3_1   = 0x6F,
    MAG_REG_DIG_XY2    = 0x70,
    MAG_REG_DIG_XY1    = 0x71,
};

}  // namespace BMX160Defs