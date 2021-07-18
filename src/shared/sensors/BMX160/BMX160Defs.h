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

/// @brief Various BMX160 register/enums definitions.
namespace BMX160Defs
{

/// @brief Temperature sensor sensibility.
const float TEMP_SENSIBILITY = 64.0f / 32768.0f;
/// @brief Magnetometer fixed sensibility.
const float MAG_SENSIBILITY = 0.3f;

/// @brief BMX160 Chip Id.
const uint8_t CHIPID = 0xD8;

/// @brief Default value for FIFO_CONFIG_1.
const uint8_t FIFO_CONFIG_1_DEFAULT = 0x10;

/// @brief Values for SELF_TEST.
enum SELF_TEST
{
    SELF_TEST_GYR         = 1 << 4,
    SELF_TEST_ACC_AMP     = 1 << 3,
    SELF_TEST_ACC_SIGN    = 1 << 2,
    SELF_TEST_ACC_ENABLE  = 0x01,
    SELF_TEST_ACC_DISABLE = 0x00
};

/// @brief Values for FIFO_CONFIG_1.
enum FIGO_CONFIG_1
{
    FIFO_CONFIG_1_ACC_EN    = 1 << 7,
    FIFO_CONFIG_1_GYR_EN    = 1 << 6,
    FIFO_CONFIG_1_MAG_EN    = 1 << 5,
    FIFO_CONFIG_1_HEADER_EN = 1 << 4,
};

/// @brief Values for FIFO_DOWNS.
enum FIFO_DOWNS {
    FIFO_DOWNS_ACC_FILT_DATA = 1 << 7,
    FIFO_DOWNS_GYR_FILT_DATA = 1 << 3,
};

/// @brief Values for INT_OUT_CTRL.
enum INT_OUT_CTRL
{
    INT_OUT_CTRL_INT2_OUT = 1 << 7,
    INT_OUT_CTRL_INT1_OUT = 1 << 3,
};

/// @brief Values for INT_MAP_1.
enum INT_MAP_1
{
    INT_MAP_1_INT_1_FIFO_WATERMARK = 1 << 6,
    INT_MAP_1_INT_1_FIFO_FULL      = 1 << 5,
    INT_MAP_1_INT_2_FIFO_WATERMARK = 1 << 2,
    INT_MAP_1_INT_2_FIFO_FULL      = 1 << 1,
};

/// @brief Values for INT_EN_1.
enum INT_EN_1
{
    INT_EN_1_FIFO_WATERMARK = 1 << 6,
    INT_EN_1_FIFO_FULL      = 1 << 5,
};

/// @brief Values for MAG_IF_0.
enum MAG_IF_0
{
    MAG_IF_0_MANUAL  = 1 << 7,
    MAG_IF_0_BURST_1 = 0x00,
    MAG_IF_0_BURST_2 = 0x01,
    MAG_IF_0_BURST_6 = 0x02,
    MAG_IF_0_BURST_8 = 0x03,
};

/// @brief Mask for PMU_STATUS.
const uint8_t PMU_STATUS_ALL_MASK = (3 << 0) | (3 << 2) | (3 << 4);
/// @brief Value for PMU_STATUS all on normal status.
const uint8_t PMU_STATUS_ALL_NORMAL = (1 << 0) | (1 << 2) | (1 << 4);

/// @brief This value indicates that the data in the FIFO stops prematurely.
const uint8_t FIFO_STOP_BYTE = 128;

/// @brief Mask for fifo header mode.
const uint8_t FIFO_HEADER_MODE_MASK = 0x03 << 6;
/// @brief Mask for fifo header parm.
const uint8_t FIFO_HEADER_PARM_MASK = 0x0f << 2;
/// @brief Mask for fifo header ext.
const uint8_t FIFO_HEADER_EXT_MASK = 0x03 << 0;

/// @brief Values for fifo header byte.
enum FIFO_HEADER
{
    FIFO_HEADER_MODE_REGULAR = 0x02 << 6,
    FIFO_HEADER_MODE_CONTROL = 0x01 << 6,

    FIFO_HEADER_PARM_ACC_DATA = 1 << 2,
    FIFO_HEADER_PARM_GYR_DATA = 1 << 3,
    FIFO_HEADER_PARM_MAG_DATA = 1 << 4,

    FIFO_HEADER_PARM_SKIP       = 0 << 2,
    FIFO_HEADER_PARM_SENSORTIME = 1 << 2,
    FIFO_HEADER_PARM_CONFIG     = 2 << 2,
};

/// @brief Values for mag RESET.
enum MAG_RESET
{
    MAG_RESET_POWER_CONTROL = 0x01
};

/// @brief Values for mag CONTROL.
enum MAG_CONTROL
{
    MAG_CONTROL_SELF_TEST = 0x01,
    MAG_CONTROL_NORMAL    = 0x00 << 1,
    MAG_CONTROL_FORCED    = 0x01 << 1,
    MAG_CONTROL_SLEEP     = 0x03 << 1,
};

/// @brief Raw struct, read directly from device.
struct MagRaw
{
    int16_t x, y, z;
    uint16_t rhall;
};

/// @brief Raw struct, read directly from device.
struct GyrRaw
{
    int16_t x, y, z;
};

/// @brief Raw struct, read directly from device.
struct AccRaw
{
    int16_t x, y, z;
};

/// @brief Struct holding trim data used for magnetomer compensation
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

/// @brief Internal register definitions
enum Reg
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

    /* INT_DATA_[0-1] not needed */
    /* INT_LOWHIGH_[0-5] not needed */
    /* INT_MOTION_[0-3] not needed */
    /* INT_TAP_[0-1] not needed */
    /* INT_ORIENT_[0-1] not needed */
    /* INT_FLAT_[0-1] not needed */

    REG_FOC_CONF = 0x69,
    REG_CONF     = 0x6A,
    REG_IF_CONF  = 0x6B,

    /* PMU_TRIGGER not needed */

    REG_SELF_TEST = 0x6D,

    /* NV_CONF not needed */
    /* OFFSET_[0-6] not needed */
    /* STEP_CNT_[0-1] not needed */
    /* STEP_CONF_[0-1] not needed */

    REG_CMD       = 0x7E,
    REG_COMM_TEST = 0x7F,
};

/// @brief Internal magnetometer definitions
enum MagReg
{
    MAG_REG_DATA    = 0x42,
    MAG_REG_RESET   = 0x4B,
    MAG_REG_CONTROL = 0x4C,

    MAG_REG_REPXY = 0x51,
    MAG_REG_REPZ  = 0x52,

    // Factory calibrated trim registers.
    // This is all undocumented territory (the datasheet mentions this as
    // "reserved")

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