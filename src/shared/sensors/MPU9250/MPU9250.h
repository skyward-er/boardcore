/* Copyright (c) 2021 Skyward Experimental Rocketry
 * Authors: Alberto Nidasio
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

#pragma once

#include "MPU9250Data.h"
#include "drivers/spi/SPIDriver.h"
#include "sensors/Sensor.h"

/**
 * @brief Driver class for MPU9250
 */
class MPU9250 : public Sensor<MPU9250Data>
{
public:
    enum MPU9250GyroFSR : uint8_t
    {
        GYRO_FSR_250DPS  = 0x0,
        GYRO_FSR_500DPS  = 0x08,
        GYRO_FSR_1000DPS = 0x10,
        GYRO_FSR_2000DPS = 0x18
    };

    enum MPU9250AccelFSR : uint8_t
    {
        ACCEL_FSR_2G  = 0x0,
        ACCEL_FSR_4G  = 0x08,
        ACCEL_FSR_8G  = 0x10,
        ACCEL_FSR_16G = 0x18
    };

    enum MPU9250I2CMasterInterfaceClock : uint8_t
    {
        I2C_MST_IF_CLK_348 = 0x0,
        I2C_MST_IF_CLK_333 = 0x1,
        I2C_MST_IF_CLK_320 = 0x2,
        I2C_MST_IF_CLK_308 = 0x3,
        I2C_MST_IF_CLK_296 = 0x4,
        I2C_MST_IF_CLK_286 = 0x5,
        I2C_MST_IF_CLK_276 = 0x6,
        I2C_MST_IF_CLK_267 = 0x7,
        I2C_MST_IF_CLK_258 = 0x8,
        I2C_MST_IF_CLK_500 = 0x9,
        I2C_MST_IF_CLK_471 = 0xA,
        I2C_MST_IF_CLK_444 = 0xB,
        I2C_MST_IF_CLK_421 = 0xC,
        I2C_MST_IF_CLK_400 = 0xD,
        I2C_MST_IF_CLK_381 = 0xE,
        I2C_MST_IF_CLK_364 = 0xF
    };

    union MPU9250RawData
    {
        struct __attribute__((packed)) MPU9250RawDataBits
        {
            int16_t accelX;
            int16_t accelY;
            int16_t accelZ;
            int16_t temp;
            int16_t gyroX;
            int16_t gyroY;
            int16_t gyroZ;
            uint8_t magSt1;
            int16_t magX;
            int16_t magY;
            int16_t magZ;
        } bits;
        int8_t bytes[21];
    };

    enum MPU9250Registers : uint8_t
    {
        REG_SMPLRT_DIV       = 0x19,
        REG_CONFIG           = 0x1A,
        REG_GYRO_CONFIG      = 0x1B,
        REG_ACCEL_CONFIG     = 0x1C,
        REG_FIFO_EN          = 0x23,
        REG_I2C_MST_CTRL     = 0x24,
        REG_I2C_SLV0_ADDR    = 0x25,
        REG_I2C_SLV0_REG     = 0x26,
        REG_I2C_SLV0_CTRL    = 0x27,
        REG_I2C_SLV1_ADDR    = 0x28,
        REG_I2C_SLV1_REG     = 0x29,
        REG_I2C_SLV1_CTRL    = 0x2A,
        REG_I2C_SLV2_ADDR    = 0x2B,
        REG_I2C_SLV2_REG     = 0x2C,
        REG_I2C_SLV2_CTRL    = 0x2D,
        REG_I2C_SLV3_ADDR    = 0x2E,
        REG_I2C_SLV3_REG     = 0x2F,
        REG_I2C_SLV3_CTRL    = 0x30,
        REG_I2C_SLV4_ADDR    = 0x31,
        REG_I2C_SLV4_REG     = 0x32,
        REG_I2C_SLV4_DO      = 0x33,
        REG_I2C_SLV4_CTRL    = 0x34,
        REG_INT_PIN_CFG      = 0x37,
        REG_INT_ENABLE       = 0x38,
        REG_INT_STATUS       = 0x3A,
        REG_ACCEL_XOUT_H     = 0x3B,
        REG_ACCEL_XOUT_L     = 0x3C,
        REG_ACCEL_YOUT_H     = 0x3D,
        REG_ACCEL_YOUT_L     = 0x3E,
        REG_ACCEL_ZOUT_H     = 0x3F,
        REG_ACCEL_ZOUT_L     = 0x40,
        REG_TEMP_OUT_H       = 0x41,
        REG_TEMP_OUT_L       = 0x42,
        REG_GYRO_XOUT_H      = 0x43,
        REG_GYRO_XOUT_L      = 0x44,
        REG_GYRO_YOUT_H      = 0x45,
        REG_GYRO_YOUT_L      = 0x46,
        REG_GYRO_ZOUT_H      = 0x47,
        REG_GYRO_ZOUT_L      = 0x48,
        REG_EXT_SENS_DATA_00 = 0x49,
        REG_I2C_SLV0_DO      = 0x63,
        REG_I2C_SLV1_DO      = 0x64,
        REG_I2C_SLV2_DO      = 0x65,
        REG_I2C_SLV3_DO      = 0x66,
        REG_MST_DELAY_CTRL   = 0x67,
        REG_USER_CTRL        = 0x6A,
        REG_PWR_MGMT_1       = 0x6B,
        REG_PWR_MGMT_2       = 0x6C,
        REG_WHO_AM_I         = 0x75
    };

    enum AK8963Registers : uint8_t
    {
        AK8963_REG_WHO_AM_I = 0x0,
        AK8963_REG_ST1      = 0x2,
        AK8963_REG_HXL      = 0x3,
        AK8963_REG_HXH      = 0x4,
        AK8963_REG_HYL      = 0x5,
        AK8963_REG_HYH      = 0x6,
        AK8963_REG_HZL      = 0x7,
        AK8963_REG_HZH      = 0x8,
        AK8963_REG_ST2      = 0x9,
        AK8963_REG_CNTL1    = 0xA,
        AK8963_REG_CNTL2    = 0xB,
        AK8963_REG_ASAX     = 0x10,
        AK8963_REG_ASAY     = 0x11,
        AK8963_REG_ASAZ     = 0x12
    };

    // Registers bits
    static constexpr uint8_t REG_CONFIG_DLPF_CFG_1      = 0x1;
    static constexpr uint8_t REG_USER_CTRL_I2C_MST_EN   = 0x20;
    static constexpr uint8_t REG_I2C_SLV_CTRL_EN        = 0x80;
    static constexpr uint8_t REG_PWR_MGMT_1_BIT_H_RESET = 0x80;
    static constexpr uint8_t REG_PWR_MGMT_1_CLKSEL_AUTO = 0x1;
    static constexpr uint8_t REG_WHO_AM_I_VAL           = 0x71;

    // AK8963 address and bits
    static constexpr uint8_t AK8963_ADDR                      = 0xC;
    static constexpr uint8_t AK8963_REG_WHO_AM_I_VAL          = 0x48;
    static constexpr uint8_t AK8963_REG_CNTL1_POWER_DOWN_MODE = 0x0;
    static constexpr uint8_t AK8963_REG_CNTL1_SINGLE_MES_MODE = 0x11;  // 16 bit
    static constexpr uint8_t AK8963_REG_CNTL1_CONT_MES_MODE_1 = 0x12;  // 16 bit
    static constexpr uint8_t AK8963_REG_CNTL1_CONT_MES_MODE_2 = 0x16;  // 16 bit
    static constexpr uint8_t AK8963_REG_CNTL1_SELF_TEST_MODE  = 0x8;
    static constexpr uint8_t AK8963_REG_CNTL1_FUSE_ROM_ACCESS_MODE = 0xF;
    static constexpr uint8_t AK8963_REG_CNTL1_BIT_16_BIT_OUT       = 0xF;
    static constexpr uint8_t AK8963_REG_CNTL2_BIT_SRST             = 0x1;

    const float ACCELERATION_FS_MAP[4] = {2.0, 4.0, 8.0, 16.0};
    const float GYROSCOPE_FS_MAP[4]    = {250, 500, 1000, 2000};

    /**
     * @brief Instantiates the driver
     *
     * @param highSpeedSpiClockDivider_ Clocl diver for 20MHz SPI communication
     * with the device
     */
    MPU9250(SPISlave spiSlave_, unsigned short samplingRate_ = 100,
            MPU9250GyroFSR gyroFsr_                   = GYRO_FSR_250DPS,
            MPU9250AccelFSR accelFsr_                 = ACCEL_FSR_2G,
            SPIClockDivider highSpeedSpiClockDivider_ = SPIClockDivider::DIV4);

    /**
     * @brief Constructs the default config for SPI Bus.
     *
     * @returns the default SPIBusConfig
     */
    static SPIBusConfig getDefaultSPIConfig();

    /**
     * @brief Initialize the device
     *
     * The AK8963 is initialized such ad its data are read by the MPU9250 at
     * each sample cycle, the host only needs to communicate with the MPU9250
     */
    bool init() override;

    /**
     * @brief Self test
     *
     * @return True if everything ok
     */
    bool selfTest() override { return false; };

private:
    MPU9250Data sampleImpl() override;

    void resetDevice();

    void selectAutoClock();

    /**
     * @brief Set the gyro full-scale range.
     *
     * @param fsr Desired full-scale range
     */
    void setGyroFsr(MPU9250GyroFSR fs);

    /**
     * @brief Set the accel full-scale range.
     *
     * @param fsr Desired full-scale range
     */
    void setAccelFsr(MPU9250AccelFSR fs);

    /**
     * @brief Set sampling rate.
     *
     * Sampling rate must be between 4Hz and 1kHz.
     *
     * @param rate Desired sampling rate (Hz)
     */
    void setSampleRate(unsigned short rate);

    void enableMpuI2CMasterInterface();

    void setMpuI2CMasterInterfaceClock(MPU9250I2CMasterInterfaceClock clk);

    /**
     * @brief Setup the I2C slave (0-3) for read operations
     */
    void setI2CMasterSlaveRead(uint8_t addr, uint8_t reg, uint8_t nBytes = 1,
                               uint8_t slave = 0);

    /**
     * @brief Setup the I2C slave (0-3) for write operations
     */
    void setI2CMasterSlaveWrite(uint8_t addr, uint8_t reg, uint8_t data,
                                uint8_t slave = 0);

    /**
     * @brief Disables the I2C slave communication
     */
    void disableI2CMasterSlave(uint8_t slave = 0);

    /**
     * @brief Read a byte form the AK8963 throug the MPU9250 I2C master
     * interface.
     *
     * Requires that MPU I2C master interface to be enabled.
     */
    uint8_t readFromAk(uint8_t reg);

    /**
     * @brief Writes a byte to the AK8963 throug the MPU9250 I2C master
     * interface.
     *
     * Requires that MPU I2C master interface to be enabled.
     */
    void writeToAk(uint8_t reg, uint8_t data);

    /**
     * @brief Initialize the AK8963.
     *
     * Requires the MPU I2C master interface to be enabled.
     */
    bool initAk();

    /**
     * @brief Check the WHO AM I code from the device.
     *
     * @return true if the device is recognized
     */
    bool checkWhoAmI();

    /**
     * @brief Check the WHO AM I code for the AK8963.
     *
     * @return true if the device is recognized
     */
    bool checkAkWhoAmI();

    inline void writeSPIWithDelay(SPITransaction& transaction, uint8_t reg,
                                  uint8_t data);

    inline float normalizeAcceleration(int16_t rawValue);

    inline float normalizeTemperature(int16_t rawValue);

    inline float normalizeGyroscope(int16_t rawValue);

    inline float normalizeMagnetometer(int16_t rawValue, float adjustmentCoeff);

    SPISlave spiSlave;

    const unsigned short samplingRate;

    const MPU9250GyroFSR gyroFsr;
    const MPU9250AccelFSR accelFsr;
    float magSensAdjCoeff[3];  // Page 32 on datasheet

    const SPIClockDivider highSpeedSpiClockDivider;

    bool initialized = false;  // Whether the sensor has been initialized
};