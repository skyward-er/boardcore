/* Copyright (c) 2021 Skyward Experimental Rocketry
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

#include "MPU9250.h"

#include <drivers/timer/TimestampTimer.h>
#include <interfaces/endianness.h>

#include "Constants.h"

namespace Boardcore
{

MPU9250::MPU9250(SPISlave spiSlave_, unsigned short samplingRate_,
                 MPU9250GyroFSR gyroFsr_, MPU9250AccelFSR accelFsr_,
                 SPI::ClockDivider highSpeedSpiClockDivider_)
    : spiSlave(spiSlave_), samplingRate(samplingRate_), gyroFsr(gyroFsr_),
      accelFsr(accelFsr_), highSpeedSpiClockDivider(highSpeedSpiClockDivider_)
{
}

SPIBusConfig MPU9250::getDefaultSPIConfig()
{
    SPIBusConfig spiConfig{};
    spiConfig.clockDivider = SPI::ClockDivider::DIV_64;
    spiConfig.mode         = SPI::Mode::MODE_3;
    return spiConfig;
}

bool MPU9250::init()
{
    // Check if already initialized
    if (initialized)
    {
        LOG_ERR(logger, "Already initialized\n");

        last_error = SensorErrors::ALREADY_INIT;

        return false;
    }

    // Reset the device
    resetDevice();
    miosix::Thread::sleep(100);

    // Wake up the chip
    selectAutoClock();

    // Check WHO AM I
    if (!checkWhoAmI())
    {
        LOG_ERR(logger, "Invalid WHO AM I\n");

        last_error = SensorErrors::INVALID_WHOAMI;

        return false;
    }

    // Setup I2C master interface to communicate with the AK8963
    enableMpuI2CMasterInterface();
    setMpuI2CMasterInterfaceClock(I2C_MST_IF_CLK_400);

    // Init the AK8963
    if (!initAk())
    {
        return false;
    }

    // Set full scale resolution for gyroscope and accelerometer (they are
    // enabled by default)
    setGyroFsr(gyroFsr);
    setAccelFsr(accelFsr);

    // Set the sample rate
    setSampleRate(samplingRate);

    LOG_DEBUG(logger, "Magnetometer sensitivity adjustment: %d, %d, %d\n",
              magSensAdjCoeff[0], magSensAdjCoeff[1], magSensAdjCoeff[2]);

    initialized = true;
    return true;
}

MPU9250Data MPU9250::sampleImpl()
{
    MPU9250RawData rawData;
    MPU9250Data data;
    SPI::ClockDivider clockDivider = spiSlave.config.clockDivider;

    // Read the data registers at high speed (up to 20MHz)
    spiSlave.config.clockDivider = highSpeedSpiClockDivider;
    {
        SPITransaction transaction(spiSlave);
        transaction.readRegisters(REG_ACCEL_XOUT_H, (uint8_t*)rawData.bytes,
                                  sizeof(MPU9250RawData));
    }
    spiSlave.config.clockDivider = clockDivider;

    // Save timestamps
    uint64_t timestamp   = TimestampTimer::getTimestamp();
    data.accel_timestamp = timestamp;
    data.temp_timestamp  = timestamp;
    data.gyro_timestamp  = timestamp;
    data.mag_timestamp   = timestamp;

    // Save data
    data.accel_x = normalizeAcceleration(swapBytes16(rawData.bits.accelX));
    data.accel_y = normalizeAcceleration(swapBytes16(rawData.bits.accelY));
    data.accel_z = normalizeAcceleration(swapBytes16(rawData.bits.accelZ));
    data.temp    = normalizeTemperature(swapBytes16(rawData.bits.temp));
    data.gyro_x  = normalizeGyroscope(swapBytes16(rawData.bits.gyroX));
    data.gyro_y  = normalizeGyroscope(swapBytes16(rawData.bits.gyroY));
    data.gyro_z  = normalizeGyroscope(swapBytes16(rawData.bits.gyroZ));
    data.mag_x   = normalizeMagnetometer(rawData.bits.magX, magSensAdjCoeff[0]);
    data.mag_y   = normalizeMagnetometer(rawData.bits.magY, magSensAdjCoeff[1]);
    data.mag_z   = normalizeMagnetometer(rawData.bits.magZ, magSensAdjCoeff[2]);

    return data;
}

void MPU9250::resetDevice()
{
    SPITransaction transaction(spiSlave);

    transaction.writeRegister(REG_PWR_MGMT_1, REG_PWR_MGMT_1_BIT_H_RESET);
}

void MPU9250::selectAutoClock()
{
    SPITransaction transaction(spiSlave);

    // Wake up by clearing sleep bit and set clock auto select
    writeSPIWithDelay(transaction, REG_PWR_MGMT_1, REG_PWR_MGMT_1_CLKSEL_AUTO);
}

void MPU9250::setGyroFsr(MPU9250GyroFSR fs)
{
    SPITransaction transaction(spiSlave);

    writeSPIWithDelay(transaction, REG_GYRO_CONFIG, fs);
}

void MPU9250::setAccelFsr(MPU9250AccelFSR fs)
{
    SPITransaction transaction(spiSlave);

    writeSPIWithDelay(transaction, REG_ACCEL_CONFIG, fs);
}

void MPU9250::setSampleRate(unsigned short rate)
{
    if (rate < 4)
        rate = 4;
    else if (rate > 1000)
        rate = 1000;

    uint8_t data = 1000 / rate - 1;

    SPITransaction transaction(spiSlave);

    writeSPIWithDelay(transaction, REG_SMPLRT_DIV, data);

    // Set the digital low pass filter in order to use the sample rate divider
    writeSPIWithDelay(transaction, REG_CONFIG, REG_CONFIG_DLPF_CFG_1);

    // We do not need to write Fchoise_b bits becouse they reset to 0 and they
    // are the inverse of Fchoise, see page 13 of MPU9250 register map
}

void MPU9250::enableMpuI2CMasterInterface()
{
    SPITransaction transaction(spiSlave);

    writeSPIWithDelay(transaction, REG_USER_CTRL, REG_USER_CTRL_I2C_MST_EN);
}

void MPU9250::setMpuI2CMasterInterfaceClock(MPU9250I2CMasterInterfaceClock clk)
{
    SPITransaction transaction(spiSlave);

    writeSPIWithDelay(transaction, REG_I2C_MST_CTRL, clk);
}

void MPU9250::setI2CMasterSlaveRead(uint8_t addr, uint8_t reg, uint8_t nBytes,
                                    uint8_t slave)
{
    uint8_t regSlvAddr, regSlvReg, regSlvCtrl;

    switch (slave)
    {
        case 0:
            regSlvAddr = REG_I2C_SLV0_ADDR;
            regSlvReg  = REG_I2C_SLV0_REG;
            regSlvCtrl = REG_I2C_SLV0_CTRL;
            break;
        case 1:
            regSlvAddr = REG_I2C_SLV1_ADDR;
            regSlvReg  = REG_I2C_SLV1_REG;
            regSlvCtrl = REG_I2C_SLV1_CTRL;
            break;
        case 2:
            regSlvAddr = REG_I2C_SLV2_ADDR;
            regSlvReg  = REG_I2C_SLV2_REG;
            regSlvCtrl = REG_I2C_SLV2_CTRL;
            break;
        case 3:
            regSlvAddr = REG_I2C_SLV3_ADDR;
            regSlvReg  = REG_I2C_SLV3_REG;
            regSlvCtrl = REG_I2C_SLV3_CTRL;
            break;

        default:
            LOG_ERR(logger,
                    "invalid slave parameter in function "
                    "setI2CMasterSlaveRead");
            return;
    }

    SPITransaction transaction(spiSlave);

    writeSPIWithDelay(transaction, regSlvAddr, addr | 0x80);  // Set read bit
    writeSPIWithDelay(transaction, regSlvReg, reg);

    // Set bytes number and enable the data transfer on each sample
    writeSPIWithDelay(transaction, regSlvCtrl, REG_I2C_SLV_CTRL_EN | nBytes);
}

void MPU9250::setI2CMasterSlaveWrite(uint8_t addr, uint8_t reg, uint8_t data,
                                     uint8_t slave)
{
    uint8_t regSlvAddr, regSlvReg, regSlvCtrl, regSlvDo;

    switch (slave)
    {
        case 0:
            regSlvAddr = REG_I2C_SLV0_ADDR;
            regSlvReg  = REG_I2C_SLV0_REG;
            regSlvCtrl = REG_I2C_SLV0_CTRL;
            regSlvDo   = REG_I2C_SLV0_DO;
            break;
        case 1:
            regSlvAddr = REG_I2C_SLV1_ADDR;
            regSlvReg  = REG_I2C_SLV1_REG;
            regSlvCtrl = REG_I2C_SLV1_CTRL;
            regSlvDo   = REG_I2C_SLV1_DO;
            break;
        case 2:
            regSlvAddr = REG_I2C_SLV2_ADDR;
            regSlvReg  = REG_I2C_SLV2_REG;
            regSlvCtrl = REG_I2C_SLV2_CTRL;
            regSlvDo   = REG_I2C_SLV2_DO;
            break;
        case 3:
            regSlvAddr = REG_I2C_SLV3_ADDR;
            regSlvReg  = REG_I2C_SLV3_REG;
            regSlvCtrl = REG_I2C_SLV3_CTRL;
            regSlvDo   = REG_I2C_SLV3_DO;
            break;

        default:
            LOG_ERR(logger,
                    "invalid slave parameter in function "
                    "setI2CMasterSlaveRead");
            return;
    }

    SPITransaction transaction(spiSlave);

    writeSPIWithDelay(transaction, regSlvAddr, addr);
    writeSPIWithDelay(transaction, regSlvReg, reg);
    writeSPIWithDelay(transaction, regSlvDo, data);

    // Enable the data transfer on each sample
    writeSPIWithDelay(transaction, regSlvCtrl, REG_I2C_SLV_CTRL_EN | 0x1);
}

void MPU9250::disableI2CMasterSlave(uint8_t slave)
{
    uint8_t regSlvCtrl;

    switch (slave)
    {
        case 0:
            regSlvCtrl = REG_I2C_SLV0_CTRL;
            break;
        case 1:
            regSlvCtrl = REG_I2C_SLV1_CTRL;
            break;
        case 2:
            regSlvCtrl = REG_I2C_SLV2_CTRL;
            break;
        case 3:
            regSlvCtrl = REG_I2C_SLV3_CTRL;
            break;
            // In the control register of slave 4 there is also the
            // configuration for the slaves sampling rate. If slave 4 is needed
            // the salmpling rate divider should not be changed case 4:
            //     regSlvCtrl = REG_I2C_SLV4_CTRL;
            //     break;

        default:
            LOG_ERR(logger,
                    "invalid slave parameter in function "
                    "setI2CMasterSlaveRead");
            return;
    }

    SPITransaction transaction(spiSlave);

    // Enable the data transfer on each sample
    writeSPIWithDelay(transaction, regSlvCtrl, 0);
}

uint8_t MPU9250::readFromAk(uint8_t reg)
{
    // Set the device address and register
    setI2CMasterSlaveRead(AK8963_ADDR, reg);

    // Wait for the sample
    miosix::Thread::sleep(1);

    SPITransaction transaction(spiSlave);

    return transaction.readRegister(REG_EXT_SENS_DATA_00);
}

void MPU9250::writeToAk(uint8_t reg, uint8_t data)
{
    // Set the device address, register and data byte
    setI2CMasterSlaveWrite(AK8963_ADDR, reg, data);
    miosix::Thread::sleep(1);
}

bool MPU9250::initAk()
{
    // Set the sample rate to 1KHz in order to communicate faster, but safely,
    // with the AK (if we use 8KHz the communication does not work properly)
    setSampleRate(1000);

    // Power down the magnetometer
    writeToAk(AK8963_REG_CNTL1, AK8963_REG_CNTL1_POWER_DOWN_MODE);

    // Wait a bit more, the first slave communication seams to be delayed by 1
    // sample
    miosix::Thread::sleep(1);

    // Reset
    writeToAk(AK8963_REG_CNTL2, AK8963_REG_CNTL2_BIT_SRST);

    // Check AK8963 WHO AM I
    if (!checkAkWhoAmI())
    {
        LOG_ERR(logger, "Invalid AK8963 WHO AM I\n");

        last_error = SensorErrors::INVALID_WHOAMI;

        return false;
    }

    // Enter rom access mode
    writeToAk(AK8963_REG_CNTL1, AK8963_REG_CNTL1_POWER_DOWN_MODE);
    writeToAk(AK8963_REG_CNTL1, AK8963_REG_CNTL1_FUSE_ROM_ACCESS_MODE);

    // Read magnetometer sensitivity adjustment data (page 53 of register map)
    magSensAdjCoeff[0] =
        (((readFromAk(AK8963_REG_ASAX) - 128) + .5) / 128 + 1) * 4912.0f /
        32760.0f;
    magSensAdjCoeff[1] =
        (((readFromAk(AK8963_REG_ASAY) - 128) + .5) / 128 + 1) * 4912.0f /
        32760.0f;
    magSensAdjCoeff[2] =
        (((readFromAk(AK8963_REG_ASAZ) - 128) + .5) / 128 + 1) * 4912.0f /
        32760.0f;

    // Set continuos measurement mode
    writeToAk(AK8963_REG_CNTL1, AK8963_REG_CNTL1_POWER_DOWN_MODE);
    writeToAk(AK8963_REG_CNTL1, AK8963_REG_CNTL1_CONT_MES_MODE_2);

    // Set I2C master slave communication on each sample
    setI2CMasterSlaveRead(AK8963_ADDR, AK8963_REG_ST1, 8);

    return true;
}

bool MPU9250::checkWhoAmI()
{
    SPITransaction transaction(spiSlave);

    uint8_t who_am_i_value = transaction.readRegister(REG_WHO_AM_I);

    return who_am_i_value == REG_WHO_AM_I_VAL;
}

bool MPU9250::checkAkWhoAmI()
{
    uint8_t who_am_i_value = readFromAk(AK8963_REG_WHO_AM_I);

    return who_am_i_value == AK8963_REG_WHO_AM_I_VAL;
}

void MPU9250::writeSPIWithDelay(SPITransaction& transaction, uint8_t reg,
                                uint8_t data)
{
    transaction.writeRegister(reg, data);
    miosix::delayUs(1);
    /*transaction.readRegister(reg);
    miosix::delayUs(1);*/
}

float MPU9250::normalizeAcceleration(int16_t rawValue)
{
    return static_cast<float>(rawValue) / 32768.0f *
           ACCELERATION_FS_MAP[accelFsr >> 3] * EARTH_GRAVITY;
}

// Page 33 of register map document
float MPU9250::normalizeTemperature(int16_t rawValue)
{
    return static_cast<float>(rawValue) / 512.0f + 21.0f;
}

float MPU9250::normalizeGyroscope(int16_t rawValue)
{
    return static_cast<float>(rawValue) / 32768.0f *
           GYROSCOPE_FS_MAP[gyroFsr >> 3] * DEGREES_TO_RADIANS;
}

float MPU9250::normalizeMagnetometer(int16_t rawValue, float adjustmentCoeff)
{
    // Page 50 and 53 of register map document
    return static_cast<float>(rawValue) * adjustmentCoeff;
}

}  // namespace Boardcore
