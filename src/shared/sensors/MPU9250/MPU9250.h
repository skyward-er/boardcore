/* MPU9250 Driver
 *
 * Copyright (c) 2016,2019 Skyward Experimental Rocketry
 * Authors: Alain Carlucci, Nuno Barcellos(Magnetometer)
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

#ifndef MPU9250_H
#define MPU9250_H
#include <drivers/BusTemplate.h>
#include "../Sensor.h"

// TODO: fix normalizeTemp() (is /512.0f correct?)
// TODO: Self-Test
template <typename Bus>
class MPU9250 : public GyroSensor,
                public AccelSensor,
                public CompassSensor,
                public TemperatureSensor
{

#pragma pack(1)
    // __extension__ is needed to prevent compiler warnings for anonymous
    // structs.
    typedef union {
        __extension__ struct
        {
            int16_t mag[3];
        };
        uint8_t raw[6];
    } akdata_t;

    typedef union {
        __extension__ struct
        {
            int16_t accel[3];
            int16_t temp;
            int16_t gyro[3];
        };
        int16_t buf[8];
    } mpudata_t;
#pragma pack()

public:
    MPU9250(uint8_t accelFullScale, uint8_t gyroFullScale)
    {
        accelFS         = accelFullScale & 0x03;
        gyroFS          = gyroFullScale & 0x03;
        magnetoFSMState = 0;
        mLastTemp       = 0.0f;
    }

    ~MPU9250() {}

    std::vector<SPIRequest> buildDMARequest() override
    {
        // clang-format off
        std::vector<uint8_t> v = 
        { 
            (REG_ACCEL_XOUT_H | 0x80),
            0,0,0,0,0,0, // accel
            0,0,         // temp
            0,0,0,0,0,0, // gyro
        };
        // clang-format on

        return {SPIRequest(0, Bus::getCSPin(), v)};
    }

    /*
     * Only reads accelerometer, gyro and temp
     */
    void onDMAUpdate(const SPIRequest &req) override
    {
        const auto &r = req.readResponseFromPeripheral();
        mpudata_t raw_data;

        memcpy(&raw_data.buf, &(r[1]), sizeof(raw_data));
        for (int i = 0; i < 3; i++)
        {
            raw_data.accel[i] = fromBigEndian16(raw_data.accel[i]);
            raw_data.gyro[i]  = fromBigEndian16(raw_data.gyro[i]);
        }
        raw_data.temp = fromBigEndian16(raw_data.temp);

        mLastAccel.setX((normalizeAccel(raw_data.accel[0])-_axb));
        mLastAccel.setY((normalizeAccel(raw_data.accel[1])-_ayb));
        mLastAccel.setZ((normalizeAccel(raw_data.accel[2])-_azb));

        mLastGyro.setX(normalizeGyro(raw_data.gyro[0]));
        mLastGyro.setY(normalizeGyro(raw_data.gyro[1]));
        mLastGyro.setZ(normalizeGyro(raw_data.gyro[2]));

        mLastTemp = normalizeTemp(raw_data.temp);
    }

    bool init() override
    {
        uint8_t whoami = Bus::read(REG_WHO_AM_I);
        // printf("MPU whoami: expected %x actual %x\n", who_am_i_value_mpu, whoami);
        if (whoami != who_am_i_value_mpu)
        {
            last_error = ERR_NOT_ME;
            return false;
        }
        int i = 0;
        while(!initMagneto() && i < 10)
        {
            ++i;
            Thread::sleep(5);
        }

        // Initialize MPU9250
        // clang-format off
        uint8_t init_data[][2] = 
        {
            {REG_PWR_MGMT_1,     0x01}, // Clock Source
            {REG_PWR_MGMT_2,     0x00}, // Enable all sensors
            {REG_CONFIG,         0x00}, // DLPF_CFG = xxxxx000
            {REG_SMPLRT_DIV,     0x00}, // Do not divide
            {REG_GYRO_CONFIG,    (uint8_t) (0x03 | ((gyroFS  & 3) << 3))},
            {REG_ACCEL_CONFIG,   (uint8_t) (0x00 | ((accelFS & 3) << 3))},
            {REG_ACCEL_CONFIG2,  0x08}, // FCHOICE = 1, A_DLPF_CFG = 000
        };
        // clang-format on

        for (size_t i = 0; i < sizeof(init_data) / sizeof(init_data[0]); i++)
        {
            Bus::write(init_data[i][0], init_data[i][1]);
            miosix::Thread::sleep(10);
        }

        return true;
    }

    /* Try to initialize AK8963 for 10 times because it doesn't always works */
    bool initMagneto()
    { 
        for (size_t i = 0; i < 10; i++)
        {
            // clang-format off
            uint8_t init_data[][2] = 
            {
                {REG_PWR_MGMT_1,     0x80}, // Reset Device
                {REG_INT_PIN_CFG,    0x16},

                // I2C (MPU9250 <-> AK8963)
                {REG_USER_CTRL,      0x20}, // I2C Master mode

                {REG_I2C_MST_CTRL,   0x48}, // I2C configuration multi-master  IIC at 258hz
                
                {REG_I2C_SLV0_ADDR,  AK8963_I2C_ADDR}, // Set the I2C slave addres of AK8963 and set for write.
                {REG_I2C_SLV0_REG,   AK8963_CNTL2}, // I2C slave 0 register address from where to begin data transfer
                {REG_I2C_SLV0_DO,    0x01}, // Reset AK8963
                {REG_I2C_SLV0_CTRL,  0x81}, // Enable I2C and set 1 byte

                {REG_I2C_SLV0_REG,   AK8963_CNTL1}, // I2C slave 0 register address from where to begin data transfer
                {REG_I2C_SLV0_DO,    0x16}, // Register value to 100Hz continuous measurement in 16bit
                // {REG_I2C_SLV0_DO,    0x12}, // Register value to 8Hz continuous measurement in 16bit
                {REG_I2C_SLV0_CTRL,  0x81}, //Enable I2C and set 1 byte
            };
            // clang-format on

            for (size_t i = 0; i < sizeof(init_data) / sizeof(init_data[0]); i++)
            {
                Bus::write(init_data[i][0], init_data[i][1]);
                miosix::Thread::sleep(10); // It won't work without this delay
            }

            uint8_t ak_wia = akReadWhoAmI();
            // printf("AK whoami: expected %x actual %x\n", who_am_i_value_ak, ak_wia);
            if (ak_wia == who_am_i_value_ak)
            {
                magnetoFSMState = 1;
                return true;
            }
        }

        // Reset the device if magnetometer initialization was unsuccessfull.
        Bus::write(REG_PWR_MGMT_1,0x80); // Reset Device
        miosix::Thread::sleep(10);

        return false;
    }

    bool onSimpleUpdate() override
    {
        uint8_t buf[14];
        mpudata_t raw_data;

        Bus::read(REG_ACCEL_XOUT_H, buf, 14);

        memcpy(&raw_data.buf, buf, 14);
        for (int i = 0; i < 3; i++)
        {
            raw_data.accel[i] = fromBigEndian16(raw_data.accel[i]);
            raw_data.gyro[i]  = fromBigEndian16(raw_data.gyro[i]);
        }
        raw_data.temp = fromBigEndian16(raw_data.temp);

        mLastAccel.setX((normalizeAccel(raw_data.accel[0])-_axb));
        mLastAccel.setY((normalizeAccel(raw_data.accel[1])-_ayb));
        mLastAccel.setZ((normalizeAccel(raw_data.accel[2])-_azb));

        mLastGyro.setX(normalizeGyro(raw_data.gyro[0]));
        mLastGyro.setY(normalizeGyro(raw_data.gyro[1]));
        mLastGyro.setZ(normalizeGyro(raw_data.gyro[2]));

        mLastTemp = normalizeTemp(raw_data.temp);

        return true;
    }

    bool updateMagneto()
    {
        /* Magnetometer FSM ( Board <-SPI-> MPU9250 <-I2C-> Magneto )
         *              ______________
         *   __________/            __v________
         *  / State 1  \           /  State 2  \
         * |  ReadI2C  |          |  CopyToMem |
         * \__________/           \___________/
         *         ^_______________/
         *
         */
        switch (magnetoFSMState)
        {
            case 1:  // ReadI2C
                akReadData1();
                magnetoFSMState = 2;
                break;
            case 2:  // CopyToMem
                akReadData2();
                magnetoFSMState = 1;
                break;
            default:
                return false;
        }
        return true;
    }

    /* Finds offset bias for the accelerometer*/
    void calibrateAccel()
    {
        accelFS = 0;
        Bus::write(REG_ACCEL_CONFIG, accelFS);  // setting the accel range to 2G
        Bus::write(REG_ACCEL_CONFIG2, 0x04);    // setting accel bandwidth to 20Hz
        Bus::write(REG_SMPLRT_DIV,19);          // setting the sample rate divider to 19
        
        // accel bias and scale factor estimation
        double _axbD = 0;
        double _aybD = 0;
        double _azbD = 0;

        for (size_t i = 0; i < _numSamples; i++)
        {
            onSimpleUpdate();
            _axbD += (mLastAccel.getX() + _axb)/((double)_numSamples);
            _aybD += (mLastAccel.getY() + _ayb)/((double)_numSamples);
            _azbD += (mLastAccel.getZ() + _azb)/((double)_numSamples);
            miosix::Thread::sleep(10);
        }

        _axb = _axbD;
        _ayb = _aybD;
        _azb = _azbD + EARTH_GRAVITY;
    }

    void getAccelCalibParams(float* bias)
    {
        bias[0] = _axb;
        bias[1] = _ayb;
        bias[2] = _azb;
    }

    void setAccelCalibParams(float* bias)
    {
        _axb = bias[0];
        _ayb = bias[1];
        _azb = bias[2];
    }

    bool selfTest() override
    {
        return true;
    }

    // clang-format off
    enum gyroFullScale
    {
        GYRO_FS_250           = 0,
        GYRO_FS_500           = 1,
        GYRO_FS_1000          = 2,
        GYRO_FS_2000          = 3
    };

    enum accelFullScale
    {
        ACC_FS_2G             = 0,
        ACC_FS_4G             = 1,
        ACC_FS_8G             = 2,
        ACC_FS_16G            = 3
    };
    // clang-format on

private:
    constexpr static uint8_t who_am_i_value_mpu = 0x71;
    constexpr static uint8_t who_am_i_value_ak = 0x48;
    constexpr static float accelFSMAP[]     = {2.0, 4.0, 8.0, 16.0};
    constexpr static float gyroFSMAP[]      = {250, 500, 1000, 2000};

    uint8_t gyroFS;
    uint8_t accelFS;
    uint8_t magnetoFSMState;

    // accel bias and scale factor estimation
    size_t _numSamples = 1000;
    float _axb = 0.0f; // offset bias
    float _ayb = 0.0f;
    float _azb = 0.0f;

    inline float normalizeAccel(int16_t val)
    {
        return static_cast<float>(val) / 32768.0f * accelFSMAP[accelFS] *
               EARTH_GRAVITY;
    }

    inline float normalizeGyro(int16_t val)
    {
        return static_cast<float>(val) / 32768.0f * gyroFSMAP[gyroFS] *
               DEGREES_TO_RADIANS;
    }

    inline float normalizeMagneto(int16_t val)
    {
        // Page 50 @ Register Map document
        return static_cast<float>(val) / 32760.0f * 4912.0f;
    }

    inline float normalizeTemp(int16_t val)
    {
        // Page 33 @ Register Map Document
        return static_cast<float>(val) / 512.0f + 21.0f;
    }


    uint8_t akReadWhoAmI()
    {
        // clang-format off
        uint8_t regs[][2] =
        {
            { REG_I2C_SLV0_ADDR,    (uint8_t)(AK8963_I2C_ADDR | 0x80)},
            { REG_I2C_SLV0_REG,     AK8963_WIA},
            { REG_I2C_SLV0_CTRL,    0x81 }, //Read 1 byte from the magnetometer
        };
        // clang-format on

        for (size_t i = 0; i < sizeof(regs) / sizeof(regs[0]); i++)
            Bus::write(regs[i][0], regs[i][1]);

        miosix::Thread::sleep(2);

        uint8_t ret = Bus::read(REG_EXT_SENS_DATA_00|0x80);
        return ret;
    }

    void akReadData()
    {
        // clang-format off
        uint8_t regs[][2] =
        {
            { REG_I2C_SLV0_ADDR,    (uint8_t)(AK8963_I2C_ADDR | 0x80)},
            { REG_I2C_SLV0_REG,     AK8963_HXL},
            { REG_I2C_SLV0_CTRL,    0x87}, // Read 6 bytes from the magnetometer
        };
        // clang-format on

        for (size_t i = 0; i < sizeof(regs) / sizeof(regs[0]); i++)
            Bus::write(regs[i][0], regs[i][1]);

        miosix::Thread::sleep(2);

        akdata_t ak;

        Bus::read(REG_EXT_SENS_DATA_00, ak.raw, sizeof(ak.raw));

        ak.mag[0] = (ak.raw[1] << 8) | ak.raw[0];
        ak.mag[1] = (ak.raw[3] << 8) | ak.raw[2];
        ak.mag[2] = (ak.raw[5] << 8) | ak.raw[4];

        mLastCompass.setX(normalizeMagneto(ak.mag[0]));
        mLastCompass.setY(normalizeMagneto(ak.mag[1]));
        mLastCompass.setZ(normalizeMagneto(ak.mag[2]));
    }

    // akReadData11 tells to the i2c master to begin reading and.. (akReadData2)
    void akReadData1()
    {
        // clang-format off
        uint8_t regs[][2] =
        {
            { REG_I2C_SLV0_ADDR,    (uint8_t)(AK8963_I2C_ADDR | 0x80)},
            { REG_I2C_SLV0_REG,     AK8963_HXL},
            { REG_I2C_SLV0_CTRL,    0x87}, // Read 6 bytes from the magnetometer
        };
        // clang-format on

        for (size_t i = 0; i < sizeof(regs) / sizeof(regs[0]); i++)
            Bus::write(regs[i][0], regs[i][1]);
    }

    // akReadData2 this one moves the data from spi to the local processor   
    void akReadData2()
    {
        akdata_t ak;

        Bus::read(REG_EXT_SENS_DATA_00, ak.raw, sizeof(ak.raw));

        ak.mag[0] = (ak.raw[1] << 8) | ak.raw[0];
        ak.mag[1] = (ak.raw[3] << 8) | ak.raw[2];
        ak.mag[2] = (ak.raw[5] << 8) | ak.raw[4];

        mLastCompass.setX(normalizeMagneto(ak.mag[0]));
        mLastCompass.setY(normalizeMagneto(ak.mag[1]));
        mLastCompass.setZ(normalizeMagneto(ak.mag[2]));
    }

    // clang-format off
    enum magnetoMap
    {
        AK8963_I2C_ADDR     = 0x0c,

        AK8963_WIA          = 0x00,
        AK8963_STATUS1      = 0x02,
        AK8963_HXL          = 0x03,
        AK8963_CNTL1        = 0x0A,
        AK8963_CNTL2        = 0x0B,
    };

    enum regMap
    {
        REG_ST_GYRO         = 0x00,
        REG_ST_ACCEL        = 0x0D,

        REG_SMPLRT_DIV      = 0x19,
        REG_CONFIG          = 0x1A,
        REG_GYRO_CONFIG     = 0x1B,
        REG_ACCEL_CONFIG    = 0x1C,
        REG_ACCEL_CONFIG2   = 0x1D,

        REG_I2C_MST_CTRL    = 0x24,
        REG_I2C_SLV0_ADDR   = 0x25,
        REG_I2C_SLV0_REG    = 0x26,
        REG_I2C_SLV0_CTRL   = 0x27,
        REG_I2C_SLV0_DO     = 0x63,

        REG_I2C_SLV4_ADDR   = 0x31,
        REG_I2C_SLV4_REG    = 0x32,
        REG_I2C_SLV4_DO     = 0x33,
        REG_I2C_SLV4_CTRL   = 0x34,
        REG_I2C_SLV4_DI     = 0x35,
        REG_I2C_MST_STATUS  = 0x36,

        REG_ACCEL_XOUT_H    = 0x3b,

        REG_INT_PIN_CFG     = 0x37,
        REG_INT_ENABLE      = 0x38,
        REG_INT_STATUS      = 0x3A,

        REG_EXT_SENS_DATA_00= 0x49,

        REG_USER_CTRL       = 0x6A,
        REG_PWR_MGMT_1      = 0x6B,
        REG_PWR_MGMT_2      = 0x6C,
        REG_WHO_AM_I        = 0x75
    };
    // clang-format on

    void readRAWData(mpudata_t &out)
    {
        Bus::read(REG_ACCEL_XOUT_H, reinterpret_cast<uint8_t *>(out.buf),
                  sizeof(out.buf));

        // BigEndian -> CPUArch (LittleEndian as usual)
        for (size_t i = 0; i < (sizeof(out.buf) / sizeof(out.buf[0])); i++)
            out.buf[i] = fromBigEndian16(out.buf[i]);
    }
};

template <typename Bus>
constexpr float MPU9250<Bus>::accelFSMAP[];

template <typename Bus>
constexpr float MPU9250<Bus>::gyroFSMAP[];

#endif /* ifndef MPU9250 */
