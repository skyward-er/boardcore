/* MAX9250 Driver 
 *
 * Copyright (c) 2016 Skyward Experimental Rocketry
 * Authors: Alain Carlucci
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

#ifndef MAX9250_H
#define MAX9250_H 
#include "Sensor.h"
#include <BusTemplate.h>

// TODO: fix normalizeTemp() (is /512.0f correct?)
// TODO: Self-Test
template <typename Bus>
class MPU9250 : public GyroSensor, public AccelSensor, 
                public CompassSensor, public TemperatureSensor 
{

#pragma pack(1)
typedef union {
    struct { 
        uint8_t status1;
        int16_t mag[3];
        uint8_t status2;
    };
    uint8_t raw[8];
} akdata_t;

typedef union {
    struct {
        int16_t accel[3];
        int16_t temp;
        int16_t gyro[3];
    };
    int16_t buf[7];
} mpudata_t;
#pragma pack()

public:
    MPU9250(uint8_t accelFullScale, uint8_t gyroFullScale)
    {
        accelFS = accelFullScale & 0x03;
        gyroFS = gyroFullScale & 0x03;
        magnetoFSMState = 0;
        mLastTemp = 0.0f;
    }

    ~MPU9250()
    {
    }

    Vec3* accelDataPtr() override { return &mLastAccel; }
    Vec3* gyroDataPtr()  override { return &mLastGyro; }
    Vec3* compassDataPtr() override { return &mLastCompass; }
    float* tempDataPtr() override { return &mLastTemp; }

    std::vector<SPIRequest> buildDMARequest() override 
    {
        printf("MPU9250::buildDMARequest()\n");
        std::vector<uint8_t> v = 
            { (REG_ACCEL_XOUT_H | 0x80), 0,0,0,0,0,0,0 };

        return { SPIRequest(0, Bus::getCSPin(), v) };
    }

    void onDMAUpdate(const SPIRequest& req) override 
    {
        printf("MPU9250::onDMAUpdate()\n");
        const auto& r = req.readResponseFromPeripheral();
        const mpudata_t* raw_data = (const mpudata_t*) r.data();

        mLastAccel.setX(normalizeAccel(raw_data->accel[0]));
        mLastAccel.setY(normalizeAccel(raw_data->accel[1]));
        mLastAccel.setZ(normalizeAccel(raw_data->accel[2]));

        mLastGyro.setX(normalizeGyro(raw_data->gyro[0]));
        mLastGyro.setY(normalizeGyro(raw_data->gyro[1]));
        mLastGyro.setZ(normalizeGyro(raw_data->gyro[2]));

        mLastTemp = normalizeTemp(raw_data->temp);
    }

    bool init() 
    {
        uint8_t whoami = Bus::read(REG_WHO_AM_I);

        if(whoami != who_am_i_value) {
            last_error = ERR_NOT_ME;
            return false;
        }

        // Warning: do not reset this sensor at startup, otherwise 
        // its magnetometer could randomly stop working.

        uint8_t init_data[][2] = 
        {
            {REG_PWR_MGMT_1,     0x01},
            {REG_PWR_MGMT_2,     0x00}, // Enable all sensors
            {REG_CONFIG,         0x00}, // DLPF_CFG = xxxxx000
            {REG_SMPLRT_DIV,     0x00}, // Do not divide
            {REG_GYRO_CONFIG,    (uint8_t) (0x03 | ((gyroFS  & 3) << 3))},
            {REG_ACCEL_CONFIG,   (uint8_t) (0x00 | ((accelFS & 3) << 3))},
            {REG_ACCEL_CONFIG2,  0x08}, // FCHOICE = 1, A_DLPF_CFG = 000
            {REG_INT_PIN_CFG,    0x30},
            {REG_INT_ENABLE,     0x00}, // No interrupts

            // I2C (MPU9250 <-> AK8963)
            {REG_USER_CTRL,      0x20}, // Master mode TODO: I2C_IF_DIS???
            {REG_I2C_MST_CTRL,   0x00}, // I2C @ 348khz(400 seems giving issues)
        }; 

        for(size_t i=0; i < sizeof(init_data)/sizeof(init_data[0]); i++)
            Bus::write(init_data[i][0], init_data[i][1]);

        uint8_t ak_wia = akReadReg(AK8963_WIA);

        if(ak_wia != 0x48) 
        {
            last_error = ERR_CANT_TALK_TO_CHILD; // TODO
            return false;
        }

        akWriteReg(AK8963_CNTL2, 0x01);
        Thread::sleep(1);
        akWriteReg(AK8963_CNTL1, 0x16);
        Thread::sleep(1);

        magnetoFSMState = 1;
        return true;
    }

    bool updateParams() 
    {
        akdata_t ak;

        /* Magnetometer FSM ( Board <-SPI-> MPU9250 <-I2C-> Magneto )
         *              ______________
         *   __________/            __v________
         *  / State 1  \           /  State 2  \
         * |  ReadI2C  |          |  CopyToMem |
         * \__________/           \___________/
         *         ^_______________/
         *
         */

        switch(magnetoFSMState)
        {
            case 1: // ReadI2C
                akReadReg_1(AK8963_STATUS1, sizeof(ak.raw));
                magnetoFSMState = 2;
                break;
            case 2: // CopyToMem
                akReadReg_2(ak.raw, sizeof(ak.raw));
                mLastCompass.setX(normalizeMagneto(ak.mag[0]));
                mLastCompass.setY(normalizeMagneto(ak.mag[1]));
                mLastCompass.setZ(normalizeMagneto(ak.mag[2]));
                magnetoFSMState = 1;
                break;
            default:
                return false;
        }

        return true;
    }

    bool selfTest() 
    {
        uint8_t st_gyro[3], st_accel[3];
        mpudata_t test, real;
        uint16_t cfg; // [ REG_GYRO_CONFIG REG_ACCEL_CONFIG ]

        readRAWData(real);
        Bus::read(REG_GYRO_CONFIG, 
                reinterpret_cast<uint8_t *>(&cfg), sizeof(cfg)); 

        // -- ENABLE SELF TEST --
        cfg |= 0xe0e0;
        Bus::write(REG_GYRO_CONFIG, 
                reinterpret_cast<uint8_t *>(&cfg), sizeof(cfg));

        // -- SELF TEST ROUTINE --
        Bus::read(REG_ST_GYRO, st_gyro, sizeof(st_gyro)); 
        Bus::read(REG_ST_ACCEL, st_accel, sizeof(st_accel)); 
        Thread::sleep(10);
        readRAWData(test);

        // -- DISABLE SELF TEST --
        cfg &= ~(0xe0e0);
        Bus::write(REG_GYRO_CONFIG,
                reinterpret_cast<uint8_t *>(&cfg), sizeof(cfg));

        // TODO Self-Test

        return false; 
    }

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

private:
    constexpr static uint8_t who_am_i_value = 0x71;
    static constexpr const float accelFSMAP[] = {2.0,4.0,8.0,16.0};
    static constexpr const float gyroFSMAP[]  = {250,500,1000,2000};

    uint8_t gyroFS;
    uint8_t accelFS;
    uint8_t magnetoFSMState;

    inline constexpr float normalizeAccel(int16_t val) 
    {
        return 
            static_cast<float>(val) / 32768.0f * accelFSMAP[accelFS]
            * EARTH_GRAVITY;
    }

    inline constexpr float normalizeGyro(int16_t val) 
    {
        return 
            static_cast<float>(val) / 32768.0f * gyroFSMAP[gyroFS]
            * DEGREES_TO_RADIANS;
    }

    inline constexpr float normalizeMagneto(int16_t val) 
    {
        // Page 50 @ Register Map document
        return static_cast<float>(val) / 32760.0f * 4912.0f;
    }

    inline constexpr float normalizeTemp(int16_t val) 
    {
        // Page 33 @ Register Map Document
        return static_cast<float>(val) / 512.0f + 21.0f;
    }

    uint8_t akReadReg(uint8_t reg) 
    {
        uint8_t regs[][2] = {
            { REG_I2C_SLV4_ADDR,    (uint8_t)(AK8963_I2C_ADDR | 0x80)},
            { REG_I2C_SLV4_REG,     reg },
            { REG_I2C_SLV4_CTRL,    0x80 },
        };

        for(size_t i=0;i<sizeof(regs)/sizeof(regs[0]);i++)
            Bus::write(regs[i][0], regs[i][1]);

        int timeout = 20; 
        while(--timeout >= 0) {
            if(Bus::read(REG_I2C_MST_STATUS) & 0x40)
                break;
            Thread::sleep(1);
        }

        if(timeout < 0)
            return -1;
        
        uint8_t ret = Bus::read(REG_I2C_SLV4_DI);
        return ret;
    }

    // This is an optimized version of the common (i2c) burst-read:
    // _1 tells to the i2c master to begin reading and.. (See _2)
    void akReadReg_1(uint8_t addr, uint8_t len) 
    {
        assert(len < 16 && len > 0);
        uint8_t regs[][2] = {
            { REG_I2C_SLV0_ADDR,    (uint8_t)(AK8963_I2C_ADDR | 0x80)},
            { REG_I2C_SLV0_REG,     addr },
            { REG_I2C_SLV0_CTRL,    (uint8_t)(0x80 + len)},
        };

        for(size_t i=0;i<sizeof(regs)/sizeof(regs[0]);i++)
            Bus::write(regs[i][0], regs[i][1]);
    }

    // _2 this one moves the data from spi to the local processor
    void akReadReg_2(uint8_t *buf, uint8_t len) 
    {
        Bus::read(REG_EXT_SENS_DATA_00, buf, len);
    }

    void akWriteReg(uint8_t reg, uint8_t data) 
    {
        uint8_t regs[][2] = {
            { REG_I2C_SLV4_ADDR,    AK8963_I2C_ADDR},
            { REG_I2C_SLV4_REG,     reg},
            { REG_I2C_SLV4_DO,      data},
            { REG_I2C_SLV4_CTRL,    0x80},
        };

        for(size_t i=0;i<sizeof(regs)/sizeof(regs[0]);i++)
            Bus::write(regs[i][0], regs[i][1]);

        Thread::sleep(1);
    }

    enum magnetoMap 
    {
        AK8963_I2C_ADDR     = 0x0c,

        AK8963_WIA          = 0x00,
        AK8963_STATUS1      = 0x02,
        AK8963_HXL          = 0x03,
        AK8963_CNTL1        = 0x0a,
        AK8963_CNTL2        = 0x0b,
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

    void readRAWData(mpudata_t &out) 
    {
        Bus::read(REG_ACCEL_XOUT_H,
                reinterpret_cast<uint8_t *>(out.buf), sizeof(out.buf));

        // BigEndian -> CPUArch (LittleEndian as usual)
        for(size_t i=0; i<(sizeof(out.buf)/sizeof(out.buf[0]));i++)
            out.buf[i] = fromBigEndian16(out.buf[i]);
    }
};  

template<typename Bus>
constexpr float MPU9250<Bus>::accelFSMAP[];

template<typename Bus>
constexpr float MPU9250<Bus>::gyroFSMAP[];

#endif /* ifndef MAX9250_H */
