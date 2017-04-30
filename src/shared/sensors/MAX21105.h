/* MAX21105 Driver
 *
 * Copyright (c) 2015-2016 Skyward Experimental Rocketry
 * Authors: Matteo Michele Piazzolla, Alain Carlucci
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

#include "Sensor.h"
#include "interfaces/endianness.h"
#include <BusTemplate.h>

template<class Bus>
class MAX21105 : public AccelSensor, public GyroSensor, 
                 public TemperatureSensor
{
    #pragma pack(1)
    union rawdata_t
    {
        struct {
            int16_t gyro[3];
            int16_t accel[3];
            int16_t temp;
        };
        int16_t raw[7];
    };
    #pragma pack()
public:
    MAX21105(uint8_t accelFullScale, uint8_t gyroFullScale)
    {
        accelFS = accelFullScale & 0x03;
        gyroFS  = gyroFullScale & 0x03;
        mLastTemp = 0.0f;
    }

    ~MAX21105()
    {
    }

    bool init() override
    {
        uint8_t who_am_i = Bus::read(WHO_AM_I);

        if(who_am_i != who_am_i_value)
        {
            last_error = ERR_NOT_ME;
            return false;
        }

        // Init this sensor
        uint8_t init_data[][2] =
        {
            {EXT_STATUS,         0x00},  // Choose the bank 0
            {SET_PWR,            0x00},  // Power down

            // Gyro: 2kHz BW
            {SNS_CFG_1,          (uint8_t)(0x20 | gyroFS)},  
            {SNS_CFG_2,          0x10},

            // Accel: Set scale & no self test
            {SET_ACC_PWR,        (uint8_t)(0x00 | (accelFS << 6))}, 
            {ACC_CFG_1,          0x02},  // 400Hz 
            {ACC_CFG_2,          0x00},  // Low pass filter
            {SET_PWR,            0x78},  // Power up (Accel + Gyro) Low-Noise
        };

        for(size_t i=0; i < sizeof(init_data)/sizeof(init_data[0]); i++)
        {
            Bus::write(init_data[i][0], init_data[i][1]);
            Thread::sleep(1);
        }

        return true;
    }

    bool selfTest() override
    {
        /*
        if(!SelfTestAcc()) {
            last_error = ERR_ACCEL_SELFTEST;
            return false;
        }
        if(!SelfTestGyro()) {
            last_error = ERR_GYRO_SELFTEST;
            return false;
        }
        return true;
        */
        return false;
    }

    std::vector<SPIRequest> buildDMARequest() override 
    {
        return { SPIRequest(0, Bus::getCSPin(),
        {
            (GYRO_X_H | 0x80),
            0,0,0,0,0,0, // gyro
            0,0,         // accel
            0,0,0,0,0,0, // temp
        }
        )};
    }

    void onDMAUpdate(const SPIRequest& req) override
    {
        const auto& r = req.readResponseFromPeripheral();
        const int16_t* ptr = (const int16_t*) &r[1];
        rawdata_t raw_data;

        constexpr size_t dSize = 7; // 3 (gyro) + 3 (accel) + 1 (temp)
        for(size_t i=0; i< dSize;i++)
            raw_data.raw[i] = fromBigEndian16(ptr[i]);

        mLastAccel.setX(normalizeAccel(raw_data.accel[0]));
        mLastAccel.setY(normalizeAccel(raw_data.accel[1]));
        mLastAccel.setZ(normalizeAccel(raw_data.accel[2]));

        mLastGyro.setX(normalizeGyro(raw_data.gyro[0]));
        mLastGyro.setY(normalizeGyro(raw_data.gyro[1]));
        mLastGyro.setZ(normalizeGyro(raw_data.gyro[2]));

        mLastTemp = normalizeTemp(raw_data.temp);
    }

    bool onSimpleUpdate() override {
        return false;
    }

    enum accelFullScale {
        ACC_FS_16G     = 0,
        ACC_FS_8G      = 1,
        ACC_FS_4G      = 2,
        ACC_FS_2G      = 3,
    };
    
    enum gyroFullScale {
        GYRO_FS_2000   = 0,
        GYRO_FS_1000   = 1,
        GYRO_FS_500    = 2,
        GYRO_FS_250    = 3,
    };

private:
    uint8_t accelFS, gyroFS;

    static constexpr const uint8_t who_am_i_value = 0xb4;
    static constexpr const float accelFSMAP[] = {16.0,8.0,4.0,2.0};
    static constexpr const float gyroFSMAP[]  = {2000,1000,500,250};

    inline constexpr float normalizeAccel(int16_t val) {
        return 
            static_cast<float>(val) / 32768.0f * accelFSMAP[accelFS]
            * EARTH_GRAVITY;
    }

    inline constexpr float normalizeGyro(int16_t val) {
        return 
            static_cast<float>(val) / 32768.0f * gyroFSMAP[gyroFS]
            * DEGREES_TO_RADIANS;
    }

    inline constexpr float normalizeTemp(int16_t val) {
        return static_cast<float>(val) / 256.0f;
    }

    enum regMap {
        SET_PWR        = 0x00,
        SNS_CFG_1      = 0x01,
        SNS_CFG_2      = 0x02,
        SNS_CFG_3      = 0x03,
        SET_ACC_PWR    = 0x04,
        ACC_CFG_1      = 0x05,
        ACC_CFG_2      = 0x06,
        SET_TEMP_DR    = 0x13,
        MIF_CFG        = 0x16,
        OTP_STS_CFG    = 0x1C,

        WHO_AM_I       = 0x20, 
        EXT_STATUS     = 0x22,

        GYRO_X_H       = 0x24,
        GYRO_X_L       = 0x25,
        GYRO_Y_H       = 0x26,
        GYRO_Y_L       = 0x27,
        GYRO_Z_H       = 0x28,
        GYRO_Z_L       = 0x29,
        ACCE_X_H       = 0x2A,
        ACCE_X_L       = 0x2B,
        ACCE_Y_H       = 0x2C,
        ACCE_Y_L       = 0x2D,
        ACCE_Z_H       = 0x2E,
        ACCE_Z_L       = 0x2F,
        TEMP_H         = 0x30,
        TEMP_L         = 0x31,

        TRM_BNK_REG    = 0x38,
        FIFO_COUNT     = 0x3C,
        FIFO_STATUS    = 0x3D,
        FIFO_DATA      = 0x3E,
        RST_REG        = 0x3F

    };

    // -------------------- the hell down here 

//TODO: capire se ha senso
/*
Self Test del Gyroscopio,
Prendo 5 campioni del giroscopio e ne faccio la media,
imposto la modalità self test e ne prendo altri 5 facendone nuovamente la media.
Se la distanza è maggiore di quella nel datasheet o non passa la condizione
X>0, Y<0, Z>0 il test fallisce e ritorna falso
*/
/*
    bool SelfTestGyro() {
        constexpr uint8_t min_x = 8;
        constexpr uint8_t min_y = -50;
        constexpr uint8_t min_z = 8;
        constexpr uint8_t max_x = 50;
        constexpr uint8_t max_y = -8;
        constexpr uint8_t max_z = 50;

      // Self Test Gyroscopio
        float Ax_no_test=0;
        float Ay_no_test=0;
        float Az_no_test=0;

        for(int i=0;i<5;++i){
            //sleep
            Ax_no_test+=readGyroX();
            Ay_no_test+=readGyroY();
            Az_no_test+=readGyroZ();
        }

        Ax_no_test=Ax_no_test/5;
        Ay_no_test=Ay_no_test/5;
        Az_no_test=Az_no_test/5;

        //seleziono il bank 0 dei registri
        writeReg(EXT_STATUS,0x00);
        //Power Down
        writeReg(SET_PWR,0x78);
        // Gyro: 2kHz BW, 1000dps FS
        //Testing Positive sign {+X, -Y, +Z}
        writeReg(SNS_CFG_1,0x7c);
        writeReg(SNS_CFG_2,0x10);
        // Acc Low-Noise + Gyro Low-Noise
        writeReg(SET_PWR,0x78);
        //sleep

        float Ax_test=0;
        float Ay_test=0;
        float Az_test=0;

        for(int i=0;i<5;++i){
            Ax_test+=readGyroX();
            Ay_test+=readGyroY();
            Az_test+=readGyroZ();
        }

        Ax_test=Ax_test/5;
        Ay_test=Ay_test/5;
        Az_test=Az_test/5;

        float deltaX,deltaY,deltaZ;
        deltaX = std::abs(Ax_test - Ax_no_test);
        deltaY = -std::abs(Ay_test - Ay_no_test);
        deltaZ = std::abs(Az_test - Az_no_test);

        if ((min_x <= deltaX && deltaX <= max_x) &&
            (min_y <= deltaY && deltaY <= max_y) &&
            (min_z <= deltaZ && deltaZ <= max_z))
            return true;
        return false;
    }

    Vec3 readGyro() {
        return Vec3(readGyroX(), readGyroY(), readGyroZ());
    }

    Vec3 readAccelerometer() {
        return Vec3(readAccX(), readAccY(), readAccZ());
    }

    bool SelfTestAcc(){
        //16g X positive force
        writeReg(SET_ACC_PWR,0x8);
        //TODO: test acc
        return true;
    }
*/
};

template<typename Bus>
constexpr float MAX21105<Bus>::accelFSMAP[];

template<typename Bus>
constexpr float MAX21105<Bus>::gyroFSMAP[];
