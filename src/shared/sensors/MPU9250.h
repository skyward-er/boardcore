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
#include "BusTemplate.h"

template <typename BusType>
class MPU9250 : public GyroSensor, public AccelSensor, 
                public CompassSensor, public TemperatureSensor {
    public:
        MPU9250() : temp(0.0f) {
            gyro_scale = GYROS_2000;
            accel_scale = ACCELS_16G;
        }

        bool init() {
            uint8_t whoami = bus.read(REG_WHO_AM_I);

            if(whoami != who_am_i_value) {
                last_error = ERR_NOT_ME;
                return false;
            }

            uint8_t init_data[][2] = {
                {REG_PWR_MGMT_1,     0x80},
                {REG_PWR_MGMT_1,     0x01},
                {REG_PWR_MGMT_2,     0x00}, // Enable all sensors
                {REG_CONFIG,         0x00}, // DLPF_CFG = xxxxx000
                {REG_SMPLRT_DIV,     0x00}, // Do not divide
                {REG_GYRO_CONFIG,    0x03 | ((gyro_scale  & 3) << 3)},
                {REG_ACCEL_CONFIG,   0x00 | ((accel_scale & 3) << 3)},
                {REG_ACCEL_CONFIG2,  0x08}, // FCHOICE = 1, A_DLPF_CFG = 000
                {REG_INT_PIN_CFG,    0x30},
                {REG_INT_ENABLE,     0x00}, // No interrupts

                // I2C
                {REG_USER_CTRL,      0x20}, // Master mode TODO: I2C_IF_DIS???
                {REG_I2C_MST_CTRL,   0x0D}, // Multi-master @ 400KHz

                // Let's try with slv4 because reg+1 = do, do+1 = ctrl
                {REG_I2C_SLV4_ADDR, AK8963_I2C_ADDR}, // AK8963 

                {REG_I2C_SLV4_REG, AK8963_CNTL2}, 
                {REG_I2C_SLV4_DO, 0x01},    // Reset AK8963
                {REG_I2C_SLV4_CTRL, 0x81},  // Enable I2C and set 1 byte

                {REG_I2C_SLV4_REG, AK8963_CNTL1}, 
                {REG_I2C_SLV4_DO, 0x12},    // Continuous measurement @ 16bit
                {REG_I2C_SLV4_CTRL, 0x81}   // Enable I2C and set 1 byte
            }; 

            for(int i=0; i < sizeof(init_data)/sizeof(init_data[0]); i++) {
                bus.write(init_data[i][0], init_data[i][1]);
                Thread::sleep(1);
            }

            return true;
        }

        void updateData() {
            //                                 xh    xl    yh    yl    zh    zl
            // uint8_t accel = {0x3b | 0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}
            //                                th    tl
            // uint8_t temp = {0x41 | 0x80, 0x00, 0x00}
            //                                xh    xl    yh    yl    zh    zl
            // uint8_t gyro = {0x3b | 0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}
            // TODO magneto
        }

        bool selfTest() {
            return false; 
        }

        Vec3 getOrientation() {
            return gyro; 
        }

        Vec3 getAccel() {
            return accel; 
        }

        Vec3 getCompass() {
            return magneto; 
        }

        float getTemperature() {
            return temp;             
        }
    private:
        BusType bus;
        constexpr static uint8_t who_am_i_value = 0x71;
        Vec3 accel, gyro, magneto;
        float temp;

        enum eMagnetoMap {
            AK8963_I2C_ADDR     = 0x0c,

            AK8963_STATUS1      = 0x02,
            AK8963_CNTL1        = 0x0a,
            AK8963_CNTL2        = 0x0b,
        };

        enum eRegMap {
            REG_SMPLRT_DIV      = 0x19,
            REG_CONFIG          = 0x1A,
            REG_GYRO_CONFIG     = 0x1B,
            REG_ACCEL_CONFIG    = 0x1C,
            REG_ACCEL_CONFIG2   = 0x1D,

            REG_I2C_MST_CTRL    = 0x24, 
            REG_I2C_SLV0_ADDR   = 0x25, // unused.
            REG_I2C_SLV0_REG    = 0x26, // unused.
            REG_I2C_SLV0_CTRL   = 0x27, // unused.
            REG_I2C_SLV0_DO     = 0x63, // unused.

            REG_I2C_SLV4_ADDR   = 0x31,
            REG_I2C_SLV4_REG    = 0x32,
            REG_I2C_SLV4_DO     = 0x33,
            REG_I2C_SLV4_CTRL   = 0x34,

            REG_INT_PIN_CFG     = 0x37,
            REG_INT_ENABLE      = 0x38,
            REG_INT_STATUS      = 0x3A,

            REG_USER_CTRL       = 0x6A,
            REG_PWR_MGMT_1      = 0x6B,
            REG_PWR_MGMT_2      = 0x6C,
            REG_WHO_AM_I        = 0x75
        };

        enum eGyroScale {
            GYROS_250           = 0,
            GYROS_500           = 1,
            GYROS_1000          = 2,
            GYROS_2000          = 3
        };

        enum eAccelScale {
            ACCELS_2G           = 0,
            ACCELS_4G           = 1,
            ACCELS_8G           = 2,
            ACCELS_16G          = 3
        };

        eGyroScale gyro_scale;
        eAccelScale accel_scale;
};  

#endif /* ifndef MAX9250_H */
