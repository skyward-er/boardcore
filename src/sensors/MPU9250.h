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
        MPU9250() {
            gyro_scale = GYROS_2000;
            accel_scale = ACCELS_16g;
        }

        bool init() {
            uint8_t whoami = mBus.ReadReg(regMap::WHO_AM_I)

            if(whoami != who_am_i_value) {
                last_error = ERR_NOT_ME;
                return false;
            }

            // MSB: reset, LSB: clock=auto
            bus.WriteReg(REG_PWR_MGMT_1, 0b10000001 );

            // Reset Timeout
            int timeout = 10;
            while(--timeout > 0) {
                if(!(mBus.ReadReg(REG_PWR_MGMT_1) & 0x80))
                    break;
                Thread::Sleep(1);
            }

            if(timeout == 0) {
                last_error = ERR_RESET_TIMEOUT;
                return false;
            }

            // Enable all sensors
            bus.WriteReg(REG_PWR_MGMT_2, 0b00000000 );

            // DLPF_CFG = xxxxx000
            bus.WriteReg(REG_CONFIG, 0b00000000);

            // Do not divide
            bus.WriteReg(REG_SMPLRT_DIV, 0x00);

            // Gyro Config: FChoice = 0b11: use DLPF_CFG
            bus.WriteReg(REG_GYRO_CONFIG, 0b00000011 | 
                    ((gyro_scale & 0x03) << 3));

            // Write Accel Scale
            bus.WriteReg(REG_ACCEL_CONFIG, 0b00000000 |
                    ((accel_scale & 0x03) << 3));

            // 0000xyyy : x = FCHOICE, y = A_DLPF_CFG 
            bus.WriteReg(REG_ACCEL_CONFIG2, 0b00001000);

            return true;
        }

        bool selfTest() {
            return false; 
        }

        Quaternion getOrientation() {
            return Quaternion(); 
        }

        Vec3 getSpeed() {
            return Vec3(); 
        }

        Vec3 getCompass() {
            return Vec3(); 
        }

        float getTemperature() {
            return 0.0f;             
        }

        enum eErrors {
            ERR_NOT_ME          = 0x01,
            ERR_RESET_TIMEOUT   = 0x02
        };
    private:
        BusType bus;
        constexpr static uint8_t who_am_i_value = 0x71;

        enum eRegMap {
            REG_SMPLRT_DIV      = 0x19,
            REG_CONFIG          = 0x1A,
            REG_GYRO_CONFIG     = 0x1B,
            REG_ACCEL_CONFIG    = 0x1C,
            REG_ACCEL_CONFIG2   = 0x1D,
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
