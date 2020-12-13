/* Copyright (c) 2020 Skyward Experimental Rocketry
 * Authors: Vincenzo Santomarco
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

#include "sensors/Sensor.h"
#include "Common.h"
#include "drivers/spi/SPIDriver.h"
#include "ASM330LHHData.h"
#include <cmath>

using miosix::getTick;
using miosix::TICK_FREQ;
 

        
/**
* @brief IMU parameters:
*           gyroscope odr (from power down up to 6667Hz)
*           gyroscope full scale (from +-125dps up to +-4000dps)
*           accelerometer odr (from power down up do 6667hz)
*           accelerometer full scale (from +-2g up to +-16g)
*           bdu (continuous update or update after read)
*           temperatuse sampling divider (after how many calls of method "onSimpleUpdate"
*               temperature must be sampled)
*/
struct asm330lhh_params
{
    uint8_t gyro_odr = 0;
    uint8_t gyro_fs = 0;
    uint8_t accel_odr = 0;
    uint8_t accel_fs = 0;
    uint8_t bdu = 0;
    uint16_t temperature_divider = 100;
};

class ASM330LHH : public virtual Sensor
{
    public:

        /**
        * @brief Constructor
        * 
        * @param bus            the SPI bus
        * @param chip_select    CS pin
        * @param params         Sensor params (bdu, accelerometer and gyroscope odr and sensitivity)
        * 
        **/
        ASM330LHH(SPIBusInterface& bus,
                    GpioPin chip_select, asm330lhh_params params) : 
                spi_slave(bus, chip_select, {}), params(params) 
        {
            spi_slave.config.clock_div = SPIClockDivider::DIV64;  // used to set the spi baud rate (maximum is 10 Mhz)
        }

        /**
        * @brief Constructor
        * 
        * @param bus            the SPI bus
        * @param chip_select    CS pin
        * @param config         SPI bus configuration
        * @param params         Sensor parameters (bdu, accelerometer and gyroscope odr and sensitivity)
        * 
        **/
        ASM330LHH(SPIBusInterface& bus,
                    GpioPin chip_select,
                    SPIBusConfig config,
                    asm330lhh_params params) :
                spi_slave(bus, chip_select, config), params(params) {}

        ~ASM330LHH() {}

        /**
         * @brief Initialize the sensor.
         * 
         * @return boolean value indicating whether the operation succeded or not
         */
        bool init() override
        { 

            miosix::Thread::sleep(100);

            assert(params.temperature_divider > 0);

            setup_device();

            if(!check_whoami()){
                TRACE("Whoami check failed!\n");
                last_error = ERR_NOT_ME;
                return false;
            }

            setup_accel(params.accel_odr, params.accel_odr);
            setup_gyro(params.gyro_odr, params.gyro_fs);
            set_bdu(params.bdu);

            return true; 
        }

         /**
         * @brief Read new data from accelerometer, gyroscope and temperature sensor.
         *        Accelerometer unit of measure: mg
         *        Gyroscope unit of measure: mdps
         *        Temperature sensor unit of measure: °C
         *        
         * 
         * @return boolean value indicating whether the operation succeded or not
         */
        bool onSimpleUpdate() override
        {
            // Increase sampling counter
            sample++;

            uint16_t buffer[12];
            asm330lhh_data new_data;

            // Read all accelerometer and gyroscope output registers
            SPITransaction spi(spi_slave);
            spi.read(OUTX_L_G_REG, (uint8_t*) buffer, 12);

            int16_t val = buffer[0] | buffer[1]<<8;
            new_data.gyro_x = ((float) val)*gyro_sensityvity;

            val = buffer[2] | buffer[3]<<8;
            new_data.gyro_y = ((float) val)*gyro_sensityvity;

            val = buffer[4] | buffer[5]<<8;
            new_data.gyro_z = ((float) val)*gyro_sensityvity;

            val = buffer[6] | buffer[7]<<8;
            new_data.accel_x = ((float) val)*accel_sensitivity;

            val = buffer[8] | buffer[9]<<8;
            new_data.accel_y = ((float) val)*accel_sensitivity;

            val = buffer[10] | buffer[11]<<8;
            new_data.accel_x = ((float) val)*accel_sensitivity;

            // Read temperature data or put 0 as default value according to temperature_divider
            if (sample == params.temperature_divider){
                spi.read(OUT_TEMP_L_REG, (uint8_t*) buffer, 2);
                val = buffer[0] | buffer[1]<<8;
                new_data.temperature = val/TSen + Toff;
                sample = 0;
            } else {
                new_data.temperature = 0;
            }

            data = new_data;
            return true;
        }

        /**
        * @brief Check if the sensor is working.
        * 
        * @return boolean indicating whether the sensor is correctly working or not
        */
        bool selfTest() override 
        {   
            // Number of samples collected
            int samples = 10;

            // Average values collected with st disabled
            float gyro_before[3] = {0, 0, 0};
            float accel_before[3] = {0, 0, 0};
            // Average values collected with st enabled
            float gyro_after[3] = {0, 0, 0};
            float accel_after[3] = {0, 0, 0};

            // Set gyroscope sensitivity to +-250
            setup_gyro(params.gyro_odr, _250DPS);

            // Read selftest disabled data
            for(int i=0; i<samples; i++){
                miosix::Thread::sleep(10);
                sample = 0;                     // Be sure to never read temperature data
                onSimpleUpdate();
                asm330lhh_data data = getData();

                gyro_before[0] += data.gyro_x;
                gyro_before[1] += data.gyro_y;
                gyro_before[2] += data.gyro_z;

                accel_before[0] += data.accel_x;
                accel_before[1] += data.accel_y;
                accel_before[2] += data.accel_z;
            }

            gyro_before[0] /= samples;
            gyro_before[1] /= samples;
            gyro_before[2] /= samples;

            accel_before[0] /= samples;
            accel_before[1] /= samples;
            accel_before[2] /= samples;

            // Enable selftest
            {
                SPITransaction spi(spi_slave);

                uint8_t gyro_positive_st = 0x1;     // 0b01
                uint8_t accel_positive_st = 0x1;    // 0b01

                uint8_t ctrl5_c_value = spi.read(CTRL5_C_REG);
                ctrl5_c_value |= accel_positive_st;
                ctrl5_c_value |= gyro_positive_st << 2;
                spi.write(CTRL5_C_REG, ctrl5_c_value);
            }

            // Read selftest enabled data
            for(int i=0; i<samples; i++){
                miosix::Thread::sleep(10);
                sample = 0;                     // Be sure to never read temperature data
                onSimpleUpdate();
                asm330lhh_data data = getData();

                gyro_after[0] += data.gyro_x;
                gyro_after[1] += data.gyro_y;
                gyro_after[2] += data.gyro_z;

                accel_after[0] += data.accel_x;
                accel_after[1] += data.accel_y;
                accel_after[2] += data.accel_z;
            }

            gyro_after[0] /= samples;
            gyro_after[1] /= samples;
            gyro_after[2] /= samples;

            accel_after[0] /= samples;
            accel_after[1] /= samples;
            accel_after[2] /= samples;

            // Disable selftest
            {
                SPITransaction spi(spi_slave);

                uint8_t ctrl5_c_value = spi.read(CTRL5_C_REG);
                ctrl5_c_value &= 0xf0;                          // 0b11110000
                spi.write(CTRL5_C_REG, ctrl5_c_value);
            }

            // Reset gyroscope sensitivity
            setup_gyro(params.gyro_odr, params.gyro_fs);

            // Check deltas
            bool selftest_ok = true;

            for(int i=0; i<3 && selftest_ok; i++){
                float delta = abs(gyro_after[i] - gyro_before[i]);
                if(delta<gyro_selftest_min || delta>gyro_selftest_max){
                    selftest_ok = false;
                    last_error = ERR_GYRO_SELFTEST;
                }
                delta =  abs(accel_after[i] - accel_before[i]);
                if(delta<accel_selftest_min || delta > accel_selftest_max){
                    selftest_ok = false;
                    last_error = ERR_ACCEL_SELFTEST;
                }
            }

            return selftest_ok; 
        }
        
        // not a Sensor class method: you should 
        // define methods to get the sampled data
        asm330lhh_data getData() 
        { 
            return data; 
        }
    
    private:
        /**
         * @brief Check that the WHO_AM_I register 
         *        contains the correct value.
         * 
         * @return boolean value indicating whether the value read
         *          from the WHO_AM_I register is correct or not
         */
        bool check_whoami() 
        {
            SPITransaction spi(spi_slave);
            
            uint8_t whoami = spi.read(WHO_AM_I_REG);

            TRACE("whoami: %u\n", whoami);

            return whoami == WHO_AM_I_DEFAULT_VALUE;
        }

        /**
         * @brief I2C_disable = 1 in CTRL4_C and
         *      DEVICE_CONF = 1 in CTRL9_XL for setting up
         *      the device for SPI
         */
        void setup_device() 
        {
            SPITransaction spi(spi_slave);

            // Read and update CTRL4_C
            uint8_t ctrl4_c_val = spi.read(CTRL4_C_REG);
            uint8_t i2c_disable_bit = 4;                    // 0b00000100
            ctrl4_c_val |= i2c_disable_bit;
            spi.write(CTRL4_C_REG, ctrl4_c_val);


            // Read and update CTRL9_XL
            uint8_t ctrl9_xl_val = spi.read(CTRL9_XL_REG);
            uint8_t device_conf = 2;                        // 0b00000010
            ctrl9_xl_val |= device_conf;
            spi.write(CTRL9_XL_REG, ctrl9_xl_val);

        }

        void setup_accel(uint8_t odr, uint8_t fs) {
            SPITransaction spi(spi_slave);

            assert(odr>=0 && odr<=10);
            assert(fs>=0 && fs <=3);

            accel_sensitivity = LA_So[fs];

            odr = odr & 15;     // 0b1111
            odr = odr << 4;

            fs = fs & 3;        // 0b11
            fs = fs << 2;

            uint8_t ctrl1_xl_value = spi.read(CTRL1_XL_REG);
            ctrl1_xl_value &= 3;                        // 0b00000011
            ctrl1_xl_value |= odr;
            ctrl1_xl_value |= fs;
            spi.write(CTRL1_XL_REG, ctrl1_xl_value);
        }

        void setup_gyro(uint8_t odr, uint8_t fs) {
            SPITransaction spi(spi_slave);

            assert(odr>=0 && odr<=10);      //0b1010
            assert(fs>=0 && fs<=5);

            gyro_sensityvity = G_So[fs];

            odr = odr & 15;                 // 0b1111
            odr = odr << 4;

            // if fs is 125 or 4000 ctrl2_g register values [1:0]
            // must be set accordingly, otherwise those must be 00
            if(fs==0)
                // FS_125 bit = 1
                // FS_4000 bit = 0
                fs = 2;    // 0b10
            else if(fs==5)
                // FS_125 bit = 0
                // FS_4000 bit = 1
                fs = 1;     // 0b01
            else {
                // Set ctrl2_g[3:2]
                fs = fs - 1;
                fs = fs << 2;
            }

            uint8_t ctrl2_g_value = spi.read(CTRL2_G_REG);
            ctrl2_g_value &= 0;
            ctrl2_g_value |= odr;
            ctrl2_g_value |= fs;
            spi.write(CTRL2_G_REG, ctrl2_g_value);
        }

        void set_bdu(uint8_t bdu){
            assert(bdu==1 || bdu==0);

            SPITransaction spi(spi_slave);

            uint8_t ctrl3_c_value = spi.read(CTRL3_C_REG);
            ctrl3_c_value &=0xbf;                       // 0b10111111
            bdu &= 1;
            bdu = bdu << 6;
            ctrl3_c_value |= bdu;
            spi.write(CTRL3_C_REG, ctrl3_c_value);
        }

    public:

        /**
         * @brief Accelerometer possible values for full scale
        */
        enum ACCEL_FS {
            _2G = 0,
            _16G = 1,
            _4G = 2,
            _8G = 3
        };

        /**
         * @brief Gyroscope possible values for full scale
        */
        enum GYRO_FS {
            _125DPS = 0,
            _250DPS = 1,
            _500DPS = 2,
            _1000DPS = 3,
            _2000DPS = 4,
            _4000DPS = 5,
        };
        
        /**
         * @brief ODR possible values for both Accelerometer and Gyroscope
        */
        enum ODR {
            _POWER_DOWN = 0,
            _12_5HZ = 1,
            _26HZ = 2,
            _52HZ = 3,
            _104HZ = 4,
            _208HZ = 5,
            _417HZ = 6,
            _833HZ = 7,
            _1667HZ = 8,
            _3333HZ = 9,
            _6667HZ = 10
        };

        /**
         * BDU options: continuous update or update after read
        */
        enum BDU {
            CONTINUOUS_UPDATE = 0,
            UPDATE_AFTER_READ = 1,
        };
    
    // Constant values
    private:
        const uint8_t WHO_AM_I_DEFAULT_VALUE = 0x6b;
        const float LA_So[4] = {0.061, 0.122, 0.244, 0.488};
        const float G_So[6] = {4.37, 8.75, 17.5, 35.0, 70.0, 140.0};
        const float TSen = 256;
        const float Toff = 25;
        const float accel_selftest_min = 40;
        const float accel_selftest_max = 1700;
        const float gyro_selftest_min = 20;
        const float gyro_selftest_max = 80;

    private:

        /**
         * @brief Registers' addresses definition.
         */
        enum REG {
            WHO_AM_I_REG = 0x0f,
            CTRL1_XL_REG = 0x10,
            CTRL2_G_REG = 0x11,
            CTRL3_C_REG = 0x12,
            CTRL4_C_REG = 0x13,
            CTRL5_C_REG = 0x14,
            CTRL9_XL_REG = 0x18,
            OUT_TEMP_L_REG = 0x20,
            OUT_TEMP_H_REG = 0X21,
            OUTX_L_G_REG = 0X22,
            OUTX_H_G_REG = 0X23,
            OUTY_L_G_REG = 0X24,
            OUTY_H_G_REG = 0X25,
            OUTZ_L_G_REG = 0X26,
            OUTZ_H_G_REG = 0X27,
            OUTX_L_A_REG = 0X28,
            OUTX_H_A_REG = 0X29,
            OUTY_L_A_REG = 0X2a,
            OUTY_H_A_REG = 0X2b,
            OUTZ_L_A_REG = 0X2c,
            OUTZ_H_A_REG = 0X2d,
        };

    private:
        const asm330lhh_params params;
        SPISlave spi_slave;
        asm330lhh_data data;
        float gyro_sensityvity;
        float accel_sensitivity;
        uint16_t sample = 0;

};