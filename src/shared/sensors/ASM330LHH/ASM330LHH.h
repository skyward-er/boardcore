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

using miosix::getTick;
using miosix::TICK_FREQ;
 

        
/**
* @brief IMU parameters:
*           gyroscope odr (from power down up to 6667Hz)
*           gyroscope full scale (from +-125dps up to +-4000dps)
*           accelerometer odr (from power down up do 6667hz)
*           accelerometer full scale (from +-2g up to +-16g)
*           bdu (continuous update or update after read)
*/
struct asm330lhh_params
{
    uint8_t gyro_odr = 0;
    uint8_t gyro_fs = 0;
    uint8_t accel_odr = 0;
    uint8_t accel_fs = 0;
    uint8_t bdu = 0;
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
            uint16_t buffer[14];
            asm330lhh_data new_data;

            SPITransaction spi(spi_slave);
            spi.read(OUT_TEMP_L_REG, (uint8_t*) buffer, 14);

            int16_t val = buffer[0] | buffer[1]<<8;
            new_data.temperature = ((float) val)/TSen + Toff;

            val = buffer[2] | buffer[3]<<8;
            new_data.gyro_x = ((float) val)*G_So[params.gyro_fs];

            val = buffer[4] | buffer[5]<<8;
            new_data.gyro_y = ((float) val)*G_So[params.gyro_fs];

            val = buffer[6] | buffer[7]<<8;
            new_data.gyro_z = ((float) val)*G_So[params.gyro_fs];

            val = buffer[8] | buffer[9]<<8;
            new_data.accel_x = ((float) val)*LA_So[params.accel_fs];

            val = buffer[10] | buffer[11]<<8;
            new_data.accel_y = ((float) val)*LA_So[params.accel_fs];

            val = buffer[12] | buffer[13]<<8;
            new_data.accel_x = ((float) val)*LA_So[params.accel_fs];

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
            // check that the sensor is properly working
            // e.g. check values tolerance over a set of samples
            //      or what the datasheet suggests
            return true; 
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

};