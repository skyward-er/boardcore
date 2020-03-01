/* LSM9DS1 accelerometer + giroscope Driver
 *
 * Copyright (c) 2016,2020 Skyward Experimental Rocketry
 * Authors: Andrea Milluzzo
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


//il sensore ha due CS e due MISO (rispettivamente per AXEL+GYRO e MAGNETOMETRO)
//verrà trattato come due sensori separati

/*
TODO:
- lettura con interrupt: ci sarà un timer hardware che gira. Quando arriva
  interrupt di data ready, salvo il valore del timer (bisogna capire quando questo va in overflow, cosa faccio?)
- formato dati: std::vector con X,Y,Z,timestamp per ogni grandezza fisica

SELFTEST: cosa sono i registri di self test??

POI:
- FIFO: ho due opzioni
    1-faccio una FIFO sw in parallelo a quella HW con solo i timestamp e ogni x timestamp mi prelevo tutta la FIFO
    2-metto nella FIFO hw i timestamp (la LSM6DS3 lo faceva)
*/



#pragma once
#include <miosix.h>

#include "drivers/spi/SPIDriver.h"
#include "Sensor.h"

using miosix::GpioPin;

class LSM9DS1_XLG : public GyroSensor, public AccelSensor
{
    public:

        enum class AxelFSR
        {
            FS_2 = 0x00,
            FS_16,
            FS_4,
            FS_8 
        };

        enum class GyroFSR
        {
            FS_245 = 0x00,
            FS_500,
            FS_2000 = 0x03
        };

        enum class ODR
        {
            PWR_DW  =   0X00,
            ODR_10,
            ODR_50,
            ODR_119,
            ODR_238,
            ODR_476,
            ODR_952,
        };

         /**
         * @brief Creates an instance of an LSM9DS1 accelerometer + gyroscope sensor
         *
         * @param    bus SPI bus the sensor is connected to
         * @param    cs Chip Select GPIO
         * @param    config (OPTIONAL) custom SPIBusConfig 
         * @param    axelRange accelerometer Full Scale Range (See datasheet)
         * @param    gyroRange gyroscope Full Scale Range (See datasheet)
         * @param    odr Output Data Rate (See datasheet)
         * @param    fifo_enabled Fifo enabled
         * @param    fifo_watermark FIFO watermark level in range [1,32] (used for
         * interrupt generation, see datasheet).
         */

        LSM9DS1_XLG(
           SPIBusInterface& bus, 
           GpioPin          cs,
           AxelFSR          axelRange           = AxelFSR::FS_2,
           GyroFSR          gyroRange           = GyroFSR::FS_245,
           ODR              odr                 = ODR::ODR_10,
           bool             fifo_enabled        = false,
           unsigned int     fifo_watermark      = 24
           ):fifo_enabled(fifo_enabled), fifo_watermark(fifo_watermark),
          spislave(bus, cs), axelFSR(axelRange), gyroFSR(gyroRange), odr(odr){
            //SPI config
            spislave.config.br = SPIBaudRate::DIV_64; //baud = fclk/64
        }

        LSM9DS1_XLG(
           SPIBusInterface& bus, 
           GpioPin          cs,
           SPIBusConfig     config,
           AxelFSR          axelRange           = AxelFSR::FS_2,
           GyroFSR          gyroRange           = GyroFSR::FS_245,
           ODR              odr                 = ODR::ODR_10,
           bool             fifo_enabled        = false,
           unsigned int     fifo_watermark      = 24
           ):fifo_enabled(fifo_enabled), fifo_watermark(fifo_watermark),
          spislave(bus, cs, config), axelFSR(axelRange), gyroFSR(gyroRange), odr(odr){
            
        }



        bool init() override
        {
            //Set FSR
            switch(axelFSR)
            {
                case AxelFSR::FS_2:
                    axelFSRval = 2.0f;
                    break;
                case AxelFSR::FS_4:
                    axelFSRval = 4.0f;
                    break;
                case AxelFSR::FS_8:
                    axelFSRval = 8.0f;
                    break;
                case AxelFSR::FS_16:
                    axelFSRval = 16.0f;
                    break;
                default:
                    axelFSRval = 2.0f;
                    break;
            }
            switch (gyroFSR)
            {
                case GyroFSR::FS_245:
                    gyroFSRval = 245.0f;
                    break;
                case GyroFSR::FS_500:
                    gyroFSRval = 500.0f;
                    break;
                case GyroFSR::FS_2000:
                    gyroFSRval = 2000.0f;
                    break;
                default:
                    gyroFSRval = 245.0f;
                    break;
            }

            SPITransaction spi(spislave);

            //Who Am I check:
            uint8_t whoami = spi.read(regMapXLG::WHO_AM_I);
            if(whoami != WHO_AM_I_XLG_VAL){
                printf("LSM9DS1 WAMI: %d\n", whoami);
                last_error = ERR_NOT_ME;
                return false;
            }

            //common setup
            spi.write(regMapXLG::CTRL_REG8, 0x04); //addr auto-increment while reading/writing
            spi.write(regMapXLG::CTRL_REG9, 0x0C); //DRDY_mask_bit ON(????), I2C OFF, FIFO OFF

            //FIFO setup
            if(fifo_enabled)
            {
                //spi.write(regMapXLG::CTRL_REG5_XL, 0x38); //FIFO decimation -> no decimation, axel out enable -> ON. Di default è OK
                spi.write(regMapXLG::FIFO_CTRL, 0xC0 | fifo_watermark); //FIFO continous mode + fifo watermark threshold setup
                //interrupt setup
                spi.write(regMapXLG::INT1_CTRL, 0x08); //interrupt on FIFO treshold
                //CTRL_REG4 //latched interrupt?
            }
            
            //Axel Setup
            spi.write(regMapXLG::CTRL_REG6_XL, (int)odr << 5 | (int)axelFSR << 3); //ODR, FSR, auto BW (max) function of ODR
            //CTRL_REG7_XL //High resolution mode enable / LPF2&HPF enable -> default: disabled
            //CTRL_REG5_XL //axel enable -> default:enable
            
            //Gyro Setup
            spi.write(regMapXLG::CTRL_REG1_G, (int)odr<<5 | (int)gyroFSR<<3); //ORD,FSR
            //CTRL_REG2_G //LPF2, HPF bypassed by default
            //CTRL_REG3_G //HPF enable, HPF cutoff 
            //ORIENT_CFG_G //angular rate sign and orientation Setup <--- BOARD DEPENDENT
            //CTRL_REG4 //gyro enable -> enabled by default

            return selfTest(); 
        }

        bool selfTest() override
        {   
            //@ startup, some samples have to be discarded (datasheet)
            uint16_t toWait_ms = samplesToDiscard * 1000 / (int)odr;
            miosix::Thread::sleep(toWait_ms); //if FIFO is disabled, just wait 
            if(fifo_enabled)
            {
                //if FIFO is enabled, read first <samplesToDiscard> samples and discard them
                SPITransaction spi(spislave);
                for(int i=0; i < samplesToDiscard; i++)
                {
                    spi.read(regMapXLG::OUT_X_L_XL, 6);
                    spi.read(regMapXLG::OUT_X_L_G,  6);
                }
            }
            return true;
        }
        
        bool onSimpleUpdate() override //SE NELLO STATUS REGISTER IL DATO NON E' PRONTO?????
        {
            
            if(!fifo_enabled){ //if FIFO disabled
                uint8_t axelData[6], gyroData[6];
                // Read output axel+gyro data X,Y,Z
                {
                    SPITransaction spi(spislave);
                    spi.read(regMapXLG::OUT_X_L_XL, axelData, 6);
                    spi.read(regMapXLG::OUT_X_L_G,  gyroData, 6);
                }

                int16_t x_xl = axelData[0] | axelData[1] << 8;
                int16_t y_xl = axelData[2] | axelData[3] << 8;
                int16_t z_xl = axelData[4] | axelData[5] << 8;

                int16_t x_gy = gyroData[0] | gyroData[1] << 8;
                int16_t y_gy = gyroData[2] | gyroData[3] << 8;
                int16_t z_gy = gyroData[4] | gyroData[5] << 8;

                // printf("LSM9DS1 axel: %02X,%02X,%02X\n", x_xl, y_xl, z_xl);
                // printf("LSM9DS1 gyro: %02X,%02X,%02X\n", x_gy, y_gy, z_gy);
                
                mLastAccel = 
                    Vec3(x_xl * axelFSRval / 65535,
                         y_xl * axelFSRval / 65535,
                         z_xl * axelFSRval / 65535);
                
                mLastGyro =
                    Vec3(x_gy * gyroFSRval / 65535,
                         y_gy * gyroFSRval / 65535,
                         z_gy * gyroFSRval / 65535);
            }
            else{ //if FIFO enabled

            }
            return true; 
        }

    private:

        bool fifo_enabled = false;
        uint8_t fifo_watermark;

        SPISlave spislave; 

        AxelFSR axelFSR;
        GyroFSR gyroFSR;
        ODR odr;
        
        float axelFSRval;
        float gyroFSRval;
        uint8_t samplesToDiscard = 8; //max possible val

        static const uint8_t WHO_AM_I_XLG_VAL = 0x68;
        enum regMapXLG
        {
            //ACT_THS             =   0x04,
            //ACT_DUR             =   0x05,    
            //INT_GEN_CFG_XL      =   0x06,
            //INT_GEN_THS_X_XL    =   0x07,
            //INT_GEN_THS_Y_XL    =   0x08,
            //INT_GEN_THS_Z_XL    =   0x09,
            //INT_GEN_DUR_XL      =   0x0A,
            //REFERENCE_G         =   0x0B,
            INT1_CTRL           =   0x0C,
            //INT2_CTRL           =   0x0D,
            WHO_AM_I            =   0x0F,
            CTRL_REG1_G         =   0x10,
            CTRL_REG2_G         =   0x11,
            CTRL_REG3_G         =   0x12,
            ORIENT_CFG_G        =   0x13,
            //INT_GEN_SRC_G       =   0x14,
            //OUT_TEMP_L          =   0x15,
            //OUT_TEMP_H          =   0x16,
            STATUS_REG_G        =   0x17,  // per check stato
            OUT_X_L_G           =   0x18,
            OUT_X_H_G           =   0x19,
            OUT_Y_L_G           =   0x1A,
            OUT_Y_H_G           =   0x1B,
            OUT_Z_L_G           =   0x1C,
            OUT_Z_H_G           =   0x1D,
            CTRL_REG4           =   0x1E,
            CTRL_REG5_XL        =   0x1F,
            CTRL_REG6_XL        =   0x20,
            CTRL_REG7_XL        =   0x21,
            CTRL_REG8           =   0x22,
            CTRL_REG9           =   0x23,
            //CTRL_REG10          =   0x24, //per self-test ma n.u.
            //INT_GEN_SRC_XL      =   0x26,
            STATUS_REG_XL       =   0x27,   // per check stato
            OUT_X_L_XL          =   0x28,
            OUT_X_H_XL          =   0x29,
            OUT_Y_L_XL          =   0x2A,
            OUT_Y_H_XL          =   0x2B,
            OUT_Z_L_XL          =   0x2C,
            OUT_Z_H_XL          =   0x2D,
            FIFO_CTRL           =   0x2E,
            FIFO_SRC            =   0x2F //FIFO status register
            //INT_GEN_CFG_G       =   0x30,
            //INT_GEN_THS_XH_G    =   0x31,
            //INT_GEN_THS_XL_G    =   0x32,
            //INT_GEN_THS_YH_G    =   0x33,
            //INT_GEN_THS_YL_G    =   0x34,
            //INT_GEN_THS_ZH_G    =   0x35,
            //INT_GEN_THS_ZL_G    =   0x36,
            //INT_GEN_DUR_G       =   0x37
        };
};
