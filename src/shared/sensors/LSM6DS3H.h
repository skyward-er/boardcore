/*  LSM6DS3H Driver
 *
 * Copyright (c) 2018-2019 Skyward Experimental Rocketry
 * Authors: Soufiane Machkouk
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
#include <drivers/BusTemplate.h>
#include "Common.h"
#include "Sensor.h"
template <typename BusG>
class LSM6DS3H : public GyroSensor, public AccelSensor
{
public:
    LSM6DS3H(uint8_t accelFullScale, uint16_t gyroFullScale)
    {
        accelFS = accelFullScale;
        gyroFS  = gyroFullScale;
    }
    bool init()
    {
        selfTest();
        BusG::write(RegMap::CTRL5_C, 0x60);  //  Rounding read enabled for both accel and gyro
        BusG::write(RegMap::CTRL3_C, 0x44 ); // incrementare l'indirizzo ad ogni lettura data LSB @ lower address
        fifoConfiguration();
        accelConfiguration();
        gyroConfiguration();
        // TODO
        uint8_t XLSamplesToDiscard=2;   //
        uint8_t gyroSamplesToDiscard=2; //since we set ODR to 416HZ
        uint8_t buffer[2]={0, 0};
        BusG::read(RegMap::FIFO_DATA_OUT_L, buffer, XLSamplesToDiscard);
        BusG::read(RegMap::FIFO_DATA_OUT_L, buffer, gyroSamplesToDiscard);
        return true;
    }
    void fifoConfiguration()
    {
        uint8_t continuousMode = 0x36;  // ODR is set to 416HZ
        BusG::write(RegMap::FIFO_CTRL_REG5, continuousMode);
        uint8_t bitToSet= BusG::read(RegMap::FIFO_CTRL_REG2);
        bitToSet = bitToSet & 0xcf;
        BusG::write(RegMap::FIFO_CTRL_REG2, bitToSet);
        uint8_t noDecimation = 0x09;
        BusG::write(RegMap::FIFO_CTRL_REG3, noDecimation);  // NO decimation for both accel and gyro data
        bitToSet = 0x00;
        BusG::write(RegMap::FIFO_CTRL_REG5, bitToSet);
        uint8_t registerStatus = BusG::read(RegMap::FIFO_STATUS4);
        bitToSet = 0x03;
        bitToSet = bitToSet & registerStatus;
        BusG::write(RegMap::FIFO_STATUS4, bitToSet);
    }
    bool selfTest()
    {
        uint8_t whoami = BusG::read(RegMap::WHO_AM_I);
        if (whoami != whoami_value)
        {
            last_error = ERR_NOT_ME;
            return false;
        }
        return true;
    }
    // da settare la frequenza di taglio del passabasso
    void accelConfiguration()
    {
        uint8_t i=0;
        uint8_t controlAccel = 0x61; //da vedere
        for (i = 0; i<=3; i++)
        {
            if ((uint8_t)accelFSMAP[i] == accelFS)
            {
                controlAccel = controlAccel | (i << 2);
                BusG::write(RegMap::CTRL1_XL, controlAccel);  // anti-aliasing filter set to 200Hz
                                            // ODR to 416HZ
            }
        }
        // TODO
        // imporre il dato sapendo che 00=ODR_XL/50; 01=ODR_XL/100;10=ODR_XL/9;
        // 11=ODR_XL/400 ---->ODR è impostato a 416 hz
        uint16_t LPFCutOff=0;
        LPFCutOff          = (LPFCutOff << 7) & 0x84;
        BusG::write(RegMap::CTRL8_XL, LPFCutOff);
        uint8_t enableXYZ = 0x38;
        BusG::write(RegMap::CTRL9_XL, enableXYZ);
    }
    void gyroConfiguration()
    {
        uint8_t i=0;
        uint8_t controlGyro = 0x60;
        for(i=0; i<=3; i++)
        {
            if ((uint16_t)gyroFSMAP[i] == gyroFS)
            {
                controlGyro = controlGyro | (i << 2);
                BusG::write(RegMap::CTRL2_G, controlGyro);
            }
        }
        uint8_t enableXYZ = 0x39;
        BusG::write(RegMap::CTRL10_C, enableXYZ);
        //HPfilter disabled
    }
    bool onSimpleUpdate() { return false; }
    void sampleUpdate()
    {
        //gyro data set ---> lettura da 6 byte
        BusG::read(FIFO_DATA_OUT_L, datiGyro, 6);
        // accel dataset ---> lettura da 6 byte
        BusG::read(FIFO_DATA_OUT_L, datiAccel, 6);
        Data.gyroXSample   = toFloat(datiGyro[0]+datiGyro[1]<<8, gyroFS);
        Data.gyroYSample   = toFloat(datiGyro[2]+datiGyro[3]<<8, gyroFS);
        Data.gyroZSample   = toFloat(datiGyro[4]+datiGyro[5]<<8, gyroFS);
        Data.accelXSample  = toFloat(datiAccel[0]+datiAccel[1]<<8, accelFS);
        Data.accelYSample  = toFloat(datiAccel[2]+datiAccel[3]<<8, accelFS);
        Data.accelZSample  = toFloat(datiAccel[4]+datiAccel[5]<<8, accelFS);
       
    }
    /*bool accelSelfTest(){
         uint8_t dataAvailable[2];
         uint8_t c=0;
         uint16_t rxData[2];
         float OUTX_NOST;
         float OUTY_NOST;
         float OUTZ_NOST;
         float OUTX_ST;
         float OUTY_ST;
         float OUTZ_ST;
         BusG::write(RegMap::CTRL1_XL, 0x30);
         BusG::write(RegMap::CTRL2_G, 0x00);
         BusG::write(RegMap::CTRL3_C, 0x44);
         BusG::write(RegMap::CTRL4_C, 0x00);
         BusG::write(RegMap::CTRL5_C, 0x00);
         BusG::write(RegMap::CTRL6_G, 0x00);
         BusG::write(RegMap::CTRL7_G, 0x00);
         BusG::write(RegMap::CTRL8_XL, 0x00);
         BusG::write(RegMap::CTRL9_XL, 0x38);
         BusG::write(RegMap::CTRL10_C, 0x00);
         //delay (200 ms);
         BusG::read(STATUS_REG, &dataAvailable[0]);
         while (dataAvailable[0] != 0x07 & dataAvailable[0] != 0x01 &dataAvailable[0] != 0x03 )
         BusG::read(RegMap::OUT_X_L_XL, dataAvailable, 2);
         BusG::read(RegMap::OUT_Y_L_XL, dataAvailable, 2);
         BusG::read(RegMap::OUT_Z_L_XL, dataAvailable, 2);
         BusG::read(STATUS_REG, &dataAvailable[0]);
         while ((dataAvailable[0] != 0x07 & dataAvailable[0] != 0x01 &dataAvailable[0] != 0x03) & c>=5 ){
             BusG::read(STATUS_REG, &dataAvailable[0]);
             c++;
         }
         BusG::read(RegMap::OUT_X_L_XL, rxData, 2);
         OUTX_NOST = toFloat(rxData[0]+rxData[1]<<8, 2); 
         BusG::read(RegMap::OUT_Y_L_XL, rxData, 2);
         OUTY_NOST = toFloat(rxData[0]+rxData[1]<<8, 2);
         BusG::read(RegMap::OUT_Z_L_XL, rxData, 2);
         OUTZ_NOST = toFloat(rxData[0]+rxData[1]<<8, 2);
         //average the data on each axis
         BusG::write(RegMap::CTRL5_C, 0x01);
         //delay(200 ms)

         BusG::read(STATUS_REG, &dataAvailable[0]);
         while (dataAvailable[0] != 0x07 & dataAvailable[0] != 0x01 &dataAvailable[0] != 0x03 )
         BusG::read(RegMap::OUT_X_L_XL, dataAvailable, 2);
         BusG::read(RegMap::OUT_Y_L_XL, dataAvailable, 2);
         BusG::read(RegMap::OUT_Z_L_XL, dataAvailable, 2);
         BusG::read(STATUS_REG, &dataAvailable[0]);
         while ((dataAvailable[0] != 0x07 & dataAvailable[0] != 0x01 &dataAvailable[0] != 0x03) & c>=5 ){
             BusG::read(STATUS_REG, &dataAvailable[0]);
             c++;
         }
         BusG::read(RegMap::OUT_X_L_XL, rxData, 2);
         OUTX_ST = toFloat(rxData[0]+rxData[1]<<8, 2); 
         BusG::read(RegMap::OUT_Y_L_XL, rxData, 2);
         OUTY_ST = toFloat(rxData[0]+rxData[1]<<8, 2);
         BusG::read(RegMap::OUT_Z_L_XL, rxData, 2);
         OUTZ_ST = toFloat(rxData[0]+rxData[1]<<8, 2);
         //average the data on each axis
         if (){
             return true;
         }
         else {
             return false;
         }
    }
    bool accelSelfTest(){

    }
    */
    float toFloat(uint16_t var, uint16_t FScale){
        float out;
        if(var > 0x7FFF){
            out= var-0xFFFF;
        }
        else {
            out= var;
        }
        out = out*FScale/0x8000;
        return out;
    }
    uint8_t datiGyro[6]  = {0, 0, 0, 0, 0, 0};
    uint8_t datiAccel[6] = {0, 0, 0, 0, 0, 0};

    struct Samples
    {
        float gyroXSample;
        float gyroYSample;
        float gyroZSample;
        float accelXSample;
        float accelYSample;
        float accelZSample;
    };
    Samples Data;

private:
    uint8_t accelFS;
    uint16_t gyroFS;
    constexpr static uint8_t whoami_value = 0x69;

    uint8_t accelFSMAP[4] = {2, 16, 4, 8};
    uint16_t gyroFSMAP[4]  = {250, 500, 1000, 2000};
    enum RegMap {
        WHO_AM_I = 0x0F,  // default value
        // FIFO configuration registers
        FIFO_CTRL_REG1 = 0x06,
        FIFO_CTRL_REG2 = 0x07,
        FIFO_CTRL_REG3 = 0x08,
        FIFO_CTRL_REG4 = 0x09,
        FIFO_CTRL_REG5 = 0x0A,

        // Accelerometer and gyroscope control registers
        CTRL1_XL = 0x10,
        CTRL2_G  = 0x11,
        CTRL3_C  = 0x12,
        CTRL4_C  = 0x13,
        CTRL5_C  = 0x14, 
        CTRL6_C  = 0x15, 
        CTRL7_G  = 0x16, 
        CTRL8_XL = 0x17, 
        CTRL9_XL = 0x18, 
        CTRL10_C = 0x19,

        // Accelerometer output registers

        OUT_X_L_XL = 0x28,
        OUT_X_H_XL = 0x29,
        OUT_Y_L_XL = 0x2A,
        OUT_Y_H_XL = 0x2B,
        OUT_Z_L_XL = 0x2C,
        OUT_Z_H_XL = 0x2D,

        // gyro data registers
        OUT_X_L_G = 0x22,
        OUT_X_H_G = 0x23,
        OUT_Y_L_G = 0x24,
        OUT_Y_H_G = 0x25,
        OUT_Z_L_G = 0x26,
        OUT_Z_H_G = 0x27,

        // FIFO status registers
        FIFO_STATUS1 = 0x3A,
        FIFO_STATUS2 = 0x3B,
        FIFO_STATUS3 = 0x3C,
        FIFO_STATUS4 = 0x3D,

        // FIFO data output registers
        FIFO_DATA_OUT_L = 0x3E,
        FIFO_DATA_OUT_H = 0x3F
    };
};