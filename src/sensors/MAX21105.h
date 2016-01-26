/* MAX21105 Driver
 *
 * Copyright (c) 2015 Skyward Experimental Rocketry
 * Authors: Matteo Piazzolla
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
#include <stdint.h>
#include "data.h"
#include <ios>
#include "stdio.h"
/*
indirizzi dei registri a 6 bit
per scrivere in un registro bisogna settare il bit più significativo dell'indirizzo
a 0 e per leggere a 1
il bit subito dopo è un bit di parità oppure per burst operation

*/


class Sensor{
public:
    void static WriteReg(uint8_t reg, uint8_t value){
        bool rw = (0x80 & reg)>>7;
        printf("0w 1r: %d  ", rw);
        bool parity = (0x40 & reg)>>6;
        printf("parity %d\n", parity );
       // std::cout << "Write to Address: "<< std::hex << reg << " Value: "<< value << std::endl;
        printf("Write to Address: Ox%02x  Value: Ox%02x \n", (0x3f & reg),value);
    }
    uint8_t static ReadReg(uint8_t reg){
        bool rw = (0x80 & reg)>>7;
        printf("0 write 1 read: %d\n", rw);
        bool parity = (0x40 & reg)>>6;
        printf("parity %d\n", parity );


        char ch[8];
        //std::cout << "Read from Address " << std::hex << reg << std::endl;
        printf("Read from Address Ox%02x\n", (0x3f & reg));

        std::cin >> ch;
        int v = atoi(ch);
        printf("scritto Ox%02x\n", (0xff & v));

        return (0xFF & v);
    }

};

class RegMap_MAX21105
{
public:
    static constexpr uint8_t WHO_AM_I       = 0x20; //default value 0xb4
    static constexpr uint8_t EXT_STATUS     = 0x22;

    static constexpr uint8_t GYRO_X_H       = 0x24;
    static constexpr uint8_t GYRO_X_L       = 0x25;
    static constexpr uint8_t GYRO_Y_H       = 0x26;
    static constexpr uint8_t GYRO_Y_L       = 0x27;
    static constexpr uint8_t GYRO_Z_H       = 0x28;
    static constexpr uint8_t GYRO_Z_L       = 0x29;
    static constexpr uint8_t ACCE_X_H       = 0x2A;
    static constexpr uint8_t ACCE_X_L       = 0x2B;
    static constexpr uint8_t ACCE_Y_H       = 0x2C;
    static constexpr uint8_t ACCE_Y_L       = 0x2D;
    static constexpr uint8_t ACCE_Z_H       = 0x2E;
    static constexpr uint8_t ACCE_Z_L       = 0x2F;
    static constexpr uint8_t TEMP_H         = 0x30;
    static constexpr uint8_t TEMP_L         = 0x31;

    static constexpr uint8_t TRM_BNK_REG    = 0x38;
    static constexpr uint8_t FIFO_COUNT     = 0x3C;
    static constexpr uint8_t FIFO_STATUS    = 0x3D;
    static constexpr uint8_t FIFO_DATA      = 0x3E;
    static constexpr uint8_t RST_REG        = 0x3F;

//Bank 00
    static constexpr uint8_t SET_PWR        = 0x00;
    static constexpr uint8_t SNS_CFG_1      = 0x01;
    static constexpr uint8_t SNS_CFG_2      = 0x02;
    static constexpr uint8_t SNS_CFG_3      = 0x03;
    static constexpr uint8_t SET_ACC_PWR    = 0x04;
    static constexpr uint8_t ACC_CFG_1      = 0x05;
    static constexpr uint8_t ACC_CFG_2      = 0x06;
    static constexpr uint8_t SET_TEMP_DR    = 0x13;


    static constexpr uint8_t OTP_STS_CFG    = 0x1C;

    static constexpr uint8_t MIF_CFG       = 0x16;


};

template<class RegMap>
//class IMUMax21105 : public Sensor<ProtocolSPI<BusSPI<2>, 3>>
class IMUMax21105 : public Sensor
{
    // assert bus class here
public:
    IMUMax21105(){
        //Init();
    }

    uint8_t ReadWhoAmI()
    {
        return ReadReg(RegMap::WHO_AM_I);
    }

    void Init(){
        //seleziono il bank 0 dei registri
        WriteReg(RegMap::EXT_STATUS,0x00);
        //Power Down
        WriteReg(RegMap::SET_PWR,0x78);

        // Gyro: 2kHz BW, 1000dps FS
        WriteReg(RegMap::SNS_CFG_1,0x3c);
        WriteReg(RegMap::SNS_CFG_2,0x10);

        //Acc 16g FS, no self test
        WriteReg(RegMap::ACC_CFG_1,0x00);
        // no high-pass, unfiltered
        WriteReg(RegMap::ACC_CFG_2,0x00);

        // Acc Low-Noise + Gyro Low-Noise
        WriteReg(RegMap::SET_PWR,0x78);



    }

/*
Self Test del Gyroscopio,
si forzano delle
*/



    SampleGyro ReadGyro(){
        float x = ReadGyroX();
        float y = ReadGyroY();
        float z = ReadGyroZ();
        return SampleGyro(x,y,z);
    }

    SampleAccelerometer ReadAccelerometer(){
        float x = ReadAccX();
        float y = ReadAccY();
        float z = ReadAccZ();
        return SampleAccelerometer(x,y,z);
    }

    inline uint16_t ReadAccX(){
        return ((ReadReg(RegMap::ACCE_X_H) <<8) | ReadReg(RegMap::ACCE_X_L))*1.0 ;
    }

    inline uint16_t ReadAccY(){
        return ((ReadReg(RegMap::ACCE_Y_H) <<8) | ReadReg(RegMap::ACCE_Y_L))*1.0 ;
    }

    inline uint16_t ReadAccZ(){
        return ((ReadReg(RegMap::ACCE_Z_H) <<8) | ReadReg(RegMap::ACCE_Z_L))*1.0 ;
    }


    inline uint16_t ReadGyroX(){
        return ((ReadReg(RegMap::GYRO_X_H) <<8) | ReadReg(RegMap::GYRO_X_L))*1.0 ;
    }

    inline uint16_t ReadGyroY(){
        return ((ReadReg(RegMap::GYRO_Y_H) <<8) | ReadReg(RegMap::GYRO_Y_L))*1.0 ;
    }

    inline uint16_t ReadGyroZ(){
        return ((ReadReg(RegMap::GYRO_Z_H) <<8) | ReadReg(RegMap::GYRO_Z_L))*1.0 ;
    }

    bool SelfTestAcc(){}
    bool SelfTestGyro(){}

    bool SelfTest(){
        bool flag=false;
        if(!SelfTestAcc()){
            std::cout << "IMUMax21105 Accelerometer not working" << std::endl;
            flag=true;
        }
        if(!SelfTestGyro()){
            std::cout << "IMUMax21105 Gyroscope not working" << std::endl;
            flag=true;
        }
        return flag;
    }

     bool Test()  { return who_am_i_value == ReadWhoAmI(); }

private:
    constexpr static uint8_t who_am_i_value = 0xb4;

    bool parity_bit=1;

    void WriteReg(uint8_t reg, uint8_t value){
        Sensor::WriteReg((0<<7) | (parity_bit << 6)| (reg & 0x3f), value);
    }
    uint8_t ReadReg(uint8_t reg){
        return Sensor::ReadReg((1<<7) | (parity_bit << 6) | (reg & 0x3f));
    }

};