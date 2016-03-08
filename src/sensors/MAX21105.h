/* MAX21105 Driver
 *
 * Copyright (c) 2015 Skyward Experimental Rocketry
 * Authors: Matteo Michele Piazzolla
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

#include "BusTemplate.h"
#include <stdint.h>
#include "Data.h"
#include <cmath>
/*
indirizzi dei registri a 6 bit
per scrivere in un registro bisogna settare il bit più significativo dell'indirizzo
a 0 e per leggere a 1
il bit subito dopo è un bit di parità oppure per burst operation

*/

class RegMap_MAX21105 {
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

    static constexpr uint8_t MIF_CFG        = 0x16;




    SET_PWR
};

template<class RegMap>
class IMUMax21105 : public Sensor< ProtocolSPI<BusSPI<2>, 3> > {
    // assert bus class here
public:
    IMUMax21105() {
        Init();
    }

    uint8_t ReadWhoAmI() {
        return ReadReg(RegMap_MAX21105::WHO_AM_I);
    }

    void Init() {
        //seleziono il bank 0 dei registri
        WriteReg(RegMap_MAX21105::EXT_STATUS,0x00);
        //Power Down
        WriteReg(RegMap_MAX21105::SET_PWR,0x78);

        // Gyro: 2kHz BW, 1000dps FS
        WriteReg(RegMap_MAX21105::SNS_CFG_1,0x3c);
        WriteReg(RegMap_MAX21105::SNS_CFG_2,0x10);

        //Acc 16g FS, no self test
        WriteReg(RegMap_MAX21105::ACC_CFG_1,0x00);
        // no high-pass, unfiltered
        WriteReg(RegMap_MAX21105::ACC_CFG_2,0x00);

        // Acc Low-Noise + Gyro Low-Noise
        WriteReg(RegMap_MAX21105::SET_PWR,0x78);

    }

    SampleGyro ReadGyro() {
        float x = ReadGyroX();
        float y = ReadGyroY();
        float z = ReadGyroZ();
        return SampleGyro(x,y,z);
    }

    SampleAccelerometer ReadAccelerometer() {
        float x = ReadAccX();
        float y = ReadAccY();
        float z = ReadAccZ();
        return SampleAccelerometer(x,y,z);
    }

    inline uint16_t ReadAccX() {
        return ((ReadReg(RegMap::ACCE_X_H) <<8) | ReadReg(RegMap::ACCE_X_L))*1.0 ;
    }

    inline uint16_t ReadAccY() {
        return ((ReadReg(RegMap::ACCE_Y_H) <<8) | ReadReg(RegMap::ACCE_Y_L))*1.0 ;
    }

    inline uint16_t ReadAccZ() {
        return ((ReadReg(RegMap::ACCE_Z_H) <<8) | ReadReg(RegMap::ACCE_Z_L))*1.0 ;
    }


    inline uint16_t ReadGyroX() {
        return ((ReadReg(RegMap::GYRO_X_H) <<8) | ReadReg(RegMap::GYRO_X_L))*1.0 ;
    }

    inline uint16_t ReadGyroY() {
        return ((ReadReg(RegMap::GYRO_Y_H) <<8) | ReadReg(RegMap::GYRO_Y_L))*1.0 ;
    }

    inline uint16_t ReadGyroZ() {
        return ((ReadReg(RegMap::GYRO_Z_H) <<8) | ReadReg(RegMap::GYRO_Z_L))*1.0 ;
    }

    bool SelfTest() {
        bool flag=false;
        if(!SelfTestAcc()) {
            std::cout << "IMUMax21105 Accelerometer not working" << std::endl;
            flag=true;
        }
        if(!SelfTestGyro()) {
            std::cout << "IMUMax21105 Gyroscope not working" << std::endl;
            flag=true;
        }
        return flag;
    }

//TODO: capire se ha senso
/*
Self Test del Gyroscopio,
Prendo 5 campioni del giroscopio e ne faccio la media, 
imposto la modalità self test e ne prendo altri 5 facendone nuovamente la media.
Se la distanza è maggiore di quella nel datasheet o non passa la condizione
X>0, Y<0, Z>0 il test fallisce e ritorna falso
*/ 
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
            Ax_no_test+=ReadGyroX();
            Ay_no_test+=ReadGyroY();
            Az_no_test+=ReadGyroZ();
        }

        Ax_no_test=Ax_no_test/5;
        Ay_no_test=Ay_no_test/5;
        Az_no_test=Az_no_test/5;

        //seleziono il bank 0 dei registri
        WriteReg(RegMap_MAX21105::EXT_STATUS,0x00);
        //Power Down
        WriteReg(RegMap_MAX21105::SET_PWR,0x78);
        // Gyro: 2kHz BW, 1000dps FS 
        //Testing Positive sign {+X, -Y, +Z}
        WriteReg(RegMap_MAX21105::SNS_CFG_1,0x7c);
        WriteReg(RegMap_MAX21105::SNS_CFG_2,0x10);
        // Acc Low-Noise + Gyro Low-Noise
        WriteReg(RegMap_MAX21105::SET_PWR,0x78);
        //sleep
        
        float Ax_test=0;
        float Ay_test=0;
        float Az_test=0;

        for(int i=0;i<5;++i){
            Ax_test+=ReadGyroX();
            Ay_test+=ReadGyroY();
            Az_test+=ReadGyroZ();
        }
        
        Ax_test=Ax_test/5;
        Ay_test=Ay_test/5;
        Az_test=Az_test/5;
        
        float deltaX,deltaY,deltaZ;
        deltaX = std::abs(Ax_st - Ax_no_test);
        deltaY = -std::abs(Ay_st - Ay_no_test);
        deltaZ = std::abs(Az_st - Az_no_test);

        if ((min_x <= deltaX && deltaX <= max_x) && 
            (min_y <= deltaY && deltaY <= max_y) && 
            (min_z <= deltaZ && deltaZ <= max_z))
            return true;
        return false;
    }

    bool SelfTestAcc(){
        //16g X positive force
        WriteReg(RegMap_MAX21105::SET_ACC_PWR,0x8);
    }

    inline bool Test() const override { 
        return who_am_i_value == ReadWhoAmI(); 
    }

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
