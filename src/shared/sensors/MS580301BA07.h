/* MS580301BA07 Driver
 *
 * Copyright (c) 2015 Skyward Experimental Rocketry
 * Authors: Davide Benini, Matteo Piazzolla
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
#include <BusTemplate.h>
#include <Common.h>
#include <data.h>



typedef struct {
    uint16_t sens;
    uint16_t off;
    uint16_t tcs;
    uint16_t tco;
    uint16_t tref;
    uint16_t tempsens;
} calibration_data;




class RegMap_MS580301BA07
{
public:


    static constexpr uint8_t RESET_DEV                 = 0x1E;

    static constexpr uint8_t CONVERT_D1_256            = 0x40;
    static constexpr uint8_t CONVERT_D1_512            = 0x42;
    static constexpr uint8_t CONVERT_D1_1024           = 0x44;
    static constexpr uint8_t CONVERT_D1_2048           = 0x46;
    static constexpr uint8_t CONVERT_D1_4096           = 0x48;

    static constexpr uint8_t CONVERT_D2_256            = 0x50;
    static constexpr uint8_t CONVERT_D2_512            = 0x52;
    static constexpr uint8_t CONVERT_D2_1024           = 0x54;
    static constexpr uint8_t CONVERT_D2_2048           = 0x56;
    static constexpr uint8_t CONVERT_D2_4096           = 0x58;

    static constexpr uint8_t ADC_READ                  = 0x00;

    static constexpr uint8_t PROM_READ_MASK            = 0xA0;
    static constexpr uint8_t PROM_SENS_MASK            = 0x02;
    static constexpr uint8_t PROM_OFF_MASK             = 0x04;
    static constexpr uint8_t PROM_TCS_MASK             = 0x06;
    static constexpr uint8_t PROM_TCO_MASK             = 0x08;
    static constexpr uint8_t PROM_TREF_MASK            = 0x0A;
    static constexpr uint8_t PROM_TEMPSENS_MASK        = 0xC;


};

template<class RegMap>
class MS580301BA07 : public Sensor<ProtocolSPI<BusSPI<2>, 3>>
{
    // assert bus class here
public:

  calibration_data cd ={0};
  uint32_t ref_pressure=0;


    MS580301BA07(){
        Init();
        Calibrate()
    }


    uint16_t ReadCalibrationRegister(uint8_t reg){
      uint8_t rcv[2];
      GpioCS::low();
      reg |= 0x80;
      Bus::Write(&reg, sizeof(reg));
      Bus::Read(&rcv, 2);
      uint16_t data = rcv[0]<<8 | rcv[1];
      GpioCS::high();
      return data;
    }

    void Init(){
      do{

      SensorReset();
      Sensor::ReadReg(RegMap::RESET_DEV);

      cd.sens = ReadCalibrationRegister(RegMap::PROM_READ_MASK | RegMap::PROM_SENS_MASK);
      } while(cd.sens==0);
      cd.off  = ReadCalibrationRegister(RegMap::PROM_READ_MASK | RegMap::PROM_OFF_MASK);
      cd.tcs  = ReadCalibrationRegister(RegMap::PROM_READ_MASK | RegMap::PROM_TCS_MASK);
      cd.tco  = ReadCalibrationRegister(RegMap::PROM_READ_MASK | RegMap::PROM_TCO_MASK);
      cd.tref = ReadCalibrationRegister(RegMap::PROM_READ_MASK | RegMap::PROM_TREF_MASK);
      cd.tempsens = ReadCalibrationRegister(RegMap::PROM_READ_MASK | RegMap::PROM_TEMPSENS_MASK);
    }


    void SensorReset(){
        Sensor::ReadReg(RegMap::RESET_DEV);
    }


    uint32_t ReadPressure(){

      uint32_t pressure=0;
      uint32_t temperature =0;

      //TODO: dubbio, non devo fare un ADC read sola?
        do{
        GpioCS::low();
        Bus::Write(RegMap::CONVERT_D1_4096,sizeof(RegMap::CONVERT_D1_4096));
        Bus::Write(RegMap::ADC_READ,sizeof(RegMap::ADC_READ));
        uint32_t rcvbuf;
        Bus::Read(&reg, 3);
        GpioCS::high();

            pressure = (uint32_t)rcvbuf[2] | (((uint32_t)rcvbuf[1])<<8) | (((uint32_t)rcvbuf[0])<<16);
        }while(pressure==0);

        do{
        GpioCS::low();
        Bus::Write(RegMap::CONVERT_D2_4096,sizeof(RegMap::CONVERT_D2_4096));
        Bus::Write(RegMap::ADC_READ,sizeof(RegMap::ADC_READ));
        uint32_t rcvbuf;
        Bus::Read(&reg, 3);
            temperature = (uint32_t)rcvbuf[2] | (((uint32_t)rcvbuf[1])<<8) | (((uint32_t)rcvbuf[0])<<16);
        } while(temperature==0);

        int32_t dt = temperature - cd.tref*pow(2,8);
        comp_temp = 2000 + dt*(cd.tempsens)/pow(2,23);
        int64_t offs = (cd.off*pow(2,16))+((cd.tco*dt)/pow(2,7));
        int64_t senst =  cd.sens*pow(2,15) + (cd.tcs*dt)/pow(2,8);
        comp_pressure = (pressure*senst/pow(2,21)-offs)/pow(2,15);
        return comp_pressure;
    }

    void SetReferencePressure(uint32_t ref){
        ref_pressure = ref;
    }

    uint32_t ReadAltitude(){
        ReadPressure();
        altitude =(int32_t)(44330769 - 44330769*pow(((double)comp_pressure/ref_pressure),0.1902));
        return altitude;
    }



    bool SelfTest(){
      //TODO: test
      return false;
    }






};
