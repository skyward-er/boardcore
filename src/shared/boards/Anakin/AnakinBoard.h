/* Copyright (c) 2015-2017 Skyward Experimental Rocketry
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

#ifndef BOARDS_ANAKINBOARD_H
#define BOARDS_ANAKINBOARD_H

#include <events/Scheduler.h>
#include <diagnostic/Log.h>
#include "Board.h"

#include <sensors/FXAS21002.h>
#include <sensors/LPS331AP.h>
#include <sensors/MAX21105.h>
#include <sensors/MPU9250.h>
#include <sensors/MS580301BA07.h>
#include <sensors/iNemo.h>

#include <drivers/spi/SensorSpi.h>
#include <sensors/SensorSampling.h>

typedef miosix::Gpio<GPIOA_BASE, 5> GpioSck;
typedef miosix::Gpio<GPIOA_BASE, 6> GpioMiso;
typedef miosix::Gpio<GPIOA_BASE, 7> GpioMosi;
typedef BusSPI<1, GpioMosi, GpioMiso, GpioSck> busSPI1;

typedef ProtocolSPI<busSPI1, miosix::Gpio<GPIOD_BASE, 13>> spiMPU9250;
typedef ProtocolSPI<busSPI1, miosix::Gpio<GPIOG_BASE, 11>> spiINEMOA;
typedef ProtocolSPI<busSPI1, miosix::Gpio<GPIOG_BASE, 9>> spiINEMOG;
typedef ProtocolSPI<busSPI1, miosix::Gpio<GPIOG_BASE, 10>> spiFXAS21002;
typedef ProtocolSPI<busSPI1, miosix::Gpio<GPIOE_BASE, 4>> spiLPS331AP;
typedef ProtocolSPI<busSPI1, miosix::Gpio<GPIOE_BASE, 2>> spiMAX21105;
typedef ProtocolSPI<busSPI1, miosix::Gpio<GPIOB_BASE, 10>> spiMS580301BA07;
// typedef ProtocolSPI<busSPI1, miosix::Gpio<GPIOB_BASE, 11>> spiMAX31856;

typedef MPU9250<spiMPU9250> mpu_t;
typedef iNEMOLSM9DS0<spiINEMOG, spiINEMOA> inemo_t;
typedef FXAS21002<spiFXAS21002> fxas_t;
typedef LPS331AP<spiLPS331AP> lps331ap_t;
typedef MAX21105<spiMAX21105> max21105_t;
typedef MS580301BA07<spiMS580301BA07> ms580_t;

// AnakinSensor numbers must be in range 0 <= x <= 65535 (uint16_t)
enum AnakinSensor
{
    ACCEL_MPU9250  = 0,
    ACCEL_INEMO    = 1,
    ACCEL_MAX21105 = 2,

    GYRO_MPU9250   = 3,
    GYRO_INEMO     = 4,
    GYRO_FXAS21002 = 5,
    GYRO_MAX21105  = 6,

    COMPASS_MPU9250 = 7,
    COMPASS_INEMO   = 8,

    TEMP_MPU9250  = 9,
    TEMP_INEMO    = 10,
    TEMP_LPS331AP = 11,
    TEMP_MAX21105 = 12,
    TEMP_MS580    = 13,

    PRESS_LPS331AP = 14,
    PRESS_MS580    = 15,

    UNUSED_16   = 16,
    RESERVED_17 = 17,
    RESERVED_18 = 18,
};

class AnakinBoard : public Singleton<AnakinBoard>, public Board
{
    friend class Singleton<AnakinBoard>;

public:
    bool init() override;

    const std::vector<SingleSensor>& debugGetSensors() const
    {
        return mSensorData;
    }

private:
    DMASensorSampler m100HzDMA, m25HzDMA;
    SimpleSensorSampler m10HzSimple;
    mpu_t* mS_MPU9250;
    inemo_t* mS_INEMO;
    fxas_t* mS_FXAS;
    lps331ap_t* mS_LPS331AP;
    max21105_t* mS_MAX21105;
    ms580_t* mS_MS580;

    AnakinBoard();
};

#ifdef sBoard
#error YOU ARE TRYING TO USE MULTIPLE BOARDS.
#else
#define sBoard AnakinBoard::getInstance()
#endif

#endif /* ifndef BOARDS_ANAKINBOARD_H */
