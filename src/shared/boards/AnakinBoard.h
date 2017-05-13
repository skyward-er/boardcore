#ifndef BOARDS_ANAKINBOARD_H
#define BOARDS_ANAKINBOARD_H

#include "Board.h"
#include <events/Scheduler.h>

#include <sensors/iNemo.h>
#include <sensors/FXAS21002.h>
#include <sensors/MPU9250.h>
#include <sensors/LPS331AP.h>
#include <sensors/MAX21105.h>
#include <sensors/MS580301BA07.h>

#include <DMA/DMA.h>
#include <DMA/SensorSampling.h>

typedef Gpio<GPIOA_BASE, 5> GpioSck;
typedef Gpio<GPIOA_BASE, 6> GpioMiso;
typedef Gpio<GPIOA_BASE, 7> GpioMosi;
typedef BusSPI < 1, GpioMosi, GpioMiso, GpioSck> busSPI1;

typedef ProtocolSPI<busSPI1, Gpio<GPIOD_BASE, 13>> spiMPU9250;
typedef ProtocolSPI<busSPI1, Gpio<GPIOG_BASE, 11>> spiINEMOA;
typedef ProtocolSPI<busSPI1, Gpio<GPIOG_BASE,  9>> spiINEMOG;
typedef ProtocolSPI<busSPI1, Gpio<GPIOG_BASE, 10>> spiFXAS21002;
typedef ProtocolSPI<busSPI1, Gpio<GPIOE_BASE,  4>> spiLPS331AP;
typedef ProtocolSPI<busSPI1, Gpio<GPIOE_BASE,  2>> spiMAX21105;
typedef ProtocolSPI<busSPI1, Gpio<GPIOB_BASE, 10>> spiMS580301BA07;
//typedef ProtocolSPI<busSPI1, Gpio<GPIOB_BASE, 11>> spiMAX31856;

typedef MPU9250<spiMPU9250> mpu_t;
typedef iNEMOLSM9DS0<spiINEMOG,spiINEMOA> inemo_t;
typedef FXAS21002<spiFXAS21002> fxas_t;
typedef LPS331AP<spiLPS331AP> lps331ap_t;
typedef MAX21105<spiMAX21105> max21105_t;
typedef MS580301BA07<spiMS580301BA07> ms580_t;

// AnakinSensor numbers must be in range 0 <= x <= 65535 (uint16_t)
enum AnakinSensor
{
    ACCEL_MPU9250    = 0,
    ACCEL_INEMO      = 1,
    ACCEL_MAX21105   = 2,

    GYRO_MPU9250     = 3,
    GYRO_INEMO       = 4,
    GYRO_FXAS21002   = 5,
    GYRO_MAX21105    = 6,

    COMPASS_MPU9250  = 7,
    COMPASS_INEMO    = 8,

    TEMP_MPU9250     = 9,
    TEMP_INEMO       = 10,
    TEMP_LPS331AP    = 11,
    TEMP_MAX21105    = 12,
    TEMP_MS580       = 13,

    PRESS_LPS331AP   = 14,
    PRESS_MS580      = 15,

    UNUSED_16        = 16,
    RESERVED_17      = 17,
    RESERVED_18      = 18,
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
    mpu_t*      mS_MPU9250;
    inemo_t*    mS_INEMO;
    fxas_t*     mS_FXAS;
    lps331ap_t* mS_LPS331AP;
    max21105_t* mS_MAX21105;
    ms580_t*    mS_MS580;

    AnakinBoard();

    AnakinBoard(const AnakinBoard&) = delete;
    AnakinBoard(AnakinBoard&&) = delete;
    AnakinBoard& operator=(const AnakinBoard&) = delete;
    AnakinBoard& operator=(AnakinBoard&&) = delete;
};

#ifdef sBoard
#error YOU ARE TRYING TO USE MULTIPLE BOARDS.
#else
#define sBoard AnakinBoard::getInstance()
#endif

#endif /* ifndef BOARDS_ANAKINBOARD_H */
