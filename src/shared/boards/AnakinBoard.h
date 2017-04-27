#ifndef BOARDS_ANAKINBOARD_H
#define BOARDS_ANAKINBOARD_H

#include "Board.h"

typedef Gpio<GPIOA_BASE, 5> GpioSck;
typedef Gpio<GPIOA_BASE, 6> GpioMiso;
typedef Gpio<GPIOA_BASE, 7> GpioMosi;
typedef BusSPI < 1, GpioMosi, GpioMiso, GpioSck> busSPI1;

typedef Gpio<GPIOD_BASE, 13> CS_MPU9250;
typedef Gpio<GPIOG_BASE, 9> CS_INEMO_G;
typedef Gpio<GPIOG_BASE, 11> CS_INEMO_A;
typedef Gpio<GPIOG_BASE, 10> CS_FXAS21002;
typedef Gpio<GPIOE_BASE, 4> CS_LPS331AP;

typedef ProtocolSPI<busSPI1, CS_MPU9250> spiMPU9250;
typedef ProtocolSPI<busSPI1, CS_INEMO_A> spiINEMOA;
typedef ProtocolSPI<busSPI1, CS_INEMO_G> spiINEMOG;
typedef ProtocolSPI<busSPI1, CS_FXAS21002> spiFXAS21002;
typedef ProtocolSPI<busSPI1, CS_LPS331AP> spiLPS331AP;

typedef MPU9250<spiMPU9250> mpu_t;
typedef iNEMOLSM9DS0<spiINEMOG,spiINEMOA> inemo_t;
typedef FXAS21002<spiFXAS21002> fxas_t;
typedef LPS331AP<spiLPS331AP> lps331ap_t;


// AnakinSensor numbers must be in range 0 <= x <= 65535 (uint16_t)
enum AnakinSensor
{
    MPU_9250_ACCEL   = 0,
    MPU_9250_GYRO    = 1,
    MPU_9250_COMPASS = 2,
    MPU_9250_TEMP    = 3,

    INEMO_ACCEL      = 4,
    INEMO_GYRO       = 5,
    INEMO_COMPASS    = 6,
    INEMO_TEMP       = 7,
    FXAS_GYRO        = 8,

    LPS331AP_PRESS   = 9,
    LPS331AP_TEMP    = 10,
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
    SimpleSensorSampler m100HzSimple, m10HzSimple;
    mpu_t*      mS_MPU9250;
    inemo_t*    mS_INEMO;
    fxas_t*     mS_FXAS;
    lps331ap_t* mS_LPS331AP;

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
