#include "AnakinBoard.h"

#define INIT_AND_CHECK(x) do {                                       \
    if (!x->init()) { sLog->logString("=== ERR: CANNOT INIT " #x); } \
} while(0)

AnakinBoard::AnakinBoard()
{
    mInited = false;
    mS_MPU9250   = nullptr;
    mS_INEMO     = nullptr;
    mS_FXAS      = nullptr;
    mS_LPS331AP  = nullptr;
    mS_MAX21105  = nullptr;
}

bool AnakinBoard::init()
{
    if(mInited)
        return false;

    mInited = true;
    mS_MPU9250   = new mpu_t        (mpu_t::ACC_FS_16G, 
                                     mpu_t::GYRO_FS_250);
    mS_INEMO     = new inemo_t      (inemo_t::ACC_FS_16G, 
                                     inemo_t::GYRO_FS_245,
                                     inemo_t::COMPASS_FS_2);
    mS_FXAS      = new fxas_t       (fxas_t::DPS500);
    mS_LPS331AP  = new lps331ap_t   (lps331ap_t::SS_25HZ);
    mS_MAX21105  = new max21105_t   (max21105_t::ACC_FS_16G, 
                                     max21105_t::GYRO_FS_250);
    mS_MS580     = new ms580_t();

    INIT_AND_CHECK(mS_MPU9250);
    INIT_AND_CHECK(mS_INEMO);
    INIT_AND_CHECK(mS_FXAS);
    INIT_AND_CHECK(mS_LPS331AP);
    INIT_AND_CHECK(mS_MAX21105);
    INIT_AND_CHECK(mS_MS580);

    AddSensor(MPU9250_ACCEL,    DATA_VEC3,     mS_MPU9250->accelDataPtr());
    AddSensor(MPU9250_GYRO,     DATA_VEC3,     mS_MPU9250->gyroDataPtr());
    AddSensor(MPU9250_COMPASS,  DATA_VEC3,     mS_MPU9250->compassDataPtr());
    AddSensor(MPU9250_TEMP,     DATA_FLOAT,    mS_MPU9250->tempDataPtr());

    AddSensor(INEMO_ACCEL,      DATA_VEC3,     mS_INEMO->accelDataPtr());
    AddSensor(INEMO_GYRO,       DATA_VEC3,     mS_INEMO->gyroDataPtr());
    AddSensor(INEMO_COMPASS,    DATA_VEC3,     mS_INEMO->compassDataPtr());
    AddSensor(INEMO_TEMP,       DATA_FLOAT,    mS_INEMO->tempDataPtr());
    AddSensor(FXAS21002_GYRO,   DATA_VEC3,     mS_FXAS->gyroDataPtr());

    AddSensor(LPS331AP_PRESS,   DATA_FLOAT,    mS_LPS331AP->pressureDataPtr());
    AddSensor(LPS331AP_TEMP,    DATA_FLOAT,    mS_LPS331AP->tempDataPtr());

    AddSensor(MAX21105_ACCEL,   DATA_VEC3,     mS_MAX21105->accelDataPtr());
    AddSensor(MAX21105_GYRO,    DATA_VEC3,     mS_MAX21105->gyroDataPtr());
    AddSensor(MAX21105_TEMP,    DATA_FLOAT,    mS_MAX21105->tempDataPtr());

    AddSensor(MS580_PRESSURE,   DATA_FLOAT,    mS_MS580->pressureDataPtr());
    AddSensor(MS580_TEMP,       DATA_FLOAT,    mS_MS580->tempDataPtr());

    sLog->logString("Adding sensors to 100Hz DMA sampler\n");
    m100HzDMA.AddSensor(mS_MPU9250);
    m100HzDMA.AddSensor(mS_INEMO);
    m100HzDMA.AddSensor(mS_FXAS);
    m100HzDMA.AddSensor(mS_MAX21105);

    sLog->logString("Adding sensors to 25Hz DMA sampler\n");
    m25HzDMA.AddSensor(mS_LPS331AP);

    sLog->logString("Adding sensors to 10Hz Simple sampler\n");
    m10HzSimple.AddSensor(mS_MPU9250);
    m10HzSimple.AddSensor(mS_FXAS);
    m10HzSimple.AddSensor(mS_MS580);

    sLog->logString("Adding samplers to scheduler\n");
    #define ADD_SAMPLER(type, name, rate) \
        sEventScheduler->add(std::bind(& type ## SensorSampler::Update,name),\
                rate, #name "-" #rate "ms")

    ADD_SAMPLER(DMA, m100HzDMA, 10); // 10ms
    ADD_SAMPLER(DMA, m25HzDMA, 40);  // 25ms
    ADD_SAMPLER(Simple, m100HzSimple, 10); // 10ms
    ADD_SAMPLER(Simple, m10HzSimple, 100); // 100ms

    return true;
}

#undef INIT_AND_CHECK
#undef ADD_SAMPLER
