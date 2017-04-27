#include "AnakinBoard.h"

#define INIT_AND_CHECK(x) do {                                      \
    if (!x->init()) { printf("=== ERR: CANNOT INIT " #x "\n"); }    \
} while(0)

#define ADD_SAMPLER(type, name, rate) \
    sEventScheduler->add(std::bind(& type ## SensorSampler::Update,name), \
            rate, #name "-" #rate "ms")

AnakinBoard::AnakinBoard()
{
    mInited = false;
    mS_MPU9250   = nullptr;
    mS_INEMO     = nullptr;
    mS_FXAS      = nullptr;
    mS_LPS331AP  = nullptr;
}

bool AnakinBoard::init()
{
    if(mInited)
        return false;

    mInited = true;
    mS_MPU9250   = new mpu_t(mpu_t::ACC_FS_2G, mpu_t::GYRO_FS_250);
    mS_INEMO     = new inemo_t(inemo_t::ACC_FS_16G, inemo_t::GYRO_FS_245,
                                    inemo_t::COMPASS_FS_2);
    mS_FXAS      = new fxas_t(fxas_t::DPS500);
    mS_LPS331AP  = new lps331ap_t(lps331ap_t::SS_25HZ);

    INIT_AND_CHECK(mS_MPU9250);
    INIT_AND_CHECK(mS_INEMO);
    INIT_AND_CHECK(mS_FXAS);
    INIT_AND_CHECK(mS_LPS331AP);

    AddSensor(MPU_9250_ACCEL,   DATA_VEC3,     mS_MPU9250->accelDataPtr());
    AddSensor(MPU_9250_GYRO,    DATA_VEC3,     mS_MPU9250->gyroDataPtr());
    AddSensor(MPU_9250_COMPASS, DATA_VEC3,     mS_MPU9250->compassDataPtr());
    AddSensor(MPU_9250_TEMP,    DATA_FLOAT,    mS_MPU9250->tempDataPtr());

    AddSensor(INEMO_ACCEL,      DATA_VEC3,     mS_INEMO->accelDataPtr());
    AddSensor(INEMO_GYRO,       DATA_VEC3,     mS_INEMO->gyroDataPtr());
    AddSensor(INEMO_COMPASS,    DATA_VEC3,     mS_INEMO->compassDataPtr());
    AddSensor(INEMO_TEMP,       DATA_FLOAT,    mS_INEMO->tempDataPtr());
    AddSensor(FXAS_GYRO,        DATA_VEC3,     mS_FXAS->gyroDataPtr());

    AddSensor(LPS331AP_PRESS,   DATA_FLOAT,    mS_LPS331AP->pressureDataPtr());
    AddSensor(LPS331AP_TEMP,    DATA_FLOAT,    mS_LPS331AP->tempDataPtr());

    printf("Adding sensors to 100Hz DMA sampler\n");
    m100HzDMA.AddSensor(mS_MPU9250);
    m100HzDMA.AddSensor(mS_INEMO);
    m100HzDMA.AddSensor(mS_FXAS);

    printf("Adding sensors to 25Hz DMA sampler\n");
    m25HzDMA.AddSensor(mS_FXAS);

    printf("Adding sensors to 10Hz Simple sampler\n");
    m10HzSimple.AddSensor(mS_MPU9250);
    m10HzSimple.AddSensor(mS_FXAS);

    printf("Adding samplers to scheduler\n");
    ADD_SAMPLER(DMA, m100HzDMA, 10); // 10ms
    ADD_SAMPLER(DMA, m25HzDMA, 40);  // 25ms
    ADD_SAMPLER(Simple, m100HzSimple, 10); // 10ms
    ADD_SAMPLER(Simple, m10HzSimple, 100); // 100ms

    return true;
}

#undef INIT_AND_CHECK
#undef ADD_SAMPLER
