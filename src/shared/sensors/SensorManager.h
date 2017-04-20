#ifndef SHARED_SENSORS_SENSORMANAGER_H
#define SHARED_SENSORS_SENSORMANAGER_H

#include <Common.h>

class SensorManager : public Singleton<SensorManager>
{
    friend class Singleton<SensorManager>;
public:
    ~SensorManager();

private:
    SensorManager();

    SensorManager& (const SensorManager&) = delete;
    SensorManager& (SensorManager&&) = delete;
    SensorManager& operator=(const SensorManager&) = delete;
    SensorManager& operator=(SensorManager&&) = delete;
};

#define sSensorManager SensorManager::getInstance()

#endif /* ifndef SHARED_SENSORS_SENSORMANAGER_H */
