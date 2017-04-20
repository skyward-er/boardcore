#ifndef SENSOR_SAMPLING_H
#define SENSOR_SAMPLING_H

#include <Common.h>
#include <DMA/DMA.h>

class Sensor;
class SensorSampling {
public:
    SensorSampling() {}
    ~SensorSampling() {}

    void AddSensor(Sensor* sensor)
    {
        mSensors.push_back(sensor);
        std::vector<SPIRequest> requests = sensor->buildDMARequest();
        for(size_t i=0; i<requests.size();i++)
            mRequests.push_back(std::move(requests[i]));
    }

    void Update()
    {
        auto& driver = SPIDriver::instance(); 
        driver.transaction(mRequests);

        for(size_t i=0; i< mSensors.size(); i++)
            mSensors[i]->onDMAUpdate(mRequests[i]);
    }
private:
    std::vector<Sensor*> mSensors;
    std::vector<SPIRequest> mRequests;
};

#endif /* ifndef SENSOR_SAMPLING_H */
