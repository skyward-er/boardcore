#pragma once

#include "events/HSM.h"
#include "boards/Homeone/Events.h"
#include "DeploymentStatus.h"
#include "ThermalCutter/Cutter.h"
#include "boards/Homeone/LogProxy/LogProxy.h"

namespace HomeoneBoard
{

class DeploymentController : public HSM<DeploymentController>
{
public:
    DeploymentController();
    ~DeploymentController();

private:
    State state_init(const Event& ev);
    State state_ready(const Event& ev);

    State state_cutterIdle(const Event& ev);
    State state_cuttingDrogue(const Event& ev);
    State state_cuttingMain(const Event& ev);

    /**
     * @brief Logs the current status of the component, updating the timestamp
     * 
     */
    void logStatus()
    {
        status.timestamp = miosix::getTick();
        logger.log(status);
    }

    Cutter cutter{};
    DeploymentStatus status;
    bool cut_main = false;
    LoggerProxy& logger = *(LoggerProxy::getInstance());
    uint16_t delayed_ev_id = 0;
};

} // HomeoneBoard