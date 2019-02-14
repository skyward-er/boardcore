#include "Deployment.h"

#include <miosix.h>
#include "Common.h"
#include "boards/Homeone/configs/DeploymentConfig.h"
#include "events/EventBroker.h"

namespace HomeoneBoard
{

DeploymentController::DeploymentController()
    : HSM(&DeploymentController::state_init)
{
    sEventBroker->subscribe(this, TOPIC_DEPLOYMENT);
    sEventBroker->subscribe(this, TOPIC_FLIGHT_EVENTS);
    sEventBroker->subscribe(this, TOPIC_CAN);
    sEventBroker->subscribe(this, TOPIC_TC);
}

DeploymentController::~DeploymentController() {}

State DeploymentController::state_init(const Event& ev)
{
    UNUSED(ev);
    return transition(&DeploymentController::state_cutterIdle);
}

State DeploymentController::state_ready(const Event& ev)
{
    State retState = HANDLED;
    switch (ev.sig)
    {
        case EV_INIT:
            retState = transition(&DeploymentController::state_cutterIdle);
            TRACE("[DPL_CTRL] INIT state_ready\n");
            break;
        case EV_ENTRY:
            TRACE("[DPL_CTRL] ENTRY state_ready\n");
            break;
        case EV_EXIT:
            TRACE("[DPL_CTRL] EXIT state_ready\n");
            break;
        case EV_NC_GETSTATUS:
            // Todo: request nc status
            break;
        case EV_NEW_CAN_MSG:
            // TODO: Handle new CAN msg
            break;
        case EV_TC_NC_OPEN:
        case EV_NC_OPEN:
            TRACE("[DPL_CTRL] NC_OPEN state_ready\n");
            // TODO: Send nosecone open command on CAN
            break;
        case EV_TC_NC_CLOSE:
            // TODO: Send nosecone close command on CAN
            break;
        default:
            retState = tran_super(&DeploymentController::Hsm_top);
            break;
    }
    return retState;
}

State DeploymentController::state_cutterIdle(const Event& ev)
{
    State retState = HANDLED;
    switch (ev.sig)
    {
        case EV_ENTRY:
            TRACE("[DPL_CTRL] ENTRY state_cutterIdle\n");
            break;
        case EV_EXIT:
            TRACE("[DPL_CTRL] EXIT state_cutterIdle\n");
            break;
        case EV_TC_CUT_MAIN:
            retState = transition(&DeploymentController::state_cuttingMain);
            break;
        case EV_TC_CUT_ALL:
            cut_main = true;
            // Continue below. No break!
        case EV_TC_CUT_FIRST_DROGUE:
        case EV_CUT_DROGUE:
            retState = transition(&DeploymentController::state_cuttingDrogue);
            break;
        default:
            retState = tran_super(&DeploymentController::state_ready);
            break;
    }
    return retState;
}

State DeploymentController::state_cuttingDrogue(const Event& ev)
{
    State retState = HANDLED;
    switch (ev.sig)
    {
        case EV_ENTRY:
            TRACE("[DPL_CTRL] ENTRY state_cuttingDrogue\n");
            cutter.startCutDrogue();
            delayed_ev_id = sEventBroker->postDelayed(
                {EV_TIMEOUT_CUTTING}, TOPIC_DEPLOYMENT, MAXIMUM_CUTTING_DURATION);
            break;
        case EV_EXIT:
            TRACE("[DPL_CTRL] EXIT state_cuttingDrogue\n");
            sEventBroker->removeDelayed(delayed_ev_id);
            cutter.stopCutDrogue();  // TODO: Check if this is done before entry
                                     // in the next state
            break;
        case EV_TC_CUT_MAIN:
        case EV_TC_CUT_ALL:
            cut_main = true;
            break;
        case EV_TIMEOUT_CUTTING:
            TRACE("[DPL_CTRL] TIMEOUT state_cuttingDrogue\n");
            if (!cut_main)
            {
                retState = transition(&DeploymentController::state_cutterIdle);
            }
            else
            {
                retState = transition(&DeploymentController::state_cuttingMain);
            }
            break;
        default:
            retState = tran_super(&DeploymentController::state_ready);
            break;
    }
    return retState;
}

State DeploymentController::state_cuttingMain(const Event& ev)
{
    State retState = HANDLED;
    switch (ev.sig)
    {
        case EV_ENTRY:
            TRACE("[DPL_CTRL] ENTRY state_cuttingMain\n");
            cut_main = false;
            cutter.startCutMainChute();
            delayed_ev_id = sEventBroker->postDelayed(
                {EV_TIMEOUT_CUTTING}, TOPIC_DEPLOYMENT, MAXIMUM_CUTTING_DURATION);
            break;
        case EV_EXIT:
            TRACE("[DPL_CTRL] EXIT state_cuttingMain\n");
            sEventBroker->removeDelayed(delayed_ev_id);
            cutter.stopCutMainChute();
            break;
        case EV_TIMEOUT_CUTTING:
            TRACE("[DPL_CTRL] TIMEOUT state_cuttingMain\n");
            retState = transition(&DeploymentController::state_cutterIdle);
            break;
        default:
            retState = tran_super(&DeploymentController::state_ready);
            break;
    }
    return retState;
}

}  // namespace HomeoneBoard