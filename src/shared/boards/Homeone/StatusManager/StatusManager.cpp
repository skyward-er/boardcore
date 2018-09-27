// /* Copyright (c) 2018 Skyward Experimental Rocketry
//  * Authors: Elvis
//  *
//  * Permission is hereby granted, free of charge, to any person obtaining a copy
//  * of this software and associated documentation files (the "Software"), to deal
//  * in the Software without restriction, including without limitation the rights
//  * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
//  * copies of the Software, and to permit persons to whom the Software is
//  * furnished to do so, subject to the following conditions:
//  *
//  * The above copyright notice and this permission notice shall be included in
//  * all copies or substantial portions of the Software.
//  *
//  * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
//  * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
//  * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.  IN NO EVENT SHALL THE
//  * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
//  * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
//  * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
//  * THE SOFTWARE.
//  */

// #include <boards/Homeone/StatusManager/StatusManager.h>

// namespace HomeoneBoard
// {
// namespace Status
// {

// /* Post an event to trigger the sending of a Low Rate Telemetry */
// void postLR()
// {
//     sEventBroker->post(Event{EV_LOW_RATE_TM}, TOPIC_STATUS);
// }

// /* Post an event to trigger the sending of a High Rate Telemetry */
// void postHR()
// {
//     sEventBroker->post(Event{EV_HIGH_RATE_TM}, TOPIC_STATUS);
// }

//  Post an event to stop the StatusManager from sending telemetries 
// void postStopTM()
// {
//     sEventBroker->post(Event{EV_STOP_TM}, TOPIC_STATUS);
// }

// /* EventHandler implementation */
// void StatusManager::handleEvent(const Event& e)
// {
//     switch (e.sig)
//     {
//         case EV_LOW_RATE_TM:
//             if (enable)
//                 TMTC::sTMTCManager->enqueueMsg(lr_tm.serialize(), lr_tm.getSize());
//             break;
//         case EV_HIGH_RATE_TM:
//             if(enable)
//                 TMTC::sTMTCManager->enqueueMsg(hr_tm.serialize(), hr_tm.getSize());
//             break;
//         case EV_NOSECONE_STATUS_REQUEST:
//             TMTC::sTMTCManager->enqueueMsg(nos_tm.serialize(), nos_tm.getSize());
//             break;
//         case EV_IGNITION_STATUS_REQUEST:
//             TMTC::sTMTCManager->enqueueMsg(ign_tm.serialize(), ign_tm.getSize());
//             break;
//         case EV_HOMEONE_STATUS_REQUEST:
//             TMTC::sTMTCManager->enqueueMsg(home_tm.serialize(), home_tm.getSize());
//             break;
//         case EV_DEBUG_INFO_REQUEST:
//             TMTC::sTMTCManager->enqueueMsg(debug_tm.serialize(), debug_tm.getSize());
//             break;
//         case EV_START_TM:
//             // TODO: quando viene mandato?
//             enable = true;
//             sEventScheduler->add(postLR, LR_rate, "HR_TM_Task");
//             sEventScheduler->add(postHR, HR_rate, "LR_TM_Task");
//             sEventScheduler->addOnce(postStopTM, TM_timeout);
//             break;
//         case EV_STOP_TM:
//             enable = false;
//             break;
//         default:
//             break;
//     }
// }

// } /* namespace Status */
// } /* namespace HomeoneBoard */


#include <boards/Homeone/StatusManager/StatusManager.h>

namespace HomeoneBoard
{
namespace Status
{

/* Post an event to trigger the sending of a Low Rate Telemetry */
void postLR()
{
    sEventBroker->post(Event{EV_LOW_RATE_TM}, TOPIC_STATUS);
}

/* Post an event to trigger the sending of a High Rate Telemetry */
void postHR()
{
    sEventBroker->post(Event{EV_HIGH_RATE_TM}, TOPIC_STATUS);
}

 Post an event to stop the StatusManager from sending telemetries 
void postStopTM()
{
    sEventBroker->post(Event{EV_STOP_TM}, TOPIC_STATUS);
}

/* EventHandler implementation */
void StatusManager::handleEvent(const Event& e)
{
    mavlink_message_t mavMsg;

    switch (e.sig)
    {
        /* Automatic Telemetries */
        case EV_START_TM:
            // TODO: quando viene mandato?
            enable = true;
            sEventScheduler->add(postLR, LR_rate, "HR_TM_Task");
            sEventScheduler->add(postHR, HR_rate, "LR_TM_Task");
            sEventScheduler->addOnce(postStopTM, TM_timeout);
            return;

        case EV_STOP_TM:
            enable = false;
            return;

        case EV_LOW_RATE_TM:
            if (autoTmEnable) {
                // TODO: get values from SensorManager
                // pressure, acc XYZ, gyro XYZ
                mavlink_msg_sample_data_tm_pack(1, 1, &mavMsg, 0, 0, 0, 0, 0, 0, 0);
            }
            break;

        case EV_HIGH_RATE_TM:
            if(autoTmEnable) {
                // TODO: get values from SensorManager
                // pressure
                mavlink_msg_hr_sample_data_tm_pack(1, 1, &mavMsg, 0);
            }
            break;

        /* Components Status Requests */
        case EV_NOSECONE_STATUS_REQUEST:
            // TODO: get values from DeploymentController
            // connected, dpl_state
            mavlink_msg_nosecone_status_tm_pack(1, 1, &mavMsg, 0, 0);
            break;

        case EV_IGNITION_STATUS_REQUEST:
            // TODO get values from IgnitionController
            // connected, ignition_state
            mavlink_msg_ignition_status_tm_pack(1, 1, &mavMsg, 0, 0);
            break;

        case EV_HOMEONE_STATUS_REQUEST:
            // TODO get values from Logger, SM, FMM and fault counters
            // sampling_status, log status, fmm state, umbilical, faultCounters (?)
            mavlink_msg_homeone_status_tm_pack(1, 1, &mavMsg, 0, 0, 0, 0, 0);
            break;

        case EV_DEBUG_INFO_REQUEST:
            // TODO get values from all boards
            // CPU, heap, all stacks, selected info...
            mavlink_msg_debug_info_tm_pack(1, 1, &mavMsg, averageCpuUtilization());
            break;

        default:
            break;
    }

    uint8_t serializedMsg[sizeof(mavlink_message_t)];
    uint16_t msgLen = mavlink_msg_to_send_buffer(serializedMsg, &mavMsg);

    TMTC::sTMTCManager->enqueueMsg(serializedMsg, msgLen);
}

} /* namespace Status */
} /* namespace HomeoneBoard */