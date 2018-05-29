/* Copyright (c) 2018 Skyward Experimental Rocketry
 * Authors: Elvis
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

#ifndef SRC_SHARED_BOARDS_HOMEONE_STATUSMANAGER_TELEMETRYBUILDERS_H_
#define SRC_SHARED_BOARDS_HOMEONE_STATUSMANAGER_TELEMETRYBUILDERS_H_

#include "boards/Homeone/FlightModeManager/FlightModeManager.h"
#include "boards/Homeone/TMTCManager/TMTCManager.h"
#include "logger/Logger.h"
#include <diagnostic/CpuMeter.h>

namespace HomeoneBoard
{
namespace Status
{

/**
 * These builders are helper structures that encapsulate the logic
 * of retrieving telemetry informations, building mavlink messages
 * and converting them in a byte stream.
 */
struct TelemetryBuilder
{
    mavlink_message_t mavMsg;
    uint8_t serializedMsg[sizeof(mavlink_message_t)];

    virtual ~TelemetryBuilder(){};

    virtual uint8_t msgLen() = 0; // Length of the mavlink message payload
    virtual void encode() = 0;    // Encode mavlink message

    uint8_t getSize()
    {
        return msgLen() + MAVLINK_NUM_NON_PAYLOAD_BYTES;
    }

    uint8_t* serialize()
    {
        // TODO: disable interrupts
        encode();
        mavlink_msg_to_send_buffer(serializedMsg, &mavMsg);
        return serializedMsg;
    }
};

/**
 * High rate telemetry builder
 */
class HR_TM_Builder: public TelemetryBuilder
{
public:
    uint8_t msgLen() override
    {
        return MAVLINK_MSG_ID_HR_SAMPLE_DATA_TM_LEN;
    }

    void encode() override
    {
        // TODO get values from SensorManager
        // pressure
        mavlink_msg_hr_sample_data_tm_pack(1, 1, &mavMsg, 0);
    }
};

/**
 * Low rate telemetry builder
 */
class LR_TM_Builder: public TelemetryBuilder
{
public:
    uint8_t msgLen() override
    {
        return MAVLINK_MSG_ID_SAMPLE_DATA_TM_LEN;
    }

    void encode() override
    {
        // TODO get values from SensorManager
        // pressure, acc XYZ, gyro XYZ
        mavlink_msg_sample_data_tm_pack(1, 1, &mavMsg, 0, 0, 0, 0, 0, 0, 0);
    }
};

/**
 * Nosecone status telemetry builder
 */
class Nosecone_TM_Builder: public TelemetryBuilder
{
public:
    uint8_t msgLen() override
    {
        return MAVLINK_MSG_ID_NOSECONE_STATUS_TM_LEN;
    }

    void encode() override
    {
        // TODO get values from DeploymentController
        // connected, dpl_state
        mavlink_msg_nosecone_status_tm_pack(1, 1, &mavMsg, 0, 0);
    }
};

/**
 * Ignition status telemetry builder
 */
class Ignition_TM_Builder: public TelemetryBuilder
{
public:
    uint8_t msgLen() override
    {
        return MAVLINK_MSG_ID_IGNITION_STATUS_TM_LEN;
    }

    void encode() override
    {
        // TODO get values from IgnitionController
        // connected, ignition_state
        mavlink_msg_ignition_status_tm_pack(1, 1, &mavMsg, 0, 0);
    }
};

/**
 * Homeone status telemetry builder
 */
class Homeone_TM_Builder: public TelemetryBuilder
{
public:
    uint8_t msgLen() override
    {
        return MAVLINK_MSG_ID_HOMEONE_STATUS_TM_LEN;
    }

    void encode() override
    {
        // TODO get values from Logger, SM, FMM and fault counters
        // sampling_status, log status, fmm state, umbilical, faultCounters (?)
        mavlink_msg_homeone_status_tm_pack(1, 1, &mavMsg, 0, 0, 0, 0, 0);
    }
};

/**
 * Ignition status telemetry builder
 */
class Debug_TM_Builder: public TelemetryBuilder
{
public:
    uint8_t msgLen() override
    {
        return MAVLINK_MSG_ID_DEBUG_INFO_TM_LEN;
    }

    void encode() override
    {
        // TODO get values from all boards
        // CPU, heap, all stacks, selected info...
        mavlink_msg_debug_info_tm_pack(1, 1, &mavMsg, averageCpuUtilization());
    }
};

}
}

#endif /* SRC_SHARED_BOARDS_HOMEONE_STATUSMANAGER_TELEMETRYBUILDERS_H_ */
