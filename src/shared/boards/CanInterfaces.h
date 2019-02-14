/* Copyright (c) 2015-2018 Skyward Experimental Rocketry
* Authors: Alvise de' Faveri Tron
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
#pragma once

#include <stdint.h>
#include <cstring>

#include <drivers/canbus/CanBus.h>

namespace CanInterfaces
{

/**
* CanTopics = Canbus FilterIds = Source of the Canbus message
* Pay attention to the ORDER: lower number => higher priority
*/
enum CanTopic : uint16_t
{
    CAN_TOPIC_HOMEONE  = 0,
    CAN_TOPIC_LAUNCH   = 2,
    CAN_TOPIC_IGNITION = 4,
    CAN_TOPIC_NOSECONE = 8
};

enum CanCommandID : uint8_t
{
    CAN_MSG_ABORT,
    CAN_MSG_REQ_IGN_STATUS,
    CAN_MSG_REQ_NSC_STATUS,
    CAN_MSG_NSC_OPEN,
    CAN_MSG_NSC_CLOSE,
    CAN_MSG_NSC_STOP
};

/**
* Boards Status structs.
*/
struct __attribute__((packed)) IgnitionBoardStatus
{
    uint8_t u1_abort_cmd : 1;
    uint8_t u1_abort_timeout : 1;
    uint8_t u1_wrong_code : 1;
    uint8_t u1_launch_done : 1;
    uint8_t u2_abort_cmd : 1;
    uint8_t u2_abort_timeout : 1;
    uint8_t u2_wrong_code : 1;
    uint8_t u2_launch_done : 1;
};

struct __attribute__((packed)) NoseconeBoardStatus
{
    uint8_t motor_active: 1;          
    uint8_t motor_last_direction: 1;  // if active=true means current direction
    uint8_t homeone_not_connected: 1;
    uint8_t padding: 5;
    
    uint8_t close_received: 1; // received at least a close command
    uint8_t close_timeout: 1;  // last close terminated for a timeout
    uint8_t close_stop: 1;     // last close terminated for a stop command
    uint8_t close_limit: 1;    // last close terminated with motor limit event
    uint8_t open_received: 1; // received at least an open command
    uint8_t open_timeout: 1;  // last open terminated for a timeout
    uint8_t open_stop: 1;     // last open terminated for a stop command
    uint8_t open_limit: 1;    // last open terminated with motor limit event

    uint16_t max_current_sensed;
    uint16_t min_current_sensed;
    uint16_t last_current_sensed;
};

/**
* Helper functions for can messages packing.
*/
static inline bool canSendHomeoneCommand(CanBus* bus, CanCommandID id)
{
    return bus->send( CAN_TOPIC_HOMEONE, reinterpret_cast<uint8_t*>(&id), sizeof(uint8_t));
}

static inline bool canSendIgnitionStatus(CanBus* bus, IgnitionBoardStatus ign_status)
{
    return bus->send(CAN_TOPIC_IGNITION, reinterpret_cast<uint8_t*>(&ign_status), sizeof(IgnitionBoardStatus));
}

static inline bool canSendNoseconeStatus(CanBus* bus, NoseconeBoardStatus nsc_status)
{
    return bus->send(CAN_TOPIC_NOSECONE, reinterpret_cast<uint8_t*>(&nsc_status), sizeof(NoseconeBoardStatus));
}

static inline bool canSendLaunch(CanBus* bus, uint64_t launch_code)
{
    return bus->send(CAN_TOPIC_LAUNCH, reinterpret_cast<uint8_t*>(&launch_code), sizeof(uint64_t));
}

} /* namespace CanInterfaces */
