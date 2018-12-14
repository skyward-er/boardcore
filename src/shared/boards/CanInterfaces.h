/* CAN-Bus interfaces
 *
 * Copyright (c) 2015-2018 Skyward Experimental Rocketry
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

#ifndef CAN_INTERFACES_H
#define CAN_INTERFACES_H

#include <stdint.h>
#include <cstring>

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

enum CanMessageID : uint8_t
{
    CAN_MSG_ABORT,
    CAN_MSG_REQ_IGN_STATUS,
    CAN_MSG_REQ_NSC_STATUS,
    CAN_MSG_IGN_STATUS,
    CAN_MSG_NSC_STATUS
};


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


static inline int canMsgSimple(uint8_t* buf, uint8_t id)
 {
    memset(buf, 0, 8);
    buf[0] = id;
    return 1;
}

static inline int canMsgIgnitionStatus(uint8_t* buf, IgnitionBoardStatus ign_status)
{
    memset(buf, 0, 8);
    buf[0] = CAN_MSG_IGN_STATUS;
    memcpy(++buf, &ign_status, sizeof(ign_status));
    return 2;
}

static inline int canMsgLaunch(uint8_t* buf, uint64_t launch_code)
{
    memcpy(buf, &launch_code, 8);
    return 8;
}

} /* namespace CanInterfaces */

#endif /* CAN_INTERFACES_H */
