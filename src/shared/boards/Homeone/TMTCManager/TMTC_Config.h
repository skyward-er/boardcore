/* Copyright (c) 2018 Skyward Experimental Rocketry
 * Authors: Alvise De Faveri
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

#ifndef TMTC_CONFIG_H
#define TMTC_CONFIG_H

#include <Common.h>
#include <libs/mavlink_skyward_lib/mavlink_lib/skyward/mavlink.h>  
#include <boards/Homeone/Events.h>
#include <boards/Homeone/Topics.h>

// Name of the device in the fs
#define RADIO_DEVICE_NAME "/dev/radio"

// Maximum number of messages in the queue
#define TMTC_OUT_BUFFER_SIZE 1000
// Minimum sleep time between sends
#define TMTC_MIN_GUARANTEED_SLEEP 10 
// Maximum number of consecutive messages sent before sleeping
#define TMTC_MAX_PKT_SIZE (1*sizeof(mavlink_message_t))

// Sender and Receiver threads parameters
#define TMTC_SENDER_STACKSIZE miosix::STACK_DEFAULT_FOR_PTHREAD
#define TMTC_SENDER_PRIORITY miosix::MAIN_PRIORITY
#define TMTC_RECEIVER_STACKSIZE miosix::STACK_DEFAULT_FOR_PTHREAD
#define TMTC_RECEIVER_PRIORITY miosix::MAIN_PRIORITY

// Mavlink messages sysID and compID
#define TMTC_MAV_SYSID 1
#define TMTC_MAV_COMPID 1

namespace HomeoneBoard
{
namespace TMTC
{

static const std::map<uint8_t, uint8_t> statusCmdToEvt = 
{
    { MAV_NOSECONE_BOARD, EV_NOSECONE_STATUS_REQUEST },
    { MAV_IGNITION_BOARD, EV_IGNITION_STATUS_REQUEST },
    { MAV_HOMEONE_BOARD,  EV_HOMEONE_STATUS_REQUEST }
};

static const std::map<uint8_t, uint8_t> noargCmdToEvt = 
{
    { MAV_CMD_ARM,              EV_TC_ARM    }, 
    { MAV_CMD_DISARM,           EV_TC_DISARM }, 
    { MAV_CMD_ABORT,            EV_TC_ABORT_LAUNCH  }, 
    { MAV_CMD_NOSECONE_OPEN,    EV_TC_NOSECONE_OPEN }, 
    { MAV_CMD_NOSECONE_CLOSE,   EV_TC_NOSECONE_CLOSE }, 
    { MAV_CMD_START_SAMPLING,   EV_TC_START_SAMPLING }, 
    { MAV_CMD_STOP_SAMPLING,    EV_TC_STOP_SAMPLING }, 
    { MAV_CMD_TEST_MODE,        EV_TC_TEST_MODE   }, 
    { MAV_CMD_BOARD_RESET,      EV_TC_RESET_BOARD }, 
    { MAV_CMD_REQ_DEBUG_INFO,   EV_DEBUG_INFO_REQUEST }
};

} /* namespace TMTC */
} /* namespace HomeoneBoard */


#endif /* TMTC_CONFIG_H */
