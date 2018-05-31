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

#ifndef TMTC_CONFIG
#define TMTC_CONFIG

#include <Common.h>
#include <drivers/gamma868/Gamma868.h>
#include <libs/mavlink_skyward_lib/mavlink_lib/skyward/mavlink.h>
#include "boards/Homeone/Events.h"
#include "boards/Homeone/Topics.h"
#include "events/EventBroker.h"     

#define RADIO_DEVICE_NAME "/dev/radio"

// Default size of the output buffer
#define TMTC_OUT_BUFFER_SIZE (10*sizeof(mavlink_ping_tc_t))
// Default timeout before sending next packet
#define TMTC_SEND_TIMEOUT 300 
// Maximum dimension a sent packet
#define TMTC_MAX_PKT_SIZE (2*sizeof(mavlink_ping_tc_t))
// Maximum number of retransmissions when sending a packet
#define TMTC_MAX_TRIES_PER_PACKET 1

#define TMTC_SENDER_STACKSIZE miosix::STACK_DEFAULT_FOR_PTHREAD
#define TMTC_SENDER_PRIORITY miosix::MAIN_PRIORITY
#define TMTC_RECEIVER_STACKSIZE miosix::STACK_DEFAULT_FOR_PTHREAD
#define TMTC_RECEIVER_PRIORITY miosix::MAIN_PRIORITY

// Mavlink messages sysID and compID
#define TMTC_MAV_SYSID 1
#define TMTC_MAV_COMPID 1

// Define a debug trace
#ifdef DEBUG
#define TMTC_TRACE(x) printf(x)
#else
#define TMTC_TRACE(x) 
#endif /* DEBUG */

#endif /* CONFIG_H */
