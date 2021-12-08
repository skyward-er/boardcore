/* Copyright (c) 2015-2016 Skyward Experimental Rocketry
 * Authors: Matteo Piazzolla, Alain Carlucci
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
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 */

#pragma once

#include <Common.h>

#include <ostream>
#include <string>

namespace Boardcore
{

#define CAN_MAX_PAYLOAD 8
#define CAN_MAX_LEN 16

enum CanMode
{
    CAN_NORMAL    = 0,
    CAN_LOOPBACK  = 1,
    CAN_SILENT    = 2,
    CAN_SILENT_LB = 3
};

enum CanFifo
{
    CAN_FIFO0 = 0,
    CAN_FIFO1 = 1
};

enum CanSJWQuantum
{
    CAN_SJW_1tq = 0,
    CAN_SJW_2tq = 1,
    CAN_SJW_3tq = 2,
    CAN_SJW_4tq = 3
};

enum CanBS1Quantum
{
    CAN_BS1_1tq  = 0,
    CAN_BS1_2tq  = 1,
    CAN_BS1_3tq  = 2,
    CAN_BS1_4tq  = 3,
    CAN_BS1_5tq  = 4,
    CAN_BS1_6tq  = 5,
    CAN_BS1_7tq  = 6,
    CAN_BS1_8tq  = 7,
    CAN_BS1_9tq  = 8,
    CAN_BS1_10tq = 9,
    CAN_BS1_11tq = 10,
    CAN_BS1_12tq = 11,
    CAN_BS1_13tq = 12,
    CAN_BS1_14tq = 13,
    CAN_BS1_15tq = 14,
    CAN_BS1_16tq = 15
};

enum CanBS2Quantum
{
    CAN_BS2_1tq = 0,
    CAN_BS2_2tq = 1,
    CAN_BS2_3tq = 2,
    CAN_BS2_4tq = 3,
    CAN_BS2_5tq = 4,
    CAN_BS2_6tq = 5,
    CAN_BS2_7tq = 6,
    CAN_BS2_8tq = 7
};

enum CanIdTypes
{
    CAN_ID_STD = 0,
    CAN_ID_EXT = 4
};

#pragma pack(1)
struct CanMsg
{
    uint32_t StdId;
    uint32_t ExtId;
    uint8_t IDE;
    uint8_t RTR;
    uint8_t DLC;
    uint8_t Data[8];
    uint8_t FMI;
};
#pragma pack()

struct CanStatus
{
    uint16_t n_sent;
    uint16_t n_rcv;
    uint8_t last_sent;
    uint8_t last_rcv;
    uint64_t last_sent_ts;
    uint64_t last_rcv_ts;
    static std::string header()
    {
        return "n_sent,n_rcv,last_sent,last_rcv,last_sent_ts,last_rcv_ts\n";
    }

    void print(std::ostream& os) const
    {
        os << n_sent << "," << n_rcv << "," << (int)last_sent << ","
           << (int)last_rcv << "," << last_sent_ts << "," << last_rcv_ts
           << "\n";
    }
};

}  // namespace Boardcore
