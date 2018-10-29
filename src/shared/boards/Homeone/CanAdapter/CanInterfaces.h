/* CAN-Bus interfaces
 *
 * Copyright (c) 2018-2019 Skyward Experimental Rocketry
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

#include <Common.h>
#include <events/Event.h>

namespace CanInterfaces 
{

/**
 * CanTopics = Canbus FilterIds = Source of the Canbus message
 * Pay attention to the ORDER: higher number => higher priority
 */
enum class CanTopic {
    CAN_TOPIC_NOS,
    CAN_TOPIC_LAUNCH,
    CAN_TOPIC_IGN,
    CAN_TOPIC_COMMANDS,
    CAN_TOPIC_LAST
};

const size_t CAN_MAX_PAYLOAD = 8;

inline uint8_t canTopicToInt(CanTopic cmd) 
{
    return static_cast<uint8_t>(cmd);
}

inline size_t canSize(const size_t msg_size) 
{
    size_t trim_size = (msg_size <= CAN_MAX_PAYLOAD) ? 
    								msg_size : CAN_MAX_PAYLOAD;
    return trim_size;
}

} /* namespace CanInterfaces */

#endif /* CAN_INTERFACES_H */