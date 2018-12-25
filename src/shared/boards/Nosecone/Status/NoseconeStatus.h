/* Copyright (c) 2018 Skyward Experimental Rocketry
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
 * OUT OF OR IN CONNECTION\ WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 */
#ifndef NSC_STATUS_H
#define NSC_STATUS_H

#include <Common.h>
#include <atomic>
#include <boards/CanInterfaces.h>


namespace NoseconeBoard 
{

/**
 * Nosecone internal status: same as the one defined in CanInterfaces, only
 * not packed and composed of atomic variables that avoid concurrency problems.
 */
struct AtomicNoseconeStatus
{
    std::atomic<bool> motor_active;          
    std::atomic<bool> motor_last_direction;
    std::atomic<bool> homeone_not_connected;

    std::atomic<bool> close_received;
    std::atomic<bool> close_timeout;
    std::atomic<bool> close_stop;
    std::atomic<bool> close_limit;
    std::atomic<bool> open_received;
    std::atomic<bool> open_timeout;
    std::atomic<bool> open_stop;
    std::atomic<bool> open_limit;

    std::atomic<uint16_t> max_current_sensed;
    std::atomic<uint16_t> min_current_sensed;
    std::atomic<uint16_t> last_current_sensed;
};

/**
 * @brief Copies the current status in the packed structure, ready
 *        to be sent on the canbus.
 *
 * @param status   where to copy the status values.
 */
void getNoseconeStatus(CanInterfaces::NoseconeBoardStatus* status);


/* Internal global variable for the status */
static AtomicNoseconeStatus status_g;

}

#endif