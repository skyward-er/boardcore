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
#pragma once

#include <Common.h>
#include <boards/CanInterfaces.h>


namespace NoseconeBoard 
{

/**
 * Parameter offsets: used for accessing the status struct as a byte array.
 */

/* Byte 0 */
const uint8_t MOTOR_STATUS_BYTE                 = 0;
const uint8_t MOTOR_ACTIVE_BYTE_OFFSET          = 0;
const uint8_t MOTOR_LAST_DIRECTION_BYTE_OFFSET  = 0;
const uint8_t HOMEONE_NOT_CONNECTED_BYTE_OFFSET = 0;

const uint8_t MOTOR_ACTIVE_BIT_OFFSET          = 0;
const uint8_t MOTOR_LAST_DIRECTION_BIT_OFFSET  = 1;
const uint8_t HOMEONE_NOT_CONNECTED_BIT_OFFSET = 2;

/* Byte 1 */
const uint8_t NOSECONE_STATUS_BYTE       = 1;
const uint8_t CLOSE_RECEIVED_BYTE_OFFSET = 1;
const uint8_t CLOSE_TIMEOUT_BYTE_OFFSET  = 1;
const uint8_t CLOSE_STOP_BYTE_OFFSET     = 1;
const uint8_t CLOSE_LIMIT_BYTE_OFFSET    = 1;
const uint8_t OPEN_RECEIVED_BYTE_OFFSET  = 1;
const uint8_t OPEN_TIMEOUT_BYTE_OFFSET   = 1;
const uint8_t OPEN_STOP_BYTE_OFFSET      = 1;
const uint8_t OPEN_LIMIT_BYTE_OFFSET     = 1;

const uint8_t CLOSE_RECEIVED_BIT_OFFSET = 0;
const uint8_t CLOSE_TIMEOUT_BIT_OFFSET  = 1;
const uint8_t CLOSE_STOP_BIT_OFFSET     = 2;
const uint8_t CLOSE_LIMIT_BIT_OFFSET    = 3;
const uint8_t OPEN_RECEIVED_BIT_OFFSET  = 4;
const uint8_t OPEN_TIMEOUT_BIT_OFFSET   = 5;
const uint8_t OPEN_STOP_BIT_OFFSET      = 6;
const uint8_t OPEN_LIMIT_BIT_OFFSET     = 7;

/* Byte 2 */
const uint8_t MAX_CURRENT_SENSED_BYTE_OFFSET = 2;
/* Byte 4 */
const uint8_t MIN_CURRENT_SENSED_BYTE_OFFSET = 4;
/* Byte 6 */
const uint8_t LAST_CURRENT_SENSED_BYTE_OFFSET = 6;

/**
 * Provides access to the status structure.
 */
class StatusManager{

public:
    /**
     * @brief Copy the current nosecone status in the given buffer.
     *        Disabling interrupts guarantees synchronization.
     *
     * @param buff   where to copy the status struct
     */
    void getStatus(CanInterfaces::NoseconeBoardStatus* buff);

    /**
     * @brief Set one ore more bytes of the nosecone status to a given value.
     *        Disabling interrupts guarantees synchronization.
     *
     * @param offset   offset in bytes of the value to modify
     * @param buff         new value
     * @param len          length of modification
     * @return             false if you are trying to modfy an invalid address
     */
    bool setStatus(uint8_t offset, uint8_t* buff, uint8_t len);

    /**
     * @brief Set the value of a single bit in the status.
     *        Disabling interrupts guarantees synchronization.
     *
     * @param byteOffset   offset in bytes of the value to modify
     * @param bitOffset    bit to set in the given byte
     * @param value        value to set
     * @return             false if you are trying to modfy an invalid address
     */
    bool setStatusBit(uint8_t byteOffset, uint8_t bitOffset, bool value);

private:
    /* The internal status is exactly the same sent on the can */
    CanInterfaces::NoseconeBoardStatus noseconeStatus_g;
};

}