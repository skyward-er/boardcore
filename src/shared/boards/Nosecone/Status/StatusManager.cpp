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
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 */

#include "StatusManager.h"

using namespace CanInterfaces;

namespace NoseconeBoard 
{


void StatusManager::getStatus(NoseconeBoardStatus* buff)
{
    miosix::FastInterruptDisableLock dLock;
    memcpy(buff, &noseconeStatus_g, sizeof(NoseconeBoardStatus));
}


bool StatusManager::setStatus(uint8_t offset, uint8_t* buff, uint8_t len)
{
    if(offset + len > sizeof(NoseconeBoardStatus)) 
        return false;

    {
        miosix::FastInterruptDisableLock dLock;
        memcpy(reinterpret_cast<uint8_t*>(&noseconeStatus_g) + offset, buff, len);
    }

    return true;
}


bool StatusManager::setStatusBit(uint8_t byteOffset, uint8_t bitOffset, bool value)
{
    if(bitOffset >= 8 || byteOffset >= sizeof(NoseconeBoardStatus)) 
        return false;

    uint8_t* paramPtr = reinterpret_cast<uint8_t*>(&noseconeStatus_g) + byteOffset;

    miosix::FastInterruptDisableLock dLock;
    if(value) {
        *paramPtr |= 1<<bitOffset;
    } else {
        *paramPtr &= ~(1<<bitOffset);
    }

    return true;
}

}