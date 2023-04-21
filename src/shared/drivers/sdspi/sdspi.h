/* Copyright (c) 2023 Skyward Experimental Rocketry
 * Authors: Federico Lolli
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

#include "sdspi.h"

#include <miosix.h>
#include <filesystem/devfs/devfs.h>

namespace Boardcore {

/**
 * Driver for interfacing to an SD card through SPI
 */
class SPISDDriver : public miosix::Device
{
public:
    virtual ssize_t readBlock(void *buffer, size_t size, off_t where);
    
    virtual ssize_t writeBlock(const void *buffer, size_t size, off_t where);
    
    virtual int ioctl(int cmd, void *arg);
private:
    /**
     * Constructor
     */
    SPISDDriver();
    
    miosix::FastMutex mutex;
};

} //namespace Boardcore
