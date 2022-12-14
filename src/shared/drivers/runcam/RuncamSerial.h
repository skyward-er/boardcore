/* Copyright (c) 2021 Skyward Experimental Rocketry
 * Author: Matteo Pignataro
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

#include <arch/common/drivers/serial.h>
#include <fcntl.h>
#include <filesystem/file_access.h>
#include <miosix.h>

namespace Boardcore
{

/**
 * @brief Class to communicate with the Runcam via serial.
 */
class RuncamSerial
{
public:
    RuncamSerial(int serialPortNumber, int serialBaudRate);

    /**
     * @brief Initialization method
     * @return Boolean which communicates the init process result
     */
    bool init();

    /**
     * @brief Writes in Serial data
     * Note: to send std::string via serial you need to pass use "c_str()"
     * method
     * @param Data to be sent via serial
     * @return Boolean which communicates the send process result
     */
    template <typename DataSend>
    bool send(DataSend *data, int length);

    /**
     * @brief Reads from serial
     * @param Pointer to the data structure where we need to store data
     * @return Boolean which communicates the recive process result
     */
    template <typename DataReceive>
    bool recv(DataReceive *data, int length);

    /**
     * @brief Closes the serial communication
     */
    bool closeSerial();

private:
    int serialPortNumber;
    int serialBaudRate;
    int serialFileDescriptor = 0;
    bool isInit              = false;
};

inline RuncamSerial::RuncamSerial(int serialPortNumber, int serialBaudRate)
    : serialPortNumber(serialPortNumber), serialBaudRate(serialBaudRate)
{
}

inline bool RuncamSerial::init()
{
    // If already initialized i avoid the procedure
    if (isInit)
    {
        return true;
    }

    // Retrieve the file system instance
    miosix::intrusive_ref_ptr<miosix::DevFs> devFs =
        miosix::FilesystemManager::instance().getDevFs();

    // Create the serial file which we read and write to communicate
    if (!(devFs->addDevice(
            "Runcam",
            miosix::intrusive_ref_ptr<miosix::Device>(
                new miosix::STM32Serial(serialPortNumber, serialBaudRate)))))
    {
        return false;
    }

    // Open the file
    serialFileDescriptor = open("/dev/Runcam", O_RDWR);

    // Check the descriptor
    if (serialFileDescriptor <= 0)
    {
        return false;
    }

    // Success
    isInit = true;
    return true;
}

template <typename DataSend>
inline bool RuncamSerial::send(DataSend *data, int length)
{
    // Check if the serial has been previously initialized
    if (!isInit)
    {
        return false;
    }

    // Write the file with the data
    if (!write(serialFileDescriptor, data, length))
    {
        return false;
    }

    return true;
}

template <typename DataReceive>
inline bool RuncamSerial::recv(DataReceive *data, int length)
{
    if (!isInit)
    {
        return false;
    }

    // Read the data and store it in memory
    if (!read(serialFileDescriptor, data, length))
    {
        return false;
    }

    return true;
}

inline bool RuncamSerial::closeSerial()
{
    // Retrieve the file system instance
    miosix::intrusive_ref_ptr<miosix::DevFs> devFs =
        miosix::FilesystemManager::instance().getDevFs();

    // Close the file descriptor
    close(serialFileDescriptor);

    // Remove the file
    if (!(devFs->remove("Runcam")))
    {
        return false;
    }

    return true;
}

}  // namespace Boardcore
