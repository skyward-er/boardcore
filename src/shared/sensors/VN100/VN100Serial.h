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

/** BRIEF INTRODUCTION
 * This class is used to create a communication easily with UART ports.
 * In particular it handles the creation and the basic primitives
 * for standard communication protocols (init, send, recv and close).
 * The user should pay more attention on the concept behind the use of this
 * class. The principle under the class is that miosix communicates via USART
 * ports using a file based principle (Unix style). Because of that you need to
 * consider that the receive file works as a LIFO queue, in fact the last
 * message is returned over the less recent messages, so if you accumulate
 * various messages of variable length, you might end up with a message which
 * represents the most recent communication + other previous stuff.
 */

#pragma once

#include <arch/common/drivers/serial.h>
#include <fcntl.h>
#include <filesystem/file_access.h>
#include <miosix.h>
namespace Boardcore
{

/**
 * @brief Class to communicate with the vn100 via the UART interface.
 */
class VN100Serial
{
public:
    VN100Serial(unsigned int serialPortNumber, unsigned int serialBaudRate);

    bool init();

    /**
     * @brief Writes in Serial data.
     *
     * Note: to send std::string via serial you need to pass use "c_str()"
     * method.
     *
     * @param data Data to be sent via serial.
     * @param length Array length.
     *
     * @return True if operation succeeded.
     */
    template <typename DataSend>
    bool send(DataSend *data, int length);

    /**
     * @brief Reads from serial.
     *
     * @param data Array where to store data.
     * @param length Array length.
     *
     * @return True if operation succeeded.
     */
    template <typename DataReceive>
    bool recv(DataReceive *data, int length);

    bool closeSerial();

private:
    unsigned int serialPortNumber;
    unsigned int serialBaudRate;

    int serialFileDescriptor = 0;

    bool isInit = false;
};

inline VN100Serial::VN100Serial(unsigned int serialPortNumber,
                                unsigned int serialBaudRate)
    : serialPortNumber(serialPortNumber), serialBaudRate(serialBaudRate)
{
}

inline bool VN100Serial::init()
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
            "vn100",
            miosix::intrusive_ref_ptr<miosix::Device>(
                new miosix::STM32Serial(serialPortNumber, serialBaudRate)))))
    {
        return false;
    }

    // Open the file
    serialFileDescriptor = open("/dev/vn100", O_RDWR);

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
inline bool VN100Serial::send(DataSend *data, int length)
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
inline bool VN100Serial::recv(DataReceive *data, int length)
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

inline bool VN100Serial::closeSerial()
{
    // Retrieve the file system instance
    miosix::intrusive_ref_ptr<miosix::DevFs> devFs =
        miosix::FilesystemManager::instance().getDevFs();

    // Close the file descriptor
    close(serialFileDescriptor);

    // Remove the file
    if (!(devFs->remove("vn100")))
    {
        return false;
    }

    return true;
}

}  // namespace Boardcore
