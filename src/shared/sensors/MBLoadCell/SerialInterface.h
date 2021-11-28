/* Copyright (c) 2020 Skyward Experimental Rocketry
 * Author: Emilio Corigliano
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
#include <fcntl.h>
#include <stdio.h>

#include <string>

#include "arch/common/drivers/serial.h"
#include "filesystem/file_access.h"
#include "miosix.h"

namespace Boardcore
{

/**
 * @brief Creates and opens a serial port on the board and provides templated
 * "sendData" and "recvData" functions in order to send and receive any data
 * structure needed, or "sendString" and "recvString" in order to send and
 * receive strings.
 */
class SerialInterface
{
public:
    /**
     * Constructor of the serial communication with default parameters
     * @param baudrate port baudrate (default: 19200 as miosix)
     * @param serialPortNum number from 1 to 3 in order to set the USART to use
     * (default: 2 [tx_board=PA2  rx_board=PA3 cts=PA0  rts=PA1])
     * @param serialPortName name of the serial port to open (default:
     * "simulation")
     */
    SerialInterface(int baudrate = 19200, int serialPortNum = 2,
                    std::string serialPortName = "load_cell")
    {
        this->baudrate       = baudrate;
        this->serialPortNum  = serialPortNum;
        this->serialPortName = serialPortName;
        initialized          = false;
        fd                   = -1;
    }

    /**
     * @brief Initializes the object if it's the first call to this function
     * @return true if initialization is successful, false otherwise
     */
    bool init()
    {
        if (initialized)
        {
            TRACE(
                "[SerialCommunication] Error : serial communication already "
                "initialized!\n");
            return false;
        }
        else if (!serialCommSetup())
        {
            TRACE(
                "[SerialCommunication] Error : can't initialize serial "
                "communication!\n");
            return false;
        }

        initialized = true;
        return true;
    }

    /**
     * @brief Sends a string to the serial
     * @param data pointer to the string '\0' terminated
     * @return bytes sent through serial
     */
    int sendString(std::string data)
    {
        return write(fd, data.c_str(), data.length());
    }

    /**
     * @brief Receives a string from the serial
     * @param data pointer to the destination string
     * @param maxLen maximum length of the string to read
     * @return bytes received from serial
     */
    int recvString(char *data, int maxLen) { return read(fd, data, maxLen); }

    /**
     * @brief Receives the data from the simulated sensors and deserializes it
     * in the templated format
     * @param data pointer to the struct for the data to receive
     */
    template <typename T>
    void recvData(T *data)
    {
        read(fd, data, sizeof(T));
    }

    /**
     * @brief Sends the position of the simulated actuators as the templated
     * type
     * @param data pointer to the struct of the data to send
     */
    template <typename U>
    void sendData(U *data)
    {
        write(fd, data, sizeof(U));
    }

    std::string getPortName() { return serialPortName; }

    bool isInit() { return initialized; }

private:
    /* Creates and opens the serial port for communication between OBSW and
     * simulation device */
    bool serialCommSetup()
    {
        // Takes the file system pointer of the devices
        miosix::intrusive_ref_ptr<miosix::DevFs> devFs =
            miosix::FilesystemManager::instance().getDevFs();

        // Creates and adds the serial port to the devices
        if (!devFs->addDevice(
                serialPortName.c_str(),
                miosix::intrusive_ref_ptr<miosix::Device>(
                    new miosix::STM32Serial(serialPortNum, baudrate))))
            return false;

        // path string "/dev/<name_of_port>" for the port we want to open
        std::string serialPortPath = "/dev/" + serialPortName;

        // open serial port
        fd = open(serialPortPath.c_str(), O_RDWR);

        if (fd <= -1)
        {
            TRACE("Cannot open %s\n", serialPortPath.c_str());
            return false;
        }

        return true;
    }

    std::string serialPortName; /**< Port name of the serial port that has to be
                              created for the communication */
    int fd; /**< Stores the file descriptor of the serial port file opened for
               trasmission */
    int serialPortNum; /**< Stores the USART<serialPortNum> used for the
                          trasmission range[1:3] */
    int baudrate;      /**< Baudrate of the serial port */
    bool initialized;  /**< True if init() already called successfully, false
                      otherwise */
};

// namespace Boardcore
