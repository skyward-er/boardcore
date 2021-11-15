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

#include <miosix.h>
#include <fcntl.h>
#include <arch/common/drivers/serial.h>
#include <filesystem/file_access.h>

using namespace miosix;


/**
 * @brief Class to communicate with the vn100 via serial
 */
class VN100Serial
{

private:

    /**
     * @brief Serial settings
     */
    int serialBaudRate;
    int serialPortNumber;
    
    /**
     * @brief Serial file descriptor
     */
    int serialFileDescriptor;

    /**
     * @brief Boolean that indicates the init status of the serial communication
     */
    bool isInit;

public:

    /**
     * @brief Constructor
     * @param serial port number that references the usart port on the STM32
     * @param serial port baud rate
     */
    VN100Serial(int serialPortNumber, int serialBaudRate)
    {
        this -> serialPortNumber    = serialPortNumber;
        this -> serialBaudRate      = serialBaudRate;

        isInit = false;
    }

    /**
     * @brief Initialization method
     * @return Boolean which communicates the init process result
     */
    bool init()
    {
        //If already initialized i avoid the procedure
        if(isInit)
        {
            return true;
        }

        //Retrieve the file system instance
        intrusive_ref_ptr<DevFs> devFs = FilesystemManager::instance().getDevFs();

        //Create the serial file which we read and write to communicate
        if(!(devFs -> addDevice("vn100", intrusive_ref_ptr<Device>(new STM32Serial(serialPortNumber, serialBaudRate)))))
        {
            return false;
        }

        //Open the file
        serialFileDescriptor = open("/dev/vn100", O_RDWR);

        //Check the descriptor
        if(serialFileDescriptor <= 0)
        {
            return false;
        }

        //Success
        isInit = true;
        return true;
    }


    /**
     * @brief Writes in Serial data
     * @param Data to be sent via serial
     * @return Boolean which communicates the send process result
     */
    template <typename DataSend>
    bool send(DataSend data)
    {
        //Check if the serial has been previously initialized
        if(!isInit)
        {
            return false;
        }

        //Write the file with the data
        write(serialFileDescriptor, data, sizeof(data));

        return true;
    }


    /**
     * @brief Reads from serial
     * @param Pointer to the data structure where we need to store data
     * @return Boolean which communicates the recive process result
     */
    template <typename DataReceive>
    bool recv(DataReceive *data)
    {
        if(!isInit)
        {
            return false;
        }

        //Read the data and store it in memory
        read(serialFileDescriptor, data, sizeof(DataReceive));

        return true;
    }


    /**
     * @brief Closes the serial communication
     */
    void closeSerial()
    {
        close(serialFileDescriptor);
    }
};