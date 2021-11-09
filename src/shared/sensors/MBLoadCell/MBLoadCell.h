/* Copyright (c) 2021 Skyward Experimental Rocketry
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
#include <sensors/Sensor.h>
#include <stdio.h>

#include <string>

#include "MBLoadCellData.h"
#include "SerialInterface.h"
#include "miosix.h"

using namespace std;
using namespace miosix;

class MBLoadCell : public Sensor<float>
{
public:
    /**
     * @brief constructor that initializes the serial communication with the
     * load cell
     * @param serialPortNum port number to which the load cell is connected
     * @param baudrate baudrate set on the TLB converter
     */
    MBLoadCell(LoadCellModes mode, int serialPortNum, int baudrate);

    /**
     * @brief reading the output in continuous mode
     * @return the value of the weight returned from the load cell
     */
    int readBusy();

    bool init();

    bool selfTest();

protected:
    /**
     * @brief override of the sample implementation method that requests the
     * weight sampled from the load cell or waits for a sample depending on
     * the mode selected (it's a blocking function)
     * @return the weight measured from the load cell
     */
    float sampleImpl() override;

    /**
     * @brief calculates the checksum as in the manual
     * @param message the string from which will be calculated the checksum
     * @return a pair of chars that represents the checksum in hexadecimal
     */
    string calculateChecksum(char *message);

private:
    SerialInterface *serial;  ///< pointer to the instance of the serial port
                              ///< opened for the connection
    LoadCellModes mode;       ///< mode in which the load cell will be used
};