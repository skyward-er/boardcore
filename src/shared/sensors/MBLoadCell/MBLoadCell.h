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

/**
 * @brief driver in order to communicate with a TLB digital-analog weight
 * transmitter attached to a loadcell
 *
 * The driver permits to use the TLB transmitter in different modes:
 * - Continuous-modT: unidirectional protocol that consists in receiving the
 * gross weight
 * - Continuous-modTd: unidirectional protocol that consists in receiving the
 * gross weight
 * - ASCII-modTd: bidirectional mode that consists in sending a request and
 * receiving a response with the data requested or an error message
 */
class MBLoadCell : public Sensor<MBLoadCellData>
{
public:
    /**
     * @brief constructor that initializes the serial communication with the
     * load cell
     * @param mode the mode in which the load cell is in
     * @param serialPortNum port number to which the load cell is connected
     * @param baudrate baudrate set on the TLB converter
     */
    MBLoadCell(LoadCellModes mode, int serialPortNum, int baudrate);

    bool init() override;

    bool selfTest() override;

    /**
     * @brief sample function overridden because the sampleImpl doesn't return
     * the last sample but modifies that struct directly. This is useful in the
     * bidirectional mode when we request data and we don't want to lose it
     */
    void sample() override;

protected:
    /**
     * @brief override of the sample implementation method that requests the
     * weight sampled from the load cell or waits for a sample depending on
     * the mode selected (it's a blocking function)
     * @return the weight measured from the load cell
     */
    MBLoadCellDataStr sampleImpl() override;

    /**
     * @brief sampling in the "continuous Mod T" mode
     */
    void sampleContModT(void);

    /**
     * @brief sampling in the "continuous Mod Td" mode
     */
    void sampleContModTd(void);

    /**
     * @brief sampling in the "ASCII Mod Td" mode
     */
    void sampleAsciiModTd(void);

    /**
     * @brief forges a request for the ascii mode
     * @param toRequest the request to forge
     * @param req reference to the request that will be generated
     */
    void generateRequest(LoadCellValues toRequest, DataAsciiRequest &req);

    template <typename T>
    void receive(T *buf);

    template <typename T>
    void transmit(T *buf);

private:
    SerialInterface *serial;  ///< pointer to the instance of the serial port
                              ///< opened for the connection
    LoadCellModes mode;       ///< mode in which the load cell will be used
};