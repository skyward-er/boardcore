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
class MBLoadCell : public Sensor<Data>
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

    /**
     * @brief initializes the serial communication with the load cell
     * @return true if initialization completed with no problems, false
     * otherwise
     */
    bool init() override;

    bool selfTest() override;

    /**
     * @brief generates and sends a request in ASCII mode, waits for the
     * response and updates the last_sample structure
     */
    ReturnsStates asciiRequest(LoadCellValuesEnum r, int value = 0);

    /**
     * @brief permits to reset the peak weight value
     */
    void resetMaxMinWeights();

    /**
     * @brief prints the last sample received
     */
    void printData();

    /**
     * @brief returns a copy of the settings
     */
    MBLoadCellSettings getSettings();

protected:
    /**
     * @brief override of the sample implementation method that requests the
     * weight sampled from the load cell or waits for a sample depending on
     * the mode selected (it's a blocking function)
     * @return the weight measured from the load cell
     */
    Data sampleImpl() override;

    /**
     * @brief sampling in the "continuous Mod T" mode
     */
    Data sampleContModT(void);

    /**
     * @brief sampling in the "continuous Mod Td" mode
     */
    Data sampleContModTd(void);

    /**
     * @brief sampling in the "ASCII Mod Td" mode
     */
    Data sampleAsciiModTd(void);

    /**
     * @brief forges a request for the ascii mode
     * @param req reference to the request that will be generated
     * @param toRequest the request to forge
     * @param value the value used in the forging of the "set_setpoint" requests
     */
    void generateRequest(DataAsciiRequest &req,
                         const LoadCellValuesEnum toRequest, int value = 0);

    /**
     * @brief wrapper to the serial sendString method. This also sets the
     * control pins to enable the transmission mode
     * @param buf the message to send
     */
    void transmitASCII(std::string buf);

    /**
     * @brief wrapper to the serial recvString method. This also sets the
     * control pins to enable the receiver mode
     * @return the message received
     */
    std::string receiveASCII();

    /**
     * @brief wrapper to the serial receive method. This also sets the control
     * pins to enable the receiver mode
     * @param buf the pointer to the buffer in which the data received will be
     * stored
     */
    template <typename T>
    void receive(T *buf);

private:
    MBLoadCellSettings
        settings;             ///< structure that contains all te configuration
    Data max_weight;          ///< the maximum weight detected by the load cell
    Data min_weight;          ///< the minimum weight detected by the load cell
    SerialInterface *serial;  ///< pointer to the instance of the serial port
                              ///< used for the connection
};