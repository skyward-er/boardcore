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

#include <fcntl.h>
#include <sensors/Sensor.h>
#include <stdio.h>

#include <string>

#include "MBLoadCellData.h"
#include "drivers/usart/USART.h"
#include "miosix.h"

namespace Boardcore
{

/**
 * @brief Driver to communicate with a TLB digital-analog weight transmitter
 * attached to a loadcell.
 *
 * The driver allows to use the TLB transmitter in different modes:
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
     * @param serial the serial port to be used
     */
    MBLoadCell(USARTInterface &serial, LoadCellModes mode);

    /**
     * @brief Initializes the serial communication with the load cell.
     *
     * @return True if initialization completed with no problems, false
     * otherwise.
     */
    bool init() override;

    /**
     * @brief Generates and sends a request in ASCII mode, waits for the
     * response and updates the lastSample structure.
     */
    ReturnsStates asciiRequest(LoadCellValuesEnum r, int value = 0);

    /**
     * @brief Permits to reset the peak weight value.
     */
    void resetMaxMinWeights();

    /**
     * @brief Prints the last sample received.
     */
    void printData();

    /**
     * @brief Returns a copy of the max weight detected.
     */
    MBLoadCellData getMaxWeight();

    /**
     * @brief Returns a copy of the min weight detected.
     */
    MBLoadCellData getMinWeight();

    bool selfTest() override;

protected:
    /**
     * @brief Requests the weight sampled from the load cell or waits for a
     * sample depending on the mode selected (it's a blocking function).
     *
     * @return The weight measured from the load cell.
     */
    MBLoadCellData sampleImpl() override;

    /**
     * @brief Sampling in the "continuous Mod T" mode.
     */
    MBLoadCellData sampleContModT(void);

    /**
     * @brief Sampling in the "continuous Mod Td" mode.
     */
    MBLoadCellData sampleContModTd(void);

    /**
     * @brief Sampling in the "ASCII Mod Td" mode.
     */
    MBLoadCellData sampleAsciiModTd(void);

    /**
     * @brief Forges a request for the ascii mode.
     *
     * @param req Reference to the request that will be generated.
     * @param toRequest The request to forge.
     * @param value The value used in the forging of the "set point"
     * requests.
     */
    void generateRequest(DataAsciiRequest &req,
                         const LoadCellValuesEnum toRequest, int value = 0);

    /**
     * @brief Wrapper to the serial sendString method. This also sets the
     * control pins to enable the transmission mode.
     *
     * @param buf The message to send.
     */
    void transmitASCII(const std::string &buf);

    /**
     * @brief Wrapper to the serial recvString method. This also sets the
     * control pins to enable the receiver mode.
     *
     * @return The message received.
     */
    std::string receiveASCII();

    /**
     * @brief Wrapper to the serial receive method. This also sets the control
     * pins to enable the receiver mode.
     *
     * @param buf The pointer to the buffer in which the data received will be
     * stored.
     */
    template <typename T>
    void receive(T *buf);

private:
    MBLoadCellSettings settings;  ///< Contains all the configuration
    MBLoadCellData maxWeight;     ///< Maximum weight detected by the load cell
    MBLoadCellData minWeight;     ///< Minimum weight detected by the load cell
    bool maxSetted;
    bool maxPrint;
    bool minSetted;
    bool minPrint;

    ///< Pointer to the instance of the serial port used for the connection
    USARTInterface &serial;
};

}  // namespace Boardcore
