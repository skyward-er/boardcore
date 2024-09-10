/* Copyright (c) 2024 Skyward Experimental Rocketry
 * Author: Matteo Pignataro, Lorenzo Cucchi, Fabrizio Monti
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

#include <diagnostic/PrintLogger.h>
#include <drivers/usart/USART.h>

#include <string>

namespace Boardcore
{

class VNCommonSerial
{
public:
    VNCommonSerial(USART &usart, int baudrate, const std::string &sensorName);

    ~VNCommonSerial();

private:
    /**
     * @brief The name of the sensor, to be displayed inside the log.
     */
    const std::string sensorName;

protected:
    /**
     * @brief Calculate the 8bit checksum on the given array.
     *
     * @param command Command on which compute the crc.
     * @param length Array length.
     *
     * @return The 8 bit checksum.
     */
    uint8_t calculateChecksum8(const uint8_t *message, int length);

    /**
     * @brief Calculate the 16bit array on the given array.
     *
     * @param command Command on which compute the crc.
     * @param length Array length.
     *
     * @return The 16 bit CRC16-CCITT error check.
     */
    uint16_t calculateChecksum16(const uint8_t *message, int length);

    /**
     * @brief Clear the buffer of the serial interface.
     *
     * This is a placeholder function for the serial interface.
     * When the usart driver is corrected this must be changed.
     */
    void clearBuffer();
    // TODO: remove and use the one in the usart driver

    /**
     * @brief Serial interface that is needed to communicate
     * with the sensor via ASCII codes.
     */
    USART &usart;
    int baudRate;

    PrintLogger logger;
};

}  // namespace Boardcore
