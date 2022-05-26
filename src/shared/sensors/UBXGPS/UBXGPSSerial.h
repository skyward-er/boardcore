/* Copyright (c) 2021 Skyward Experimental Rocketry
 * Authors: Davide Bonomini, Davide Mor, Alberto Nidasio
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

#include <ActiveObject.h>
#include <diagnostic/PrintLogger.h>
#include <miosix.h>
#include <sensors/Sensor.h>

#include "UBXFrame.h"
#include "UBXGPSData.h"

namespace Boardcore
{

/**
 * @brief Driver for Ublox GPSs
 *
 * This driver handles communication and setup with Ublox GPSs. It uses the
 * binary UBX protocol to retrieve and parse navigation data quicker than using
 * the string based NMEA.
 *
 * At initialization it configures the device with the specified baudrate, reset
 * the configuration and sets up UBX messages and GNSS parameters.
 *
 * Communication with the device is performed through a file, the driver opens
 * the serial port under the filepath /dev/<serialPortName>.
 * There is no need for the file to be setted up beforhand.
 *
 * This driver was written for a NEO-M9N gps with the latest version of UBX.
 */
class UBXGPSSerial : public Sensor<UBXGPSData>, public ActiveObject
{
public:
    /**
     * @brief Construct a new UBXGPSSerial object.
     *
     * @param baudrate Baudrate to communicate with the device (max: 921600,
     * min: 4800 for NEO-M9N).
     * @param sampleRate GPS sample rate (max: 25 for NEO-M9N).
     * @param serialPortNumber Number of the serial port connected to the GPS.
     * @param serialPortName Name of the file for the gps device.
     * @param defaultBaudrate Startup baudrate (38400 for NEO-M9N).
     */
    UBXGPSSerial(int baudrate = 921600, uint8_t sampleRate = 10,
                 int serialPortNumber = 2, const char *serialPortName = "gps",
                 int defaultBaudrate = 38400);

    /**
     * @brief Sets up the serial port baudrate, disables the NMEA messages,
     * configures GNSS options and enables UBX-PVT message.
     *
     * @return True if the operation succeeded.
     */
    bool init() override;

    /**
     * @brief Read a single message form the GPS, waits 2 sample cycle.
     *
     * @return True if a message was sampled.
     */
    bool selfTest() override;

private:
    UBXGPSData sampleImpl() override;

    /**
     * @brief Resets the device to its default configuration.
     *
     * @return True if the device reset succeeded.
     */
    bool reset();

    /**
     * @brief Sets the UART port baudrate and enables UBX and disables NMEA on
     * the UART port.
     *
     * @param safe Whether to expect an ack after the command.
     * @return True if the configuration received an acknowledgement.
     */
    bool setBaudrateAndUBX(bool safe = true);

    /**
     * @brief Sets up the serial port with the correct baudrate.
     *
     * Opens the serial port with the default baudrate and changes it to the
     * value specified in the constructor, then it reopens the serial port. If
     * the device is already using the correct baudrate this won't have effect.
     * However if the gps is using a different baudrate the diver won't be able
     * to communicate.
     */
    bool setSerialCommunication();

    /**
     * @brief Configures the dynamic model to airborn 4g.
     *
     * @return True if the configuration received an acknowledgement.
     */
    bool setDynamicModelToAirborne4g();

    /**
     * @brief Configures the navigation solution sample rate.
     *
     * @return True if the configuration received an acknowledgement.
     */
    bool setSampleRate();

    /**
     * @brief Configures the PVT message output rate.
     *
     * @return True if the configuration received an acknowledgement.
     */
    bool setPVTMessageRate();

    bool setPOSLLHMessageRate();

    bool setSOLMessageRate();

    /**
     * @brief Reads a UBX frame.
     *
     * @param frame The received frame.
     * @return True if a valid frame was read.
     */
    bool readUBXFrame(UBXFrame &frame);

    /**
     * @brief Writes a UBX frame.
     *
     * @param frame The frame to write.
     * @return True if the frame is valid.
     */
    bool writeUBXFrame(const UBXFrame &frame);

    /**
     * @brief Writes a UBX frame and waits for its acknowledgement.
     *
     * @param frame The frame to write.
     * @return True if the frame is valid and acknowledged.
     */
    bool safeWriteUBXFrame(const UBXFrame &frame);

    void run() override;

    const int baudrate;        // [baud]
    const uint8_t sampleRate;  // [Hz]
    const int serialPortNumber;
    const char *serialPortName;
    const int defaultBaudrate;  // [baud]

    char gpsFilePath[16];  ///< Allows for a filename of up to 10 characters
    int gpsFile = -1;

    mutable miosix::FastMutex mutex;
    UBXGPSData threadSample{};

    PrintLogger logger = Logging::getLogger("ubloxgps");

    static constexpr unsigned int RESET_SLEEP_TIME = 5000;  // [ms]
    static constexpr unsigned int MAX_TRIES        = 5;     // [1]
};

}  // namespace Boardcore
