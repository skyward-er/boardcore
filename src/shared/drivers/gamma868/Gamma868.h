/* Copyright (c) 2015-2018 Skyward Experimental Rocketry
 * Authors: Alvise de'Faveri Tron, Nuno Barcellos
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
#include <drivers/Transceiver.h>
#include <fcntl.h>

#include "GammaTypes.h"

namespace Boardcore
{

class Gamma868 : public Transceiver
{
public:
    const int LEARN_MODE_TIMEOUT = 5000;
    /*
     * Create a Gamma868 object using the given path as the serial port to use.
     * @param serialPath        Name of the serial port (es. /dev/tty)
     * @param multiplier        If defined, after each send the send function
     *                          will block for multiplier*nBytesSent
     * milliseconds.
     */
    explicit Gamma868(const char* serialPath, const uint16_t multiplier = 0);

    /*
     * Create a Gamma868 that can be configured through the LRN pin.
     * @param serialPath        Name of the serial port (es. /dev/tty)
     * @param learnModePin           pin connected to the Learn Mode pin of the
     * Gamma
     * @param multiplier        If defined, after each send the send() function
     *                          will block for multiplier*nBytesSent
     * milliseconds.
     */
    Gamma868(const char* serialPath, miosix::GpioPin* learnModePin,
             const uint16_t multiplier = 0);

    ~Gamma868();

    /*
     * Send a message through the serial port to the gamma868 module (blocking).
     * @param pkt               Pointer to the packet (needs to be at least
     * packetLength bytes).
     * @param packetLength           Lenght of the packet to be sent.
     * @return                  True if the message was sent correctly.
     */
    bool send(uint8_t* pkt, size_t packetLength) override;

    /*
     * Receive a message through the serial port to the gamma868 module
     * (blocking).
     * @param pkt               Pointer to the buffer (needs to be at least
     * packetLength bytes).
     * @param packetLength           Maximum lenght of the packet to be
     * received.
     * @return                  Size of the data received or -1 if failure
     */
    ssize_t receive(uint8_t* pkt, size_t packetLength) override;

    /*
     * Set a new configuration to the gamma868 module. Can be done only if the
     * learn pin has been specified in the constructor.
     * @return       True if the configuration was set correctly.
     */
    bool configure(const GammaConf& newConf);

    /*
     * Reads the configuration from the device, updates the internal
     * configuration variable and returns it. Meaningful only if the learn pin
     * has been specified in the constructor.
     * @return       The current configuration.
     */
    GammaConf readConfig();

    /*
     * Immediately returns the value of the configuration variable, without
     * reading it from the device.
     * @return       The current configuration.
     * @warning      check the isValid bit to see if the returned configuration
     * is meaningful.
     */
    GammaConf getConfig() { return conf; }

private:
    int fd;
    const uint16_t sendTimeoutMultiplier;

    GammaConf conf;
    bool confEnabled;
    miosix::GpioPin* gammaSwitch;

    void enterLearnMode();
    void exitLearnMode();

    void writeConfig(const GammaConf& conf);
    bool updateConfig();
    void waitForOk();

    PrintLogger logger = Logging::getLogger("gamma868");
};

}  // namespace Boardcore
