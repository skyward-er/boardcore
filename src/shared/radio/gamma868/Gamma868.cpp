/* Copyright (c) 2017 Skyward Experimental Rocketry
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

#include "Gamma868.h"

using namespace miosix;

namespace Boardcore
{

/*
 * A serial port attached to the Gamma868 RX and TX pins is expected
 * to be passed to the object in order to communicate with the device.
 *
 */
Gamma868::Gamma868(const char* serialPath, const uint16_t multiplier)
    : sendTimeoutMultiplier(multiplier), conf{}, gammaSwitch{}
{
    confEnabled = false;
    fd          = open(serialPath, O_RDWR);

    if (fd < 0)
        LOG_ERR(logger, "Cannot open {}", serialPath);
    // TODO: Signal an error if we fail here?
}

/*
 * A serial port attached to the Gamma868 RX and TX pins is expected
 * to be passed to the object in order to communicate with the device.
 *
 */
Gamma868::Gamma868(const char* serialPath, GpioPin* learnModePin,
                   const uint16_t multiplier)
    : Gamma868(serialPath, multiplier)
{
    gammaSwitch = learnModePin;
    confEnabled = true;

    gammaSwitch->mode(Mode::OUTPUT);
    gammaSwitch->high();
}

Gamma868::~Gamma868() { close(fd); }

/*
 * Immediately sends command (blocking).
 */
bool Gamma868::send(uint8_t* pkt, size_t packetLength)
{
    bool ret = (write(fd, pkt, packetLength) > 0);
    Thread::sleep(sendTimeoutMultiplier * packetLength);
    return ret;
}

/*
 * Reads from the gamma868 serial (blocking).
 */
ssize_t Gamma868::receive(uint8_t* pkt, size_t packetLength)
{
    if (packetLength > 0)
        return read(fd, pkt, 1);
    else
        return 0;
}

/*
 * Reads the configuration from the device and updates the conf varaiable.
 */
GammaConf Gamma868::readConfig()
{
    if (!confEnabled)
    {
        conf.isValid = false;
    }
    else
    {
        enterLearnMode();

        bool ok = updateConfig();
        if (!ok)
            conf.isValid = false;

        exitLearnMode();
    }

    return conf;
}

/*
 * Set a new configuration to gamma.
 * Returns true if the configuration was set right.
 */
bool Gamma868::configure(const GammaConf& newConf)
{
    bool retValue;

    if (!confEnabled)
        return false;

    enterLearnMode();

    GammaConf oldConf;
    memcpy(&oldConf, &conf, sizeof(GammaConf));

    // Write the new configuration
    LOG_DEBUG(logger, "Writing new configuration...");
    writeConfig(newConf);

    // Check the current configuration
    bool ok = updateConfig();

    if (ok && conf == newConf)
    {
        LOG_DEBUG(logger, "Config ok");
        retValue = true;
    }
    else
    {
        LOG_ERR(logger, "Config error");
        memcpy(&conf, &oldConf, sizeof(GammaConf));  // rollback
        retValue = false;
    }

    exitLearnMode();

    return retValue;
}

/*
 * Puts the gamma868 in "learn mode" (configuration mode).
 */
void Gamma868::enterLearnMode()
{
    // TODO: switch baudrate to 9600

    // Enter learn mode
    LOG_DEBUG(logger, "Entering learn mode...");
    gammaSwitch->low();

    // Wait 5 seconds
    miosix::Thread::sleep(LEARN_MODE_TIMEOUT);

    gammaSwitch->high();  // Stop "pushing" the button
}

/*
 * Puts the gamma868 out of "learn mode" (configuration mode).
 */
void Gamma868::exitLearnMode()
{
    // TODO: switch baudrate according to configuration

    LOG_DEBUG(logger, "Exiting learn mode");
    write(fd, "#Q", 2);
}

/*
 * Sends configuration to the gamma868 module.
 */
void Gamma868::writeConfig(const GammaConf& newConf)
{
    uint8_t confAddress[8] = "#A";
    memcpy(confAddress + 2, &(newConf.localAddress), 3);
    memcpy(confAddress + 5, &(newConf.destinationAddress), 3);

    write(fd, confAddress, 8);
    waitForOk();

    char confBaud[3] = "#B";
    confBaud[2]      = (uint8_t)newConf.baudrate;
    write(fd, confBaud, 3);
    waitForOk();

    char confHandshake[3] = "#H";
    confHandshake[2]      = (uint8_t)newConf.handshake;
    write(fd, confHandshake, 3);
    waitForOk();

    char confLora[4] = "#C";
    confLora[2]      = (uint8_t)newConf.loraSf;
    confLora[3]      = (uint8_t)newConf.loraPower;
    write(fd, confLora, 4);
    waitForOk();

    memcpy(&conf, &newConf, sizeof(GammaConf));
    conf.isValid = true;
}

/*
 * Reads the configuration from the device and updates the conf varaiable.
 */
bool Gamma868::updateConfig()
{
    if (!confEnabled)
        return false;

    GammaMessage msg;

    // Read from device
    write(fd, "#?", 2);
    read(fd, &(msg.buf), sizeof(GammaMessage));

    // Check values validity
    if (msg.conf.loraMode >= LAST_SF || msg.conf.loraPower >= LAST_POWER ||
        msg.conf.baudrate >= LAST_BAUDRATE)
    {
        return false;
    }

    // Update conf variable
    conf.isValid = true;

    // Addresses
    memcpy(&conf.localAddress, msg.conf.localAddress, 3);
    memcpy(&conf.destinationAddress, msg.conf.destinationAddress, 3);

    // LoRa values
    conf.loraSf    = static_cast<GammaSF>(msg.conf.loraMode);
    conf.loraPower = static_cast<GammaPower>(msg.conf.loraPower);
    conf.baudrate  = static_cast<GammaBaudrate>(msg.conf.baudrate);

    conf.handshake = (msg.conf.handshake > 0) ? true : false;

    return true;
}

/*
 * Waits until an "OK" is received on the serial port (blocking).
 */
void Gamma868::waitForOk()
{
    char reply[3];
    read(fd, reply, 3);
    LOG_DEBUG(logger, "Device replied: {}", reply);
    Thread::sleep(100);
}

}  // namespace Boardcore
