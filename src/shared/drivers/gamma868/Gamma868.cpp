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
    : send_timeout_multiplier(multiplier), conf{}, gammaSwitch{}
{
    conf_enabled = false;
    fd           = open(serialPath, O_RDWR);

    if (fd < 0)
        LOG_ERR(logger, "Cannot open {}", serialPath);
    // TODO: Signal an error if we fail here?
}

/*
 * A serial port attached to the Gamma868 RX and TX pins is expected
 * to be passed to the object in order to communicate with the device.
 *
 */
Gamma868::Gamma868(const char* serialPath, GpioPin* lrn_pin,
                   const uint16_t multiplier)
    : Gamma868(serialPath, multiplier)
{
    gammaSwitch  = lrn_pin;
    conf_enabled = true;

    gammaSwitch->mode(Mode::OUTPUT);
    gammaSwitch->high();
}

Gamma868::~Gamma868() { close(fd); }

/*
 * Immediately sends command (blocking).
 */
bool Gamma868::send(uint8_t* pkt, size_t pkt_len)
{
    bool ret = (write(fd, pkt, pkt_len) > 0);
    Thread::sleep(send_timeout_multiplier * pkt_len);
    return ret;
}

/*
 * Reads from the gamma868 serial (blocking).
 */
ssize_t Gamma868::receive(uint8_t* pkt, size_t pkt_len)
{
    if (pkt_len > 0)
        return read(fd, pkt, 1);
    else
        return 0;
}

/*
 * Reads the configuration from the device and updates the conf varaiable.
 */
GammaConf Gamma868::readConfig()
{
    if (!conf_enabled)
    {
        conf.is_valid = false;
    }
    else
    {
        enterLearnMode();

        bool ok = updateConfig();
        if (!ok)
            conf.is_valid = false;

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

    if (!conf_enabled)
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
    uint8_t conf_addr[8] = "#A";
    memcpy(conf_addr + 2, &(newConf.local_addr), 3);
    memcpy(conf_addr + 5, &(newConf.dest_addr), 3);

    write(fd, conf_addr, 8);
    waitForOk();

    char conf_baud[3] = "#B";
    conf_baud[2]      = (uint8_t)newConf.baudrate;
    write(fd, conf_baud, 3);
    waitForOk();

    char conf_handshake[3] = "#H";
    conf_handshake[2]      = (uint8_t)newConf.handshake;
    write(fd, conf_handshake, 3);
    waitForOk();

    char conf_lora[4] = "#C";
    conf_lora[2]      = (uint8_t)newConf.lora_sf;
    conf_lora[3]      = (uint8_t)newConf.lora_power;
    write(fd, conf_lora, 4);
    waitForOk();

    memcpy(&conf, &newConf, sizeof(GammaConf));
    conf.is_valid = true;
}

/*
 * Reads the configuration from the device and updates the conf varaiable.
 */
bool Gamma868::updateConfig()
{
    if (!conf_enabled)
        return false;

    gamma_msg msg;

    // Read from device
    write(fd, "#?", 2);
    read(fd, &(msg.buf), sizeof(gamma_msg));

    // Check values validity
    if (msg.conf.lora_mode >= LAST_SF || msg.conf.lora_power >= LAST_POWER ||
        msg.conf.baudrate >= LAST_BAUDRATE)
    {
        return false;
    }

    // Update conf variable
    conf.is_valid = true;

    // Addresses
    memcpy(&conf.local_addr, msg.conf.local_addr, 3);
    memcpy(&conf.dest_addr, msg.conf.dest_addr, 3);

    // LoRa values
    conf.lora_sf    = static_cast<GammaSF>(msg.conf.lora_mode);
    conf.lora_power = static_cast<GammaPower>(msg.conf.lora_power);
    conf.baudrate   = static_cast<GammaBaudrate>(msg.conf.baudrate);

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
