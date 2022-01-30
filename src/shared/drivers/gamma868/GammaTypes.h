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

#include <array>
#include <iostream>

namespace Boardcore
{

enum GammaBaudrate : uint8_t
{
    B_9600   = 0,
    B_192000 = 1,
    B_28800  = 2,
    B_38400  = 3,
    B_57600  = 4,
    LAST_BAUDRATE
};

enum GammaSF : uint8_t
{
    SF6  = 0,
    SF7  = 1,
    SF8  = 2,
    SF9  = 3,
    SF10 = 4,
    SF11 = 5,
    SF12 = 6,
    LAST_SF
};

enum GammaPower : uint8_t
{
    dbm5 = 0,
    dbm6,
    dbm7,
    dbm8,
    dbm9,
    dbm10,
    dbm11,
    dbm12,
    dbm13,
    dbm14,
    dbm15,
    dbm16,
    dbm17,
    dbm18,
    dbm19,
    dbm20,
    LAST_POWER
};

/* Device configuration */
struct GammaConf
{
    bool isValid =
        false;  // True if the configuration has been read from the device
    bool handshake                            = false;
    std::array<uint8_t, 3> localAddress       = {{125}};
    std::array<uint8_t, 3> destinationAddress = {{125}};
    GammaSF loraSf                            = SF6;
    GammaPower loraPower                      = dbm15;
    GammaBaudrate baudrate                    = B_57600;
};

/* Comparison operator */
inline bool operator==(const GammaConf& lhs, const GammaConf& rhs)
{
    return lhs.handshake == rhs.handshake &&
           lhs.localAddress == rhs.localAddress &&
           lhs.destinationAddress == rhs.destinationAddress &&
           lhs.loraSf == rhs.loraSf && lhs.loraPower == rhs.loraPower &&
           lhs.baudrate == rhs.baudrate;
}

/* Stream operator */
inline std::ostream& operator<<(std::ostream& o, const GammaConf& conf)
{
    return o << "local_address: " << conf.localAddress[0]
             << "\tdest_addr: " << conf.destinationAddress[0]
             << "spreading factor: " << (uint8_t)conf.loraSf
             << "\toutput power: " << (uint8_t)conf.loraPower
             << "handshake: " << (uint8_t)conf.handshake
             << "\tbaudrate: " << (uint8_t)conf.baudrate << std::endl;
}

/* Message received from the Gamma, as per datasheet */
union GammaMessage
{
    struct gamma_msg_t
    {
        uint8_t commandEcho[2];
        uint8_t localAddress[3];
        uint8_t destinationAddress[3];
        uint8_t nullValue;
        uint8_t loraMode;
        uint8_t loraPower;
        uint8_t handshake;
        uint8_t baudrate;
    } conf;

    uint8_t buf[13];
};

}  // namespace Boardcore
