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
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.  IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 */

#ifndef GAMMA_TYPES_H
#define GAMMA_TYPES_H

#include <Common.h>
#include <iostream>
#include <array>

enum GammaBaudrate : uint8_t 
{
    B9600   = 0,
    B192000 = 1,
    B28800  = 2,
    B38400  = 3,
    B57600  = 4,
    LAST_BAUDRATE
};

enum GammaSF : uint8_t
{
    SF6     = 0, 
    SF7     = 1,
    SF8     = 2,
    SF9     = 3,
    SF10    = 4,
    SF11    = 5,
    SF12    = 6,
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
    bool is_valid  = false; // True if the configuration has been read from the device
    bool handshake = false;
    std::array<uint8_t, 3> local_addr = {{ 125 }};
    std::array<uint8_t, 3> dest_addr  = {{ 125 }};
    GammaSF         lora_sf     = SF6;  
    GammaPower      lora_power  = dbm15;   
    GammaBaudrate   baudrate    = B57600;
};

/* Comparison operator */
inline bool operator==(const GammaConf& lhs, const GammaConf& rhs)
{
    return lhs.handshake == rhs.handshake &&
           lhs.local_addr == rhs.local_addr &&
           lhs.dest_addr == rhs.dest_addr &&
           lhs.lora_sf == rhs.lora_sf &&
           lhs.lora_power == rhs.lora_power &&
           lhs.baudrate == rhs.baudrate;
}

/* Stream operator */
inline std::ostream& operator<<(std::ostream& o, const GammaConf& conf)
{
    return o << "local_addr: " << conf.local_addr[0] << "\tdest_addr: " << 
     conf.dest_addr[0] << "spreading factor: " << (uint8_t)conf.lora_sf << 
     "\toutput power: " << (uint8_t)conf.lora_power << "handshake: " << 
     (uint8_t)conf.handshake << "\tbaudrate: " << (uint8_t)conf.baudrate << 
     std::endl;
}

/* Message received from the Gamma, as per datasheet */
union gamma_msg 
{
    struct gamma_msg_t
    {
        uint8_t cmd_echo[2];
        uint8_t local_addr[3];
        uint8_t dest_addr[3];
        uint8_t null_value;
        uint8_t lora_mode;
        uint8_t lora_power;
        uint8_t handshake;
        uint8_t baudrate;
    } conf;

    uint8_t buf[13];
};

#endif /* GAMMA_TYPES_H */