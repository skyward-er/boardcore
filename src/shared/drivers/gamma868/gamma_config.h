/* Copyright (c) 2017 Skyward Experimental Rocketry
 * Authors: Alvise De Faveri
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

#ifndef CONFIG_H
#define CONFIG_H

// DISCOVERY gpio configuration
typedef Gpio<GPIOB_BASE, 0> gammaLed;
typedef Gpio<GPIOB_BASE, 2> gammaSwitch;

// Module internal config
struct Configuration
{
    int local_addr[3] = {126, 126, 126};
    int dest_addr[3]  = {126, 126, 126};
    int lora_mode     = 1;   // SF6
    int lora_pow      = 15;  //+20dbm
    int handshake     = 0;   // No handshake
    int baudrate      = 0;   // 9600 baud
};

// Buffer config
#define OUT_BUFFER_SIZE 10

/*
 * Change this if you want to send even if the buffer contains
 * less chars than the packet (note that the packet has fixed size).
 */
#define MIN_TO_SEND 5

#define SEND_SLEEP_TIME 0

// Protocol config
#define HEAD_LEN 2
#define CMD_LEN 1
#define DATA_LEN 5
#define END_LEN 1

#define START '#'
#define CMD '!'
#define DATA '?'
#define END '%'

#endif
