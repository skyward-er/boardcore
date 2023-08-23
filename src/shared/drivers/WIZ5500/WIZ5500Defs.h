/* Copyright (c) 2023 Skyward Experimental Rocketry
 * Author: Davide Mor
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

#include <cstdint>

namespace Boardcore {

namespace Wiz {

static constexpr uint8_t VERSION = 0x04;

inline uint8_t buildControlWord(uint8_t block, bool write) {
    return (block & 0b11111) << 3 | (write ? 1 << 2 : 0);
}

inline uint8_t getSocketRegBlock(int n) {
    return (n << 2) | 0b01;
}

inline uint8_t getSocketTxBlock(int n) {
    return (n << 2) | 0b10;
}

inline uint8_t getSocketRxBlock(int n) {
    return (n << 2) | 0b11;
}

namespace Common {

enum Registers {
    REG_MR = 0x0000, //< Mode Register.
    REG_GAR = 0x0001, //< Gateway IP Address Register.
    REG_SUBR = 0x0005, //< Subnet Mask Register.
    REG_SHAR = 0x0009, //< Source Hardware Address Register.
    REG_SIPR = 0x000f, //< Source IP Address Register.
    REG_INTLEVEL = 0x0013, //< Interrupt Low Level Timer Register.
    REG_IR = 0x0015, //< Interrupt Register.
    REG_IMR = 0x0016, //< Interrupt Mask Register.
    REG_SIR = 0x0017, //< Socket Interrupt Register.
    REG_RTR = 0x0019, //< Retry Time-value Register.
    REG_PTIMER = 0x001C, //< PPP Link Control Protocol Request Timer Register.
    REG_PMAGIC = 0x001D, //< PPP Link Control Protocol Magic number Register.
    REG_PHAR = 0x001E, //< Destination Hardware Register in PPPoE mode.
    REG_PSID = 0x0024, //< Session ID Register in PPPoE mode.
    REG_PMRU = 0x0026, //< Maximum Receive Unit in PPPoE mode.
    REG_UIPR = 0x0028, //< Unreachable IP Address Register.
    REG_UPORTR = 0x002C, //< Unreachable Port Register.
    REG_PHYCFGR = 0x002E, //< W5500 PHY Configuration Register.
    REG_VERSIONR = 0x0039, //< W5500 Chip Version Register.
};

}

namespace Socket {

enum Command {
    CMD_OPEN = 0x01,
    CMD_LISTEN = 0x02,
    CMD_CONNECT = 0x04,
    CMD_DISCON = 0x08,
    CMD_CLOSE = 0x10,
    CMD_SEND = 0x20,
    CMD_SEND_MAC = 0x21,
    CMD_SEND_KEEP = 0x22,
    CMD_RECV = 0x23
};

enum Status {
    STAT_CLOSED = 0x00,
    STAT_INIT = 0x13,
    STAT_LISTEN = 0x14,
    STAT_SYNSENT = 0x15,
    STAT_SYNRECV = 0x16,
    STAT_ESTABLISHED = 0x17,
    STAT_FIN_WAIT = 0x18,
    STAT_CLOSING = 0x1A,
    STAT_TIME_WAIT = 0x1B,
    STAT_CLOSE_WAIT = 0x1C,
    STAT_LAST_ACK = 0x1D,
    STAT_UDP = 0x22,
    STAT_MACRAW = 0x42,
};

enum Registers {
    REG_MR = 0x0000, //< Socket n Mode Register.
    REG_CR = 0x0001, //< Socket n Command Register.
    REG_IR = 0x0002, //< Socket n Interrupt Register.
    REG_SR = 0x0003, //< Socket n Status Register.
    REG_PORT = 0x0004, //< Socket n Source Port Register.
    REG_DHAR = 0x0006, //< Socket n Destination Hardware Address Register.
    REG_DIPR = 0x000C, //< Socket n Destination IP Address Register.
    REG_DPORT = 0x0010, //< Socket n Destination Port Register.
    REG_MSSR = 0x0012, //< Socket n Maximum Segment Size Register.
    REG_TOS = 0x0015, //< Socket n IP Type of Server Register.
    REG_TTL = 0x0016, //< Socket n TTL Register.
    REG_RXBUF_SIZE = 0x001E, //< Socket n RX Buffer Size Register.
    REG_TXBUF_SIZE = 0x001F, //< Socket n TX Buffer Size Register.
    REG_TX_FSR = 0x0020, //< Socket n TX Free Size Register.
    REG_TX_RD = 0x0022, //< Socket n TX Read Pointer Register.
    REG_TX_WR = 0x0024, //< Socket n TX Write Pointer Register.
    REG_RX_RSR = 0x0026, //< Socket n Received Size Register.
    REG_RX_RD = 0x0028, //< Socket n RX Read Data Pointer Register.
    REG_RX_WR = 0x002A, //< Socket n RX Write Pointer Register.
    REG_IMR = 0x002C, //< Socket n Interrupt Mask Register.
    REG_FRAG = 0x002D, //< Socket n Fragment Register.
    REG_KPALVTR = 0x002F, //< Socket n Keep Alive Time Register.
};

}

}

};