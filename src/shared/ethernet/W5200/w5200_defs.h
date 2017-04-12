/* Copyright (c) 2015-2016 Skyward Experimental Rocketry
 * Authors: Silvano Seva
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

#ifndef W5200_DEFS_H
#define W5200_DEFS_H

#include <Common.h>

/* Maximum number of sockets managed by the device */
constexpr uint8_t MAX_SOCK_NUM = 8;

static constexpr uint32_t COMMON_BASE = 0x0000;

//TX buffer memory base address
constexpr uint32_t TX_BUF_BASE = COMMON_BASE + 0x8000;

//RX buffer memory base address
constexpr uint32_t RX_BUF_BASE = COMMON_BASE + 0xC000;

/** Common registers **/
enum eW5200Registers {
    MR            = COMMON_BASE + 0x0000, //mode register address
    GAR_BASE      = COMMON_BASE + 0x0001, //Gateway IP Register base address
    SUBR_BASE     = COMMON_BASE + 0x0005, //Subnet mask Register base address
    SHAR_BASE     = COMMON_BASE + 0x0009, //Source MAC Register base address
    SIPR_BASE     = COMMON_BASE + 0x000F, //Source IP Register base address
    IR            = COMMON_BASE + 0x0015, //Interrupt Register address
    IR_MASK       = COMMON_BASE + 0x0036, //Interrupt mask register
    RTR_BASE      = COMMON_BASE + 0x0017, //retransmission Timeout register
    RCR           = COMMON_BASE + 0x0019, //retransmission count register
    SOCK_IR       = COMMON_BASE + 0x0034, //Socket Interrupt Register
    SOCK_IR_MASK  = COMMON_BASE + 0x0016, //Socket Interrupt mask register
    PHY           = COMMON_BASE + 0x0035, //PHY Status Register
    VERSION       = COMMON_BASE + 0x001F, //chip version number register
    PPP_AUTH_REG  = COMMON_BASE + 0x001C, //autentication type in PPPoE mode
    PPP_TIME_REG  = COMMON_BASE + 0x0028, //LCP Request Timer in PPPoE mode
    PPP_MAGIC_REG = COMMON_BASE + 0x0029, //PPP LCP Magic number in PPPoE mode
    INTLEVEL0     = COMMON_BASE + 0x0030, //set Interrupt LL timer register
    INTLEVEL1     = COMMON_BASE + 0x0031,
};

/* MODE register values */
enum eW5200MODERegisters {
    MR_RST        = 0x80, // Reset
    MR_PB         = 0x10, // Ping block enable
    MR_PPPOE      = 0x08, // PPPoE enable
};

/* IR register values */
enum eW5200IRRegisters {
    IR_CONFLICT   = 0x80, //IP conflict
    IR_PPPoE      = 0x20, //PPPoE connection close
};

/* Socket registers */
enum eW5200SockRegisters {
    SR_BASE         = COMMON_BASE + 0x4000, //Socket registers base address
    SR_SIZE         = 0x100,                //Size of each channel register map
};

enum eW5200SockNRegisters {
    SOCKn_MR         = SR_BASE + 0x0000, // Mode register
    SOCKn_CR         = SR_BASE + 0x0001, // Command register
    SOCKn_IR         = SR_BASE + 0x0002, // Interrupt register
    SOCKn_SR         = SR_BASE + 0x0003, // Status register
    SOCKn_SPORT0     = SR_BASE + 0x0004, // Source port register
    SOCKn_DHAR0      = SR_BASE + 0x0006, // Destination MAC address register
    SOCKn_DIPR0      = SR_BASE + 0x000C, // Destination IP address register
    SOCKn_DPORT0     = SR_BASE + 0x0010, // Destination port register
    SOCKn_IMR        = SR_BASE + 0x002C, // Interrupt mask register
    
    SOCKn_MSSR0      = SR_BASE + 0x0012, // MSS in TCP mode
    
    SOCKn_PROTO      = SR_BASE + 0x0014, // Protocol number in IPRAW mode
    
    SOCKn_TOS        = SR_BASE + 0x0015, // IP header ToS field value
    SOCKn_TTL        = SR_BASE + 0x0016, // IP header TTL field value
    SOCKn_FRAG0      = SR_BASE + 0x002D, // IP header Fragment field value 
    
    SOCKn_RXMEM_SIZE = SR_BASE + 0x001E, // RX buffer size register
    SOCKn_TXMEM_SIZE = SR_BASE + 0x001F, // TX buffer size register
    
    SOCKn_TX_FSR0    = SR_BASE + 0x0020, // TX buffer free size register
    SOCKn_TX_RD0     = SR_BASE + 0x0022, // TX buffer read pointer address
    SOCKn_TX_WR0     = SR_BASE + 0x0024, // TX buffer write pointer address
    
    SOCKn_RX_RSR0    = SR_BASE + 0x0026, // Received data size register
    SOCKn_RX_RD0     = SR_BASE + 0x0028, // RX buffer read pointer address
    SOCKn_RX_WR0     = SR_BASE + 0x002A, // RX buffer write pointer address
};

/* SOCKn_MR values */
enum eW5200SockNMRValues {
    SOCKn_MR_CLOSE   = 0x00, // Socket closed
    SOCKn_MR_TCP     = 0x01, // TCP mode
    SOCKn_MR_UDP     = 0x02, // UDP mode
    SOCKn_MR_IPRAW   = 0x03, // IP layer raw socket
    SOCKn_MR_MACRAW  = 0x04, // MAC layer raw socket
    SOCKn_MR_PPPOE   = 0x05, // PPPoE mode
    SOCKn_MR_ND      = 0x20, // No delayed ACK enable
    SOCKn_MR_MULTI   = 0x80, // Enable multicasting (only in UDP mode)
};

/* SOCKn_CR values */
enum eW5200SockNCRValues {
    SOCKn_CR_OPEN      = 0x01, // Initialize and open socket
    SOCKn_CR_LISTEN    = 0x02, // Wait conn request in TCP mode (Server mode)
    SOCKn_CR_CONNECT   = 0x04, // Send conn request in TCP mode (Client mode)
    SOCKn_CR_DISCON    = 0x08, // Disconnect request in TCP mode
    SOCKn_CR_CLOSE     = 0x10, // Close socket
    SOCKn_CR_SEND      = 0x20, // Send all data stored in TX buffer
    SOCKn_CR_SEND_MAC  = 0x21, // Send data + MAC without ARP (only in UDP mode)
    SOCKn_CR_SEND_KEEP = 0x22, // Check if TCP connection is still alive
    SOCKn_CR_RECV      = 0x40, // Receive data
};

/* SOCKn_IR values */
enum eW5200SockNIRValues {
    SOCKn_IR_CON       = 0x01, // Connection established
    SOCKn_IR_DISCON    = 0x02, // Disconnected (TCP mode)
    SOCKn_IR_RECV      = 0x04, // Some data received
    SOCKn_IR_TIMEOUT   = 0x08, // Timeout occurred in ARP or TCP
    SOCKn_IR_SEND_OK   = 0x10, // SEND command completed
};

/* SOCKn_SR values */
enum eW5200SockStatus {
    SOCK_CLOSED        = 0x00, // Socket closed
    SOCK_INIT          = 0x13, // TCP init state
    SOCK_LISTEN        = 0x14, // TCP server listen for connection state
    SOCK_SYNSENT       = 0x15, // TCP connection request sent to server
    SOCK_SYNRECV       = 0x16, // TCP connection request received from client
    SOCK_ESTABLISHED   = 0x17, // TCP connection established
    SOCK_FIN_WAIT      = 0x18, // TCP closing state
    SOCK_CLOSING       = 0x1A, // TCP closing state
    SOCK_TIME_WAIT     = 0x1B, // TCP closing state
    SOCK_CLOSE_WAIT    = 0x1C, // TCP closing state
    SOCK_LAST_ACK      = 0x1D, // TCP closing state
    SOCK_UDP           = 0x22, // Socket opened in UDP mode
    SOCK_IPRAW         = 0x32, // Socket opened in IP raw mode
    SOCK_MACRAW        = 0x42, // Socket opened in MAC raw mode
    SOCK_PPPOE         = 0x5F, // Socket opened in PPPoE mode
};

/* IPProto values */
enum eW5200IPProtoValues {
    IPPROTO_IP         = 0,    // Dummy for IP
    IPPROTO_ICMP       = 1,    // ICMP protocol
    IPPROTO_IGMP       = 2,    // IGMP protocol
    IPPROTO_GGP        = 3,    // GGP protocol
    IPPROTO_TCP        = 6,    // TCP
    IPPROTO_PUP        = 12,   // PUP
    IPPROTO_UDP        = 17,   // UDP
    IPPROTO_IDP        = 22,   // XNS idp
    IPPROTO_RAW        = 255,  // Raw IP packet */
};

#endif
