/*
 * Copyright (C) 2015  Silvano Seva
 * 
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 * 
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 * 
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 * 
 */

#ifndef W5200_DEFS_H
#define W5200_DEFS_H

//maximum number of sockets managed by the device
const unsigned char MAX_SOCK_NUM = 8;

const unsigned int COMMON_BASE = 0x0000;

const unsigned int TX_BUF_BASE = COMMON_BASE + 0x8000;  //TX buffer memory base address

const unsigned int RX_BUF_BASE = COMMON_BASE + 0xC000;  //RX buffer memory base address

/** common registers **/

const unsigned int MR              = COMMON_BASE + 0x0000;  //mode register address
const unsigned int GAR_BASE        = COMMON_BASE + 0x0001;  //Gateway IP Register base address
const unsigned int SUBR_BASE       = COMMON_BASE + 0x0005;  //Subnet mask Register base address
const unsigned int SHAR_BASE       = COMMON_BASE + 0x0009;  //Source MAC Register base address
const unsigned int SIPR_BASE       = COMMON_BASE + 0x000F;  //Source IP Register base address
const unsigned int IR              = COMMON_BASE + 0x0015;  //Interrupt Register address
// const unsigned int IR_MASK         = COMMON_BASE + 0x0016;  //Interrupt mask register
const unsigned int IR_MASK         = COMMON_BASE + 0x0036;  //Interrupt mask register
const unsigned int RTR_BASE        = COMMON_BASE + 0x0017;  //retransmission Timeout register
const unsigned int RCR             = COMMON_BASE + 0x0019;  //retransmission count register
const unsigned int SOCK_IR         = COMMON_BASE + 0x0034;  //Socket Interrupt Register
// const unsigned int SOCK_IR_MASK    = COMMON_BASE + 0x0036;  //Socket Interrupt mask register
const unsigned int SOCK_IR_MASK    = COMMON_BASE + 0x0016;  //Socket Interrupt mask register
const unsigned int PHY             = COMMON_BASE + 0x0035;  //PHY Status Register

const unsigned int VERSION         = COMMON_BASE + 0x001F;  //chip version number register

const unsigned int PPP_AUTH_REG    = COMMON_BASE + 0x001C;  //autentication type in PPPoE mode
const unsigned int PPP_TIME_REG    = COMMON_BASE + 0x0028;  //LCP Request Timer register  in PPPoE mode
const unsigned int PPP_MAGIC_REG   = COMMON_BASE + 0x0029;  //PPP LCP Magic number register  in PPPoE mode

const unsigned int INTLEVEL0       = COMMON_BASE + 0x0030;  //set Interrupt low level timer register
const unsigned int INTLEVEL1       = COMMON_BASE + 0x0031;


/* MODE register values */
const unsigned char MR_RST          = 0x80; //reset
const unsigned char MR_PB           = 0x10; //ping block enable
const unsigned char MR_PPPOE        = 0x08; //PPPoE enable

/* IR register values */
const unsigned char IR_CONFLICT     = 0x80; //IP conflict
const unsigned char IR_PPPoE        = 0x20; //PPPoE connection close
//#define IR_SOCK(ch)     (0x01 << ch) //check socket interrupt


/** socket registers **/

const unsigned int SR_BASE         = COMMON_BASE + 0x4000;  //socket registers base address
const unsigned int SR_SIZE         = 0x100;                 //size of each channel register map

const unsigned int SOCKn_MR            = SR_BASE + 0x0000; //socket Mode register
const unsigned int SOCKn_CR            = SR_BASE + 0x0001; //socket command register
const unsigned int SOCKn_IR            = SR_BASE + 0x0002; //socket interrupt register
const unsigned int SOCKn_SR            = SR_BASE + 0x0003; //socket status register
const unsigned int SOCKn_SPORT0        = SR_BASE + 0x0004; //socket source port register
const unsigned int SOCKn_DHAR0         = SR_BASE + 0x0006; //socket destination MAC address register
const unsigned int SOCKn_DIPR0         = SR_BASE + 0x000C; //socket destination IP address register
const unsigned int SOCKn_DPORT0        = SR_BASE + 0x0010; //socket destination port register
const unsigned int SOCKn_IMR           = SR_BASE + 0x002C; //socket's interrupt mask register

const unsigned int SOCKn_MSSR0         = SR_BASE + 0x0012; //socket MSS in TCP mode

const unsigned int SOCKn_PROTO         = SR_BASE + 0x0014; //socket protocol number in IPRAW mode

const unsigned int SOCKn_TOS           = SR_BASE + 0x0015; //socket's IP header's Type of Service field value
const unsigned int SOCKn_TTL           = SR_BASE + 0x0016; //socket's IP header's TTL field value
const unsigned int SOCKn_FRAG0         = SR_BASE + 0x002D; //socket's IP header's Fragment field value 

const unsigned int SOCKn_RXMEM_SIZE    = SR_BASE + 0x001E; //socket's RX buffer size register
const unsigned int SOCKn_TXMEM_SIZE    = SR_BASE + 0x001F; //socket's TX buffer size register

const unsigned int SOCKn_TX_FSR0       = SR_BASE + 0x0020; //socket's TX buffer free size register
const unsigned int SOCKn_TX_RD0        = SR_BASE + 0x0022; //socket's TX buffer read pointer address
const unsigned int SOCKn_TX_WR0        = SR_BASE + 0x0024; //socket's TX buffer write pointer address

const unsigned int SOCKn_RX_RSR0       = SR_BASE + 0x0026; //socket's received data size register
const unsigned int SOCKn_RX_RD0        = SR_BASE + 0x0028; //socket's RX buffer read pointer address
const unsigned int SOCKn_RX_WR0        = SR_BASE + 0x002A; //socket's RX buffer write pointer address



/* SOCKn_MR values */
const unsigned char SOCKn_MR_CLOSE     = 0x00;        //socket closed
const unsigned char SOCKn_MR_TCP       = 0x01;        //TCP mode
const unsigned char SOCKn_MR_UDP       = 0x02;        //UDP mode
const unsigned char SOCKn_MR_IPRAW     = 0x03;        //IP layer raw socket
const unsigned char SOCKn_MR_MACRAW    = 0x04;        //MAC layer raw socket
const unsigned char SOCKn_MR_PPPOE     = 0x05;        //PPPoE mode
const unsigned char SOCKn_MR_ND        = 0x20;        //No delayed ACK enable
const unsigned char SOCKn_MR_MULTI     = 0x80;        //enable multicasting (only in UDP mode)

/* SOCKn_CR values */
const unsigned char SOCKn_CR_OPEN      = 0x01;        //initialize and open socket
const unsigned char SOCKn_CR_LISTEN    = 0x02;        //wait connection request in TCP mode (Server mode)
const unsigned char SOCKn_CR_CONNECT   = 0x04;        //send connection request in TCP mode (Client mode)
const unsigned char SOCKn_CR_DISCON    = 0x08;        //disconnect request in TCP mode
const unsigned char SOCKn_CR_CLOSE     = 0x10;        //close socket
const unsigned char SOCKn_CR_SEND      = 0x20;        //send all data stored in TX buffer
const unsigned char SOCKn_CR_SEND_MAC  = 0x21;        //send data with MAC address without ARP process (only in UDP mode)
const unsigned char SOCKn_CR_SEND_KEEP = 0x22;        //check if TCP connection is still alive
const unsigned char SOCKn_CR_RECV      = 0x40;        //receive data

// #ifdef __DEF_IINCHIP_PPP__
//     #define SOCKn_CR_PCON      0x23         
//     #define SOCKn_CR_PDISCON       0x24         
//     #define SOCKn_CR_PCR       0x25         
//     #define SOCKn_CR_PCN       0x26        
//     #define SOCKn_CR_PCJ       0x27        
// #endif

/* SOCKn_IR values */
// #ifdef __DEF_IINCHIP_PPP__
//     #define SOCKn_IR_PRECV     0x80        
//     #define SOCKn_IR_PFAIL     0x40        
//     #define SOCKn_IR_PNEXT     0x20        
// #endif
const unsigned char SOCKn_IR_CON       = 0x01;        //connection established
const unsigned char SOCKn_IR_DISCON    = 0x02;        //disconnected (TCP mode)
const unsigned char SOCKn_IR_RECV      = 0x04;        //some data received
const unsigned char SOCKn_IR_TIMEOUT   = 0x08;        //Timeout occurred in ARP or TCP
const unsigned char SOCKn_IR_SEND_OK   = 0x10;        //SEND command completed

/* SOCKn_SR values */
const unsigned char SOCK_CLOSED        = 0x00;        //socket closed
const unsigned char SOCK_INIT          = 0x13;        //TCP init state
const unsigned char SOCK_LISTEN        = 0x14;        //TCP server listen for connection state
const unsigned char SOCK_SYNSENT       = 0x15;        //TCP connection request sent to server
const unsigned char SOCK_SYNRECV       = 0x16;        //TCP connection request received from client
const unsigned char SOCK_ESTABLISHED   = 0x17;        //TCP connection established
const unsigned char SOCK_FIN_WAIT      = 0x18;        //TCP closing state
const unsigned char SOCK_CLOSING       = 0x1A;        //TCP closing state
const unsigned char SOCK_TIME_WAIT     = 0x1B;        //TCP closing state
const unsigned char SOCK_CLOSE_WAIT    = 0x1C;        //TCP closing state
const unsigned char SOCK_LAST_ACK      = 0x1D;        //TCP closing state
const unsigned char SOCK_UDP           = 0x22;        //socket opened in UDP mode
const unsigned char SOCK_IPRAW         = 0x32;        //socket opened in IP raw mode
const unsigned char SOCK_MACRAW        = 0x42;        //socket opened in MAC raw mode
const unsigned char SOCK_PPPOE         = 0x5F;        //socket opened in PPPoE mode

/* IP PROTOCOL */
const unsigned char IPPROTO_IP         = 0;           // Dummy for IP
const unsigned char IPPROTO_ICMP       = 1;           // ICMP protocol
const unsigned char IPPROTO_IGMP       = 2;           // IGMP protocol
const unsigned char IPPROTO_GGP        = 3;           // GGP protocol
const unsigned char IPPROTO_TCP        = 6;           // TCP
const unsigned char IPPROTO_PUP        = 12;          // PUP
const unsigned char IPPROTO_UDP        = 17;          // UDP
const unsigned char IPPROTO_IDP        = 22;          // XNS idp
const unsigned char IPPROTO_RAW        = 255;         // Raw IP packet */

#endif