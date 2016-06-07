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

//Fuck the linter!!!! :P :P 
typedef const unsigned int cui;
typedef const unsigned char cuc;

//maximum number of sockets managed by the device
cuc MAX_SOCK_NUM = 8;

cui COMMON_BASE = 0x0000;

cui TX_BUF_BASE = COMMON_BASE + 0x8000;  //TX buffer memory base address

cui RX_BUF_BASE = COMMON_BASE + 0xC000;  //RX buffer memory base address

/** common registers **/

cui MR              = COMMON_BASE + 0x0000;  //mode register address
cui GAR_BASE        = COMMON_BASE + 0x0001;  //Gateway IP Register base address
cui SUBR_BASE       = COMMON_BASE + 0x0005;  //Subnet mask Register base address
cui SHAR_BASE       = COMMON_BASE + 0x0009;  //Source MAC Register base address
cui SIPR_BASE       = COMMON_BASE + 0x000F;  //Source IP Register base address
cui IR              = COMMON_BASE + 0x0015;  //Interrupt Register address
// cui IR_MASK         = COMMON_BASE + 0x0016;  //Interrupt mask register
cui IR_MASK         = COMMON_BASE + 0x0036;  //Interrupt mask register
cui RTR_BASE        = COMMON_BASE + 0x0017;  //retransmission Timeout register
cui RCR             = COMMON_BASE + 0x0019;  //retransmission count register
cui SOCK_IR         = COMMON_BASE + 0x0034;  //Socket Interrupt Register
// cui SOCK_IR_MASK    = COMMON_BASE + 0x0036;  //Socket Interrupt mask register
cui SOCK_IR_MASK    = COMMON_BASE + 0x0016;  //Socket Interrupt mask register
cui PHY             = COMMON_BASE + 0x0035;  //PHY Status Register

cui VERSION         = COMMON_BASE + 0x001F;  //chip version number register

cui PPP_AUTH_REG    = COMMON_BASE + 0x001C;  //autentication type in PPPoE mode
cui PPP_TIME_REG    = COMMON_BASE + 0x0028;  //LCP Request Timer register in PPPoE mode
cui PPP_MAGIC_REG   = COMMON_BASE + 0x0029;  //PPP LCP Magic number register in PPPoE mode

cui INTLEVEL0       = COMMON_BASE + 0x0030;  //set Interrupt low level timer register
cui INTLEVEL1       = COMMON_BASE + 0x0031;


/* MODE register values */
cuc MR_RST          = 0x80; //reset
cuc MR_PB           = 0x10; //ping block enable
cuc MR_PPPOE        = 0x08; //PPPoE enable

/* IR register values */
cuc IR_CONFLICT     = 0x80; //IP conflict
cuc IR_PPPoE        = 0x20; //PPPoE connection close
//#define IR_SOCK(ch)     (0x01 << ch) //check socket interrupt


/** socket registers **/

cui SR_BASE         = COMMON_BASE + 0x4000;  //socket registers base address
cui SR_SIZE         = 0x100;                 //size of each channel register map

cui SOCKn_MR            = SR_BASE + 0x0000; //socket Mode register
cui SOCKn_CR            = SR_BASE + 0x0001; //socket command register
cui SOCKn_IR            = SR_BASE + 0x0002; //socket interrupt register
cui SOCKn_SR            = SR_BASE + 0x0003; //socket status register
cui SOCKn_SPORT0        = SR_BASE + 0x0004; //socket source port register
cui SOCKn_DHAR0         = SR_BASE + 0x0006; //socket destination MAC address register
cui SOCKn_DIPR0         = SR_BASE + 0x000C; //socket destination IP address register
cui SOCKn_DPORT0        = SR_BASE + 0x0010; //socket destination port register
cui SOCKn_IMR           = SR_BASE + 0x002C; //socket's interrupt mask register

cui SOCKn_MSSR0         = SR_BASE + 0x0012; //socket MSS in TCP mode

cui SOCKn_PROTO         = SR_BASE + 0x0014; //socket protocol number in IPRAW mode

cui SOCKn_TOS           = SR_BASE + 0x0015; //socket's IP header's Type of Service field value
cui SOCKn_TTL           = SR_BASE + 0x0016; //socket's IP header's TTL field value
cui SOCKn_FRAG0         = SR_BASE + 0x002D; //socket's IP header's Fragment field value 

cui SOCKn_RXMEM_SIZE    = SR_BASE + 0x001E; //socket's RX buffer size register
cui SOCKn_TXMEM_SIZE    = SR_BASE + 0x001F; //socket's TX buffer size register

cui SOCKn_TX_FSR0       = SR_BASE + 0x0020; //socket's TX buffer free size register
cui SOCKn_TX_RD0        = SR_BASE + 0x0022; //socket's TX buffer read pointer address
cui SOCKn_TX_WR0        = SR_BASE + 0x0024; //socket's TX buffer write pointer address

cui SOCKn_RX_RSR0       = SR_BASE + 0x0026; //socket's received data size register
cui SOCKn_RX_RD0        = SR_BASE + 0x0028; //socket's RX buffer read pointer address
cui SOCKn_RX_WR0        = SR_BASE + 0x002A; //socket's RX buffer write pointer address



/* SOCKn_MR values */
cuc SOCKn_MR_CLOSE     = 0x00;  //socket closed
cuc SOCKn_MR_TCP       = 0x01;  //TCP mode
cuc SOCKn_MR_UDP       = 0x02;  //UDP mode
cuc SOCKn_MR_IPRAW     = 0x03;  //IP layer raw socket
cuc SOCKn_MR_MACRAW    = 0x04;  //MAC layer raw socket
cuc SOCKn_MR_PPPOE     = 0x05;  //PPPoE mode
cuc SOCKn_MR_ND        = 0x20;  //No delayed ACK enable
cuc SOCKn_MR_MULTI     = 0x80;  //enable multicasting (only in UDP mode)

/* SOCKn_CR values */
cuc SOCKn_CR_OPEN      = 0x01;  //initialize and open socket
cuc SOCKn_CR_LISTEN    = 0x02;  //wait connection request in TCP mode (Server mode)
cuc SOCKn_CR_CONNECT   = 0x04;  //send connection request in TCP mode (Client mode)
cuc SOCKn_CR_DISCON    = 0x08;  //disconnect request in TCP mode
cuc SOCKn_CR_CLOSE     = 0x10;  //close socket
cuc SOCKn_CR_SEND      = 0x20;  //send all data stored in TX buffer
cuc SOCKn_CR_SEND_MAC  = 0x21;  //send data with MAC address without ARP process (only in UDP mode)
cuc SOCKn_CR_SEND_KEEP = 0x22;  //check if TCP connection is still alive
cuc SOCKn_CR_RECV      = 0x40;  //receive data

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
cuc SOCKn_IR_CON       = 0x01;        //connection established
cuc SOCKn_IR_DISCON    = 0x02;        //disconnected (TCP mode)
cuc SOCKn_IR_RECV      = 0x04;        //some data received
cuc SOCKn_IR_TIMEOUT   = 0x08;        //Timeout occurred in ARP or TCP
cuc SOCKn_IR_SEND_OK   = 0x10;        //SEND command completed

/* SOCKn_SR values */
cuc SOCK_CLOSED        = 0x00;        //socket closed
cuc SOCK_INIT          = 0x13;        //TCP init state
cuc SOCK_LISTEN        = 0x14;        //TCP server listen for connection state
cuc SOCK_SYNSENT       = 0x15;        //TCP connection request sent to server
cuc SOCK_SYNRECV       = 0x16;        //TCP connection request received from client
cuc SOCK_ESTABLISHED   = 0x17;        //TCP connection established
cuc SOCK_FIN_WAIT      = 0x18;        //TCP closing state
cuc SOCK_CLOSING       = 0x1A;        //TCP closing state
cuc SOCK_TIME_WAIT     = 0x1B;        //TCP closing state
cuc SOCK_CLOSE_WAIT    = 0x1C;        //TCP closing state
cuc SOCK_LAST_ACK      = 0x1D;        //TCP closing state
cuc SOCK_UDP           = 0x22;        //socket opened in UDP mode
cuc SOCK_IPRAW         = 0x32;        //socket opened in IP raw mode
cuc SOCK_MACRAW        = 0x42;        //socket opened in MAC raw mode
cuc SOCK_PPPOE         = 0x5F;        //socket opened in PPPoE mode

/* IP PROTOCOL */
cuc IPPROTO_IP         = 0;           // Dummy for IP
cuc IPPROTO_ICMP       = 1;           // ICMP protocol
cuc IPPROTO_IGMP       = 2;           // IGMP protocol
cuc IPPROTO_GGP        = 3;           // GGP protocol
cuc IPPROTO_TCP        = 6;           // TCP
cuc IPPROTO_PUP        = 12;          // PUP
cuc IPPROTO_UDP        = 17;          // UDP
cuc IPPROTO_IDP        = 22;          // XNS idp
cuc IPPROTO_RAW        = 255;         // Raw IP packet */

#endif