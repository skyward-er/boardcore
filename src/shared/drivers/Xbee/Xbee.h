/* Copyright (c) 2015-2018 Skyward Experimental Rocketry
 * Authors: Andrea Milluzzo, Artem Glukhov
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

#pragma once;

#define START_DELIMITER 0x7E
#define BROADCAST_ADDR 0x000000000000FFFF
#define ADDR_LEN_BYTE 8
#define ACK_DISABLE 0x00
#define MAX_BROADCAST_HOP 0x00

#include <drivers/Transceiver.h>

class Xbee : public Transceiver
{
    public:

        uint64_t target_addr = BROADCAST_ADDR;

        /*
        * Send a message through the serial port to the Xbee module (blocking).
        * @param pkt               Pointer to the packet (needs to be at least pkt_len bytes).
        * @param pkt_len           Lenght of the packet to be sent (max 64KB per packet)
        * @return                  True if the message was sent correctly.
        */
        bool send(uint8_t* pkt, const uint32_t pkt_len) //posso cambiare il tipo da uint32 a uint 16? non serve così grande
        {
            uint8_t tx_pkt[18+pkt_len];
            uint32_t checksum = 0;

            //pkt contruction:
            tx_pkt[DELIMITER] = START_DELIMITER;

            tx_pkt[PKT_LEN_MSB] = (pkt_len & 0xff00)>>8;
            tx_pkt[PKT_LEN_LSB] = pkt_len & 0xff; 

            tx_pkt[AP_COMMAND] = AP_TX;

            //API frame construction

            tx_pkt[HOST_ACKNOWLEDGE] = ACK_DISABLE;

            for(int i=0; i<ADDR_LEN_BYTE; i++)
                tx_pkt[i+ADDR_MSB] = (target_addr & (0xff00000000000000>>8*i)) >> (56 - 8*i);
            
            tx_pkt[RESERVED] = 0xff;  //reserved bytes
            tx_pkt[RESERVED+1] = 0xfe;

            tx_pkt[BROADCAST_HOP_NUM] = MAX_BROADCAST_HOP;
            tx_pkt[OPTION_BYTE] = 0x43; //see option bits - Transmission packet structure

            //adding payload           

            for(int i=0; i<pkt_len; i++)
                tx_pkt[i+PAYLOAD_START_BYTE] = *(pkt+i); 

            //adding checksum

            for(int i=0; i<PAYLOAD_START_BYTE+pkt_len; i++)
                checksum += tx_pkt[i];
            tx_pkt[PAYLOAD_START_BYTE+pkt_len] = 0xff - (checksum & 0xff);
            //TODO
            //send packet via SPI
            //wait for IRQ
            //receive send ”ok”
            
            

        }

        /*
        * Receive a message through the serial port to the Xbee module (blocking).
        * @param pkt               Pointer to the buffer (needs to be at least pkt_len bytes).
        * @param pkt_len           Lenght of the packet to be received.
        */
        void receive(uint8_t* pkt, const uint32_t pkt_len)
        {

        }

    private:

        enum APCommands : uint8_t
        {
            AP_TX           = 0x10,
            AP_TX_STATUS    = 0x8b,
            AP_RX           = 0x90            
        };

        enum TX_PACKET_STRUCT   : uint8_t
        {
            DELIMITER = 0,
            PKT_LEN_MSB,
            PKT_LEN_LSB,
            AP_COMMAND,
            HOST_ACKNOWLEDGE,
            ADDR_MSB,
            RESERVED = 13,
            BROADCAST_HOP_NUM = 15,
            OPTION_BYTE,
            PAYLOAD_START_BYTE = 17
        };
};

/*

XBee: SPI packet structure (API mode):

  |                             |             frame data             |          |
  | start delimiter |   lenght  |   frame type   |         data      | checksum |
  |        1        |  2  |  3  |       4        |        5 to n     |    n+1   |
  |-----------------|-----|-----|----------------|-------------------|----------|
  |       0x7E      | MSB | LSB | API frame type |  frame + Payload  |    CRC   |



---------------------------------------------------------------------------------------- 
                                        WRITE   
*   Transmission packet structure (SPI) [HEXADECIMAL frame type: 0x10] - see page 151 of datasheet:
    
    "HOW TO SEND DATA"

    Byte:   0   1   2   3   4   5      to     12    13 & 14 15  16  17 to n   n+1

            7E  XX  XX  10  00  000000000000FFFF    FFFE    00  43  PAYLOAD   CRC             

    DESCRIPTION:

    byte 0      = 0x7E                  -   start delimiter
    byte 1-2    = 0xXXXX                -   frame data lenght (byte 4 to n)
    byte 3      = 0x10                  -   transmission API command
    byte 4      = 0x00                  -   ACK request for host: if '0' device doesn't send a response
    byte 5-12   = 0x000000000000FFFF    -   ADDR destination device (in this case is "broadcasting")
    byte 13-14  = 0xFFFE                -   reserved
    byte 15     = 0x00                  -   maximum number of hops a broadcast transmission can occur. '00' means max value
    byte 16     = 0b01000011 = 0x43     -   option bits
                    |||||||+--------        disable ACK:       disabled
                    ||||||+---------        disable RD:        disabled
                    |||||+----------        NACK:              disabled
                    ||||+-----------        trace route:       disabled
                    ||++------------        reserved
                    ++--------------        delivery method:   point-multipoint
    byte 17-n   = XX                   -    PAYLOAD
    byte n+1    = 0xXX                 -    checksum

    CALULATE & VERIFY CHECKSUM:
        -   CALCULATE:
            1   add all bytes of the packet (bytes 0 to n)
            2   keep only 8 bits from the result: result2 = result && 0xFF (bit mask)
            3   subtract this quantity from 0xFF: CRC = 0xFF - result2
        -   VERIFY:
            1   add all bytes including CRC (bytes 0 to n+1)
            2   if checksum is correct, the LSB two digits of the sum equal to 0xFF


----------------------------------------------------------------------------------------
                                        READ
*   Transmission status packet structure (SPI) [HEXADECIMAL frame type: 0x8B] - see page 162 of datasheet:
    
    "XBEE SENDS YOU BACK THIS MESSAGE AFTER A TRANSMISSION OCCURS"

    Byte:   0   1   2   3   4   5 & 6   7   8   9   n+1

            7E  XX  XX  8B  00  FF FE   XX  XX  00  CRC             

    DESCRIPTION:

    byte 0      = 0x7E                  -   start delimiter
    byte 1-2    = 0xXXXX                -   frame data lenght (byte 4 to n)
    byte 3      = 0x8B                  -   transmission STATUS API command
    byte 4      = 0x00                  -   no response frame is delivered (transmission without ACK)
    byte 5-6    = 0xFFFE                -   reserved
    byte 7      = 0xXX                  -   transmit retry counter
    byte 8      = 0x00                  -   delivery status:
                                            **  0x00 = Success
                                            **  0x01 = MAC ACK failure
                                            **  0x02 = Collision avoidance failure
                                            **  0x02 = LBT Failure
                                            **  0x03 = No Spectrum Available
                                            **  0x21 = Network ACK failure
                                            **  0x25 = Route not found
                                            **  0x31 = Internal resource error
                                            **  0x32 = Internal error
                                            **  0x74 = Payload too large
                                            **  0x75 = Indirect message requested
    byte 9      = 0x00                  -   discovery status (0x00 means "no discovery overhead")
    byte n+1    = 0xXX                  -   checksum

    CALULATE & VERIFY CHECKSUM:
        -   CALCULATE:
            1   add all bytes of the packet (bytes 0 to n)
            2   keep only 8 bits from the result: result2 = result && 0xFF (bit mask)
            3   subtract this quantity from 0xFF: CRC = 0xFF - result2
        -   VERIFY:
            1   add all bytes including CRC (bytes 0 to n+1)
            2   if checksum is correct, the LSB two digits of the sum equal to 0xFF  


----------------------------------------------------------------------------------------
                                        READ
*   Receive packet structure (SPI) [HEXADECIMAL frame type: 0x90] - see page 169 of datasheet:
    
    "SOMEONE TRANSMIT TO YOU DATA AND YOU HAVE TO READ IT"

     Byte:   0   1   2   3   4      to     11    12 & 13 14  15 to n   n+1

             7E  XX  XX  90  YYYYYYYYYYYYYYYY    XXXX    XX  PAYLOAD   CRC            

    DESCRIPTION:

    byte 0      = 0x7E                  -   start delimiter
    byte 1-2    = 0xXXXX                -   frame data lenght (byte 4 to n)
    byte 3      = 0x90                  -   receive API command
    byte 4-11   = 0xYYYYYYYYYYYYYYYY    -   
    byte 12-13  = 0xXXXX                -   reserved
    byte 14     = 0xXX                  -   Bit field:
                                            **  0x00 = Packet acknowledged
                                            **  0x01 = Packet was a broadcast packet
                                            **  0x40 = point-multipoint
                                            **  0x80 = Repeater Mode (Directed Broadcast)
                                            **  0xC0 = DigiMesh (Not available on the 10k data rate)
                                            Option values can be combined, for example: 0x40 
                                            and 0x1 will show as 0x41
                                            Ignore all other bits.
    byte 15-N   = XX                    -   PAYLOAD
    byte n+1    = 0xXX                  -   checksum

    CALULATE & VERIFY CHECKSUM:
        -   CALCULATE:
            1   add all bytes of the packet (bytes 0 to n)
            2   keep only 8 bits from the result: result2 = result && 0xFF (bit mask)
            3   subtract this quantity from 0xFF: CRC = 0xFF - result2
        -   VERIFY:
            1   add all bytes including CRC (bytes 0 to n+1)
            2   if checksum is correct, the LSB two digits of the sum equal to 0xFF  
*/  