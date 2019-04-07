/* Copyright (c) 2015-2019 Skyward Experimental Rocketry
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

#pragma once

#include <Common.h>
#include <drivers/Transceiver.h>
#include <drivers/BusTemplate.h>
#include <vector>
#include <interfaces-impl/hwmapping.h>

using std::vector;

namespace XbeeIRQ
{
    // Thread pointers needed for the IRQHandler to know which thread to wake up
    static miosix::Thread* send_thread = nullptr;
    static miosix::Thread* rcv_thread = nullptr;
    // Flag to indicate to the IRQHandler if the module is sending or receiving
    static bool sending = false;

    /**
     * Handle ATTN interrupt, waking up the correct thread.
     */
    static void handleInterrupt()
    {
        miosix::Thread* waiting;

        // Check which thread to wake up
        if(sending) 
        {
            waiting = send_thread;
            send_thread = 0;
        }
        else
        {
            waiting = rcv_thread;
            rcv_thread = 0;
        }

        // Wake
        waiting->IRQwakeup();
        if (waiting->IRQgetPriority() >
            miosix::Thread::IRQgetCurrentThread()->IRQgetPriority())
        {
            miosix::Scheduler::IRQfindNextThread();
        }
    }
}

// Next to lines are for IDE autocomplete. TODO REMOVE
//using prova=BusSPI<1,miosix::interfaces::spi1::mosi, miosix::interfaces::spi1::miso, miosix::interfaces::spi1::sck>; 
//using Bus=ProtocolSPI<prova,miosix::sensors::ad7994::ab>;


/**
 * WARNING: An IRQ linked with the ATTN pin of the Xbee module must be enabled before using this class.
 *          See test/misc/xbee-bitrate for an example.
 */
template<typename Bus, class ATTN>
class Xbee : public Transceiver
{
    public:
        /*
        * Send a message through the serial port to the Xbee module (blocking).
        * @param pkt               Pointer to the packet (needs to be at least pkt_len bytes).
        * @param pkt_len           Lenght of the packet to be sent (max 64KB per packet)
        * @return true             True if the message was sent correctly. Information about transmission available. 
        * @return false            If pkt is NULL or Xbee returns errors. Information about transmission available.
        */
        bool send(uint8_t* pkt, const uint32_t pkt_len) override
        {
            if(pkt == NULL || pkt_len > MAX_PAYLOAD_LEN)
                return false;

            // If I'm receiving, wait until everything is received
            uint32_t to = SENDING_TIMEOUT;
            while(hasData() && to > 0)
            {
                miosix::Thread::sleep(1);
                to--;
            }

            vector<uint8_t> tx_pkt;
            tx_pkt.reserve(18+pkt_len);

            //pkt contruction
            tx_pkt.push_back(START_DELIMITER);
            //invert pkt_len bytes order
            tx_pkt.push_back((pkt_len & 0xff00)>>8);
            tx_pkt.push_back(pkt_len & 0xff); 

            tx_pkt.push_back(AP_TX);

            //API frame construction
            tx_pkt.push_back(ID_ACK_DISABLE);
            uint8_t addr_buffer[ADDR_LEN];
            memcpy(addr_buffer, &BROADCAST_ADDR, ADDR_LEN);

            for(int i=ADDR_LEN; i>0; i--)
            {
                // NOTE: tx_pkt_address is stored in big endian format
                tx_pkt.push_back(addr_buffer[i]); 
            }
            
            tx_pkt.push_back(0xff);  //reserved bytes
            tx_pkt.push_back(0xfe);

            tx_pkt.push_back(MAX_BROADCAST_HOP);
            // see options bits
            // NOTE: 0x43 = Point-Multipoint, No Ack, No route discovery 
            tx_pkt.push_back(0x43); 

            // payload           
            for(uint32_t i=0; i<pkt_len; i++)
            {
                tx_pkt.push_back(*(pkt+i)); 
            }

            // checksum
            uint32_t checksum = 0;
            for(uint32_t i=3; i < (HEADER_LEN + pkt_len); i++)
            {
                checksum += tx_pkt.at(i);
            }

            tx_pkt.push_back(0xff - (checksum & 0xff)); 

            //send packet via SPI
            Bus::write(tx_pkt.data(), tx_pkt.size());
            XbeeIRQ::sending = true;

            // Wait for interrupt to wake the thread (BLOCKING)
            XbeeIRQ::send_thread = miosix::Thread::getCurrentThread();
            {
                miosix::FastInterruptDisableLock dLock;

                while (XbeeIRQ::send_thread != 0)
                {
                    XbeeIRQ::send_thread->IRQwait();
                    {
                        miosix::FastInterruptEnableLock eLock(dLock);
                        miosix::Thread::yield();
                    }
                }
            }
            XbeeIRQ::sending = false;

            //receive TX status pkt
            uint8_t tx_status_pkt[TX_STATUS_PKT_LEN];
            Bus::read(tx_status_pkt, TX_STATUS_PKT_LEN);

            if ( !checksum_check(tx_status_pkt, TX_STATUS_PKT_LEN) )
                return false;
            
            // Get status from TX Status Packet
            TXStatus = tx_status_pkt[TX_STATUS_BYTE];
            if(TXStatus == TX_SUCCESS)
            {
                TRACE("[XBEE] Tx unsuccessful\n");
                return false;   
            }
            
            return true;
        }

        /*
        * Wait for a new message to be received by the XBee
        * @param buff         Pointer to the buffer (needs to be at least pkt_len bytes).
        * @param buff_len     Lenght of the packet to be received.
        * @return true        A message was found
        * @return false       No valid message (no start frame or not an RX packet)
        *                     Wrong checksum
        *                     Buffer too small
        */
        bool receive(uint8_t* buff, const uint32_t buff_len) override
        {   
            // Wait for the IRQ
            if(!hasData())
            {
                XbeeIRQ::rcv_thread = miosix::Thread::getCurrentThread();
                {
                    miosix::FastInterruptDisableLock dLock;

                    while (XbeeIRQ::rcv_thread != 0)
                    {
                        XbeeIRQ::rcv_thread->IRQwait();
                        {
                            miosix::FastInterruptEnableLock eLock(dLock);
                            miosix::Thread::yield();
                        }
                    }
                }
            }

            vector<uint8_t> rcv_buffer;
            uint8_t rcv_byte =0x00;

            // Read bytes until start frame is found
            while(rcv_byte != START_DELIMITER)
            {
                // In case no frame delimiter was found at all, return false
                if(!hasData())
                {
                    return false;
                }
                else
                {
                    Bus::read(&rcv_byte, 1);
                }
            }

            rcv_buffer.push_back(rcv_byte);

            // Read packet len
            uint16_t pkt_len;
            Bus::read(&rcv_byte, 1); // Invert byte order: first byte
            pkt_len = rcv_byte<<8;
            rcv_buffer.push_back(rcv_byte);

            Bus::read(&rcv_byte, 1); // Second byte
            pkt_len |= rcv_byte;
            rcv_buffer.push_back(rcv_byte);

            // Read rest of packet
            rcv_buffer.reserve(pkt_len + CHECKSUM_LEN + 3);
            Bus::read(rcv_buffer.data() + 3, pkt_len + CHECKSUM_LEN);

            uint32_t payload_len = pkt_len-12;

            if(rcv_buffer.at(3) == AP_RX            // Received a RX frame
                && payload_len <= buff_len       // The buffer is big enough
                && checksum_check(rcv_buffer.data(), rcv_buffer.size()))  // Checksum ok
            {
                memcpy(buff, rcv_buffer.data()+15, payload_len);
                return true;
            }

            return false;
        }

        /*
         * @return  Status sent by the XBee after the last trasmission
         */
        uint8_t getStatus()
        {
            return TXStatus;
        }

    private:
        const uint8_t  START_DELIMITER = 0x7E;
        const uint8_t  HEADER_LEN      = 17;
        const uint8_t  CHECKSUM_LEN    = 1;
        const uint64_t BROADCAST_ADDR  = 0xFFFF;  
        static constexpr uint8_t  ADDR_LEN        = 8;
        const uint8_t  ID_ACK_DISABLE    = 0x00;
        const uint8_t  MAX_BROADCAST_HOP = 0x00;
        const uint16_t MAX_PAYLOAD_LEN   = 0xFFFF;
        static constexpr uint8_t  TX_STATUS_PKT_LEN = 11;
        const uint8_t  TX_STATUS_BYTE    = 8; 
        const uint8_t  TX_SUCCESS        = 0x00;

        const uint32_t SENDING_TIMEOUT = 10;

        uint8_t TXStatus = 0; 

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

        /*
         * Check validity of a received packet.
         * This means calculating the checksum on the whole packet (payload + checksum)
         */
        bool checksum_check(uint8_t* pkt, const uint32_t pkt_len)
        {
            uint64_t checksum=0;
            for(uint32_t i=3; i< (HEADER_LEN + pkt_len + CHECKSUM_LEN); i++)
            {
                checksum += pkt[i];
            }

            return ( (checksum & 0xFF) == 0xFF);
        }

        /**
         * Check the ATTN pin of the XBee (attention pin)
         * @return The Xbee has data to send on the SPI
         */
        bool hasData()
        {
            return (ATTN::value() == 0);
        }
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


    ADDENDUM:                                                   [cfr. datasheet pg.39]
        -   SPI parameters:
                Bit order: send MSB first
                Clock phase (CPHA): sample data on first (leading) edge
                Clock polarity (CPOL): first (leading) edge rises
                (SPI mode 0)
        -   Data format:
                Only API mode 1
        
*/  