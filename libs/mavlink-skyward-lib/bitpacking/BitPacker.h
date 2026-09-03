/**
 * Copyright (c) 2019 Skyward Experimental Rocketry
 * Authors: Luca Conterio
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

#include <math.h>
#include <string>

class BitPacker {

    public:
        /* Constructor: receives an array (packet) and its size
                        (if the size is not specified, the default one is used) */
        BitPacker(uint8_t *packet, unsigned int size = DEFAULT_PACKET_SIZE)
                                            : packet(packet), packet_size(size) {
            // do nothing
        }

        /*
            PAY ATTENTION TO THE POSITION AND THE NUMBER OF BITS YOU WRITE IN THE PACKET.
            IF THOSE TWO PARAMETERS ARE FOR SOME REASON NOT CORRECT, YOU MAY OVERWRITE 
            VALUES THAT THE PACKET ARRAY ALREADY CONTAINS.
            IN THAT CASE YOU HAVE TO CLEAR THE PACKET (see the clear() method) AND RE-WRITE
            ALL THE VALUES YOU NEED.
        */
        /* Write a number of bits equal to bits_num of the 
           parameter value to the packet array, starting from the 
           position identified by bit_pos (first bit of the array
           in which the vlaue will be written).
           Return false in case some parameters are invalid, true if 
           the value has been correctly written. */
        template<typename T>
        bool pack(unsigned int bit_pos, unsigned int bits_num, T value) {

            // check parameters validity
            if (!checkParameters(bit_pos, bits_num, value))
                return false;

            //uint64_t write_value = 0; // the actual variable that will be written in the packet
            // cast the received value to 64-bits variable
            //      dest | source | number of bits
            //memcpy(&write_value, &value, sizeof(uint64_t));

            // reset the bits to 0 before writing in them
            reset(bit_pos, bits_num);

            uint8_t curr_byte; // the current extracted byte from the value paramter
            unsigned int byte_count = 0;  /* the current number of extracted byte (i.e. iteration number)
                                             ---> byte_count must start from 0, since to extract the first
                                                  byte there is no need to shift the value (1 for the second and so on)*/
            unsigned int byte_index = 0; // the index of the packet byte in which the value has to be written
            int remaining_bits_num = bits_num; // the current number of bits that remain to be written in the packet
            
            while (remaining_bits_num > 0) {
                /* extract the i-th byte (identified by byte_count)
                   from value parameter  (from right to left) */
                curr_byte = extractByte(byte_count, value);

                /* note that bytes need to be written in reverse
                   order w.r.t. the order in which they are extracted from value,
                   so this index is computed and used to know where to
                   write the current byte inside the packet (from which bit) */
                unsigned int curr_byte_dest_index = 0;
                if (remaining_bits_num > 8) // necessary condition to avoid accessing the array at negative positions
                    curr_byte_dest_index = bit_pos + bits_num - (byte_count+1) * 8;
                else 
                    curr_byte_dest_index = bit_pos;
                
                // compute the destination byte in the packet
                byte_index = (curr_byte_dest_index - (curr_byte_dest_index % 8)) / 8;

                /* set the byte starting at the bit identified
                   by curr_byte_dest_index (not necessairily 
                   aligned with array elements) equal to curr_byte */
                writeByte(byte_index, curr_byte_dest_index, remaining_bits_num, curr_byte);

                remaining_bits_num = remaining_bits_num - 8; // write 1 byte at each iteration
                byte_count++;  // go to the next byte to be extracted from value
            }

            return true;
        }

        /* Get a value contained in the packet, reading a number
           of bits equal to bits_num, starting from the bit specified by bit_pos.
           The read value is put in the result parameter.
           Return false in case some parameters are invalid, true if the value
           has been correctly read. */
        template<typename T>
        bool unpack(unsigned int bit_pos, unsigned int bits_num, T *result) {

            // check parameters validity
            if (!checkParameters(bit_pos, bits_num, *result))
                return false;

            unsigned int read_bits_num = 0; // current number of bits unpacked
            unsigned int byte_index = (bit_pos - (bit_pos % 8)) / 8; // packet element where to start reading data
            //unsigned int byte_count = 0; // counter used to know how many bytes have been read so far from the packet 
                                         // (i.e. it indiscates the iteration number)
            *result = 0; // the value to be returned

            // until not all the bits have been read
            while(read_bits_num < bits_num) {

                // starting bit position of the current read, relative to the entire packet
                unsigned int starting_pos = bit_pos + read_bits_num;

                // read one element of the packet
                uint64_t read_value = readByte(byte_index, bits_num, &read_bits_num, starting_pos);

                // update the result
                *result |= read_value;

                // go to the next packet element
                byte_index++;
            }

            return true;
        }

        // return the current packet
        uint8_t * getPacket() {
            return packet; 
        }

        // return the packet size
        unsigned int getPacketSize() {
            return packet_size;
        }

        // reset all the elements of packet to zero
        void resetAll() {
            for (unsigned int i = 0; i < packet_size; i++)
                packet[i] = 0;
        }

        // reset to zero a specific byte (element) of packet
        void resetByte(unsigned int index) {
            packet[index] = 0;
        }

        /* reset to zero a number of bits equal to bits_num, 
           starting from position bit_index in the packet.
        */
        void reset(unsigned int bit_index, unsigned int bits_num) {

            // the byte where starting to reset the bits
            unsigned int byte_index = (bit_index - (bit_index % 8)) / 8;

            // the starting bit position, relative to the current byte 
            uint8_t index_in_byte;
            // the number of remaining bits to be reset
            unsigned int remaining_bits_num = bits_num;
            // the number of bits to be reset in the current byte
            uint8_t to_be_reset;
            // the mask used to reset the wanted bits in the current byte
            uint8_t mask;

            while (remaining_bits_num > 0) {

                // index of the starting bit, relative to the current byte
                index_in_byte = bit_index % 8;
                // get the number of bits to be reset in the current bye
                to_be_reset = getRelevantBitsNum(remaining_bits_num, index_in_byte);
                // compute the mask
                mask = getMask(to_be_reset, index_in_byte);
                // invert mask bits to reset the wanted bits to zero
                mask = ~ mask;
                // in order to reset bits to zero perform an AND with the inverted mask
                packet[byte_index] &= mask;

                // update the number of remaining bits by removing the number of reset bits
                remaining_bits_num = remaining_bits_num - to_be_reset; 
                byte_index++; // go to the next byte
                bit_index = bit_index + to_be_reset; // update the starting bit_index
            }
        }

        void printPacket() {
            printf("packet = [ ");
                for (unsigned int j = 0; j < packet_size; j++)
                    printf("%d ", packet[j]);
            printf("]\n");
        }

    /* default number of bytes contained in one packet
       (used in case a different value is not passed to the constructor) */
    static const int DEFAULT_PACKET_SIZE = 25;

    private:
        // extract a specific byte from parameter val
        template<typename T>
        uint8_t extractByte(uint8_t index, T val) {
            uint8_t byte = (val >> (index * 8)) & 0xff;
            if (!byte)
                byte = 0x00; // needed, otherwise bytes containing all zeros are ignored
            return byte;
        }

        /* set a byte of packet to the given one, starting from a specific
           bit, given by the value of the paramter bit_index
           (not necessarly aligned with a packet element,
           the written byte can overlap with multiple ones)
           byte_index: the destination byte position
           bit_index:  the destination bit position
           byte:       the value to be written in bit_index
           bits_num:   number of bits to be written (8 or less) */
        void writeByte(unsigned int byte_index, unsigned int bit_index, unsigned int bits_num, uint8_t byte) {
           
            /* get the relative index inside the found byte
               corresponding to the position given by bit_index */
            uint8_t index_in_byte = bit_index % 8;

            // in case we want to write less then 1 byte
            if (bits_num < 8) {
                // keep only the least significant bits_num of bits of the byte
                // (remove the unwanted bits on the left of the value to be written)
                byte = byte << (8-bits_num);
            }

            /* extract left and right part of the byte to be written
               (those two parts will go in subsequent bytes if the 
               bit_index position is not multiple of 8, otherwise the
               entire value to be written would be contained in left_part) */
            // shift to move left_part aligned to the requested position
            packet[byte_index] |= getLeftPart(byte, index_in_byte);

            /* if index_in_byte == 0 means that the byte to be written
               is aligned with the destination byte in packet
               (i.e. right_part is empty and left_part contains the 
               entire byte to be written) => useless to compute right_part
               in this case.
               Useless also to write a value that is 0 (in OR with what
               contained in  the packet will not change the current 
               packet content) */
            if (index_in_byte != 0 && byte != 0) {
                packet[byte_index+1] |= getRightPart(byte, index_in_byte);
            }
        }

        /* read a byte (8 bits, not necessarly aligned with packet's emlements)
           from the packet.
           bits_num: total number of bits to be read
           read_bits_num: current number of read bits   
           byte_index: the current byte from which the bits have to be read
           starting_pos: position from which starting to read
        */
        uint64_t readByte(unsigned int byte_index, unsigned int bits_num, unsigned int * read_bits_num, unsigned int starting_pos) {
            
            // remaining bits number
            unsigned int remaining_bits_num = bits_num - *read_bits_num;
            // index of the starting bit, relative to the current byte
            uint8_t index_in_byte = starting_pos % 8;
            // number of bits to be read from the current byte
            uint8_t to_be_read = getRelevantBitsNum(remaining_bits_num, index_in_byte);

            // compute the bit mask
            uint8_t mask = getMask(to_be_read, index_in_byte);
            // Notice that this can be actually avoided: 0b11111111 >> index_in_byte would create a mask
            // that removes only unwanted bits on the left, but the possible unwanted bits on the right
            // are then removed in the following if...else statements

            // compute the resulting byte
            uint64_t read_value = packet[byte_index] & mask; // this only removes left unwanted bits

            /* the shift needed by the read value in order to 
               align it with its correct position in the final result
               (in order to be returned) */
            uint8_t shift = 0;
            if (index_in_byte == 0 && to_be_read < 8) { // few bits on the left of the byte
                shift = 8 - to_be_read;
                read_value = read_value >> shift; // shift right to remove unwanted bits on the right
            }
            else if (index_in_byte != 0 && to_be_read < 8) { // few bits in the middle of a byte
                shift = 8 - index_in_byte - to_be_read;
                read_value = read_value >> shift; // shift right to remove unwanted bits on the right
            }
            // few bits on the right of the byte or 8 bits (the entire byte)
            // in case also one of the above if is executed, shift here becomes 0
            shift = bits_num - *read_bits_num - to_be_read;
            read_value = read_value << shift; // shift left to put the read byte in its correct position in the result

            // update the total number of read bits
            *read_bits_num = *read_bits_num + to_be_read;

            return read_value;
        }

        // shift parameter var by i positions to the right
        uint8_t getLeftPart(uint8_t var, uint8_t i) {
            return var >> i;
        }

        // shift parameter var by 8-i positions to the left
        uint8_t getRightPart(uint8_t var, uint8_t i) {
            return var << (8-i);
        }

        // get number of relevant bits (to be read/written) for the current byte
        uint8_t getRelevantBitsNum(unsigned int bits_num, uint8_t index_in_byte) {
            uint8_t relevant_bits_num = 0;
            if (bits_num > 8) { // if more than 8 bits are remaining
                if (index_in_byte != 0)    // if the wanted bits don't start from the first bit of the current byte
                    relevant_bits_num = 8 - index_in_byte; // only get the right ones (i.e. 8 - bit_position_relative_to_current_byte)
                else 
                    relevant_bits_num = 8;  // get the entire byte
            }
            else { // if less than 8 bits are remaining
                if (index_in_byte + bits_num > 8) // if the remaining bits are split among the current byte and the following one
                    relevant_bits_num = 8 - index_in_byte; // just take the ones contained in the current byte
                else
                    relevant_bits_num = bits_num; // else if the remaining bits are entirely contained in the current byte, get all of them
            }

            return relevant_bits_num;
        }

        /* get a bit mask, given how many bits you want to 
           read/write and the starting position, relative
           to a byte (from 0 to 7)
        */
        uint8_t getMask(uint8_t relevant_bits_num, uint8_t index_in_byte) {
            // create a mask with relevant_bits_num set to 1 as least significant bits
            uint8_t mask = pow(2, relevant_bits_num) - 1; 
            // shift the mask to align it with the requested position
            mask = mask << (8 - index_in_byte - relevant_bits_num);

            return mask;
        }

        /* get the position where to write, number of bits to be written
           and value to be written in the packet and check some conditions */
        template<typename T>
        bool checkParameters(int pos,  int num, T val) {
            (void)val; // Avoid unused warning
            // check that T is a numerical type
            static_assert(std::is_arithmetic<T>::value, "The specified value is not a number (T must be an arithmetic type)");
            // check that T is uintX_t
            static_assert(std::is_unsigned<T>::value, "The specified value is not unsigned (T must be an unsigned integer type)");

            // check if nothing to write or negative number of bits
            if (num <= 0)
                return false;
            
            // check if indexes exceed packet dimension
            if (pos < 0 || ((unsigned int)(pos + num) > packet_size * 8))
                return false;
            
            // error if trying to set more bytes than the val variable has
            if (num / 8.0 > sizeof(T)) // num / 8.0 converted to float for correct result  
                return false;

            return true;
        }

        // the packet to be created
        uint8_t * packet;
        // size of the packet array (in bytes)
        unsigned int packet_size;
};