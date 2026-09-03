
#define CATCH_CONFIG_MAIN
#include "include/catch.hpp"

#include "../BitPacker.h"

SCENARIO("check parameters validity", "[param_check]") {

    GIVEN("an array of bytes and a BitPacker object") {
        uint8_t array[5] = {0};
        BitPacker packer(array, 5);
        bool res; 

        WHEN("the position is negative") {
            res = packer.pack(-1, 5, (uint8_t)4);
            THEN("the array is unchanged") {
                bool ok = true;
                for (unsigned int i = 0; i < sizeof(array) && ok; i++) {
                    if (array[i] != 0) {
                        ok = false;
                    }
                }
                REQUIRE(ok == true);
                REQUIRE(res == false);
            }
        }
        WHEN("trying to write 0 bits") {
            res = packer.pack(0, 0, (uint8_t)2);
            THEN("the array is unchanged") {
                bool ok = true;
                for (unsigned int i = 0; i < sizeof(array) && ok; i++) {
                    if (array[i] != 0) {
                        ok = false;
                    }
                }
                REQUIRE(ok == true);
                REQUIRE(res == false);
            }
        }
        WHEN("trying to write a negative number of bits") {
            res = packer.pack(43, -5, (uint8_t)12);
            THEN("the array is unchanged") {
                bool ok = true;
                for (unsigned int i = 0; i < sizeof(array) && ok; i++) {
                    if (array[i] != 0) {
                        ok = false;
                    }
                }
                REQUIRE(ok == true);
                REQUIRE(res == false);
            }
        }
        WHEN("the position is greater than array size") {
            res = packer.pack(25*8, 1, (uint8_t)10);
            THEN("the array is unchanged") {
                bool ok = true;
                for (unsigned int i = 0; i < sizeof(array) && ok; i++) {
                    if (array[i] != 0) {
                        ok = false;
                    }
                }
                REQUIRE(ok == true);
                REQUIRE(res == false);
            }
        }
        WHEN("the sum between position and number of bits is greater than array size") {
            res = packer.pack(32, 15, (uint64_t)10);
            THEN("the array is unchanged") {
                bool ok = true;
                for (unsigned int i = 0; i < sizeof(array) && ok; i++) {
                    if (array[i] != 0) {
                        ok = false;
                    }
                }
                REQUIRE(ok == true);
                REQUIRE(res == false);
            }
        }
        WHEN("the value has less bits than the bits to be written") {
            res = packer.pack(0, 16, (uint8_t)10);
            bool res2 = packer.pack(26, 17, (uint16_t)74);
            bool res3 = packer.pack(13, 23, (uint16_t)74);
            bool res4 = packer.pack(13, 24, (uint16_t)74);
            THEN("the array is unchanged") {
                bool ok = true;
                for (unsigned int i = 0; i < sizeof(array) && ok; i++) {
                    if (array[i] != 0) {
                        ok = false;
                    }
                }
                REQUIRE(ok == true);
                REQUIRE(res == false);
                REQUIRE(res2 == false);
                REQUIRE(res3 == false);
                REQUIRE(res4 == false);
            }
        }
        WHEN("trying to unpack to much bits, w.r.t. those of the result variable") {
            uint8_t result = 0;
            packer.pack(0, 16, (uint32_t)65535); // 16 bits to 1
            res = packer.unpack(0, 16, &result);
            THEN("the array is unchanged") {
                REQUIRE(res == false);
                REQUIRE(result == 0);  // should have not read the 65535 value 
            }
        }
        
        /*
            Uncomment this to check static_asserts conditions.
            The compilation should fail.
            Leave this commented if you want to try all the 
            other tests (compilation needs to be successful)
        
        WHEN("passing a number that is not unsigned") {
            packer.pack(0, 16, 5.4); // 16 bits to 1
            THEN("the compilation fails") {

            }
        }
        WHEN("passing a number that is not unsigned") {
            packer.pack(0, 16, 'c'); // 16 bits to 1
            THEN("the compilation fails") {

            }
        }
        WHEN("passing a number that is not unsigned") {
            packer.pack(0, 16, (int)5); // 16 bits to 1
            THEN("the compilation fails") {

            }
        }
        WHEN("passing a number that is not unsigned") {
            //packer.pack(0, 16, 5.4); // 16 bits to 1
            float result;
            res = packer.unpack(0, 16, &result);
            THEN("the compilation fails") {

            }
        }*/
    }
}

SCENARIO("writing correct values, aligned with array bytes") {

    GIVEN("an array of bytes and a BitPacker object") {
        uint8_t array[BitPacker::DEFAULT_PACKET_SIZE] = {0};
        BitPacker packer(array);
        uint64_t result;

        WHEN("writing in the first array byte") {
            packer.pack(0, 8, (uint8_t)129);
            packer.unpack(0, 8, &result);
            THEN("bits are correctly written") {
                REQUIRE(array[0] == 129);
                REQUIRE(result == 129);
            }
        }
        WHEN("writing in an intermediate array byte") {
            packer.pack(16, 8, (uint8_t)255);
            packer.unpack(16, 8, &result);
            THEN("bits are correctly written") {
                REQUIRE(array[2] == 255);
                REQUIRE(result == 255);
            }
        }
        WHEN("writing in the last array byte") {
            packer.pack(192, 8, (uint8_t)53);
            packer.unpack(192, 8, &result);
            THEN("bits are correctly written") {
                REQUIRE(array[24] == 53);
                REQUIRE(result == 53);
            }
        }
        WHEN("writing only the first bit of the array") {
            packer.pack(0, 1, (uint8_t)53); // 53 = 00110101
            packer.unpack(0, 1, &result);
            THEN("bits are correctly written") {
                REQUIRE(array[0] == 128);
                REQUIRE(result == 1); // only the last bit of 53 has been kept
            }
        }
        WHEN("writing only the last bit of the array") {
            packer.pack(199, 1, (uint8_t)53);
            packer.unpack(199, 1, &result);
            THEN("bits are correctly written") {
                REQUIRE(array[24] == 1);
                REQUIRE(result == 1); // only the last bit of 53 has been kept
            }
        }
    }
}

SCENARIO("writing correct values that are not aligned with array elements") {

    GIVEN("an array of bytes and a BitPacker object") {
        uint8_t array[BitPacker::DEFAULT_PACKET_SIZE] = {0};
        BitPacker packer(array);
        uint64_t result;

        WHEN("writing in the array with valid parameters") {
            packer.pack(2, 1, (uint8_t)128);
            packer.unpack(2, 1, &result);
            THEN("bits are correctly written") {
                REQUIRE(array[0] == 0);
                REQUIRE(result == 0);
            }
        }
        WHEN("writing in the array with valid parameters") {
            packer.pack(2, 1, (uint8_t)1);
            packer.unpack(2, 1, &result);
            THEN("bits are correctly written") {
                REQUIRE(array[0] == 32);
                REQUIRE(result == 1);
            }
        }
        WHEN("writing in the array with valid parameters") {
            packer.pack(46, 8, (uint8_t)23); // 23 = 00010111 
                                    // 11 = 00001011
            packer.unpack(46, 8, &result);
            // 00000000 00000000 00000000 00000000 00000000 00000000 01011100 00000000 00000000
            THEN("bits are correctly written") {
                REQUIRE(array[5] == 0);
                REQUIRE(array[6] == 92);
                REQUIRE(result == 23);
            }
        }
        WHEN("writing less than 8 bits (less than 1 byte) in position 0") {
            packer.pack(0, 5, (uint8_t)55); // 55 keeping only 5 least significant bits becomes 23
            // 00110111 -> 10111000
            packer.unpack(0, 5, &result);
            THEN("bits are correctly written") {
                REQUIRE(array[0] == 184);
                REQUIRE(result == 23);
            }
        }
        WHEN("writing less than 8 bits (less than 1 byte) in an intermediate position") {
            packer.pack(9, 5, (uint8_t)55); // 55 keeping only 5 least significant bits becomes 23
            // 00110111 -> 10111000
            packer.unpack(9, 5, &result);
            THEN("bits are correctly written") {
                REQUIRE(array[0] == 0);
                REQUIRE(array[1] == 92);
                REQUIRE(array[2] == 0);
                REQUIRE(result == 23);
            }
        }
    }
}

SCENARIO("writing multiple overlapping correct values") {

    GIVEN("an array of bytes and a BitPacker object") {
        uint8_t array[BitPacker::DEFAULT_PACKET_SIZE] = {0};
        BitPacker packer(array);
        uint64_t result;
        WHEN("writing overlapping values (on adjacent bytes)") {
            packer.pack(0, 5, (uint8_t)63);
            packer.pack(2, 7, (uint8_t)23);
            // 63 = 00111111 -> 11111000  = 11111000
            // 23 = 00010111 -> 00101110 >> 00001011 10(000000)
            // bits in position 2-3-4-5-6-7-8 are reset before writing 23
            // ---> 11001011 10000000 ... = [203, 128, ...]
            THEN("bits are correctly updated and written") {
                REQUIRE(array[0] == 203);
                REQUIRE(array[1] == 128);

                packer.unpack(0, 5, &result);
                REQUIRE(result == 25);
                packer.unpack(2, 7, &result);
                REQUIRE(result == 23);
            }
        }
    }
}

SCENARIO("writing contiguous sequences of bits") {

    GIVEN("an array of bytes and a BitPacker object") {
        uint8_t array[BitPacker::DEFAULT_PACKET_SIZE] = {0};
        BitPacker packer(array);
        uint64_t result;

        WHEN("writing contiguous values (on adjacent bytes)") {
            packer.pack(10, 4, (uint8_t)20);
            packer.pack(14, 7, (uint8_t)65);
            // 20 = 00010100 -> 01000000 -> 00010000
            // 65 = 01000001 -> 10000010 -> 00000010 00001000
            //                            | 00010010 00001000 
            //                            = 00010010 00001000
            THEN("bits are correctly updated and written") {
                REQUIRE(array[1] == 18);
                REQUIRE(array[2] == 8);

                packer.unpack(10, 4, &result);
                REQUIRE(result == 4);
                packer.unpack(14, 7, &result);
                REQUIRE(result == 65);
            }
        }
        WHEN("writing contiguous values (on adjacent bytes)") {
            packer.pack(10, 4, (uint32_t)20);
            packer.pack(14, 16, (uint32_t)65);
            // 20 = 00010100 -> 01000000 -> 00010000
            // 65 = 00000000 00000000 00000000 01000001 -> 00000000 01000001 00000000 00000000
            //                                          -> 00000000 00000001 00000100 00000000
            // => 00010000 ...
            // |  00000000 00000001 00000100 00000000
            THEN("bits are correctly updated and written") {
                REQUIRE(array[1] == 16);
                REQUIRE(array[2] == 1);
                REQUIRE(array[3] == 4);

                packer.unpack(10, 4, &result);
                REQUIRE(result == 4);
                packer.unpack(14, 16, &result);
                REQUIRE(result == 65);
            }
        }
    }
}

SCENARIO("writing numbers that spread over multiple bytes") {

    GIVEN("an array of bytes and a BitPacker object") {
        uint8_t array[BitPacker::DEFAULT_PACKET_SIZE] = {0};
        BitPacker packer(array);
        uint64_t result;

        WHEN("writing a value that touches 3 bytes of the array") {
            packer.pack(12, 16, (uint32_t)65); // 65 = 00000000 00000000 00000000 01000001
            // DESIRED BEHAVIOUR:
            // shift left by 16 << 00000000 01000001
            // shift right by 4 >> 00000000 00000100 00010000
            // packet = 00000000 00000000 00000100 00010000 ...
            packer.unpack(12, 16, &result);
            THEN("bits are correctly updated and written") {
                REQUIRE(array[0] == 0);
                REQUIRE(array[1] == 0);
                REQUIRE(array[2] == 4);
                REQUIRE(array[3] == 16);
                REQUIRE(array[4] == 0);
                REQUIRE(result == 65);
            }
        }
        WHEN("writing a value that touches 3 bytes of the array") {
            packer.pack(12, 15, (uint32_t)65); // 65 = 00000000 00000000 00000000 01000001
            // DESIRED BEHAVIOUR:
            // shift left by 17 (32-15) << 00000000 10000010
            // shift right by 4 >> 00000000 00001000 0010000
            // packet = 00000000 00000000 00001000 0010000 ...
            packer.unpack(12, 15, &result);
            THEN("bits are correctly updated and written") {
                REQUIRE(array[0] == 0);
                REQUIRE(array[1] == 0);
                REQUIRE(array[2] == 8);
                REQUIRE(array[3] == 32);
                REQUIRE(array[4] == 0);
                REQUIRE(result == 65);
            }
        }
        WHEN("writing a value that touches 5 bytes of the array") {
            packer.pack(12, 32, (uint32_t)65); // 65 = 00000000 00000000 00000000 01000001
            // DESIRED BEHAVIOUR:
            // keep all the bits
            // shift right by 4 >> 00000000 00000000 00000000 00000100 00010000 ...
            // packet = 00000000 00000000 00000000 00000000 00000100 00010000 ...
            packer.unpack(12, 32, &result);
            THEN("bits are correctly updated and written") {
                REQUIRE(array[0] == 0);
                REQUIRE(array[1] == 0);
                REQUIRE(array[2] == 0);
                REQUIRE(array[3] == 0);
                REQUIRE(array[4] == 4);
                REQUIRE(array[5] == 16);
                REQUIRE(array[6] == 0);
                REQUIRE(result == 65);
            }
        }
        WHEN("writing a value that touches 5 bytes of the array") {
            packer.pack(13, 32, (uint64_t)2155872321); // 10000000 10000000 00000000 01000001
            packer.unpack(13, 32, &result);
            THEN("bits are correctly updated and written") {
                // DESIRED BEHAVIOUR:
                // keep all the bits
                // shift by 5 to the right >> 00000100 00000100 00000000 00000010 00001000
                // packet = 00000000 00000100 00000100 00000000 00000010 00001000 ...
                REQUIRE(array[0] == 0);
                REQUIRE(array[1] == 4);
                REQUIRE(array[2] == 4);
                REQUIRE(array[3] == 0);
                REQUIRE(array[4] == 2);
                REQUIRE(array[5] == 8);
                REQUIRE(array[6] == 0);
                REQUIRE(result == 2155872321);
            }
        }
        WHEN("writing a value that touches 4 bytes of the array") {
            packer.pack(8, 25, (uint32_t)16777217); // 00000001 00000000 00000000 00000001
                                                    // -> 10000000 00000000 00000000 1(0000000)
            packer.unpack(8, 25, &result);
            THEN("bits are correctly updated and written") {
                REQUIRE(array[0] == 0);
                REQUIRE(array[1] == 128);
                REQUIRE(array[2] == 0);
                REQUIRE(array[3] == 0);
                REQUIRE(array[4] == 128);
                REQUIRE(result == 16777217);
            }
        }
        WHEN("writing a value that touches 8 bytes of the array") {
            packer.pack(100, 60, (uint64_t)0b1000000010000000100000001000000010000000100000001000000010000000);
            // 10000000 10000000 10000000 10000000 10000000 10000000 10000000 10000000
            // shift by 4 left  (keep 64-60 bits)                  << 00001000 00001000 00001000 00001000 00001000 00001000 00001000 00000(000)
            // shift by 4 right (position 100-96 in the 12th byte) >> 00000000 10000000 10000000 10000000 10000000 10000000 10000000 10000000 00000000
            packer.unpack(100, 60, &result);
            THEN("bits are correctly updated and written") {
                REQUIRE(array[10] == 0);
                REQUIRE(array[11] == 0);
                REQUIRE(array[12] == 0);
                REQUIRE(array[13] == 128);
                REQUIRE(array[14] == 128);
                REQUIRE(array[15] == 128);
                REQUIRE(array[16] == 128);
                REQUIRE(array[17] == 128);
                REQUIRE(array[18] == 128);
                REQUIRE(array[19] == 128);
                REQUIRE(array[20] == 0);
                REQUIRE(result == (uint64_t)0b000010000000100000001000000010000000100000001000000010000000); // only 60 of the above 64 bits number
            }
        }
        WHEN("writing a value that touches 4 bytes of the array from 0 position") {
            packer.pack(0, 27, (uint32_t)67108865); // 00000100 00000000 00000000 00000001
                                                    // -> 10000000 00000000 00000000 001(00000)
            packer.unpack(0, 27, &result);
            THEN("bits are correctly updated and written") {
                REQUIRE(array[0] == 128);
                REQUIRE(array[1] == 0);
                REQUIRE(array[2] == 0);
                REQUIRE(array[3] == 32);
                REQUIRE(array[4] == 0);
                REQUIRE(result == 67108865);
            }
        }
        WHEN("writing a value that touches 4 bytes, reaching the end of the array") {
            packer.pack(175, 25, (uint32_t)2164260865); // 10000001 00000000 00000000 00000001
            packer.unpack(175, 25, &result);
            THEN("bits are correctly updated and written") {
                REQUIRE(array[20] == 0);
                REQUIRE(array[21] == 1);
                REQUIRE(array[22] == 0);
                REQUIRE(array[23] == 0);
                REQUIRE(array[24] == 1); 
                REQUIRE(result == (uint64_t)16777217); // only 25 of the above 32 bits number
            }
        }
    }
}

SCENARIO("writing on contiguous bytes, but not in the order they appear in packet") {

    GIVEN("an array of bytes and a BitPacker object") {
        uint8_t array[BitPacker::DEFAULT_PACKET_SIZE] = {0};
        BitPacker packer(array);
        uint64_t result;
        
         // 00100000 01100000 00000011 00000000 ...
        WHEN("writing the first byte, then the third and then the one in the middle") {
            packer.pack(2, 8, (uint8_t)129);
            packer.pack(23, 8,(uint8_t) 128);
            packer.pack(10, 13, (uint16_t)4097);
            THEN("array elements are correctly updated") {
                REQUIRE(array[0] == 32);
                REQUIRE(array[1] == 96);
                REQUIRE(array[2] == 3);
                REQUIRE(array[3] == 0);

                packer.unpack(2, 8, &result); 
                REQUIRE(result == 129);
                packer.unpack(23, 8, &result);
                REQUIRE(result == 128);
                packer.unpack(10, 13, &result);
                REQUIRE(result == 4097);
            }
        }
    }
}

SCENARIO("writing a complete packet") {

    GIVEN("an array of bytes and a BitPacker object") {
        uint8_t array[BitPacker::DEFAULT_PACKET_SIZE] = {0};
        BitPacker packer(array);
        uint64_t result;

        WHEN("writing all the values in the array") {
            packer.pack(0, 25, (uint32_t)16777216); // 16777216 = 2^24 = 10000000 00000000 00000000 0 (25 bits)
            packer.pack(25, 12, (uint32_t)2048);    // 2048 = 2^11 =     10000000 0000 (12 bits)
            packer.pack(37, 13, (uint32_t)4096);    // 4096 = 2^12 =     10000000 00000 (13 bits)
            packer.pack(50, 12, (uint32_t)2048);    // 2048 = 2^11 =     10000000 0000 (12 bits)
            packer.pack(62, 9, (uint32_t)256);      // 256 = 2^8 =       10000000 0 (9 bits)
            packer.pack(71, 10, (uint32_t)512);     // 512 = 2^9 =       10000000 00 (10 bits)
            packer.pack(81, 10, (uint32_t)512);     // 512 = 2^9 =       10000000 00 (10 bits)
            packer.pack(91, 11, (uint32_t)1024);    // 1024 = 2^10 =     10000000000 (11 bits)
            packer.pack(102, 11, (uint32_t)1024);   // 1024 = 2^10 =     10000000 000 (11 bits)
            packer.pack(113, 11, (uint32_t)1024);   // 1024 = 2^10 =     10000000 000 (11 bits)
            packer.pack(124, 11, (uint32_t)1024);   // 1024 = 2^10 =     10000000 000 (11 bits)
            packer.pack(135, 11, (uint32_t)1024);   // 1024 = 2^10 =     10000000 000 (11 bits)
            packer.pack(146, 11, (uint32_t)1024);   // 1024 = 2^10 =     10000000 000 (11 bits)
            packer.pack(157, 11, (uint32_t)1024);   // 1024 = 2^10 =     10000000 000 (11 bits)
            packer.pack(168, 11, (uint32_t)1024);   // 1024 = 2^10 =     10000000 000 (11 bits)
            packer.pack(179, 10, (uint32_t)512);    // 512 = 2^9 =       10000000 00 (10 bits)
            packer.pack(189, 5, (uint8_t)16);       // 16 = 2^4 =        10000 (5 bits)
            packer.pack(194, 3, (uint8_t)4);        // 4 = 2^2 =         100 (3 bits)
            packer.pack(197, 1, (uint8_t)1);        // 1 = 2^0 =         1 (1 bit)
            packer.pack(198, 1, (uint8_t)1);        // 1 = 2^0 =         1 (1 bit)
            packer.pack(199, 1, (uint8_t)1);        // 1 = 2^0 =         1 (1 bit)
            // total = 200 bit (from 0 to 199)

            // packer.printPacket();
            
            // it should be:
            // 10000000 00000000 00000000 01000000 00000100 00000000 00100000 00000010 00000001 00000000  
            // 01000000 00010000 00000010 00000000 01000000 00001000 00000001 00000000 00100000 00000100 
            // 00000000 10000000 00010000 00000100 00100111
            THEN("bits are correctly updated and written") {
                // 00000001 00000000 00000000 00000000
                REQUIRE(array[0] == 128);
                REQUIRE(array[1] == 0);
                REQUIRE(array[2] == 0);
                REQUIRE(array[3] == 64);
                REQUIRE(array[4] == 4);
                REQUIRE(array[5] == 0);
                REQUIRE(array[6] == 32); 
                REQUIRE(array[7] == 2);
                REQUIRE(array[8] == 1);
                REQUIRE(array[9] == 0);
                REQUIRE(array[10] == 64);
                REQUIRE(array[11] == 16);
                REQUIRE(array[12] == 2);
                REQUIRE(array[13] == 0);
                REQUIRE(array[14] == 64);
                REQUIRE(array[15] == 8);
                REQUIRE(array[16] == 1);
                REQUIRE(array[17] == 0);
                REQUIRE(array[18] == 32);
                REQUIRE(array[19] == 4);
                REQUIRE(array[20] == 0);
                REQUIRE(array[21] == 128);
                REQUIRE(array[22] == 16);
                REQUIRE(array[23] == 4);
                REQUIRE(array[24] == 39);

                packer.unpack(0, 25, &result);
                REQUIRE(result == (uint32_t)16777216);
                packer.unpack(25, 12, &result);
                REQUIRE(result == (uint32_t)2048);
                packer.unpack(37, 13, &result);
                REQUIRE(result == (uint32_t)4096);   
                packer.unpack(50, 12, &result);              
                REQUIRE(result == (uint32_t)2048);
                packer.unpack(62, 9, &result);               
                REQUIRE(result == (uint32_t)256);  
                packer.unpack(71, 10, &result);    
                REQUIRE(result == (uint32_t)512); 
                packer.unpack(81, 10, &result);   
                REQUIRE(result == (uint32_t)512);   
                packer.unpack(91, 11, &result);  
                REQUIRE(result == (uint32_t)1024); 
                packer.unpack(102, 11, &result);
                REQUIRE(result == (uint32_t)1024); 
                packer.unpack(113, 11, &result);
                REQUIRE(result == (uint32_t)1024); 
                packer.unpack(124, 11, &result);
                REQUIRE(result == (uint32_t)1024); 
                packer.unpack(135, 11, &result);
                REQUIRE(result == (uint32_t)1024); 
                packer.unpack(146, 11, &result);
                REQUIRE(result == (uint32_t)1024); 
                packer.unpack(157, 11, &result);
                REQUIRE(result == (uint32_t)1024); 
                packer.unpack(168, 11, &result);
                REQUIRE(result == (uint32_t)1024); 
                packer.unpack(179, 10, &result);
                REQUIRE(result == (uint32_t)512);    
                packer.unpack(189, 5, &result);
                REQUIRE(result == (uint8_t)16);     
                packer.unpack(194, 3, &result);        
                REQUIRE(result == (uint8_t)4);   
                packer.unpack(197, 1, &result);            
                REQUIRE(result == (uint8_t)1);    
                packer.unpack(198, 1, &result);    
                REQUIRE(result == (uint8_t)1); 
                packer.unpack(199, 1, &result);        
                REQUIRE(result == (uint8_t)1);  
            }
        }
    }
}

SCENARIO("writing a complete packet that has a size which is different from the default one") {

    GIVEN("an array of bytes and a BitPacker object") {
        uint8_t array[12] = {0};
        BitPacker packer(array, 12);
        uint64_t result;

        WHEN("the array has a size which is different from the default one") {
            THEN("an array with the specified size is created") {
                REQUIRE(packer.getPacketSize() == 12);
            }
        }
        WHEN("writing all the values in the array") {
            THEN("bits are correctly updated and written") {
                packer.pack(0, 8, (uint8_t)255);         // 11111111
                                                // NOTICE THE 2 MISSING BITS!
                packer.pack(10, 15, (uint32_t)16400);     // 10000000 0010000
                packer.pack(25, 3, (uint8_t)7);          // 111
                packer.pack(28, 145, (uint64_t)0);        // this should fail: too much bits (exceed array size)
                packer.pack(28, 20, (uint32_t)128);       // 00000000 00001000 0000
                packer.pack(48, 23, (uint32_t)4210752);   // 10000000 10000000 1000000
                packer.pack(71, 25, (uint32_t)16843011);  // 10000000 10000000 10000001 1
                // ---> 11111111 00100000 00001000 01110000 00000000 10000000
                //      10000000 10000000 10000001 00000001 00000001 00000011
                REQUIRE(array[0] == 255);
                REQUIRE(array[1] == 32);
                REQUIRE(array[2] == 8);
                REQUIRE(array[3] == 112);
                REQUIRE(array[4] == 0);
                REQUIRE(array[5] == 128);
                REQUIRE(array[6] == 128);
                REQUIRE(array[7] == 128);
                REQUIRE(array[8] == 129);
                REQUIRE(array[9] == 1);
                REQUIRE(array[10] == 1);
                REQUIRE(array[11] == 3);

                packer.unpack(0, 8, &result);
                REQUIRE(result == 255);
                packer.unpack(10, 15, &result);
                REQUIRE(result == 16400);
                packer.unpack(25, 3, &result);
                REQUIRE(result == 7);
                packer.unpack(28, 20, &result);
                REQUIRE(result == 128);
                packer.unpack(48, 23, &result);
                REQUIRE(result == 4210752);
                packer.unpack(71, 25, &result);
                REQUIRE(result == 16843011);
            }
        }
    }
}

SCENARIO("clearing the packet") {

    GIVEN("an array of bytes and a BitPacker object") {
        uint8_t array[BitPacker::DEFAULT_PACKET_SIZE] = {0};
        BitPacker packer(array);
        packer.pack(8, 8, (uint8_t)255);
        WHEN("writing a value in the array and then clearing the entire array") {
            packer.resetAll();
            bool allZeros = true;
            for(unsigned int i = 0; i < sizeof(array) && allZeros; i++) {
                if (array[i] != 0)
                    allZeros = false;
            }
            THEN("array elements are all set to 0") {
                REQUIRE(allZeros == true);
            }
        }
        WHEN("writing a value in the array and then clearing one element") {
            packer.resetByte(1);
            THEN("that element is set to 0") {
                REQUIRE(array[1] == 0);
            }
        }
        WHEN("writing a value in the array and then clearing one element") {
            packer.pack(8, 8, (uint8_t)255);      // set array[1] = 255
            packer.pack(16, 16, (uint16_t)65535);  // set array[2] = array[3] = 255
            packer.resetByte(2);         // only array[2] is cleared
            THEN("that element is set to 0") {
                REQUIRE(array[0] == 0);
                REQUIRE(array[1] == 255);
                REQUIRE(array[2] == 0);
                REQUIRE(array[3] == 255);
            }
        }
        WHEN("resetting specific bits of the array") {
            THEN("only the specified bits are reset to zero") {

            }
        }
    }
}

SCENARIO("writing multiple times the same bits of the array") {
    GIVEN("an array of bytes and a BitPacker object") {
        uint8_t array[BitPacker::DEFAULT_PACKET_SIZE] = {0};
        BitPacker packer(array);
        uint64_t result = 0;
    
        WHEN("reading the same bits") {
            packer.pack(2, 4, (uint8_t)4);
            packer.pack(2, 4, (uint8_t)8);
            packer.unpack(2, 4, &result);
            THEN("the last written value is read when accessing the array") {
                REQUIRE(result == 8);
            }
        }
        WHEN("reading the same bits") {
            packer.pack(0, 25, (uint32_t)65535);
            packer.pack(0, 25, (uint32_t)12345678);
            packer.unpack(0, 25, &result);
            THEN("the last written value is read when accessing the array") {
                REQUIRE(result == 12345678);
            }
        }
        WHEN("reading the same bits") {
            packer.pack(100, 60, (uint64_t)0b1000100010001000100010001000100010001000100010001000100010001000);
            packer.pack(100, 60, (uint64_t)0b1000000010000000100000001000000010000000100000001000000010000000);
            // 10000000 10000000 10000000 10000000 10000000 10000000 10000000 10000000
            // shift by 4 left  (keep 64-60 bits)                  << 00001000 00001000 00001000 00001000 00001000 00001000 00001000 00000(000)
            // shift by 4 right (position 100-96 in the 12th byte) >> 00000000 10000000 10000000 10000000 10000000 10000000 10000000 10000000 00000000
            packer.unpack(100, 60, &result);
            THEN("the last written value is read when accessing the array") {
                //REQUIRE(array[10] == 0);
                REQUIRE(array[11] == 0);
                REQUIRE(array[12] == 0);
                REQUIRE(array[13] == 128);
                REQUIRE(array[14] == 128);
                REQUIRE(array[15] == 128);
                REQUIRE(array[16] == 128);
                REQUIRE(array[17] == 128);
                REQUIRE(array[18] == 128);
                REQUIRE(array[19] == 128);
                REQUIRE(array[20] == 0);
                REQUIRE(result == (uint64_t)0b000010000000100000001000000010000000100000001000000010000000); // only 60 of the above 64 bits number
            }
        }
    }
}

SCENARIO("writing a complete packet multiple times") {

    GIVEN("an array of bytes and a BitPacker object") {
        uint8_t array[12] = {0};
        BitPacker packer(array, 12);
        uint64_t result;
        
        uint8_t v1 = 200;
        uint32_t v2 = 16400;
        uint8_t v3 = 7;
        uint32_t v4 = 128;
        uint32_t v5 = 4210752;
        uint32_t v6 = 16843011;

        WHEN("reading the values in the array") {
            THEN("the last written values are retrieved") {
                for (uint8_t i = 0; i <= 10; i++) {
                    packer.pack(0, 8, (uint8_t)(v1+i));         // 11111111
                                                    // NOTICE THE 2 MISSING BITS!
                    packer.pack(10, 15, (uint32_t)(v2+i));     // 10000000 0010000
                    packer.pack(25, 3, (uint8_t)(v3+i));          // 111
                    packer.pack(28, 20, (uint32_t)(v4+i));       // 00000000 00001000 0000
                    packer.pack(48, 23, (uint32_t)(v5+i));   // 10000000 10000000 1000000
                    packer.pack(71, 25, (uint32_t)(v6+i));  // 10000000 10000000 10000001 1
                    // ---> 11111111 00100000 00001000 01110000 00000000 10000000
                    //      10000000 10000000 10000001 00000001 00000001 00000011
                }

                packer.unpack(0, 8, &result);
                REQUIRE(result == v1+10);
                packer.unpack(10, 15, &result);
                REQUIRE(result == v2+10);
                packer.unpack(25, 3, &result);
                REQUIRE(result == 1); // v3 = 7 with only 3 bits => it overflows
                                          // 7  = 00000111
                                          // 10 = 00001010
                                          // +  = 00010001 = 1 (keeping only 3 lsb bits)
                packer.unpack(28, 20, &result);
                REQUIRE(result == v4+10);
                packer.unpack(48, 23, &result);
                REQUIRE(result == v5+10);
                packer.unpack(71, 25, &result);
                REQUIRE(result == v6+10);
            }
        }
    }
}