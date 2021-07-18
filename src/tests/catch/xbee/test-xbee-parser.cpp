/* Copyright (c) 2021 Skyward Experimental Rocketry
 * Author: Luca Erbetta
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

// Disable asserts
#define NDEBUG

#include <cstdio>

#ifdef STANDALONE_CATCH1_TEST
#include "../catch-tests-entry.cpp"
#endif

#include <cstdio>
#include <cstring>

#include "drivers/Xbee/APIFrameParser.h"
#include "utils/testutils/catch.hpp"

using namespace Xbee;

/**
 * @brief Calculates the checksum for a sequence of bytes representing an API
 * frame.
 *
 * @param frame bytes (including start delimiter, and checksum set to 0)
 * @param frame_size size of the frame, including start delimiter and checksum
 */
void calcChecksum(uint8_t* frame, size_t frame_size)
{
    frame[frame_size - 1] = 0;
    for (size_t i = 3; i < frame_size - 1; i++)
    {
        frame[frame_size - 1] += frame[i];
    }
    frame[frame_size - 1] = 0xFF - frame[frame_size - 1];
}

void printHex(uint8_t* frame, size_t frame_size)
{
    for (size_t i = 0; i < frame_size; i++)
    {
        printf("%02X ", frame[i]);
    }
    printf("\n");
}

void printu64(uint64_t v)
{
    uint8_t* p = reinterpret_cast<uint8_t*>(&v);

    for (int i = 0; i < 8; i++)
    {
        printf("%02X ", p[i]);
    }

    printf("\n");
}

void printBuf(uint8_t* buf, size_t size)
{
    for (size_t i = 0; i < size; i++)
    {
        printf("%02X ", buf[i]);
    }

    printf("\n");
}

bool compareAPIFrames(const APIFrame& a, const APIFrame& b)
{
    bool result = a.start_del == b.start_del && a.frame_type == b.frame_type &&
                  a.length == b.length && a.checksum == b.checksum;

    return result &&
           memcmp(a.frame_data, b.frame_data, a.getFrameDataLength()) == 0;
}

APIFrame parse(APIFrameParser& parser, uint8_t* data, size_t len)
{
    APIFrame out;
    for (size_t i = 0; i < len; i++)
    {
        APIFrameParser::ParseResult pres = parser.parse(data[i], &out);

        if (i < len - 1)
        {
            CAPTURE(i);
            REQUIRE(pres == APIFrameParser::ParseResult::PARSING);
        }
        else
        {
            REQUIRE(pres == APIFrameParser::ParseResult::SUCCESS);
        }
    }

    REQUIRE(out.verifyChecksum());

    return out;
}

template <typename FrameType>
void testParse(APIFrameParser& parser, FrameType orig)
{
    uint8_t bytes[MAX_API_FRAME_SIZE];

    size_t len = orig.toBytes(bytes);

    APIFrame parsed_api = parse(parser, bytes, len);

    REQUIRE(compareAPIFrames(orig, parsed_api));
}

// Taken from examples in the datasheet
TEST_CASE("Frame serialization")
{
    uint8_t bytes[MAX_API_FRAME_SIZE];
    APIFrame deserialized;

    SECTION("AT Command")
    {
        ATCommandFrame at_orig;
        at_orig.setATCommand("NH");
        REQUIRE(memcmp(at_orig.getATCommand(), "NH", 2) == 0);

        at_orig.setFrameID(0x01);
        REQUIRE(at_orig.getFrameID() == 0x01);

        at_orig.setParameterSize(2);
        at_orig.getCommandDataPointer()[0] = 0x07;
        at_orig.getCommandDataPointer()[1] = 0x17;

        REQUIRE(at_orig.getCommandDataLength() == 2);
        REQUIRE(at_orig.getFrameDataLength() == 5);

        at_orig.calcChecksum();

        uint8_t expected_bytes[] = {0x7E, 0,    6,    0x08, 1,
                                    0x4E, 0x48, 0x07, 0x17, 0x00};

        calcChecksum(expected_bytes, 10);

        size_t len = at_orig.toBytes(bytes);

        REQUIRE(len == 10);

        REQUIRE(memcmp(bytes, expected_bytes, len) == 0);

        REQUIRE(APIFrame::fromBytes(bytes, len, &deserialized));
        REQUIRE(compareAPIFrames(deserialized, at_orig));
    }

    SECTION("AT Command, No parameters")
    {
        ATCommandFrame at_orig;

        at_orig.setATCommand("NH");

        at_orig.setFrameID(0x01);
        REQUIRE(at_orig.getFrameID() == 0x01);

        REQUIRE(at_orig.getCommandDataLength() == 0);
        REQUIRE(at_orig.getFrameDataLength() == 3);

        at_orig.calcChecksum();

        uint8_t expected_bytes[] = {0x7E, 0, 4, 0x08, 1, 0x4E, 0x48, 0x60};

        size_t len = at_orig.toBytes(bytes);

        REQUIRE(len == 8);

        REQUIRE(memcmp(bytes, expected_bytes, len) == 0);

        REQUIRE(APIFrame::fromBytes(bytes, len, &deserialized));
        REQUIRE(compareAPIFrames(deserialized, at_orig));
    }

    SECTION("AT Command Response")
    {
        ATCommandResponseFrame at_orig;
        at_orig.setATCommand("BD");
        REQUIRE(memcmp(at_orig.getATCommand(), "BD", 2) == 0);

        at_orig.setFrameID(0x01);
        REQUIRE(at_orig.getFrameID() == 0x01);

        at_orig.setCommandDataSize(3);
        at_orig.getCommandDataPointer()[0] = 0x07;
        at_orig.getCommandDataPointer()[1] = 0x17;
        at_orig.getCommandDataPointer()[2] = 0x27;

        REQUIRE(at_orig.getCommandDataLength() == 3);
        REQUIRE(at_orig.getFrameDataLength() == 7);

        at_orig.setCommandStatus(0x55);
        REQUIRE(at_orig.getCommandStatus() == 0x55);

        at_orig.calcChecksum();

        uint8_t expected_bytes[] = {0x7E, 0,    8,    0x88, 0x01, 0x42,
                                    0x44, 0x55, 0x07, 0x17, 0x27, 0};

        calcChecksum(expected_bytes, 12);

        size_t len = at_orig.toBytes(bytes);
        REQUIRE(len == 12);

        REQUIRE(memcmp(bytes, expected_bytes, len) == 0);

        REQUIRE(APIFrame::fromBytes(bytes, len, &deserialized));
        REQUIRE(compareAPIFrames(deserialized, at_orig));
    }

    SECTION("AT Command Response, No cmd data")
    {
        ATCommandResponseFrame at_orig;
        at_orig.setATCommand("BD");

        at_orig.setFrameID(0x01);
        REQUIRE(at_orig.getFrameID() == 0x01);

        REQUIRE(at_orig.getCommandDataLength() == 0);
        REQUIRE(at_orig.getFrameDataLength() == 4);

        at_orig.setCommandStatus(0x0);
        REQUIRE(at_orig.getCommandStatus() == 0x00);

        at_orig.calcChecksum();

        uint8_t expected_bytes[] = {0x7E, 0,    5,    0x88, 0x01,
                                    0x42, 0x44, 0x00, 0xF0};

        calcChecksum(expected_bytes, 9);

        size_t len = at_orig.toBytes(bytes);
        REQUIRE(len == 9);

        REQUIRE(memcmp(bytes, expected_bytes, len) == 0);
        REQUIRE(APIFrame::fromBytes(bytes, len, &deserialized));
        REQUIRE(compareAPIFrames(deserialized, at_orig));
    }

    SECTION("TX Request Frame")
    {
        TXRequestFrame tx_orig;

        tx_orig.setFrameID(0x01);
        REQUIRE(tx_orig.getFrameID() == 0x01);

        tx_orig.setDestAddress(0x0013A200400A0127);

        bool addr_cmp = tx_orig.getDestAddress() == 0x0013A200400A0127;
        REQUIRE(addr_cmp);

        tx_orig.setBroadcastRadius(0x55);
        REQUIRE(tx_orig.getBroadcastRadius() == 0x55);

        tx_orig.setTransmitOptions(0x40);
        REQUIRE(tx_orig.getTrasmitOptions() == 0x40);

        uint8_t* rf_data = tx_orig.getRFDataPointer();

        rf_data[0] = 0x54;
        rf_data[1] = 0x78;
        rf_data[2] = 0x44;
        rf_data[3] = 0x61;
        rf_data[4] = 0x74;
        rf_data[5] = 0x61;
        rf_data[6] = 0x30;
        rf_data[7] = 0x41;

        tx_orig.setRFDataLength(8);

        REQUIRE(tx_orig.getRFDataLength() == 8);
        REQUIRE(tx_orig.getFrameDataLength() == 21);

        uint8_t expected_bytes[] = {0x7E, 0,    0x16, 0x10, 0x01, 0x00, 0x13,
                                    0xA2, 0x00, 0x40, 0x0A, 0x01, 0x27, 0xFF,
                                    0xFE, 0x55, 0x40, 0x54, 0x78, 0x44, 0x61,
                                    0x74, 0x61, 0x30, 0x41, 0x00};

        // Datasheet example checksum is wrong
        calcChecksum(expected_bytes, 26);
        tx_orig.calcChecksum();
        size_t len = tx_orig.toBytes(bytes);

        REQUIRE(len == 26);

        REQUIRE(memcmp(bytes, expected_bytes, len) == 0);
        REQUIRE(APIFrame::fromBytes(bytes, len, &deserialized));
        REQUIRE(compareAPIFrames(deserialized, tx_orig));
    }

    SECTION("TX Request Frame - No payload")
    {
        TXRequestFrame tx_orig;

        tx_orig.setFrameID(0x01);
        REQUIRE(tx_orig.getFrameID() == 0x01);

        tx_orig.setDestAddress(0x0013A200400A0127);
        bool addr_cmp = tx_orig.getDestAddress() == 0x0013A200400A0127;
        REQUIRE(addr_cmp);

        tx_orig.setBroadcastRadius(0x55);
        REQUIRE(tx_orig.getBroadcastRadius() == 0x55);

        tx_orig.setTransmitOptions(0x40);
        REQUIRE(tx_orig.getTrasmitOptions() == 0x40);

        uint8_t* rf_data = tx_orig.getRFDataPointer();

        REQUIRE(tx_orig.getRFDataLength() == 0);
        REQUIRE(tx_orig.getFrameDataLength() == 13);

        tx_orig.calcChecksum();

        uint8_t expected_bytes[] = {0x7E, 0,    14,   0x10, 0x01, 0x00,
                                    0x13, 0xA2, 0x00, 0x40, 0x0A, 0x01,
                                    0x27, 0xFF, 0xFE, 0x55, 0x40, 0x00};

        calcChecksum(expected_bytes, 18);

        size_t len = tx_orig.toBytes(bytes);

        REQUIRE(len == 18);

        REQUIRE(memcmp(bytes, expected_bytes, len) == 0);
        REQUIRE(APIFrame::fromBytes(bytes, len, &deserialized));
        REQUIRE(compareAPIFrames(deserialized, tx_orig));
    }

    SECTION("Modem status frame")
    {
        ModemStatusFrame mod_orig;

        mod_orig.setStatus(0x55);

        mod_orig.calcChecksum();

        uint8_t expected_bytes[] = {0x7E, 0, 2, 0x8A, 0x55, 0x00};

        calcChecksum(expected_bytes, 6);

        size_t len = mod_orig.toBytes(bytes);

        REQUIRE(memcmp(bytes, expected_bytes, len) == 0);
        REQUIRE(APIFrame::fromBytes(bytes, len, &deserialized));
        REQUIRE(compareAPIFrames(deserialized, mod_orig));
    }

    SECTION("TX Status frame")
    {
        constexpr size_t frame_size = 11;
        TXStatusFrame tx_orig{};

        tx_orig.setFrameID(0x47);
        REQUIRE(tx_orig.getFrameID() == 0x47);
        tx_orig.setTransmitRetryCount(0x55);
        REQUIRE(tx_orig.getTransmitRetryCount() == 0x55);

        tx_orig.setDeliveryStatus(0x44);
        REQUIRE(tx_orig.getDeliveryStatus() == 0x44);

        tx_orig.setDiscoveryStatus(2);
        REQUIRE(tx_orig.getDiscoveryStatus() == 2);

        tx_orig.calcChecksum();

        uint8_t expected_bytes[] = {0x7E, 0,    0x07, 0x8B, 0x47, 0xFF,
                                    0xFE, 0x55, 0x44, 0x02, 0x00};

        calcChecksum(expected_bytes, 11);

        size_t len = tx_orig.toBytes(bytes);
        REQUIRE(len == 11);

        REQUIRE(memcmp(bytes, expected_bytes, len) == 0);
        REQUIRE(APIFrame::fromBytes(bytes, len, &deserialized));
        REQUIRE(compareAPIFrames(deserialized, tx_orig));
    }

    SECTION("RX Packet frame")
    {
        RXPacketFrame rx;

        rx.setSourceAddress(0x0013A20040522BAA);
        bool addr_cmp = rx.getSourceAddress() == 0x0013A20040522BAA;
        REQUIRE(addr_cmp);

        rx.setReceiveOptions(0x01);
        REQUIRE(rx.getReceiveOptions() == 0x01);

        uint8_t* rf_data = rx.getRXDataPointer();

        rf_data[0] = 0x52;
        rf_data[1] = 0x78;
        rf_data[2] = 0x44;
        rf_data[3] = 0x61;
        rf_data[4] = 0x74;
        rf_data[5] = 0x61;

        rx.setRXDataLength(6);
        REQUIRE(rx.getRXDataLength() == 6);
        REQUIRE(rx.getFrameDataLength() == 0x11);

        rx.calcChecksum();

        uint8_t expected_bytes[] = {
            0x7E, 0,    0x12, 0x90, 0x00, 0x13, 0xA2, 0x00, 0x40, 0x52, 0x2B,
            0xAA, 0xFF, 0xFE, 0x01, 0x52, 0x78, 0x44, 0x61, 0x74, 0x61, 0x11};

        // calcChecksum(expected_bytes, frame_size);

        size_t len = rx.toBytes(bytes);

        REQUIRE(len == 22);

        REQUIRE(memcmp(bytes, expected_bytes, len) == 0);
        REQUIRE(APIFrame::fromBytes(bytes, len, &deserialized));
        REQUIRE(compareAPIFrames(deserialized, rx));
    }

    SECTION("RX Packet frame - no payload")
    {
        RXPacketFrame rx;

        rx.setSourceAddress(0x0013A20040522BAA);
        bool addr_cmp = rx.getSourceAddress() == 0x0013A20040522BAA;
        REQUIRE(addr_cmp);

        rx.setReceiveOptions(0x01);
        REQUIRE(rx.getReceiveOptions() == 0x01);

        REQUIRE(rx.getRXDataLength() == 0);
        REQUIRE(rx.getFrameDataLength() == 11);

        rx.calcChecksum();

        uint8_t expected_bytes[] = {0x7E, 0,    12,   0x90, 0x00, 0x13,
                                    0xA2, 0x00, 0x40, 0x52, 0x2B, 0xAA,
                                    0xFF, 0xFE, 0x01, 0x00};

        calcChecksum(expected_bytes, 16);

        size_t len = rx.toBytes(bytes);

        REQUIRE(len == 16);

        REQUIRE(memcmp(bytes, expected_bytes, len) == 0);

        memset(deserialized.frame_data, 0, FRAME_DATA_SIZE);
        REQUIRE(APIFrame::fromBytes(bytes, len, &deserialized));
        REQUIRE(compareAPIFrames(deserialized, rx));
    }
}

TEST_CASE("Frame parsing")
{
    APIFrameParser parser;

    SECTION("RXPacketFrame Parsing")
    {
        RXPacketFrame rx;
        rx.setSourceAddress(0x0013A20040522BAA);
        rx.setReceiveOptions(0x01);
        uint8_t* rf_data = rx.getRXDataPointer();

        rf_data[0] = 0x52;
        rf_data[1] = 0x78;
        rf_data[2] = 0x44;
        rf_data[3] = 0x61;
        rf_data[4] = 0x74;
        rf_data[5] = 0x61;

        rx.setRXDataLength(6);
        rx.calcChecksum();

        // Parse 3 times the same frame
        testParse(parser, rx);
        testParse(parser, rx);
        testParse(parser, rx);
    }
}

TEST_CASE("Parser edge cases")
{
    APIFrameParser parser;

    SECTION("Wrong checksum")
    {
        RXPacketFrame rx;
        rx.setSourceAddress(0x0013A20040522BAA);
        rx.setReceiveOptions(0x01);
        uint8_t* rf_data = rx.getRXDataPointer();

        rf_data[0] = 0x52;
        rf_data[1] = 0x78;
        rf_data[2] = 0x44;
        rf_data[3] = 0x61;
        rf_data[4] = 0x74;
        rf_data[5] = 0x61;

        rx.setRXDataLength(6);
        rx.calcChecksum();

        uint8_t bytes[MAX_API_FRAME_SIZE];

        size_t len = rx.toBytes(bytes);

        bytes[len - 1] += 1;

        APIFrame out;
        for (size_t i = 0; i < len; i++)
        {
            APIFrameParser::ParseResult res = parser.parse(bytes[i], &out);
            REQUIRE_FALSE(res == APIFrameParser::ParseResult::SUCCESS);

            if (i == len - 1)
            {
                REQUIRE(res == APIFrameParser::ParseResult::FAIL);
            }
        }
    }

    SECTION("Wrong start delimiter")
    {
        RXPacketFrame rx;
        rx.start_del = 0x55;
        rx.setSourceAddress(0x0013A20040522BAA);
        rx.setReceiveOptions(0x01);
        uint8_t* rf_data = rx.getRXDataPointer();

        rf_data[0] = 0x52;
        rf_data[1] = 0x78;
        rf_data[2] = 0x44;
        rf_data[3] = 0x61;
        rf_data[4] = 0x74;
        rf_data[5] = 0x61;

        rx.setRXDataLength(6);
        rx.calcChecksum();

        uint8_t bytes[MAX_API_FRAME_SIZE];

        size_t len = rx.toBytes(bytes);

        APIFrame out;
        for (size_t i = 0; i < len; i++)
        {
            APIFrameParser::ParseResult res = parser.parse(bytes[i], &out);
            REQUIRE(res == APIFrameParser::ParseResult::IDLE);
        }
    }
}