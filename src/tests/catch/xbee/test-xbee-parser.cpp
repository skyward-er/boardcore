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
#ifndef NDEBUG
#define NDEBUG
#endif  // NDEBUG

#include <cstdio>

#ifdef STANDALONE_CATCH1_TEST
#include "../catch-tests-entry.cpp"
#endif

#include <radio/Xbee/APIFrameParser.h>

#include <catch2/catch.hpp>
#include <cstdio>
#include <cstring>

using namespace Boardcore;
using namespace Xbee;

/**
 * @brief Calculates the checksum for a sequence of bytes representing an API
 * frame.
 *
 * @param frame bytes (including start delimiter, and checksum set to 0)
 * @param frameSize size of the frame, including start delimiter and checksum
 */
void calcChecksum(uint8_t* frame, size_t frameSize)
{
    frame[frameSize - 1] = 0;
    for (size_t i = 3; i < frameSize - 1; i++)
    {
        frame[frameSize - 1] += frame[i];
    }
    frame[frameSize - 1] = 0xFF - frame[frameSize - 1];
}

void printHex(uint8_t* frame, size_t frameSize)
{
    for (size_t i = 0; i < frameSize; i++)
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
    bool result = a.startDel == b.startDel && a.frameType == b.frameType &&
                  a.length == b.length && a.checksum == b.checksum;

    return result &&
           memcmp(a.frameData, b.frameData, a.getFrameDataLength()) == 0;
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

    APIFrame parsedApi = parse(parser, bytes, len);

    REQUIRE(compareAPIFrames(orig, parsedApi));
}

// Taken from examples in the datasheet
TEST_CASE("Frame serialization")
{
    uint8_t bytes[MAX_API_FRAME_SIZE];
    APIFrame deserialized;

    SECTION("AT Command")
    {
        ATCommandFrame atOrig;
        atOrig.setATCommand("NH");
        REQUIRE(memcmp(atOrig.getATCommand(), "NH", 2) == 0);

        atOrig.setFrameID(0x01);
        REQUIRE(atOrig.getFrameID() == 0x01);

        atOrig.setParameterSize(2);
        atOrig.getCommandDataPointer()[0] = 0x07;
        atOrig.getCommandDataPointer()[1] = 0x17;

        REQUIRE(atOrig.getCommandDataLength() == 2);
        REQUIRE(atOrig.getFrameDataLength() == 5);

        atOrig.calcChecksum();

        uint8_t expectedBytes[] = {0x7E, 0,    6,    0x08, 1,
                                   0x4E, 0x48, 0x07, 0x17, 0x00};

        calcChecksum(expectedBytes, 10);

        size_t len = atOrig.toBytes(bytes);

        REQUIRE(len == 10);

        REQUIRE(memcmp(bytes, expectedBytes, len) == 0);

        REQUIRE(APIFrame::fromBytes(bytes, len, &deserialized));
        REQUIRE(compareAPIFrames(deserialized, atOrig));
    }

    SECTION("AT Command, No parameters")
    {
        ATCommandFrame atOrig;

        atOrig.setATCommand("NH");

        atOrig.setFrameID(0x01);
        REQUIRE(atOrig.getFrameID() == 0x01);

        REQUIRE(atOrig.getCommandDataLength() == 0);
        REQUIRE(atOrig.getFrameDataLength() == 3);

        atOrig.calcChecksum();

        uint8_t expectedBytes[] = {0x7E, 0, 4, 0x08, 1, 0x4E, 0x48, 0x60};

        size_t len = atOrig.toBytes(bytes);

        REQUIRE(len == 8);

        REQUIRE(memcmp(bytes, expectedBytes, len) == 0);

        REQUIRE(APIFrame::fromBytes(bytes, len, &deserialized));
        REQUIRE(compareAPIFrames(deserialized, atOrig));
    }

    SECTION("AT Command Response")
    {
        ATCommandResponseFrame atOrig;
        atOrig.setATCommand("BD");
        REQUIRE(memcmp(atOrig.getATCommand(), "BD", 2) == 0);

        atOrig.setFrameID(0x01);
        REQUIRE(atOrig.getFrameID() == 0x01);

        atOrig.setCommandDataSize(3);
        atOrig.getCommandDataPointer()[0] = 0x07;
        atOrig.getCommandDataPointer()[1] = 0x17;
        atOrig.getCommandDataPointer()[2] = 0x27;

        REQUIRE(atOrig.getCommandDataLength() == 3);
        REQUIRE(atOrig.getFrameDataLength() == 7);

        atOrig.setCommandStatus(0x55);
        REQUIRE(atOrig.getCommandStatus() == 0x55);

        atOrig.calcChecksum();

        uint8_t expectedBytes[] = {0x7E, 0,    8,    0x88, 0x01, 0x42,
                                   0x44, 0x55, 0x07, 0x17, 0x27, 0};

        calcChecksum(expectedBytes, 12);

        size_t len = atOrig.toBytes(bytes);
        REQUIRE(len == 12);

        REQUIRE(memcmp(bytes, expectedBytes, len) == 0);

        REQUIRE(APIFrame::fromBytes(bytes, len, &deserialized));
        REQUIRE(compareAPIFrames(deserialized, atOrig));
    }

    SECTION("AT Command Response, No cmd data")
    {
        ATCommandResponseFrame atOrig;
        atOrig.setATCommand("BD");

        atOrig.setFrameID(0x01);
        REQUIRE(atOrig.getFrameID() == 0x01);

        REQUIRE(atOrig.getCommandDataLength() == 0);
        REQUIRE(atOrig.getFrameDataLength() == 4);

        atOrig.setCommandStatus(0x0);
        REQUIRE(atOrig.getCommandStatus() == 0x00);

        atOrig.calcChecksum();

        uint8_t expectedBytes[] = {0x7E, 0,    5,    0x88, 0x01,
                                   0x42, 0x44, 0x00, 0xF0};

        calcChecksum(expectedBytes, 9);

        size_t len = atOrig.toBytes(bytes);
        REQUIRE(len == 9);

        REQUIRE(memcmp(bytes, expectedBytes, len) == 0);
        REQUIRE(APIFrame::fromBytes(bytes, len, &deserialized));
        REQUIRE(compareAPIFrames(deserialized, atOrig));
    }

    SECTION("TX Request Frame")
    {
        TXRequestFrame txOrig;

        txOrig.setFrameID(0x01);
        REQUIRE(txOrig.getFrameID() == 0x01);

        txOrig.setDestAddress(0x0013A200400A0127);

        bool addrCmp = txOrig.getDestAddress() == 0x0013A200400A0127;
        REQUIRE(addrCmp);

        txOrig.setBroadcastRadius(0x55);
        REQUIRE(txOrig.getBroadcastRadius() == 0x55);

        txOrig.setTransmitOptions(0x40);
        REQUIRE(txOrig.getTrasmitOptions() == 0x40);

        uint8_t* rfData = txOrig.getRFDataPointer();

        rfData[0] = 0x54;
        rfData[1] = 0x78;
        rfData[2] = 0x44;
        rfData[3] = 0x61;
        rfData[4] = 0x74;
        rfData[5] = 0x61;
        rfData[6] = 0x30;
        rfData[7] = 0x41;

        txOrig.setRFDataLength(8);

        REQUIRE(txOrig.getRFDataLength() == 8);
        REQUIRE(txOrig.getFrameDataLength() == 21);

        uint8_t expectedBytes[] = {0x7E, 0,    0x16, 0x10, 0x01, 0x00, 0x13,
                                   0xA2, 0x00, 0x40, 0x0A, 0x01, 0x27, 0xFF,
                                   0xFE, 0x55, 0x40, 0x54, 0x78, 0x44, 0x61,
                                   0x74, 0x61, 0x30, 0x41, 0x00};

        // Datasheet example checksum is wrong
        calcChecksum(expectedBytes, 26);
        txOrig.calcChecksum();
        size_t len = txOrig.toBytes(bytes);

        REQUIRE(len == 26);

        REQUIRE(memcmp(bytes, expectedBytes, len) == 0);
        REQUIRE(APIFrame::fromBytes(bytes, len, &deserialized));
        REQUIRE(compareAPIFrames(deserialized, txOrig));
    }

    SECTION("TX Request Frame - No payload")
    {
        TXRequestFrame txOrig;

        txOrig.setFrameID(0x01);
        REQUIRE(txOrig.getFrameID() == 0x01);

        txOrig.setDestAddress(0x0013A200400A0127);
        bool addrCmp = txOrig.getDestAddress() == 0x0013A200400A0127;
        REQUIRE(addrCmp);

        txOrig.setBroadcastRadius(0x55);
        REQUIRE(txOrig.getBroadcastRadius() == 0x55);

        txOrig.setTransmitOptions(0x40);
        REQUIRE(txOrig.getTrasmitOptions() == 0x40);

        uint8_t* rfData = txOrig.getRFDataPointer();
        UNUSED(rfData);  // TODO: Check rfData

        REQUIRE(txOrig.getRFDataLength() == 0);
        REQUIRE(txOrig.getFrameDataLength() == 13);

        txOrig.calcChecksum();

        uint8_t expectedBytes[] = {0x7E, 0,    14,   0x10, 0x01, 0x00,
                                   0x13, 0xA2, 0x00, 0x40, 0x0A, 0x01,
                                   0x27, 0xFF, 0xFE, 0x55, 0x40, 0x00};

        calcChecksum(expectedBytes, 18);

        size_t len = txOrig.toBytes(bytes);

        REQUIRE(len == 18);

        REQUIRE(memcmp(bytes, expectedBytes, len) == 0);
        REQUIRE(APIFrame::fromBytes(bytes, len, &deserialized));
        REQUIRE(compareAPIFrames(deserialized, txOrig));
    }

    SECTION("Modem status frame")
    {
        ModemStatusFrame modOrig;

        modOrig.setStatus(0x55);

        modOrig.calcChecksum();

        uint8_t expectedBytes[] = {0x7E, 0, 2, 0x8A, 0x55, 0x00};

        calcChecksum(expectedBytes, 6);

        size_t len = modOrig.toBytes(bytes);

        REQUIRE(memcmp(bytes, expectedBytes, len) == 0);
        REQUIRE(APIFrame::fromBytes(bytes, len, &deserialized));
        REQUIRE(compareAPIFrames(deserialized, modOrig));
    }

    SECTION("TX Status frame")
    {
        // constexpr size_t frameSize = 11;
        TXStatusFrame txOrig{};

        txOrig.setFrameID(0x47);
        REQUIRE(txOrig.getFrameID() == 0x47);
        txOrig.setTransmitRetryCount(0x55);
        REQUIRE(txOrig.getTransmitRetryCount() == 0x55);

        txOrig.setDeliveryStatus(0x44);
        REQUIRE(txOrig.getDeliveryStatus() == 0x44);

        txOrig.setDiscoveryStatus(2);
        REQUIRE(txOrig.getDiscoveryStatus() == 2);

        txOrig.calcChecksum();

        uint8_t expectedBytes[] = {0x7E, 0,    0x07, 0x8B, 0x47, 0xFF,
                                   0xFE, 0x55, 0x44, 0x02, 0x00};

        calcChecksum(expectedBytes, 11);

        size_t len = txOrig.toBytes(bytes);
        REQUIRE(len == 11);

        REQUIRE(memcmp(bytes, expectedBytes, len) == 0);
        REQUIRE(APIFrame::fromBytes(bytes, len, &deserialized));
        REQUIRE(compareAPIFrames(deserialized, txOrig));
    }

    SECTION("RX Packet frame")
    {
        RXPacketFrame rx;

        rx.setSourceAddress(0x0013A20040522BAA);
        bool addrCmp = rx.getSourceAddress() == 0x0013A20040522BAA;
        REQUIRE(addrCmp);

        rx.setReceiveOptions(0x01);
        REQUIRE(rx.getReceiveOptions() == 0x01);

        uint8_t* rfData = rx.getRXDataPointer();

        rfData[0] = 0x52;
        rfData[1] = 0x78;
        rfData[2] = 0x44;
        rfData[3] = 0x61;
        rfData[4] = 0x74;
        rfData[5] = 0x61;

        rx.setRXDataLength(6);
        REQUIRE(rx.getRXDataLength() == 6);
        REQUIRE(rx.getFrameDataLength() == 0x11);

        rx.calcChecksum();

        uint8_t expectedBytes[] = {
            0x7E, 0,    0x12, 0x90, 0x00, 0x13, 0xA2, 0x00, 0x40, 0x52, 0x2B,
            0xAA, 0xFF, 0xFE, 0x01, 0x52, 0x78, 0x44, 0x61, 0x74, 0x61, 0x11};

        // calcChecksum(expectedBytes, frameSize);

        size_t len = rx.toBytes(bytes);

        REQUIRE(len == 22);

        REQUIRE(memcmp(bytes, expectedBytes, len) == 0);
        REQUIRE(APIFrame::fromBytes(bytes, len, &deserialized));
        REQUIRE(compareAPIFrames(deserialized, rx));
    }

    SECTION("RX Packet frame - no payload")
    {
        RXPacketFrame rx;

        rx.setSourceAddress(0x0013A20040522BAA);
        bool addrCmp = rx.getSourceAddress() == 0x0013A20040522BAA;
        REQUIRE(addrCmp);

        rx.setReceiveOptions(0x01);
        REQUIRE(rx.getReceiveOptions() == 0x01);

        REQUIRE(rx.getRXDataLength() == 0);
        REQUIRE(rx.getFrameDataLength() == 11);

        rx.calcChecksum();

        uint8_t expectedBytes[] = {0x7E, 0,    12,   0x90, 0x00, 0x13,
                                   0xA2, 0x00, 0x40, 0x52, 0x2B, 0xAA,
                                   0xFF, 0xFE, 0x01, 0x00};

        calcChecksum(expectedBytes, 16);

        size_t len = rx.toBytes(bytes);

        REQUIRE(len == 16);

        REQUIRE(memcmp(bytes, expectedBytes, len) == 0);

        memset(deserialized.frameData, 0, FRAME_DATA_SIZE);
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
        uint8_t* rfData = rx.getRXDataPointer();

        rfData[0] = 0x52;
        rfData[1] = 0x78;
        rfData[2] = 0x44;
        rfData[3] = 0x61;
        rfData[4] = 0x74;
        rfData[5] = 0x61;

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
        uint8_t* rfData = rx.getRXDataPointer();

        rfData[0] = 0x52;
        rfData[1] = 0x78;
        rfData[2] = 0x44;
        rfData[3] = 0x61;
        rfData[4] = 0x74;
        rfData[5] = 0x61;

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
        rx.startDel = 0x55;
        rx.setSourceAddress(0x0013A20040522BAA);
        rx.setReceiveOptions(0x01);
        uint8_t* rfData = rx.getRXDataPointer();

        rfData[0] = 0x52;
        rfData[1] = 0x78;
        rfData[2] = 0x44;
        rfData[3] = 0x61;
        rfData[4] = 0x74;
        rfData[5] = 0x61;

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
