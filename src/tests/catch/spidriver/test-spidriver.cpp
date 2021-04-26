/**
 * Copyright (c) 2020 Skyward Experimental Rocketry
 * Authors: Luca Erbetta (luca.erbetta@skywarder.eu)
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

#ifdef STANDALONE_CATCH1_TEST
#include "../catch-tests-entry.cpp"
#endif

#ifndef USE_MOCK_PERIPHERALS
#error "This test requires SpiBusInterface built with MockGpioPin (-DUSE_MOCK_PERIPHERALS)"
#endif

#include <utils/testutils/catch.hpp>

#include "drivers/spi/SPIBus.h"
#include "drivers/spi/SPIDriver.h"
#include "drivers/spi/test/FakeSpiTypedef.h"
#include "drivers/spi/test/MockSPIBus.h"

template <typename T1, typename T2>
bool bufcmp(T1* buf1, T2* buf2, size_t size)
{
    for (size_t i = 0; i < size; i++)
    {
        if (*buf1 != *buf2)
            return false;

        buf1++;
        buf2++;
    }
    return true;
}

TEST_CASE("SPIBus - Bus Configuration")
{
    FakeSpiTypedef spi;

    SPIBus bus{&spi};

    REQUIRE(spi.CR1 == 0);

    SECTION("Configure & check CR1")
    {
        SPIBusConfig config;
        REQUIRE(spi.CR1 == 0);

        SECTION("Mode")
        {
            config.mode           = SPIMode::MODE0;
            uint32_t expected_CR1 = 0x0344;
            bus.acquire(config);
            REQUIRE(spi.CR1 == expected_CR1);
            bus.release();

            config.mode  = SPIMode::MODE1;
            expected_CR1 = 0x0345;
            bus.acquire(config);
            REQUIRE(spi.CR1 == expected_CR1);
            bus.release();

            config.mode  = SPIMode::MODE2;
            expected_CR1 = 0x0346;
            bus.acquire(config);
            REQUIRE(spi.CR1 == expected_CR1);
            bus.release();

            config.mode  = SPIMode::MODE3;
            expected_CR1 = 0x0347;
            bus.acquire(config);
            REQUIRE(spi.CR1 == expected_CR1);
            bus.release();
        }

        SECTION("Clock Divider")
        {
            config.clock_div      = SPIClockDivider::DIV2;
            uint32_t expected_CR1 = 0x0344;
            bus.acquire(config);
            REQUIRE(spi.CR1 == expected_CR1);
            bus.release();

            config.clock_div = SPIClockDivider::DIV4;
            expected_CR1     = 0x034C;
            bus.acquire(config);
            REQUIRE(spi.CR1 == expected_CR1);
            bus.release();

            config.clock_div = SPIClockDivider::DIV8;
            expected_CR1     = 0x0354;
            bus.acquire(config);
            REQUIRE(spi.CR1 == expected_CR1);
            bus.release();

            config.clock_div = SPIClockDivider::DIV16;
            expected_CR1     = 0x035C;
            bus.acquire(config);
            REQUIRE(spi.CR1 == expected_CR1);
            bus.release();

            config.clock_div = SPIClockDivider::DIV32;
            expected_CR1     = 0x0364;
            bus.acquire(config);
            REQUIRE(spi.CR1 == expected_CR1);
            bus.release();

            config.clock_div = SPIClockDivider::DIV64;
            expected_CR1     = 0x036C;
            bus.acquire(config);
            REQUIRE(spi.CR1 == expected_CR1);
            bus.release();

            config.clock_div = SPIClockDivider::DIV128;
            expected_CR1     = 0x0374;
            bus.acquire(config);
            REQUIRE(spi.CR1 == expected_CR1);
            bus.release();

            config.clock_div = SPIClockDivider::DIV256;
            expected_CR1     = 0x037C;
            bus.acquire(config);
            REQUIRE(spi.CR1 == expected_CR1);
            bus.release();
        }

        SECTION("Bit order")
        {
            config.bit_order      = SPIBitOrder::MSB_FIRST;
            uint32_t expected_CR1 = 0x0344;
            bus.acquire(config);
            REQUIRE(spi.CR1 == expected_CR1);
            bus.release();

            config.bit_order = SPIBitOrder::LSB_FIRST;
            expected_CR1     = 0x03C4;
            bus.acquire(config);
            REQUIRE(spi.CR1 == expected_CR1);
            bus.release();
        }
    }

    SECTION("Disable configuration")
    {
        SPIBusConfig config;
        config.clock_div = SPIClockDivider::DIV16;

        config.mode      = SPIMode::MODE3;
        config.bit_order = SPIBitOrder::LSB_FIRST;

        bus.disableBusConfiguration();
        bus.acquire(config);
        REQUIRE(spi.CR1 == 0);
        bus.release();
    }
}

TEST_CASE("SPIBus - Chip select")
{
    FakeSpiTypedef spi;

    SPIBus bus{&spi};

    REQUIRE(spi.cs.value() == 1);

    bus.select(spi.cs);
    REQUIRE(spi.cs.value() == 0);

    bus.deselect(spi.cs);
    REQUIRE(spi.cs.value() == 1);
}

TEST_CASE("SPIBus - One byte operations")
{
    FakeSpiTypedef spi;

    spi.DR.in_buf    = {1, 2, 3, 4, 5, 6, 7, 8, 9};
    spi.CR1_expected = 0x03DF;

    SPIBus bus{&spi};

    SPIBusConfig config;
    config.clock_div = SPIClockDivider::DIV16;

    config.mode      = SPIMode::MODE3;
    config.bit_order = SPIBitOrder::LSB_FIRST;

    bus.acquire(config);
    bus.select(spi.cs);

    SECTION("Write")
    {
        bus.write(1);
        REQUIRE(spi.DR.out_buf.back() == 1);
        REQUIRE(spi.DR.out_buf.size() == 1);

        bus.write(2);
        REQUIRE(spi.DR.out_buf.back() == 2);
        REQUIRE(spi.DR.out_buf.size() == 2);
    }
    SECTION("Read")
    {
        REQUIRE(bus.read() == spi.DR.in_buf[0]);
        REQUIRE(spi.DR.out_buf.size() == 1);
        REQUIRE(spi.DR.out_buf.back() == 0);

        REQUIRE(bus.read() == spi.DR.in_buf[1]);
        REQUIRE(spi.DR.out_buf.size() == 2);
        REQUIRE(spi.DR.out_buf.back() == 0);
    }

    SECTION("Transfer")
    {
        REQUIRE(bus.transfer(55) == spi.DR.in_buf[0]);
        REQUIRE(spi.DR.out_buf.back() == 55);
        REQUIRE(spi.DR.out_buf.size() == 1);

        REQUIRE(bus.transfer(255) == spi.DR.in_buf[1]);
        REQUIRE(spi.DR.out_buf.back() == 255);
        REQUIRE(spi.DR.out_buf.size() == 2);
    }

    bus.deselect(spi.cs);
    bus.release();
}

TEST_CASE("SPIBus - Multi byte operations")
{
    FakeSpiTypedef spi;

    spi.DR.in_buf    = {100, 101, 102, 103, 104, 105, 106, 107, 108};
    spi.CR1_expected = 0x03DF;

    SPIBus bus{&spi};

    SPIBusConfig config;
    config.clock_div = SPIClockDivider::DIV16;

    config.mode      = SPIMode::MODE3;
    config.bit_order = SPIBitOrder::LSB_FIRST;

    bus.acquire(config);
    bus.select(spi.cs);

    // 2 identical buffers
    uint8_t buf[]  = {5, 4, 3, 2, 1};
    uint8_t bufc[] = {5, 4, 3, 2, 1};

    SECTION("Write")
    {
        bus.write(buf, 0);
        REQUIRE(spi.DR.out_buf.size() == 0);

        bus.write(buf, 1);
        REQUIRE(spi.DR.out_buf.size() == 1);
        REQUIRE(spi.DR.out_buf.back() == bufc[0]);

        bus.write(buf, 4);
        REQUIRE(spi.DR.out_buf.size() == 5);
        REQUIRE(bufcmp(bufc, spi.DR.out_buf.data() + 1, 4));
    }

    SECTION("Read")
    {
        bus.read(buf, 0);
        // Nothing read
        REQUIRE(bufcmp(bufc, buf, 5));

        bus.read(buf, 1);
        REQUIRE(bufcmp(buf, spi.DR.in_buf.data(), 1));
        // No overflows
        REQUIRE(bufcmp(bufc + 1, buf + 1, 4));

        bus.read(buf, 4);
        REQUIRE(bufcmp(buf, spi.DR.in_buf.data() + 1, 4));
        // No overflows
        REQUIRE(bufcmp(bufc + 4, buf + 4, 1));
    }

    SECTION("Transfer")
    {
        bus.transfer(buf, 0);
        // Nothing read
        REQUIRE(bufcmp(bufc, buf, 4));
        // Nothing written
        REQUIRE(spi.DR.out_buf.size() == 0);

        bus.transfer(buf, 1);
        REQUIRE(spi.DR.out_buf.size() == 1);
        REQUIRE(bufcmp(bufc, spi.DR.out_buf.data(), 1));
        REQUIRE(bufcmp(buf, spi.DR.in_buf.data(), 1));
        // No overflows
        REQUIRE(bufcmp(bufc + 1, buf + 1, 4));

        bus.transfer(buf + 1, 3);
        REQUIRE(spi.DR.out_buf.size() == 4);
        REQUIRE(bufcmp(bufc + 1, spi.DR.out_buf.data() + 1, 3));
        REQUIRE(bufcmp(buf + 1, spi.DR.in_buf.data() + 1, 3));
        // No overflows
        REQUIRE(bufcmp(bufc + 4, buf + 4, 1));
    }

    bus.deselect(spi.cs);
    bus.release();
}

TEST_CASE("SPITransaction - writes")
{
    SPIBusConfig config1{};

    config1.mode      = SPIMode::MODE1;
    config1.clock_div = SPIClockDivider::DIV32;

    MockSPIBus bus(config1);
    MockGpioPin cs;

    SECTION("Transaction")
    {
        SPITransaction spi(bus, cs, config1);

        REQUIRE(bus.getOutBuf().size() == 0);

        SECTION("cmd write")
        {
            spi.write(9);
            REQUIRE_FALSE(bus.isSelected());
            REQUIRE(bus.getOutBuf().size() == 1);
            REQUIRE(bus.getOutBuf().back() == 9);
        }

        SECTION("1 byte reg write")
        {
            spi.write(10, 77);
            REQUIRE_FALSE(bus.isSelected());

            REQUIRE(bus.getOutBuf().size() == 2);
            REQUIRE(bus.getOutBuf()[0] == 10);
            REQUIRE(bus.getOutBuf()[1] == 77);
        }

        SECTION("multi byte reg write")
        {
            uint8_t buf[] = {1, 2, 3, 4, 5, 6};

            SECTION("0 size write")
            {
                spi.write(10, buf, 0);
                REQUIRE_FALSE(bus.isSelected());

                REQUIRE(bus.getOutBuf().size() == 1);
                REQUIRE(bus.getOutBuf()[0] == 10);
            }

            SECTION("2 writes")
            {
                spi.write(10, buf, 4);
                REQUIRE_FALSE(bus.isSelected());

                REQUIRE(bus.getOutBuf().size() == 5);

                REQUIRE(bus.getOutBuf()[0] == 10);
                REQUIRE(bufcmp(buf, bus.getOutBuf().data() + 1, 4));

                spi.write(99, buf, 6);
                REQUIRE_FALSE(bus.isSelected());

                REQUIRE(bus.getOutBuf().size() == 12);

                REQUIRE(bus.getOutBuf()[5] == 99);
                REQUIRE(bufcmp(buf, bus.getOutBuf().data() + 6, 6));
            }
        }

        SECTION("raw write")
        {
            uint8_t buf[] = {1, 2, 3, 4, 5, 6};

            spi.write(buf, 0);
            REQUIRE_FALSE(bus.isSelected());

            REQUIRE(bus.getOutBuf().size() == 0);

            spi.write(buf, 4);
            REQUIRE_FALSE(bus.isSelected());

            REQUIRE(bus.getOutBuf().size() == 4);

            REQUIRE(bufcmp(buf, bus.getOutBuf().data(), 4));

            spi.write(buf, 6);
            REQUIRE_FALSE(bus.isSelected());

            REQUIRE(bus.getOutBuf().size() == 10);

            REQUIRE(bufcmp(buf, bus.getOutBuf().data() + 4, 6));
        }
    }
}

TEST_CASE("SPITransaction - reads")
{
    SPIBusConfig config1;

    config1.mode      = SPIMode::MODE1;
    config1.clock_div = SPIClockDivider::DIV32;

    MockSPIBus bus(config1);
    MockGpioPin cs;

    uint8_t in_data[10] = {100, 101, 102, 103, 104, 105, 106, 107, 108, 109};
    bus.push(in_data, 10);

    SECTION("Transaction")
    {
        SPISlave slave(bus, cs, config1);
        SPITransaction spi(slave);

        REQUIRE(bus.getOutBuf().size() == 0);

        SECTION("1 byte reg read")
        {

            REQUIRE(spi.read(0x05) == in_data[0]);
            REQUIRE_FALSE(bus.isSelected());

            REQUIRE(bus.getOutBuf().size() == 1);
            REQUIRE(bus.getOutBuf().back() == 0x85);

            REQUIRE(spi.read(0x05, true) == in_data[1]);
            REQUIRE_FALSE(bus.isSelected());

            REQUIRE(bus.getOutBuf().size() == 2);
            REQUIRE(bus.getOutBuf().back() == 0x85);

            REQUIRE(spi.read(0x05, false) == in_data[2]);
            REQUIRE_FALSE(bus.isSelected());

            REQUIRE(bus.getOutBuf().size() == 3);
            REQUIRE(bus.getOutBuf().back() == 0x05);
        }

        SECTION("multi byte reg read")
        {
            const int buf_size = 7;
            uint8_t buf[]      = {1, 2, 3, 4, 5, 6, 7};
            uint8_t cmp[]      = {1, 2, 3, 4, 5, 6, 7};

            spi.read(0x05, buf, 0);
            REQUIRE_FALSE(bus.isSelected());
            REQUIRE(bus.getOutBuf().size() == 1);
            REQUIRE(bus.getOutBuf().back() == 0x85);
            REQUIRE(bufcmp(buf, cmp, buf_size));

            spi.read(0x05, buf, 3);
            REQUIRE_FALSE(bus.isSelected());
            REQUIRE(bus.getOutBuf().size() == 2);
            REQUIRE(bus.getOutBuf().back() == 0x85);
            REQUIRE(bufcmp(buf, &in_data[0], 3));
            REQUIRE(bufcmp(buf + 3, cmp + 3, buf_size - 3));

            spi.read(0x05, buf, 3, true);
            REQUIRE_FALSE(bus.isSelected());
            REQUIRE(bus.getOutBuf().size() == 3);
            REQUIRE(bus.getOutBuf().back() == 0x85);
            REQUIRE(bufcmp(buf, &in_data[3], 3));
            REQUIRE(bufcmp(buf + 3, cmp + 3, buf_size - 3));

            spi.read(0x05, buf, 4, false);
            REQUIRE_FALSE(bus.isSelected());
            REQUIRE(bus.getOutBuf().size() == 4);
            REQUIRE(bus.getOutBuf().back() == 0x05);
            REQUIRE(bufcmp(buf,&in_data[6], 4));
            REQUIRE(bufcmp(buf + 4, cmp + 4, buf_size - 4));
        }

        SECTION("multi byte raw read")
        {
            const int buf_size = 7;
            uint8_t buf[]      = {1, 2, 3, 4, 5, 6, 7};
            uint8_t cmp[]      = {1, 2, 3, 4, 5, 6, 7};

            spi.read(buf, 0);
            REQUIRE_FALSE(bus.isSelected());
            REQUIRE(bus.getOutBuf().size() == 0);
            REQUIRE(bufcmp(buf, cmp, buf_size));

            spi.read(buf, 3);
            REQUIRE_FALSE(bus.isSelected());
            REQUIRE(bus.getOutBuf().size() == 0);
            REQUIRE(bufcmp(buf, &in_data[0], 3));
            REQUIRE(bufcmp(buf + 3, cmp + 3, buf_size - 3));
        }
    }
}

TEST_CASE("SPITransaction - transfer")
{
    SPIBusConfig config1;

    config1.mode      = SPIMode::MODE1;
    config1.clock_div = SPIClockDivider::DIV32;

    MockSPIBus bus(config1);
    MockGpioPin cs;

    uint8_t data[10] = {1, 2, 3, 4, 5, 6, 7, 8, 9, 10};
    
    bus.push(data, 10);

    SECTION("Transaction")
    {
        SPISlave slave(bus, cs, config1);
        SPITransaction spi(slave);

        const int buf_size = 7;
        uint8_t buf[]      = {1, 2, 3, 4, 5, 6, 7};
        uint8_t cmp[]      = {1, 2, 3, 4, 5, 6, 7};

        spi.transfer(buf, 0);
        REQUIRE_FALSE(bus.isSelected());
        REQUIRE(bus.getOutBuf().size() == 0);
        REQUIRE(bufcmp(buf, cmp, buf_size));

        spi.transfer(buf, 4);
        REQUIRE_FALSE(bus.isSelected());
        REQUIRE(bus.getOutBuf().size() == 4);
        REQUIRE(bufcmp(buf, &data[0], 4));
        REQUIRE(bufcmp(cmp, bus.getOutBuf().data(), 4));
        REQUIRE(bufcmp(buf + 4, cmp + 4, buf_size - 4));
    }
}