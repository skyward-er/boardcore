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

#include <utils/testutils/catch.hpp>

#include "FakeSPIBus.h"
#include "drivers/spi/MockSPIBus.h"
#include "drivers/spi/SPIDriver.h"

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

    FakeSPIBus bus{&spi};

    REQUIRE(spi.CR1 == 0);

    SECTION("Configure & check CR1")
    {
        SPIBusConfig config;
        REQUIRE(spi.CR1 == 0);

        SECTION("Mode")
        {
            config.mode = SPIMode::MODE0;
            uint32_t expected_CR1 = 0x0344;   
            bus.configure(config);
            REQUIRE(spi.CR1 == expected_CR1);

            config.mode = SPIMode::MODE1;
            expected_CR1 = 0x0345;   
            bus.configure(config);
            REQUIRE(spi.CR1 == expected_CR1);

            config.mode = SPIMode::MODE2;
            expected_CR1 = 0x0346;   
            bus.configure(config);
            REQUIRE(spi.CR1 == expected_CR1);

            config.mode = SPIMode::MODE3;
            expected_CR1 = 0x0347;   
            bus.configure(config);
            REQUIRE(spi.CR1 == expected_CR1);

        }

        SECTION("Clock Divider")
        {
            config.clock_div = SPIClockDivider::DIV2;
            uint32_t expected_CR1 = 0x0344;   
            bus.configure(config);
            REQUIRE(spi.CR1 == expected_CR1);

            config.clock_div = SPIClockDivider::DIV4;
            expected_CR1 = 0x034C;   
            bus.configure(config);
            REQUIRE(spi.CR1 == expected_CR1);

            config.clock_div = SPIClockDivider::DIV8;
            expected_CR1 = 0x0354;   
            bus.configure(config);
            REQUIRE(spi.CR1 == expected_CR1);

            config.clock_div = SPIClockDivider::DIV16;
            expected_CR1 = 0x035C;   
            bus.configure(config);
            REQUIRE(spi.CR1 == expected_CR1);

            config.clock_div = SPIClockDivider::DIV32;
            expected_CR1 = 0x0364;   
            bus.configure(config);
            REQUIRE(spi.CR1 == expected_CR1);
            
            config.clock_div = SPIClockDivider::DIV64;
            expected_CR1 = 0x036C;   
            bus.configure(config);
            REQUIRE(spi.CR1 == expected_CR1);

            config.clock_div = SPIClockDivider::DIV128;
            expected_CR1 = 0x0374;   
            bus.configure(config);
            REQUIRE(spi.CR1 == expected_CR1);

            config.clock_div = SPIClockDivider::DIV256;
            expected_CR1 = 0x037C;   
            bus.configure(config);
            REQUIRE(spi.CR1 == expected_CR1);
        }

        SECTION("Bit order")
        {
            config.bit_order = SPIBitOrder::MSB_FIRST;
            uint32_t expected_CR1 = 0x0344;   
            bus.configure(config);
            REQUIRE(spi.CR1 == expected_CR1);

            config.bit_order = SPIBitOrder::LSB_FIRST;
            expected_CR1 = 0x03C4;   
            bus.configure(config);
            REQUIRE(spi.CR1 == expected_CR1);
        }
      
    }

    SECTION("Disable configuration")
    {
        SPIBusConfig config;
        config.clock_div = SPIClockDivider::DIV16;

        config.mode      = SPIMode::MODE3;
        config.bit_order = SPIBitOrder::LSB_FIRST;

        bus.enableSlaveConfiguration(false);
        bus.configure(config);
        REQUIRE(spi.CR1 == 0);
    }
}

TEST_CASE("SPIBus - Chip select")
{
    FakeSpiTypedef spi;

    FakeSPIBus bus{&spi};

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

    FakeSPIBus bus{&spi};

    SPIBusConfig config;
    config.clock_div = SPIClockDivider::DIV16;

    config.mode      = SPIMode::MODE3;
    config.bit_order = SPIBitOrder::LSB_FIRST;

    bus.configure(config);
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
}

TEST_CASE("SPIBus - Multi byte operations")
{
    FakeSpiTypedef spi;

    spi.DR.in_buf    = {1, 2, 3, 4, 5, 6, 7, 8, 9};
    spi.CR1_expected = 0x03DF;

    FakeSPIBus bus{&spi};

    SPIBusConfig config;
    config.clock_div = SPIClockDivider::DIV16;

    config.mode      = SPIMode::MODE3;
    config.bit_order = SPIBitOrder::LSB_FIRST;

    bus.configure(config);
    bus.select(spi.cs);

    // 2 identical buffers
    uint8_t buf[]  = {4, 3, 2, 1};
    uint8_t bufc[] = {4, 3, 2, 1};

    SECTION("Write")
    {
        bus.write(buf, 0);
        REQUIRE(spi.DR.out_buf.size() == 0);

        bus.write(buf, 4);
        REQUIRE(spi.DR.out_buf.size() == 4);
        REQUIRE(bufcmp(bufc, spi.DR.out_buf.data(), 4));
    }

    SECTION("Read")
    {
        bus.read(buf, 0);
        // Nothing read
        REQUIRE(bufcmp(bufc, buf, 4));

        bus.read(buf, 4);

        REQUIRE(bufcmp(buf, spi.DR.in_buf.data(), 4));
    }

    SECTION("Transfer")
    {
        bus.transfer(buf, 0);
        // Nothing read
        REQUIRE(bufcmp(bufc, buf, 4));
        // Nothing written
        REQUIRE(spi.DR.out_buf.size() == 0);

        bus.transfer(buf, 4);
        REQUIRE(spi.DR.out_buf.size() == 4);

        REQUIRE(bufcmp(bufc, spi.DR.out_buf.data(), 4));
        REQUIRE(bufcmp(buf, spi.DR.in_buf.data(), 4));
    }
}

TEST_CASE("SPITransaction - writes")
{
    MockSPIBus bus{};
    SPIBusConfig config1{};

    config1.mode = SPIMode::MODE1;
    config1.clock_div  = SPIClockDivider::DIV32;

    bus.expected_config = config1;

    SECTION("Transaction")
    {
        SPITransaction spi(bus, GpioPin(GPIOA_BASE, 1), config1);

        REQUIRE(bus.out_buf.size() == 0);

        SECTION("cmd write")
        {
            spi.write(9);
            REQUIRE_FALSE(bus.isSelected());
            REQUIRE(bus.out_buf.size() == 1);
            REQUIRE(bus.out_buf.back() == 9);
        }

        SECTION("1 byte reg write")
        {
            spi.write(10, 77);
            REQUIRE_FALSE(bus.isSelected());

            REQUIRE(bus.out_buf.size() == 2);
            REQUIRE(bus.out_buf[0] == 10);
            REQUIRE(bus.out_buf[1] == 77);
        }

        SECTION("multi byte reg write")
        {
            uint8_t buf[] = {1, 2, 3, 4, 5, 6};

            SECTION("0 size write")
            {
                spi.write(10, buf, 0);
                REQUIRE_FALSE(bus.isSelected());

                REQUIRE(bus.out_buf.size() == 1);
                REQUIRE(bus.out_buf[0] == 10);
            }

            SECTION("2 writes")
            {
                spi.write(10, buf, 4);
                REQUIRE_FALSE(bus.isSelected());

                REQUIRE(bus.out_buf.size() == 5);

                REQUIRE(bus.out_buf[0] == 10);
                REQUIRE(bufcmp(buf, bus.out_buf.data() + 1, 4));

                spi.write(99, buf, 6);
                REQUIRE_FALSE(bus.isSelected());

                REQUIRE(bus.out_buf.size() == 12);

                REQUIRE(bus.out_buf[5] == 99);
                REQUIRE(bufcmp(buf, bus.out_buf.data() + 6, 6));
            }
        }

        SECTION("raw write")
        {
            uint8_t buf[] = {1, 2, 3, 4, 5, 6};

            spi.write(buf, 0);
            REQUIRE_FALSE(bus.isSelected());

            REQUIRE(bus.out_buf.size() == 0);

            spi.write(buf, 4);
            REQUIRE_FALSE(bus.isSelected());

            REQUIRE(bus.out_buf.size() == 4);

            REQUIRE(bufcmp(buf, bus.out_buf.data(), 4));

            spi.write(buf, 6);
            REQUIRE_FALSE(bus.isSelected());

            REQUIRE(bus.out_buf.size() == 10);

            REQUIRE(bufcmp(buf, bus.out_buf.data() + 4, 6));
        }
    }
}

TEST_CASE("SPITransaction - reads")
{
    MockSPIBus bus;

    bus.in_buf = {1, 2, 3, 4, 5, 6, 7, 8, 9, 10};

    SPIBusConfig config1;

    config1.mode = SPIMode::MODE1;
    config1.clock_div  = SPIClockDivider::DIV32;

    bus.expected_config = config1;

    SECTION("Transaction")
    {
        SPISlave slave(bus, GpioPin(GPIOA_BASE, 1), config1);
        SPITransaction spi(slave);

        REQUIRE(bus.out_buf.size() == 0);

        SECTION("1 byte reg read")
        {

            REQUIRE(spi.read(0x05) == 1);
            REQUIRE_FALSE(bus.isSelected());

            REQUIRE(bus.out_buf.size() == 1);
            REQUIRE(bus.out_buf.back() == 0x85);

            REQUIRE(spi.read(0x05, true) == 2);
            REQUIRE_FALSE(bus.isSelected());

            REQUIRE(bus.out_buf.size() == 2);
            REQUIRE(bus.out_buf.back() == 0x85);

            REQUIRE(spi.read(0x05, false) == 3);
            REQUIRE_FALSE(bus.isSelected());

            REQUIRE(bus.out_buf.size() == 3);
            REQUIRE(bus.out_buf.back() == 0x05);
        }

        SECTION("multi byte reg read")
        {
            uint8_t read[4] = {0, 0, 0, 0};
            uint8_t zero[4] = {0, 0, 0, 0};

            spi.read(0x05, read, 0);
            REQUIRE_FALSE(bus.isSelected());
            REQUIRE(bus.out_buf.size() == 1);
            REQUIRE(bus.out_buf.back() == 0x85);
            REQUIRE(bufcmp(read, zero, 4));

            spi.read(0x05, read, 3);
            REQUIRE_FALSE(bus.isSelected());
            REQUIRE(bus.out_buf.size() == 2);
            REQUIRE(bus.out_buf.back() == 0x85);
            REQUIRE(bufcmp(read, bus.in_buf.data(), 3));

            spi.read(0x05, read, 3, true);
            REQUIRE_FALSE(bus.isSelected());
            REQUIRE(bus.out_buf.size() == 3);
            REQUIRE(bus.out_buf.back() == 0x85);
            REQUIRE(bufcmp(read, bus.in_buf.data() + 3, 3));

            spi.read(0x05, read, 4, false);
            REQUIRE_FALSE(bus.isSelected());
            REQUIRE(bus.out_buf.size() == 4);
            REQUIRE(bus.out_buf.back() == 0x05);
            REQUIRE(bufcmp(read, bus.in_buf.data() + 6, 4));
        }

        SECTION("multi byte raw read")
        {
            uint8_t read[4] = {0, 0, 0, 0};
            uint8_t zero[4] = {0, 0, 0, 0};

            spi.read(read, 0);
            REQUIRE_FALSE(bus.isSelected());
            REQUIRE(bus.out_buf.size() == 0);
            REQUIRE(bufcmp(read, zero, 4));

            spi.read(read, 3);
            REQUIRE_FALSE(bus.isSelected());
            REQUIRE(bus.out_buf.size() == 0);
            REQUIRE(bufcmp(read, bus.in_buf.data(), 3));
        }
    }
}

TEST_CASE("SPITransaction - transfer")
{
    MockSPIBus bus;

    bus.in_buf = {1, 2, 3, 4, 5, 6, 7, 8, 9, 10};

    SPIBusConfig config1;

    config1.mode = SPIMode::MODE1;
    config1.clock_div  = SPIClockDivider::DIV32;

    bus.expected_config = config1;

    SECTION("Transaction")
    {
        SPISlave slave(bus, GpioPin(GPIOA_BASE, 1), config1);
        SPITransaction spi(slave);

        uint8_t buf[4] = {4, 3, 2, 1};
        uint8_t cmp[4] = {4, 3, 2, 1};

        spi.transfer(buf, 0);
        REQUIRE_FALSE(bus.isSelected());
        REQUIRE(bus.out_buf.size() == 0);
        REQUIRE(bufcmp(buf, cmp, 4));

        spi.transfer(buf, 4);
        REQUIRE_FALSE(bus.isSelected());
        REQUIRE(bus.out_buf.size() == 4);
        REQUIRE(bufcmp(buf, bus.in_buf.data(), 4));
        REQUIRE(bufcmp(cmp, bus.out_buf.data(), 4));
    }
}