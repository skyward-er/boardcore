/* Copyright (c) 2018 Skyward Experimental Rocketry
 * Authors: Luca Erbetta
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

#define CATCH_CONFIG_RUNNER
#define CATCH_CONFIG_NO_POSIX_SIGNALS
#define CATCH_CONFIG_NO_CPP11_GENERATED_METHODS
#define CATCH_CONFIG_NO_CPP11_TYPE_TRAITS

#include <miosix.h>
#include <catch.hpp>
#include <cstdio>
#include "boards/CanInterfaces.h"

using miosix::Thread;
using namespace CanInterfaces;

int main()
{
    int result = Catch::Session().run();

    for (;;)
    {
        printf("end.\n");
        Thread::sleep(10000);
    }
}

TEST_CASE("Test Bitfield", "[bitfield]")
{
    REQUIRE(sizeof(IgnitionBoardStatus) == 1);

    uint8_t bitmap;
    IgnitionBoardStatus brd{1, 0, 0, 0, 1, 1, 1, 1};
    memcpy(&bitmap, &brd, 1);

    printf("%d\n", bitmap);
    REQUIRE(bitmap == 0xF1);
}

TEST_CASE("Test msgpackets", "[pack]")
{
    uint8_t buf[8] = {0};
    SECTION("Simple message")
    {
        canMsgSimple(buf, CAN_MSG_ABORT);
        REQUIRE(buf[0] == CAN_MSG_ABORT);
        bool zero = true;
        for(int i = 1; i < 8; i++)
        {
            if(buf[i] != 0)
            {
                zero = false;
                break;
            }
        }

        REQUIRE(zero);
    }


    SECTION("Ign status message")
    {
        IgnitionBoardStatus brd{1, 0, 0, 0, 1, 1, 1, 1};
        canMsgIgnitionStatus(buf, brd);
        REQUIRE(buf[0] == CAN_MSG_IGN_STATUS);
        REQUIRE(buf[1] == 0xF1);

        bool zero = true;
        for(int i = 2; i < 8; i++)
        {
            if(buf[i] != 0)
            {
                zero = false;
                break;
            }
        }

        REQUIRE(zero);
    }

}