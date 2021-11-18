/* Copyright (c) 2019 Skyward Experimental Rocketry
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

#define private public

#ifdef STANDALONE_CATCH1_TEST
#include "catch-tests-entry.cpp"
#endif

#include <catch2/catch.hpp>
#include <Common.h>
#include "sensors/LM75B.h"
#include "drivers/BusTemplate.h"

using namespace miosix;

#include <interfaces-impl/hwmapping.h>
using I2CProtocol = ProtocolI2C<miosix::I2C1Driver>;
typedef LM75B<I2CProtocol> LM75BType;

//TODO sistemare la codifica di {0x80, 0x00}, che
//per qualche ragione, diventa positiva
TEST_CASE("[LM75B] temperature")
{
    uint8_t addr1 = 0x48 << 1;
    LM75BType temp1{addr1};

    SECTION("[LM75B] positive temperature"){
        uint8_t temp_array[2] = {0x10, 0x10};
        REQUIRE(temp1.computeTemp(temp_array) 
                == Approx(16.0).epsilon(0.001));
    }

    SECTION("[LM75B] negative temperature"){
        uint8_t temp_array[2] = {0x80, 0x10};
        REQUIRE(temp1.computeTemp(temp_array) 
                == Approx(-127.875).epsilon(0.001));
    }

    SECTION("[LM75B] zero temperature"){
        uint8_t temp_array[2] = {0x00, 0x00};
        REQUIRE(temp1.computeTemp(temp_array) 
                == Approx(0).epsilon(0.001));
    }

    SECTION("[LM75B] max temperature"){
        uint8_t temp_array[2] = {0x7F, 0xFF};
        REQUIRE(temp1.computeTemp(temp_array) 
                == Approx(127.875).epsilon(0.001));
    }

    SECTION("[LM75B] min temperature"){
        uint8_t temp_array[2] = {0x80, 0x01};
        REQUIRE(temp1.computeTemp(temp_array) 
                == Approx(-127.875).epsilon(0.001));
    }
}