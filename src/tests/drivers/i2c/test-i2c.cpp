/* Copyright (c) 2022 Skyward Experimental Rocketry
 * Author: Emilio Corigliano
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

#include "drivers/i2c/I2C.h"
#include "miosix.h"
#include "string"
#include "string.h"
#include "thread"

using namespace miosix;
using namespace Boardcore;

/**
 * SETUP:
 */

typedef struct
{
    char dataChar;
    int dataInt;
    float dataFloat;
    double dataDouble;

    std::string print()
    {
        return fmt::format("{},{:d},{:f},{:f}", dataChar, dataInt, dataFloat,
                           dataDouble);
    }
} StructToSend;

StructToSend struct_tx = {'C', 42, 420.69, 48.84};

char buf_tx[64] = "Testing communication, but very very very loong :D";

int main()
{
    I2C i2c(I2C1, I2C::Speed::STANDARD, I2C::Addressing::BIT7, 42);

    i2c.init();

    return 0;
}
