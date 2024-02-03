/* Copyright (c) 2024 Skyward Experimental Rocketry
 * Authors: Adriano Longo, Davide Mor, Damiano Procaccia, Emilio Corigliano
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

#include <miosix.h>

using namespace miosix;
typedef uint32_t u32;
typedef uint8_t u8;

inline u32 xor_next(u32 current)
{
    current ^= current >> 7;
    current ^= current << 9;
    current ^= current >> 13;
    return current;
}

void test_ram1()
{
    const u32 pattern0     = 0xaaaaaaaa;
    const u32 nBanks       = 4;
    const u32 base_address = 0xd0000000;
    const u32 end_address  = 0xd1000000;
    const u32 inc          = 4;
    u32 wait               = 10;
    u32 n                  = 1;

    while (1)
    {
        u32 fullError  = 0;
        u32 otherError = 0;
        u32 pattern    = pattern0;
        u32 iPattern   = 0;
        for (u32 i = 0; i < (end_address - base_address) / nBanks; i += inc)
        {
            for (u32 iBank = 0; iBank < nBanks; iBank++)
            {
                u32 address = base_address + i +
                              iBank * (end_address - base_address) / nBanks;

                *((volatile u32*)address) = pattern;

                if (iPattern % n)
                {
                    pattern = ~pattern;
                }

                iPattern++;
            }
        }
        delayUs(wait);

        // printf("reading\n");
        pattern  = pattern0;
        iPattern = 0;

        for (u32 i = 0; i < (end_address - base_address) / nBanks; i += inc)
        {
            for (u32 iBank = 0; iBank < nBanks; iBank++)
            {
                u32 address = base_address + i +
                              iBank * (end_address - base_address) / nBanks;

                u32 read = *((volatile u32*)address);

                if (read != pattern)
                {
                    if ((pattern ^ read) == 0xffffffff)
                        fullError++;
                    else
                        otherError++;
                    // printf("%lx %lx %lx %lx\n", address, pattern, read,
                    //        pattern ^ read);
                };

                if (iPattern % n)
                {
                    pattern = ~pattern;
                }

                iPattern++;
            }
        }

        printf("n: %ld, full: %ld, other: %ld\n", n, fullError, otherError);
        if (n == (end_address - base_address))
            break;
        n += n;
        // break;
    }
    printf("end\n");
    ledOn();
    while (1)
    {
    }
}

void test_ram1bis()
{
    const u32 pattern0     = 0xaaaaaaaa;
    const u32 base_address = 0xd0000000;
    const u32 end_address  = 0xd1000000;
    const u32 inc          = 4;
    u32 wait               = 10;
    u32 n                  = 1;

    while (1)
    {
        u32 fullError  = 0;
        u32 otherError = 0;
        u32 pattern    = pattern0;
        u32 iPattern   = 0;
        for (u32 i = base_address; i < end_address; i += inc)
        {
            u32 address = i;

            *((volatile u32*)address) = pattern;

            if (iPattern % n)
            {
                pattern = ~pattern;
            }

            iPattern++;
        }
        delayUs(wait);

        // printf("reading\n");
        pattern  = pattern0;
        iPattern = 0;

        for (u32 i = base_address; i < end_address; i += inc)
        {
            u32 address = i;

            u32 read = *((volatile u32*)address);

            if (read != pattern)
            {
                if ((pattern ^ read) == 0xffffffff)
                    fullError++;
                else
                    otherError++;
                // printf("%lx %lx %lx %lx\n", address, pattern, read,
                //        pattern ^ read);
            };

            if (iPattern % n)
            {
                pattern = ~pattern;
            }

            iPattern++;
        }

        printf("n: %ld, full: %ld, other: %ld\n", n, fullError, otherError);
        if (n == (end_address - base_address))
            break;
        n += n;
        // break;
    }
    printf("end\n");
    ledOn();
    while (1)
    {
    }
}

void test_ram2()
{
    const u32 pattern0     = 0xdeadbeef;
    const u32 pattern1     = 0xcafebabe;
    const u32 nBanks       = 4;
    const u32 base_address = 0xd0000000;
    const u32 end_address  = 0xd1000000;
    const u32 inc          = 4;
    u32 wait               = 10;
    u32 n                  = 1;

    while (1)
    {
        u32 fullError  = 0;
        u32 otherError = 0;
        u32 pattern    = pattern0;
        u32 iPattern   = 0;
        for (u32 i = 0; i < (end_address - base_address) / nBanks; i += inc)
        {
            for (u32 iBank = 0; iBank < nBanks; iBank++)
            {
                u32 address = base_address + i +
                              iBank * (end_address - base_address) / nBanks;

                *((volatile u32*)address) = pattern;

                if (iPattern % n)
                {
                    pattern = (pattern == pattern0 ? pattern1 : pattern0);
                }

                iPattern++;
            }
        }
        delayUs(wait);

        // printf("reading\n");
        pattern  = pattern0;
        iPattern = 0;

        for (u32 i = 0; i < (end_address - base_address) / nBanks; i += inc)
        {
            for (u32 iBank = 0; iBank < nBanks; iBank++)
            {
                u32 address = base_address + i +
                              iBank * (end_address - base_address) / nBanks;

                u32 read = *((volatile u32*)address);

                if (read != pattern)
                {
                    if ((pattern ^ read) == 0xffffffff)
                        fullError++;
                    else
                        otherError++;
                    // printf("%lx %lx %lx %lx\n", address, pattern, read,
                    //        pattern ^ read);
                };

                if (iPattern % n)
                {
                    pattern = (pattern == pattern0 ? pattern1 : pattern0);
                }

                iPattern++;
            }
        }

        printf("n: %ld, full: %ld, other: %ld\n", n, fullError, otherError);
        if (n == (end_address - base_address))
            break;
        n += n;
        // break;
    }
    printf("end\n");
    ledOn();
    while (1)
    {
    }
}

// u8 rtt_up_buffer[2048];

int main()
{
    while (true)
    {
        ledOn();
        printf("Serial is working!\n");
        Thread::sleep(1000);
        ledOff();
        Thread::sleep(1000);
        test_ram1bis();
    }

    return 0;
}
