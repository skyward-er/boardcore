/* Copyright (c) 2018 Skyward Experimental Rocketry
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

// #include <drivers/HardwareTimer.h>
#include <array>
// #include <chrono>
#include <cstdio>
#include <ctime>
#include <iostream>
// #include <thread>
#include <math/Stats.h>
#include <miosix.h>

#include <vector>

using namespace Boardcore;
using namespace miosix;
using namespace std;

const unsigned int NUM_WRITES = 5000;

vector<unsigned int> BUF_SIZES{128, 256, 512, 1024, 2048, 4096, 1024 * 8};

// Fills a buffer with random bytes
void rndFill(uint8_t* buf, unsigned int size)
{
    for (unsigned int i = 0; i < size; i++)
    {
        *buf = rand() % 256;
        ++buf;
    }
}

bool benchmark(unsigned int buf_size, array<float, NUM_WRITES>& results)
{
    bool success = false;

    uint8_t* buf = new uint8_t[buf_size];

    FILE* f = fopen("/sd/buf.dat", "w");

    if (f == NULL)
    {
        printf("Error opening file!\n");
        goto clean;
    }

    for (unsigned int i = 0; i < NUM_WRITES; i++)
    {
        rndFill(buf, buf_size);
        long long t = getTick();
        int w       = fwrite(buf, 1, buf_size, f);
        t           = getTick() - t;

        if (w != (int)buf_size)
        {
            printf("fwrite error: %d\n", ferror(f));
            goto clean;
        }
        results[i] = t;
    }

    success = true;

clean:
    // Cleanup
    fclose(f);
    remove("buf.dat");
    delete[] buf;

    return success;
}

void printResults(unsigned int buf_size, array<float, NUM_WRITES>& results)
{
    Stats s;
    for (float f : results)
    {
        s.add(f);
    }
    StatsResult res = s.getStats();

    cout << "***BUF SIZE: " << buf_size << "\n";
    cout << "Times: \n";
    cout << "- mean: " << res.mean << " ms \n";
    cout << "- stddev: " << res.stdev << " ms \n";
    cout << "- min: " << res.minValue << " ms \n";
    cout << "- max: " << res.maxValue << " ms \n";

    cout << "Speeds: \n";
    cout << "- mean: " << buf_size / (res.mean * 1024) << " KiB/s \n";
    cout << "- min: " << buf_size / (res.maxValue * 1024) << " KiB/s \n";
    cout << "- max: " << buf_size / (res.minValue * 1024) << " KiB/s \n";

    cout << "\n\n";
}

array<float, NUM_WRITES> data;

int main()
{
    srand(time(NULL));

    for (unsigned int s : BUF_SIZES)
    {
        if (benchmark(s, data))
        {
            printResults(s, data);
        }
        else
        {
            cout << "Error (buf_size: " << s << ").\nAborting.\n";
        }
    }
    for (;;)
    {
        Thread::sleep(60000);
        cout << "Aborted!\n";
    }

    return 0;
}