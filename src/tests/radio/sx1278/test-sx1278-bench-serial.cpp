/* Copyright (c) 2021 Skyward Experimental Rocketry
 * Author: Davide Mor
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

// Include the body of the test
#include "test-sx1278-bench.cpp"

int main()
{
    initBoard();
    if (!initRadio())
    {
        while (1)
            ;
    }

    // Initialize backgrounds threads
    spawnThreads();

    while (1)
    {
        printf(
            "\n[sx1278] Stats:\n"
            "Tx bitrate:        %.2f kb/s\n"
            "Packet sent:       %d\n"
            "Rx bitrate:        %.2f kb/s\n"
            "Packet received:   %d\n"
            "Corrupted packets: %d\n"
            "RSSI:              %.2f dBm\n"
            "FEI:               %.2f Hz\n"
            "SNR:               %.2f\n",
            static_cast<float>(stats.txBitrate()) / 1000.0f, stats.sent_count,
            static_cast<float>(stats.rxBitrate()) / 1000.0f, stats.recv_count,
            stats.corrupted_count, stats.rssi, stats.fei, stats.snr);

        miosix::Thread::sleep(2000);
    }

    return 0;
}
