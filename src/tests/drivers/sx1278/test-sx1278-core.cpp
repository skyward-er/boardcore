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

#include "test-sx1278-core.h"

using namespace Boardcore;

const char *stringFromErr(SX1278::Error err)
{
    switch (err)
    {
        case SX1278::Error::BAD_VALUE:
            return "Error::BAD_VALUE";

        case SX1278::Error::BAD_VERSION:
            return "Error::BAD_VERSION";

        default:
            return "<unknown>";
    }
}

const char *stringFromRxBw(SX1278::RxBw rx_bw)
{
    switch (rx_bw)
    {
        case SX1278::RxBw::HZ_2600:
            return "RxBw::HZ_2600";

        case SX1278::RxBw::HZ_3100:
            return "RxBw::HZ_3100";

        case SX1278::RxBw::HZ_3900:
            return "RxBw::HZ_3900";

        case SX1278::RxBw::HZ_5200:
            return "RxBw::HZ_5200";

        case SX1278::RxBw::HZ_6300:
            return "RxBw::HZ_6300";

        case SX1278::RxBw::HZ_7800:
            return "RxBw::HZ_7800";

        case SX1278::RxBw::HZ_10400:
            return "RxBw::HZ_10400";

        case SX1278::RxBw::HZ_12500:
            return "RxBw::HZ_12500";

        case SX1278::RxBw::HZ_15600:
            return "RxBw::HZ_15600";

        case SX1278::RxBw::HZ_20800:
            return "RxBw::HZ_20800";

        case SX1278::RxBw::HZ_25000:
            return "RxBw::HZ_25000";

        case SX1278::RxBw::HZ_31300:
            return "RxBw::HZ_31300";

        case SX1278::RxBw::HZ_41700:
            return "RxBw::HZ_41700";

        case SX1278::RxBw::HZ_50000:
            return "RxBw::HZ_50000";

        case SX1278::RxBw::HZ_62500:
            return "RxBw::HZ_62500";

        case SX1278::RxBw::HZ_83300:
            return "RxBw::HZ_83300";

        case SX1278::RxBw::HZ_100000:
            return "RxBw::HZ_100000";

        case SX1278::RxBw::HZ_125000:
            return "RxBw::HZ_125000";

        case SX1278::RxBw::HZ_166700:
            return "RxBw::HZ_166700";

        case SX1278::RxBw::HZ_200000:
            return "RxBw::HZ_200000";

        case SX1278::RxBw::HZ_250000:
            return "RxBw::HZ_250000";

        default:
            return "<unknown>";
    }
}

void printConfig(SX1278::Config config)
{
    printf("config.freq_rf = %d\n", config.freq_rf);
    printf("config.freq_dev = %d\n", config.freq_dev);
    printf("config.bitrate = %d\n", config.bitrate);
    printf("config.rx_bw = %s\n", stringFromRxBw(config.rx_bw));
    printf("config.afc_bw = %s\n", stringFromRxBw(config.afc_bw));
    printf("config.ocp = %d\n", config.ocp);
    printf("config.power = %d\n", config.power);
}