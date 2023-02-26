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

#pragma once

#include <radio/SX1278/SX1278Fsk.h>

static const char *stringFromErr(Boardcore::SX1278Fsk::Error err)
{
    switch (err)
    {
        case Boardcore::SX1278Fsk::Error::BAD_VALUE:
            return "Error::BAD_VALUE";
        case Boardcore::SX1278Fsk::Error::BAD_VERSION:
            return "Error::BAD_VERSION";
        default:
            return "<unknown>";
    }
}

static const char *stringFromRxBw(Boardcore::SX1278Fsk::RxBw rx_bw)
{
    switch (rx_bw)
    {
        case Boardcore::SX1278Fsk::RxBw::HZ_2600:
            return "RxBw::HZ_2600";
        case Boardcore::SX1278Fsk::RxBw::HZ_3100:
            return "RxBw::HZ_3100";
        case Boardcore::SX1278Fsk::RxBw::HZ_3900:
            return "RxBw::HZ_3900";
        case Boardcore::SX1278Fsk::RxBw::HZ_5200:
            return "RxBw::HZ_5200";
        case Boardcore::SX1278Fsk::RxBw::HZ_6300:
            return "RxBw::HZ_6300";
        case Boardcore::SX1278Fsk::RxBw::HZ_7800:
            return "RxBw::HZ_7800";
        case Boardcore::SX1278Fsk::RxBw::HZ_10400:
            return "RxBw::HZ_10400";
        case Boardcore::SX1278Fsk::RxBw::HZ_12500:
            return "RxBw::HZ_12500";
        case Boardcore::SX1278Fsk::RxBw::HZ_15600:
            return "RxBw::HZ_15600";
        case Boardcore::SX1278Fsk::RxBw::HZ_20800:
            return "RxBw::HZ_20800";
        case Boardcore::SX1278Fsk::RxBw::HZ_25000:
            return "RxBw::HZ_25000";
        case Boardcore::SX1278Fsk::RxBw::HZ_31300:
            return "RxBw::HZ_31300";
        case Boardcore::SX1278Fsk::RxBw::HZ_41700:
            return "RxBw::HZ_41700";
        case Boardcore::SX1278Fsk::RxBw::HZ_50000:
            return "RxBw::HZ_50000";
        case Boardcore::SX1278Fsk::RxBw::HZ_62500:
            return "RxBw::HZ_62500";
        case Boardcore::SX1278Fsk::RxBw::HZ_83300:
            return "RxBw::HZ_83300";
        case Boardcore::SX1278Fsk::RxBw::HZ_100000:
            return "RxBw::HZ_100000";
        case Boardcore::SX1278Fsk::RxBw::HZ_125000:
            return "RxBw::HZ_125000";
        case Boardcore::SX1278Fsk::RxBw::HZ_166700:
            return "RxBw::HZ_166700";
        case Boardcore::SX1278Fsk::RxBw::HZ_200000:
            return "RxBw::HZ_200000";
        case Boardcore::SX1278Fsk::RxBw::HZ_250000:
            return "RxBw::HZ_250000";
        default:
            return "<unknown>";
    }
}

static void printConfig(const Boardcore::SX1278Fsk::Config &config)
{
    printf("config.freq_rf = %d\n", config.freq_rf);
    printf("config.freq_dev = %d\n", config.freq_dev);
    printf("config.bitrate = %d\n", config.bitrate);
    printf("config.rx_bw = %s\n", stringFromRxBw(config.rx_bw));
    printf("config.afc_bw = %s\n", stringFromRxBw(config.afc_bw));
    printf("config.ocp = %d\n", config.ocp);
    printf("config.power = %d\n", config.power);
}
