/* Copyright (c) 2015-2018 Skyward Experimental Rocketry
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
#ifndef SRC_SHARED_LOGGER_LOGPROXY_H
#define SRC_SHARED_LOGGER_LOGPROXY_H

#include "logger/Logger.h"
#include "Singleton.h"

#include "sensors/MPU9250/MPU9250Data.h"

class LoggerProxy : public Singleton<LoggerProxy>
{
    friend class Singleton<LoggerProxy>;

public:
    struct LowRateData
    {
        Vec3 mpu9250_accel;
    };

    struct HighRateData
    {
        float pressure_sample;
        uint8_t last_fmm_event;
        uint8_t last_ign_event;
        uint8_t last_nsc_event;
    };

    LoggerProxy() : lr_data(), hr_data(), logger(Logger::instance()) {}

    template <typename T>
    inline LogResult log(const T& t)
    {
        return logger.log(t);
    }

    inline LogResult log(const LowRateData& t)
    {
        {
            miosix::PauseKernelLock kLock;
            lr_data = t;
        }
        return logger.log(t);
    }

    LowRateData getLowRateData() { return lr_data; }

    HighRateData getHighRateData()
    {
        return hr_data;
    }

private:
    LowRateData lr_data;
    HighRateData hr_data;

    Logger& logger;
};

#endif /* SRC_SHARED_LOGGER_LOGPROXY_H */
