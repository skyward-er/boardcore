/* Copyright (c) 2015-2018 Skyward Experimental Rocketry
 * Authors: Alvise de'Faveri Tron
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

#include <logger/Logger.h>
#include "Singleton.h"

#include <boards/CanInterfaces.h>

/**
 * This class is used to intercept calls to the logger and update internal state
 * before logging.
 */
class LoggerProxy : public Singleton<LoggerProxy>
{
    friend class Singleton<LoggerProxy>;

public:
    /**
     * @brief Create instance of Logger.
     */
    LoggerProxy() : status(), logger(Logger::instance()) {}

    /**
     * @brief Simple log function. You have to specify what structure you are 
     * logging.
     * Note that specific log() functions are provided in the .cpp file for those  
     * structures that have to be saved before logging.
     */
    template <typename T>
    inline LogResult log(const T& t)
    {
        return logger.log(t);
    }

    /**
     * @brief Internal status getter. Synchronization is guaranteed.
     */
    CanInterfaces::NoseconeBoardStatus getNoseconeStatus() { 
        return status; 
    }

private:
    CanInterfaces::NoseconeBoardStatus status;

    Logger& logger;
};

#endif /* SRC_SHARED_LOGGER_LOGPROXY_H */
