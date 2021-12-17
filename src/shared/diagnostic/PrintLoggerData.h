/* Copyright (c) 2021 Skyward Experimental Rocketry
 * Authors: Luca Erbetta, Luca Conterio
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

#include <string>

namespace Boardcore
{

static constexpr unsigned int MAX_LOG_STRING_SIZE =
    250; /**< Max length of messages to be logged through the buffered logger */

struct LogRecord
{
    int level;
    std::string function;
    std::string file;
    int line;
    std::string name;
    std::string message;
};

enum LogLevel : uint8_t
{
    LOGL_NOTSET   = 0,
    LOGL_DEBUG    = 10,
    LOGL_INFO     = 20,
    LOGL_WARNING  = 30,
    LOGL_ERROR    = 40,
    LOGL_CRITICAL = 50
};

/**
 * This class is used to output to filesystem (SD card) log messages
 * through the buffered logger (src/shared/logger/Logger.h).
 */
struct LoggingString
{
    char log_string[MAX_LOG_STRING_SIZE];

    static std::string header() { return "log_string\n"; }

    void print(std::ostream& os) const { os << log_string << "\n"; }
};

}  // namespace Boardcore
