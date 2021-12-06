/* Copyright (c) 2021 Skyward Experimental Rocketry
 * Author: Luca Erbetta, Luca Conterio
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

#include <logger/Logger.h>
#include <miosix.h>

#include "PrintLoggerData.h"

using miosix::FastMutex;

namespace Boardcore
{

class LogSink
{
public:
    LogSink() {}
    LogSink(const LogSink&) = delete;
    LogSink& operator=(const LogSink&) = delete;

    void log(const LogRecord& record);

    void enable() { enabled = true; }

    void disable() { enabled = false; }

    bool isEnabled() { return enabled; }

    void setLevel(uint8_t level) { min_level = level; }

    int getLevel() { return min_level; }

    void setFormatString(std::string format) { this->format = format; }

protected:
    virtual void logImpl(std::string l) = 0;

private:
    bool enabled       = true;  // enabled by the default when created
    uint8_t min_level  = LOGL_NOTSET;
    std::string format = "{ts} {file}:{line} {fun} {lvl} [{name}] {msg}\n";
};

/**
 * This class directly outputs the log to file.
 */
class FileLogSink : public LogSink
{
public:
    FileLogSink(FILE* f) : f(f) {}

    FileLogSink() {}

    void setFile(FILE* f_) { f = f_; }

protected:
    void logImpl(std::string l);

    FILE* f;
    FastMutex mutex;
};

/**
 * This class uses the skyward-boardcore logger in order to output
 * strings to filesystem. It can be used for example to log error strings
 * during flight, but use it carefully.
 */
class FileLogSinkBuffered : public FileLogSink
{
public:
    FileLogSinkBuffered() : logger(Logger::instance()) {}

protected:
    void logImpl(std::string l);

private:
    Logger& logger;
    FastMutex mutex;
};

}  // namespace Boardcore
