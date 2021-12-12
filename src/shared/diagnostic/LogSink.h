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

#include <Common.h>
#include <logger/Logger.h>

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

    virtual ~LogSink() {}

#ifdef DISABLE_PRINTLOGGER
    void log(const LogRecord& record) { UNUSED(record); }
#else
    void log(const LogRecord& record);
#endif
    void enable() { enabled = true; }

    void disable() { enabled = false; }

    bool isEnabled() { return enabled; }

    void setLevel(uint8_t level) { min_level = level; }

    int getLevel() { return min_level; }

    void setFormatString(const std::string& format) { this->format = format; }

protected:
    virtual void logImpl(const std::string& l) = 0;

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
    FileLogSink() {}

    explicit FileLogSink(FILE* f) : f(f) {}

    void setFile(FILE* f_) { f = f_; }

protected:
#ifdef DISABLE_PRINTLOGGER
    void logImpl(const std::string& l) override { UNUSED(l); }
#else
    void logImpl(const std::string& l) override;
#endif
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
#ifdef DISABLE_PRINTLOGGER
    void logImpl(const std::string& l) override { UNUSED(l); }
#else
    void logImpl(const std::string& l) override;
#endif

private:
    Logger& logger;
};

}  // namespace Boardcore
