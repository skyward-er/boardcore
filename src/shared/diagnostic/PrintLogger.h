/* Copyright (c) 2021 Skyward Experimental Rocketry
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

#pragma once

#include <ActiveObject.h>
#include <Singleton.h>
#include <fmt/format.h>
#include <logger/Logger.h>
#include <utils/Constants.h>
#include <utils/collections/CircularBuffer.h>

#include <memory>
#include <string>
#include <vector>

#include "LogSink.h"
#include "PrintLoggerData.h"

using std::string;
using std::unique_ptr;
using std::vector;

using miosix::ConditionVariable;
using miosix::FastMutex;

namespace Boardcore
{

#ifndef DEFAULT_STDOUT_LOG_LEVEL
#define DEFAULT_STDOUT_LOG_LEVEL 0
#endif

static constexpr unsigned int ASYNC_LOG_BUFFER_SIZE = 100;

class Logging;

class PrintLogger
{
public:
    PrintLogger(Logging& logging, const string& name)
        : parent(logging), name(name)
    {
    }

    PrintLogger getChild(const string& name);

    template <typename... Args>
    void log(uint8_t level, const string& function, const string& file,
             int line, string format, Args&&... args)
    {
        vlog(level, function, file, line, format,
             fmt::make_args_checked<Args...>(format, args...));
    }

    template <typename... Args>
    void logAsync(uint8_t level, const string& function, const string& file,
                  int line, string format, Args&&... args)
    {
        vlogAsync(level, function, file, line, format,
                  fmt::make_args_checked<Args...>(format, args...));
    }

private:
    void vlog(uint8_t level, const string& function, const string& file,
              int line, fmt::string_view format, fmt::format_args args);
    void vlogAsync(uint8_t level, const string& function, const string& file,
                   int line, fmt::string_view format, fmt::format_args args);

    LogRecord buildLogRecord(uint8_t level, const string& function,
                             const string& file, int line,
                             fmt::string_view format, fmt::format_args args);

    Logging& parent;
    string name;
};

class Logging : public Singleton<Logging>
{
    friend class Singleton<Logging>;
    friend class PrintLogger;

public:
    static PrintLogger getLogger(const string& name)
    {
        return PrintLogger(getInstance(), name);
    }

    static void addLogSink(unique_ptr<LogSink>& sink)
    {
        getInstance().sinks.push_back(std::move(sink));
    }

    static LogSink& getStdOutLogSink() { return *getInstance().sinks.at(0); }

    static void startAsyncLogger() { getInstance().asyncLog.start(); }

private:
    void log(const LogRecord& record);
    void logAsync(const LogRecord& record);

    class AsyncLogger : public ActiveObject
    {
    public:
        explicit AsyncLogger(Logging& parent);
        void log(const LogRecord& record);

    protected:
        void run() override;

    private:
        Logging& parent;
        CircularBuffer<LogRecord, ASYNC_LOG_BUFFER_SIZE> records;
        FastMutex mutex;
        ConditionVariable cv;
    };

    Logging() : asyncLog(*this)
    {
        unique_ptr<FileLogSink> serial = std::make_unique<FileLogSink>(stdout);
        serial->setLevel(DEFAULT_STDOUT_LOG_LEVEL);
#ifndef DEBUG  // do not output to serial if not in DEBUG mode
        serial->disable();
#endif
        sinks.push_back(std::move(serial));
    }

    AsyncLogger asyncLog;
    vector<unique_ptr<LogSink>> sinks;
};

}  // namespace Boardcore

#define LOG(logger, level, ...) \
    logger.log(level, __FUNCTION__, __FILE__, __LINE__, __VA_ARGS__)

#define LOG_DEBUG(logger, ...) \
    LOG(logger, Boardcore::LogLevel::LOGL_DEBUG, __VA_ARGS__)

#define LOG_INFO(logger, ...) \
    LOG(logger, Boardcore::LogLevel::LOGL_INFO, __VA_ARGS__)

#define LOG_WARN(logger, ...) \
    LOG(logger, Boardcore::LogLevel::LOGL_WARNING, __VA_ARGS__)

#define LOG_ERR(logger, ...) \
    LOG(logger, Boardcore::LogLevel::LOGL_ERROR, __VA_ARGS__)

#define LOG_CRIT(logger, ...) \
    LOG(logger, Boardcore::LogLevel::LOGL_CRITICAL, __VA_ARGS__)

#define LOG_ASYNC(logger, level, ...) \
    logger.logAsync(level, __FUNCTION__, __FILE__, __LINE__, __VA_ARGS__)

#define LOG_DEBUG_ASYNC(logger, ...) \
    LOG_ASYNC(logger, Boardcore::LogLevel::LOGL_DEBUG, __VA_ARGS__)

#define LOG_INFO_ASYNC(logger, ...) \
    LOG_ASYNC(logger, Boardcore::LogLevel::LOGL_INFO, __VA_ARGS__)

#define LOG_WARN_ASYNC(logger, ...) \
    LOG_ASYNC(logger, Boardcore::LogLevel::LOGL_WARNING, __VA_ARGS__)

#define LOG_ERR_ASYNC(logger, ...) \
    LOG_ASYNC(logger, Boardcore::LogLevel::LOGL_ERROR, __VA_ARGS__)

#define LOG_CRIT_ASYNC(logger, ...) \
    LOG_ASYNC(logger, Boardcore::LogLevel::LOGL_CRITICAL, __VA_ARGS__)
