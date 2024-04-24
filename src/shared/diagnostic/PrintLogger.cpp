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

#include "PrintLogger.h"

#include <utils/KernelTime.h>

using miosix::Lock;

namespace Boardcore
{

static string getLevelString(uint8_t level)
{
    switch (level)
    {
        case LOGL_DEBUG:
            return "DEBUG";
        case LOGL_INFO:
            return "INFO";
        case LOGL_WARNING:
            return "WARNING";
        case LOGL_ERROR:
            return "ERROR";
        case LOGL_CRITICAL:
            return "CRITICAL";
        default:
            return std::to_string(level);
    }
}

static string truncateFileName(const string& name, int depth = 0)
{
    // Find the first separator
    auto start = name.find_last_of("\\/");

    // Now traverse the path until we reach the end or the required depth
    for (int i = 0; i < depth && start != string::npos; i++)
    {
        start = name.find_last_of("\\/", start - 1);
    }

    // Truncate the path if needed
    return start == string::npos
               ? name
               : "..." + string{name.cbegin() + start, name.cend()};
}

void LogSink::log(const LogRecord& record)
{
    using namespace fmt::literals;
    if (record.level >= minimumLevel)
    {
        float ts     = Kernel::getOldTick() / 1000.0f;
        int min      = ts / 60;
        string tsStr = fmt::format("{:02d}:{:06.3f}", min, (ts - min * 60));

        logImpl(fmt::format(
            format, "ts"_a = tsStr, "file"_a = truncateFileName(record.file, 1),
            "line"_a =  // cppcheck-suppress AssignmentIntegerToAddress
            record.line,
            "fun"_a = record.function, "lvl"_a = getLevelString(record.level),
            "name"_a = record.name, "msg"_a = record.message));
    }
}

void FileLogSink::logImpl(const string& l)
{
    Lock<FastMutex> lock(mutex);
    fwrite(l.c_str(), sizeof(char), l.length(), f);
}

void FileLogSinkBuffered::logImpl(const string& l)
{
    Lock<FastMutex> lock(mutex);
    LoggingString s;
    strncpy(s.logString, l.c_str(), MAX_LOG_STRING_SIZE - 1);
    logger.log(s);
}

PrintLogger PrintLogger::getChild(const string& name)
{
    return PrintLogger(parent, this->name + "." + name);
}

LogRecord PrintLogger::buildLogRecord(uint8_t level, const string& function,
                                      const string& file, int line,
                                      fmt::string_view format,
                                      fmt::format_args args)
{
    LogRecord record;
    record.level    = level;
    record.function = function;
    record.file     = file;
    record.line     = line;
    record.name     = name;
    try
    {
        record.message = fmt::vformat(format, args);
    }
    catch (const std::exception& e)
    {
        record.level   = LOGL_ERROR;
        record.message = "FMT Formatting error! " + string(e.what());
    }

    return record;
}

void PrintLogger::vlog(uint8_t level, const string& function,
                       const string& file, int line, fmt::string_view format,
                       fmt::format_args args)
{
    parent.log(buildLogRecord(level, function, file, line, format, args));
}

void PrintLogger::vlogAsync(uint8_t level, const string& function,
                            const string& file, int line,
                            fmt::string_view format, fmt::format_args args)
{
    parent.logAsync(buildLogRecord(level, function, file, line, format, args));
}

void Logging::log(const LogRecord& record)
{
    for (auto& s : sinks)
    {
        if (s->isEnabled())
        {
            s->log(record);
        }
    }
}

void Logging::logAsync(const LogRecord& record) { asyncLog.log(record); }

Logging::AsyncLogger::AsyncLogger(Logging& parent) : parent(parent) {}

void Logging::AsyncLogger::log(const LogRecord& record)
{
    {
        Lock<FastMutex> l(mutex);
        records.put(record);
    }

    cv.signal();
}

void Logging::AsyncLogger::run()
{
    while (!shouldStop())
    {
        LogRecord rec;
        {
            Lock<FastMutex> l(mutex);
            while (records.isEmpty())
            {
                cv.wait(mutex);
            }

            rec = records.pop();
        }

        parent.log(rec);
    }
}

}  // namespace Boardcore
