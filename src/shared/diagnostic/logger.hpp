/* Logger
 * Inspired by http://www.drdobbs.com/cpp/a-lightweight-logger-for-c/240147505.
 *
 * Copyright (c) 2016 Skyward Experimental Rocketry
 * Authors: Gabriele Farina
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

#ifndef LOGGER_HPP
#define LOGGER_HPP

#include <iostream>
#include <string>
#include <sstream>
#include <ctime>
#include <iomanip>

namespace logging {

class LogLevel {
 public:
  LogLevel(int numeric_value, const std::string& string_value) :
      _numeric_value(numeric_value), _string_value(string_value) {}

  int getNumericValue() const {
    return _numeric_value;
  }

  const std::string& getStringValue() const {
    return _string_value;
  }

  bool operator<(const LogLevel& ref) const {
    return _numeric_value < ref._numeric_value;
  }

 private:
  int _numeric_value;
  std::string _string_value;
};

// Define some sugary constants
auto INFO = LogLevel(1, "INFO");
auto WARNING = LogLevel(2, "WARN");
auto ERROR = LogLevel(3, "ERR!");
auto CRITICAL = LogLevel(4, "CRIT");

class Logger {
 public:
  void setLogLevel(const LogLevel& new_level) {
    _log_level = &new_level;
  }
  void setOstream(std::ostream* new_ostream) {
    _out_stream = new_ostream;
  }

  // Print function. Use it like this:
  //   .print<logging::WARNING>(arg1, arg2, arg3)
  template <const LogLevel& level, typename... Args>
  void log(Args...args) {
    if (level < *_log_level)
      return;

    std::stringstream stream;

    stream << "[" << level.getStringValue() << "] ";
    _printOnSStream(&stream, args...);
    *_out_stream << stream.str() << std::endl;
  }

 private:
  void _printOnSStream(std::stringstream*) {}

  template <typename FirstArg, typename... Args>
  void _printOnSStream(std::stringstream* stream,
                       const FirstArg& arg, Args...args) {
    *stream << arg;
    _printOnSStream(stream, args...);
  }

  std::ostream* _out_stream = &std::cout;
  const LogLevel* _log_level = &INFO;
};

}  // namespace logging

#endif  // LOGGER_HPP
