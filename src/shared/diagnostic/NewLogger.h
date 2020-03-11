/* Copyright (c) 2015-2017 Skyward Experimental Rocketry
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

#ifndef SRC_SHARED_DIAGNOSTIC_NEWLOGGER_H_
#define SRC_SHARED_DIAGNOSTIC_NEWLOGGER_H_

#include <iostream>
#include <string>
#include <sstream>
#include <ctime>
#include <iomanip>

namespace logging
{

enum class LogLevel : int
 {
  INFO = 0,
  NOTIFY = 1,
  WARNING = 2,
  ERROR = 3,
  CRITICAL = 4
};

template <typename T>
std::string hex(T val)
{
  std::stringstream stream;
  stream << std::hex << val;
  std::string result( stream.str() );
  return result;
}

class Logger
{
 public:
  Logger() : out_stream_(&std::cout)
  {

  }

  Logger(std::ostream* ostream) : out_stream_(ostream)
  {

  }

  void setOstream(std::ostream* new_ostream)
  {
    out_stream_ = new_ostream;
  }

  template<typename ... Args>
  void log(LogLevel level, std::string tag, Args ...args) const
  {
    if (level < log_level_)
      return;

    std::stringstream stream;
    stream << getStringPrefix(level) << formatTag(tag) << " ";
    _printOnSStream(&stream, args...);
    *out_stream_ << stream.str() << std::endl;
  }

  template<typename ... Args>
  void info(std::string tag, Args ... args) const
  {
    log(LogLevel::INFO, tag, args...);
  }

  template<typename ... Args>
  void notify(std::string tag, Args ... args) const
  {
    log(LogLevel::NOTIFY, tag, args...);
  }

  template<typename ... Args>
  void warning(std::string tag, Args ... args) const
  {
    log(LogLevel::WARNING, tag, args...);
  }

  template<typename ... Args>
  void error(std::string tag, Args ... args) const
  {
    log(LogLevel::ERROR, tag, args...);
  }

  template<typename ... Args>
  void critical(std::string tag, Args ... args) const
  {
    log(LogLevel::CRITICAL, tag, args...);
  }

 private:

  std::string getStringPrefix(LogLevel level) const
  {
    //Text is colored and formatted using Ansi Escape Codes:
    //https://en.wikipedia.org/wiki/ANSI_escape_code

    switch (level)
    {
      case LogLevel::INFO:
        return "\033[37m[INFO]\033[0m     ";
        break;
      case LogLevel::NOTIFY:
        return "\033[32m[NOTIFY]\033[0m   ";
        break;
      case LogLevel::WARNING:
        return "\033[33m[WARNING]\033[0m  ";
        break;
      case LogLevel::ERROR:
        return "\033[1;31m[ERROR]\033[0m    ";
        break;
      case LogLevel::CRITICAL:
        return "\033[1;41;37m[CRITICAL]\033[0m ";
        break;
    }
    return "";
  }

  static std::string formatTag(std::string tag)
  {
    static const unsigned int maxlen = 14;
    if(tag.length() > maxlen)
      tag = tag.substr(0, maxlen);

    tag + tag + ": ";

    //Add trailing whitespace
    std::string whitespace(maxlen - tag.length() + 2, ' ');
    tag = tag + whitespace;

    return tag;
  }

  void _printOnSStream(std::stringstream*) const
  {
  }

  template<typename FirstArg, typename ... Args>
  void _printOnSStream(std::stringstream* stream, const FirstArg& arg,
                       Args ...args) const
  {
    *stream << arg;
    _printOnSStream(stream, args...);
  }

  LogLevel log_level_ = LogLevel::INFO;
  std::ostream* out_stream_;
};

//Default logger on cout
static const Logger logger;

}   //namespace logger

#endif /* SRC_SHARED_DIAGNOSTIC_NEWLOGGER_H_ */
