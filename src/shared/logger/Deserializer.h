/* Copyright (c) 2015-2018 Skyward Experimental Rocketry
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

#include <sys/stat.h>
#include <tscpp/stream.h>

#include <cstdio>
#include <fstream>
#include <limits>
#include <ostream>
#include <string>
#include <vector>

using std::ostream;
using std::string;
using std::vector;
using tscpp::TypePoolStream;
using tscpp::UnknownInputArchive;

namespace Boardcore
{

typedef std::numeric_limits<float> flt;

// linter off

/**
 * Class used to deserialize log files created using fedetft's logger.
 */
class Deserializer
{
public:
    Deserializer(std::string logfile, std::string prefix = "")
        : prefix(prefix), logFile(logfile),
          logFileWithExt(prefix + logFile + ".dat")
    {
    }

    ~Deserializer()
    {
        if (!closed)
        {
            for (auto it = fileStreams.begin(); it != fileStreams.end(); it++)
            {
                (*it)->close();
                delete *it;
            }
        }
    }

    /**
     * Register a type to be deserialized, and the associated print function
     *
     * @param t the object to be deserialized
     * @param fncPrint function that prints the deserialized data on the
     * provided output stream.
     */
    template <typename T>
    bool registerType(std::function<void(T& t, std::ostream& os)> fncPrint,
                      std::string header = "")
    {
        if (closed)
        {
            printf("Error: Deserializer is closed.\n");
            return false;
        }

        char cFilename[128];
        sprintf(cFilename, "%s%s_%s.csv", prefix.c_str(), logFile.c_str(),
                typeid(T).name());

        std::string filename(cFilename);

        std::ofstream* stream = new std::ofstream();
        stream->open(filename);

        if (!stream->is_open())
        {
            printf("Error opening file %s.\n", filename.c_str());
            perror("Error is:");
            delete stream;
            return false;
        }

        fileStreams.push_back(stream);
        stream->precision(flt::maxDigits10);  // Set stream precision to
                                              // maximum float precision
        // Print the header
        if (header.length() > 0)
        {
            *stream << header;
        }

        using namespace std::placeholders;  // for _1

        std::function<void(T & t)> callback =
            std::bind(fncPrint, _1, std::ref(*stream));

        tps.registerType<T>(callback);

        return true;
    }

    /**
     * @brief Deserializes the provided file.
     *
     * @return Wheter the deserialization was successful.
     */
    bool deserialize()
    {
        if (closed)
        {
            return false;
        }

        bool success = true;
        std::string unknownTypeName;

        struct stat st;
        if (stat(logFileWithExt.c_str(), &st) != 0)
        {
            printf("File %s does not exists.\n", logFileWithExt.c_str());
            return false;
        }

        std::ifstream file(logFileWithExt);
        // file.open;
        tscpp::UnknownInputArchive ia(file, tps);
        int i = 0;
        while (success)
        {
            try
            {
                ia.unserialize();
            }
            catch (tscpp::TscppException& ex)
            {
                // Reached end of file
                if (strcmp(ex.what(), "eof") == 0)
                {
                    break;
                }
                else if (strcmp(ex.what(), "unknown type") == 0)
                {
                    unknownTypeName = ex.name();
                    success         = false;
                    printf("Unknown type found: %s\n", unknownTypeName.c_str());
                    break;
                }
            }
        }
        file.close();
        return success;
    }

    void close()
    {
        if (!closed)
        {
            closed = true;
            for (auto it = fileStreams.begin(); it != fileStreams.end(); it++)
            {
                (*it)->close();
                delete *it;
            }
        }
    }

private:
    bool closed = false;

    std::vector<std::ofstream*> fileStreams;
    tscpp::TypePoolStream tps;

    std::string prefix;
    std::string logFile;
    std::string logFileWithExt;
};

}  // namespace Boardcore
