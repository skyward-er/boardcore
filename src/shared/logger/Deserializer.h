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
#include <iostream>
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
    Deserializer(std::string fileName);

    ~Deserializer();

    /**
     * Register a type to be deserialized, and the associated print function.
     *
     * @param t The object to be deserialized.
     * @param fncPrint Function that prints the deserialized data on the
     * provided output stream.
     * @param header Optional CSV header text.
     */
    template <typename T>
    bool registerType(std::function<void(T& t, std::ostream& os)> fncPrint,
                      std::string header = "");

    /**
     * @brief Deserializes the provided file.
     *
     * @return Whether the deserialization was successful.
     */
    bool deserialize();

    void close();

private:
    bool closed = false;

    std::vector<std::ofstream*> fileStreams;
    tscpp::TypePoolStream tps;

    std::string fileName;
};

Deserializer::Deserializer(std::string fileName) : fileName(fileName) {}

Deserializer::~Deserializer()
{
    if (!closed)
        for (auto it = fileStreams.begin(); it != fileStreams.end(); it++)
        {
            (*it)->close();
            delete *it;
        }
}

template <typename T>
bool Deserializer::registerType(
    std::function<void(T& t, std::ostream& os)> fncPrint, std::string header)
{
    if (closed)
    {
        printf("Error: Deserializer is closed.\n");
        return false;
    }

    char cFilename[128];
    sprintf(cFilename, "%s_%s.csv", fileName.c_str(), typeid(T).name());

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
    stream->precision(flt::max_digits10);  // Set stream precision to
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

bool Deserializer::deserialize()
{
    if (closed)
        return false;

    bool success = true;
    std::string unknownTypeName;

    std::ifstream file(fileName);

    // Check if the file exists
    if (!file)
    {
        std::cout << fileName << " does not exists." << std::endl;
        return false;
    }

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
                std::cout << "Unknown type found: " << unknownTypeName
                          << std::endl;
                break;
            }
        }
    }

    file.close();
    return success;
}

void Deserializer::close()
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

}  // namespace Boardcore
