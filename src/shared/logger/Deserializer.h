/* Copyright (c) 2015-2022 Skyward Experimental Rocketry
 * Authors: Luca Erbetta, Alberto Nidasio
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
 * @brief Class used to deserialize the binary logs created using fedetft's
 * logger into csv files.
 */
class Deserializer
{
public:
    /**
     * @brief Initializes the deserializer with the filename provided.
     */
    Deserializer(std::string fileName);

    ~Deserializer();

    /**
     * @brief Register a type to be deserialized.
     *
     * Node: The object type must provide a static header function and a print
     * function with the following prototypes:
     * static std::string header()
     * void print(std::ostream& os) const
     *
     * @tparam T The object type to be deserialized.
     */
    template <typename T>
    void registerType();

    /**
     * @brief Deserializes the provided file.
     *
     * @return Whether the deserialization was successful.
     */
    bool deserialize();

    /**
     * @brief Closes all the openend files.
     */
    void close();

private:
    /**
     * @brief Function to print a data element into its csv file.
     *
     * @tparam T Type of the object.
     * @param t Object to be printed in the file.
     * @param path Path where to save the file.
     * @param prefix Prefix added to the file.
     */
    template <typename T>
    void printType(T& t, std::string path = "", std::string prefix = "");

    bool closed = false;

    std::map<std::string, std::ofstream*> fileStreams;
    tscpp::TypePoolStream tps;

    std::string logFilename;
    std::string logFilenameWithoutExtension;
    std::string logFolderPath;
};

Deserializer::Deserializer(std::string logFilename) : logFilename(logFilename)
{
    // Prepare the folder path
    logFilenameWithoutExtension =
        logFilename.substr(0, logFilename.find_last_of("."));
    logFolderPath = logFilenameWithoutExtension + "/";
}

Deserializer::~Deserializer() { close(); }

template <typename T>
void Deserializer::registerType()
{
    std::function<void(T & t)> callback =
        std::bind(&Deserializer::printType<T>, this, std::placeholders::_1,
                  logFolderPath, logFilenameWithoutExtension);

    tps.registerType<T>(callback);
}

template <typename T>
void Deserializer::printType(T& t, std::string path, std::string prefix)
{
    std::string demangledTypeName = tscpp::demangle(typeid(T).name());
    static std::ofstream* stream;

    try
    {
        stream = fileStreams.at(demangledTypeName);
    }
    // If not already initialize, open the file and write the header
    catch (std::out_of_range e)
    {
        stream               = new std::ofstream();
        std::string filename = path + prefix + "_" + demangledTypeName + ".csv";
        std::cout << "Creating file " + filename << std::endl;
        stream->open(filename);

        if (!stream->is_open())
        {
            printf("Error opening file %s.\n", filename.c_str());
            perror("Error is:");
            return;
        }

        // Print the header in the file
        *stream << T::header();

        // Set stream precision to maximum float precision
        stream->precision(flt::max_digits10);

        // Add the file to the vector such that it will be closed
        fileStreams.emplace(demangledTypeName, stream);
    }

    // Print data into the file if it is open
    if (stream->is_open())
        t.print(std::ref(*stream));
}

bool Deserializer::deserialize()
{
    if (closed)
        return false;

    // Create the folder
    mkdir(logFolderPath.c_str(), 0777);

    // Move the log file into the folder
    if (rename(logFilename.c_str(), (logFolderPath + logFilename).c_str()))
    {
        std::cout << logFilename + " does not exists." << std::endl;
        return false;
    }

    // Open the log file
    std::ifstream file(logFolderPath + logFilename);

    // Check if the file exists
    if (!file)
    {
        std::cout << logFolderPath + logFilename + " does not exists."
                  << std::endl;
        return false;
    }

    tscpp::UnknownInputArchive inputArchive(file, tps);
    while (true)
    {
        try
        {
            inputArchive.unserialize();
        }
        catch (tscpp::TscppException& ex)
        {
            // Reached end of file
            if (strcmp(ex.what(), "eof") == 0)
            {
                return true;
            }
            // Unknown type found
            else if (strcmp(ex.what(), "unknown type") == 0)
            {
                std::string unknownTypeName = ex.name();
                std::cout << "Unknown type found: " << unknownTypeName
                          << std::endl;
                return false;
            }
        }
    }

    file.close();
    return true;
}

void Deserializer::close()
{
    if (!closed)
    {
        closed = true;
        for (auto it = fileStreams.begin(); it != fileStreams.end(); it++)
        {
            it->second->close();
            delete it->second;
        }
    }
}

}  // namespace Boardcore
