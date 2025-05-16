/* Copyright (c) 2015-2025 Skyward Experimental Rocketry
 * Authors: Luca Erbetta, Alberto Nidasio, Pietro Bortolus
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
#include <map>
#include <ostream>
#include <regex>
#include <sstream>
#include <stdexcept>
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
typedef float float32_t;
typedef double float64_t;

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

    /**
     * @brief Register all types from the mapping file.
     *
     * @param mapping The mapping file stream
     */
    template <typename F>
    void registerTypes(F&& f);

    bool closed = false;

    std::map<std::string, std::ofstream*> fileStreams;
    std::map<std::string, std::function<void(std::ifstream&)>> types;

    tscpp::TypePoolStream tps;

    std::string logFilename;
    std::string logFilenameWithoutExtension;
    std::string logFolderPath;
    std::string mappingFileName;
};

Deserializer::Deserializer(std::string logFilename) : logFilename(logFilename)
{
    // Prepare the folder path
    logFilenameWithoutExtension =
        logFilename.substr(0, logFilename.find_last_of("."));
    logFolderPath = logFilenameWithoutExtension + "/";

    // prepare the mapping file name
    mappingFileName = logFolderPath + "mapping" +
                      logFilenameWithoutExtension.erase(0, 3) + ".dat";
}

Deserializer::~Deserializer() { close(); }

template <typename T>
void Deserializer::printType(T& t, std::string path, std::string prefix)
{
    static std::ofstream* stream;
    std::string demangledTypeName = tscpp::demangle(typeid(T).name());

    // Replace the :: with the _ in order to make the format string cross
    // platform compatible
    demangledTypeName =
        std::regex_replace(demangledTypeName, std::regex("::"), "_");

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

/**
 * @brief Register the types from the mapping file
 *
 * @param mapping The mapping file stream
 */
template <typename F>
void registerTypes(F&& f)
{
    // open the mapping file
    std::ifstream mapping(mappingFileName);

    // iterate over all of the types' descriptions
    while (mapping.peek() != -1)
    {
        // get the name of the type
        std::string type_name;
        // Read null terminated name string
        while (mapping.peek() != 0)
            type_name += mapping.get();
        // Dummy null read
        mapping.get();

        // read the structure of the type and save it

        // store the number of fields
        uint8_t field_count;
        mapping.read((char*)&field_count, sizeof field_count);

        // Dummy null read
        mapping.get();

        // create an array to store the field types
        std::vector<char> field_types(field_count);

        for (int i = 0; i < field_count; i++)
        {
            // get the name of the field, we don't actually need it for this
            // part but we need to read it to get the type
            std::string field_name;
            // Read null terminated name string
            while (mapping.peek() != 0)
                field_name += mapping.get();
            // Dummy null read
            mapping.get();

            // get the type of the field
            std::string field_type;
            // Read null terminated name string
            while (mapping.peek() != 0)
                field_type += mapping.get();
            // Dummy null read
            mapping.get();

            // add the field type to field_types
            field_types[i] = field_type[0];
        }

        // register a callback for this type that takes field_types as an
        // argument and prints them to std::cout after having converted the
        // value read from the input stream into the correct type
        types.insert({type_name, [f = std::move(f), field_types,
                                  field_count](std::istream& is)
                      {
                          for (int i = 0; i < field_count; i++)
                          {
                              std::ostringstream
                                  oss;  // Create a string stream for each field

                              // Read the value from the input stream and
                              // store it as the correct type based on the
                              // strcutre described in field_types
                              switch (field_types[i])
                              {
                                  case 'a':
                                      int8_t int8;
                                      is.read((char*)&int8, sizeof int8);
                                      // The cast is necessary because int8_t
                                      // could be interpreted as char
                                      // otherwhise
                                      oss << static_cast<int>(int8);
                                      break;
                                  case 's':
                                      int16_t int16;
                                      is.read((char*)&int16, sizeof int16);
                                      oss << int16;
                                      break;
                                  case 'i':
                                      int32_t int32;
                                      is.read((char*)&int32, sizeof int32);
                                      oss << int32;
                                      break;
                                  case 'l':
                                      int64_t int64;
                                      is.read((char*)&int64, sizeof int64);
                                      oss << int64;
                                      break;
                                  case 'h':
                                      uint8_t uint8;
                                      is.read((char*)&uint8, sizeof uint8);
                                      // The cast is necessary because C++
                                      // sucks
                                      oss << static_cast<unsigned int>(uint8);
                                      break;
                                  case 't':
                                      uint16_t uint16;
                                      is.read((char*)&uint16, sizeof uint16);
                                      oss << uint16;
                                      break;
                                  case 'j':
                                      uint32_t uint32;
                                      is.read((char*)&uint32, sizeof uint32);
                                      oss << uint32;
                                      break;
                                  case 'm':
                                      uint64_t uint64;
                                      is.read((char*)&uint64, sizeof uint64);
                                      oss << uint64;
                                      break;
                                  case 'f':
                                      float32_t float32;
                                      is.read((char*)&float32, sizeof float32);
                                      oss << float32;
                                      break;
                                  case 'd':
                                      float64_t float64;
                                      is.read((char*)&float64, sizeof float64);
                                      oss << float64;
                                      break;
                                  default:
                                      std::cerr
                                          << "Unknown type: " << field_types[i]
                                          << std::endl
                                          << "Aborting deserialization"
                                          << std::endl;
                                      throw std::runtime_error(
                                          "Unknown typeID: " +
                                          std::string(1, field_types[i]));
                                      return;
                              }
                              // Print the converted string
                              f(oss);
                          }
                      }});
        std::cout << "Registered type: " << type_name << std::endl;
    }
    mapping.close();
    return;
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

    while (file.peek() != -1)
    {
        std::string type_name;

        // Read null terminated name string
        while (file.peek() != 0)
            type_name += file.get();
        // Dummy null read
        file.get();

        try
        {
            // Invoke correct parser for this type
            std::cout << "Deserializing type: " << type_name << std::endl;
            (types.at(type_name))(file);
        }
        catch (const std::out_of_range& e)
        {
            std::cerr << "Type not found: " << type_name << std::endl;
            return false;
        }
        catch (std::runtime_error& e)
        {
            std::cerr << "Error deserializing type: " << type_name << " - "
                      << e.what() << std::endl;
            return false;
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
