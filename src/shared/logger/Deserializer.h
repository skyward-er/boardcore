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

#include <fstream>
#include <iostream>
#include <limits>
#include <ostream>
#include <regex>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <vector>

namespace Boardcore
{

using float32_t = float;
using float64_t = double;

namespace detail
{
template <typename T>
void deserializeField(std::ifstream& in, std::ofstream& out)
{
    auto value = T{};
    in.read(reinterpret_cast<char*>(&value), sizeof(T));
    out << value << ',';
}

template <>
void deserializeField<int8_t>(std::ifstream& in, std::ofstream& out)
{
    using T    = int8_t;
    auto value = T{};
    in.read(reinterpret_cast<char*>(&value), sizeof(T));
    out << static_cast<int>(value) << ',';
}

template <>
void deserializeField<uint8_t>(std::ifstream& in, std::ofstream& out)
{
    using T    = uint8_t;
    auto value = T{};
    in.read(reinterpret_cast<char*>(&value), sizeof(T));
    out << static_cast<unsigned int>(value) << ',';
}
}  // namespace detail

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
    using FieldDeserializer =
        std::function<void(std::ifstream&, std::ofstream&)>;

    using DeserializeInstructions = std::vector<FieldDeserializer>;

    /**
     * @brief Creates a CSV file for the given type name.
     *
     * This method generates a CSV file in the log folder path using the
     * provided type name. It replaces "::" in the type name with "_" to ensure
     * cross-platform compatibility. The created file stream is stored in the
     * fileStreams map for later use and proper cleanup.
     *
     * @param typeName The name of the type for which the CSV file is created.
     * @return A pointer to the created std::ofstream object.
     * @throws std::runtime_error If the file cannot be opened.
     */
    std::ofstream* createCSV(const std::string& typeName);

    /**
     * @brief Registers the types and their deserialization instructions from
     * the mapping file.
     *
     * This method reads the mapping file to extract type names, field names,
     * and field types. It creates corresponding CSV files for each type and
     * stores deserialization instructions for each field in the `types` map.
     * Throws an exception if the mapping file is missing or contains invalid
     * data.
     */
    void registerTypes();

    bool closed = false;

    std::unordered_map<std::string, std::ofstream*> fileStreams;
    std::unordered_map<std::string, DeserializeInstructions> types;

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
    mappingFileName =
        "mapping" + logFilenameWithoutExtension.erase(0, 3) + ".dat";
}

Deserializer::~Deserializer() { close(); }

std::ofstream* Deserializer::createCSV(const std::string& typeName)
{
    // Replace the :: with the _ in order to make the format string cross
    // platform compatible
    std::string demangledTypeName =
        std::regex_replace(typeName, std::regex("::"), "_");

    std::string filename = logFolderPath + demangledTypeName + ".csv";

    std::ofstream* stream = new std::ofstream(filename);

    if (!stream->is_open())
    {
        std::cerr << "Error opening file " << filename.c_str() << std::endl;
        throw std::runtime_error("Error opening file " + filename);
    }

    stream->precision(std::numeric_limits<float>::max_digits10);

    // Add the file to the vector such that it will be closed
    fileStreams.emplace(typeName, stream);

    return stream;
}

/**
 * @brief Register the types from the mapping file
 */
void Deserializer::registerTypes()
{
    // open the mapping file
    std::ifstream mapping(logFolderPath + mappingFileName);

    if (!mapping)
    {
        std::cerr << logFolderPath + mappingFileName + " does not exists."
                  << std::endl;

        throw std::runtime_error(logFolderPath + mappingFileName +
                                 " does not exists.");
    }

    // iterate over all of the type mappings
    while (mapping.peek() != -1)
    {
        // get the name of the type
        std::string typeName;
        // Read null terminated name string
        while (mapping.peek() != 0)
            typeName += mapping.get();
        // Dummy null read
        mapping.get();

        // Create a new csv file for this type
        std::ofstream* outFile = createCSV(typeName);

        // Read the number of fields
        uint8_t fieldCount = mapping.get();
        // Dummy null read
        mapping.get();

        // create an array to store the field types
        DeserializeInstructions deserializeInstructions;
        std::string header;

        for (int i = 0; i < fieldCount; i++)
        {
            // get the name of the field
            std::string fieldName;
            // Read null terminated name string
            while (mapping.peek() != 0)
                fieldName += mapping.get();
            // Dummy null read
            mapping.get();
            header += fieldName + ",";

            // Read the field type
            char fieldType = mapping.get();
            // Dummy null read
            mapping.get();

            switch (fieldType)
            {
                case 'b':
                    deserializeInstructions.push_back(
                        detail::deserializeField<bool>);
                    break;
                case 'c':
                    deserializeInstructions.push_back(
                        detail::deserializeField<char>);
                    break;
                case 'h':
                    deserializeInstructions.push_back(
                        detail::deserializeField<int8_t>);
                    break;
                case 'i':
                    deserializeInstructions.push_back(
                        detail::deserializeField<int16_t>);
                    break;
                case 'j':
                    deserializeInstructions.push_back(
                        detail::deserializeField<int32_t>);
                    break;
                case 'l':
                    deserializeInstructions.push_back(
                        detail::deserializeField<int64_t>);
                    break;
                case 'H':
                    deserializeInstructions.push_back(
                        detail::deserializeField<uint8_t>);
                    break;
                case 'I':
                    deserializeInstructions.push_back(
                        detail::deserializeField<uint16_t>);
                    break;
                case 'J':
                    deserializeInstructions.push_back(
                        detail::deserializeField<uint32_t>);
                    break;
                case 'L':
                    deserializeInstructions.push_back(
                        detail::deserializeField<uint64_t>);
                    break;
                case 'f':
                    deserializeInstructions.push_back(
                        detail::deserializeField<float32_t>);
                    break;
                case 'd':
                    deserializeInstructions.push_back(
                        detail::deserializeField<float64_t>);
                    break;
                default:
                    std::cerr << "Unknown field type: " << fieldType
                              << std::endl
                              << "Aborting deserialization" << std::endl;
                    throw std::runtime_error("Unknown field type: " +
                                             std::to_string(fieldType));
            }
        }

        // print the header to the CSV and remove the trailing comma
        *outFile << header.substr(0, header.size() - 2) << '\n';

        auto result =
            types.insert(std::make_pair(typeName, deserializeInstructions));

        if (!result.second)
        {
            std::cerr << "Duplicate type: " << typeName << std::endl;
            throw std::runtime_error("Duplicate type: " + typeName);
        }

        std::cout << "Registered " << typeName << " with " << (int)fieldCount
                  << " fields" << std::endl;
    }

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

    // Move the mapping file into the folder
    if (rename(mappingFileName.c_str(),
               (logFolderPath + mappingFileName).c_str()))
    {
        std::cout << mappingFileName + " does not exists." << std::endl;
        return false;
    }

    registerTypes();

    // Open the log and the mapping file
    std::ifstream file(logFolderPath + logFilename);

    // Check if the files exist
    if (!file)
    {
        std::cout << logFolderPath + logFilename + " does not exists."
                  << std::endl;
        return false;
    }

    while (file.peek() != -1)
    {
        std::string typeName;

        // Read null terminated name string
        while (file.peek() != 0)
            typeName += file.get();
        // Dummy null read
        file.get();

        try
        {
            // Invoke correct parser for this type

            auto deserInstructions = types.at(typeName);
            auto& outFile          = *fileStreams.at(typeName);

            for (auto& deserFunction : deserInstructions)
                deserFunction(file, outFile);

            // replace the trailing comma with a newline character
            outFile.seekp(outFile.tellp() - std::streampos{1});
            outFile << '\n';
        }
        catch (const std::out_of_range& e)
        {
            std::cerr << "Type not found: " << typeName << std::endl;
            return false;
        }
        catch (std::runtime_error& e)
        {
            std::cerr << "Error deserializing type: " << typeName << " - "
                      << e.what() << std::endl;
            return false;
        }
    }

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
