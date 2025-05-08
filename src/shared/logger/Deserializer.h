/* Copyright (c) 2015-2025 Skyward Experimental Rocketry
 * Authors: Luca Erbetta, Alberto Nidasio, Pietro Bortolus, Niccolò Betto
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

#include <filesystem>
#include <fstream>
#include <iostream>
#include <limits>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <vector>

#include "Logger.h"

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
    Deserializer(std::filesystem::path fileName);

    /**
     * @brief Returns the path to the directory where the log will be decoded.
     */
    std::filesystem::path outputDirectory() const
    {
        return filename.parent_path() / filename.stem();
    }

    /**
     * @brief Deserializes the provided file.
     *
     * @return Whether the deserialization was successful.
     */
    bool deserialize();

private:
    using FieldDeserializer =
        std::function<void(std::ifstream&, std::ofstream&)>;

    using DeserializeInstructions = std::vector<FieldDeserializer>;

    /**
     * @brief Creates a CSV file for the given type name.
     *
     * This method generates a CSV file in the log folder path using the
     * provided type name.
     *
     * @param typeName The name of the type for which the CSV file is created.
     * @return The output stream of the file.
     * @throws std::runtime_error If the file cannot be opened.
     */
    std::ofstream createCSV(const std::string& typeName);

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
    void registerType(std::string typeName, std::ifstream& file);

    struct MappingRecord
    {
        std::ofstream file;
        DeserializeInstructions instructions;
    };

    std::unordered_map<std::string, MappingRecord> typeMap;

    std::filesystem::path filename;
};

Deserializer::Deserializer(std::filesystem::path filename)
    : filename(std::move(filename))
{
}

std::ofstream Deserializer::createCSV(const std::string& typeName)
{
    auto outputFile = outputDirectory() / (typeName + ".csv");
    std::ofstream stream(outputFile);

    if (!stream)
    {
        std::cerr << "Error opening file " << outputFile << std::endl;
        throw std::runtime_error("Error opening file " + outputFile.string());
    }

    stream.precision(std::numeric_limits<float>::max_digits10);

    return stream;
}

/**
 * @brief Register a type so that it can be deserialized
 */
void Deserializer::registerType(std::string typeName, std::ifstream& file)
{
    // Create a new csv file for this type
    std::ofstream outFile = createCSV(typeName);

    // Read the number of fields
    uint8_t fieldCount = file.get();
    // Dummy null read
    file.get();

    // create an array to store the field types
    DeserializeInstructions deserializeInstructions;
    deserializeInstructions.reserve(fieldCount);
    std::string header;

    for (int i = 0; i < fieldCount; i++)
    {
        // get the name of the field
        std::string fieldName;
        // Read null terminated name string
        while (file.peek() != 0)
            fieldName += file.get();
        // Dummy null read
        file.get();
        header += fieldName + ",";

        // Read the field type
        char fieldType = file.get();
        // Dummy null read
        file.get();

        switch (fieldType)
        {
            case TypeIDByte::Bool:
                deserializeInstructions.push_back(
                    detail::deserializeField<bool>);
                break;
            case TypeIDByte::Char:
                deserializeInstructions.push_back(
                    detail::deserializeField<char>);
                break;
            case TypeIDByte::Int8:
                deserializeInstructions.push_back(
                    detail::deserializeField<int8_t>);
                break;
            case TypeIDByte::UInt8:
                deserializeInstructions.push_back(
                    detail::deserializeField<uint8_t>);
                break;
            case TypeIDByte::Int16:
                deserializeInstructions.push_back(
                    detail::deserializeField<int16_t>);
                break;
            case TypeIDByte::UInt16:
                deserializeInstructions.push_back(
                    detail::deserializeField<uint16_t>);
                break;
            case TypeIDByte::Int32:
                deserializeInstructions.push_back(
                    detail::deserializeField<int32_t>);
                break;
            case TypeIDByte::UInt32:
                deserializeInstructions.push_back(
                    detail::deserializeField<uint32_t>);
                break;
            case TypeIDByte::Int64:
                deserializeInstructions.push_back(
                    detail::deserializeField<int64_t>);
                break;
            case TypeIDByte::UInt64:
                deserializeInstructions.push_back(
                    detail::deserializeField<uint64_t>);
                break;
            case TypeIDByte::Float:
                deserializeInstructions.push_back(
                    detail::deserializeField<float32_t>);
                break;
            case TypeIDByte::Double:
                deserializeInstructions.push_back(
                    detail::deserializeField<float64_t>);
                break;
            default:
                std::cerr << "Unknown field type: " << fieldType << std::endl
                          << "Aborting deserialization" << std::endl;
                throw std::runtime_error("Unknown field type: " +
                                         std::to_string(fieldType));
        }
    }

    // print the header to the CSV and remove the trailing comma
    outFile << header.substr(0, header.size() - 1) << '\n';

    auto result = typeMap.emplace(
        typeName,
        MappingRecord{std::move(outFile), std::move(deserializeInstructions)});

    if (!result.second)
    {
        std::cerr << "Duplicate type: " << typeName << std::endl;
        throw std::runtime_error("Duplicate type: " + typeName);
    }

    // Indent to reduce output clutter
    std::cout << "\tFound type " << typeName << " with " << (int)fieldCount
              << " fields" << std::endl;

    return;
}

bool Deserializer::deserialize()
{
    std::ifstream file(filename, std::ios::binary);
    if (!file)
    {
        std::cerr << filename << " does not exists." << std::endl;
        return false;
    }

    std::filesystem::create_directory(outputDirectory());

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
            // if the first char of the type name is the mapping marker then
            // this is the definition of a mapping and not data to be
            // deserialized
            if (typeName[0] == MappingMarker)
            {
                registerType(typeName.substr(1), file);
                continue;
            }

            // get the correct parser for this type
            auto& type = typeMap.at(typeName);

            for (auto& fieldInstruction : type.instructions)
                fieldInstruction(file, type.file);

            // replace the trailing comma with a newline character
            type.file.seekp(type.file.tellp() - std::streampos{1});
            type.file << '\n';
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

}  // namespace Boardcore
