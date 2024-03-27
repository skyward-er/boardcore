/* Copyright (c) 2023 Skyward Experimental Rocketry
 * Author: Nicolò Caruso
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

#include "RegistrySerializer.h"

#include <functional>
#include <numeric>

namespace Boardcore
{
RegistrySerializer::RegistrySerializer(std::vector<uint8_t>& vector)
    : serializationVector(vector), vectorWritePosition(0)
{
}

RegistryError RegistrySerializer::serializeConfiguration(
    RegistryConfiguration& configuration)
{
    RegistryError error = RegistryError::OK;
    bool success        = true;

    // Resizes the serialization vector
    serializationVector.resize(size(configuration));

    // Write the header
    RegistryHeader header;
    header.startBytes = 1;
    header.totalSize  = serializationVector.size();
    header.nrEntries  = configuration.size();
    error             = serialize(header);
    if (error != RegistryError::OK)
        return RegistryError::NO_SPACE_FOR_HEADER;

    // Add the configuration entries one after the other
    for (auto& entry : configuration)
    {
        TypesEnum type;
        // Appends the entry ID
        success &= (serialize(entry.first) == RegistryError::OK);
        // Appends the configuration entry
        type = entry.second.getType();
        success &= (serialize(type) == RegistryError::OK);
        if (!success)
            return RegistryError::WRONG_WRITES_SIZE;

        switch (entry.second.getType())
        {
            case TypesEnum::COORDINATES:
            {
                Coordinates coordinate{0, 0};
                entry.second.get(coordinate);
                success &= (serialize(coordinate) == RegistryError::OK);
                break;
            }

            case TypesEnum::UINT32:
            {
                uint32_t uint32Value = 0;
                entry.second.get(uint32Value);
                success &= (serialize(uint32Value) == RegistryError::OK);
                break;
            }

            case TypesEnum::FLOAT:
            {
                float floatValue = 0;
                entry.second.get(floatValue);
                success &= (serialize(floatValue) == RegistryError::OK);
                break;
            }

            default:
                return RegistryError::NO_SUCH_TYPE;
                break;
        }
        if (!success)
            return RegistryError::WRONG_WRITES_SIZE;
    }

    // Compute the Footer and write it
    RegistryFooter footer;
    footer.crc = computeCRC();

    // Add the RegistryHeader at vector head position
    return serialize(footer);
}

RegistryError RegistrySerializer::deserializeConfiguration(
    RegistryConfiguration& configuration)
{
    bool success = true;

    // Case the vector is empty/not have even the vector size
    if (serializationVector.size() <
        sizeof(RegistryHeader) + sizeof(RegistryFooter))
        return RegistryError::MALFORMED_SERIALIZED_DATA;

    RegistryHeader header;
    success &= (deserialize(header) == RegistryError::OK);
    if (!success)
        return RegistryError::MALFORMED_SERIALIZED_DATA;

    if (header.startBytes != 1)
    {
        return RegistryError::WRONG_ENDIANESS;
    }

    // Save the current vector position before jumping to the footer
    uint32_t previousPos = vectorWritePosition;
    vectorWritePosition  = header.totalSize - sizeof(RegistryFooter);
    RegistryFooter footer;
    success &= (deserialize(footer) == RegistryError::OK);
    if (!success)
        return RegistryError::MALFORMED_SERIALIZED_DATA;
    // Restore vector position
    vectorWritePosition = previousPos;

    uint32_t savedCRC = computeCRC();
    if (footer.crc != savedCRC)
        return RegistryError::CRC_FAIL;

    // Clears the configuration for the correct insertion
    configuration.clear();

    // Set the configuration from the saved configuration
    uint32_t counter = 0;

    while (vectorWritePosition < serializationVector.size() - sizeof(footer) &&
           counter < header.nrEntries)
    {
        ConfigurationId id = 0;
        TypesEnum typeId;
        // Gets the ID of the entry, the ID of the data type, the value
        success &= (deserialize(id) == RegistryError::OK);
        success &= (deserialize(typeId) == RegistryError::OK);
        if (!success)
            return RegistryError::MALFORMED_SERIALIZED_DATA;

        switch (typeId)
        {
            case TypesEnum::COORDINATES:
            {
                Coordinates coordinate{0, 0};
                success &= (deserialize(coordinate) == RegistryError::OK);
                if (!success)
                    return RegistryError::MALFORMED_SERIALIZED_DATA;
                EntryStructsUnion entry = EntryStructsUnion::make(coordinate);
                success &= configuration.insert({id, entry}).second;
                break;
            }
            case TypesEnum::FLOAT:
            {
                float floatValue = 0;
                success &= (deserialize(floatValue) == RegistryError::OK);
                if (!success)
                    return RegistryError::MALFORMED_SERIALIZED_DATA;
                EntryStructsUnion entry = EntryStructsUnion::make(floatValue);
                success &= configuration.insert({id, entry}).second;
                break;
            }
            case TypesEnum::UINT32:
            {
                uint32_t uint32Value = 0;
                success &= (deserialize(uint32Value) == RegistryError::OK);
                if (!success)
                    return RegistryError::MALFORMED_SERIALIZED_DATA;
                EntryStructsUnion entry = EntryStructsUnion::make(uint32Value);
                success &= configuration.insert({id, entry}).second;
                break;
            }
            default:
            {
                return RegistryError::NO_SUCH_TYPE;
            }
        }

        if (!success)
            return RegistryError::CANNOT_INSERT;

        counter++;
    }

    return RegistryError::OK;
}

uint32_t RegistrySerializer::computeCRC()
{
    uint32_t counter = 0;
    return std::accumulate(
        serializationVector.begin() + sizeof(RegistryHeader),
        serializationVector.end() - sizeof(RegistryFooter), 0,
        [&counter](uint32_t acc, uint8_t element)
        {
            acc ^= static_cast<uint32_t>(element >> (3 - (counter % 4)) * 8);
            counter++;
            return acc;
        });
}

size_t RegistrySerializer::size(RegistryConfiguration& configuration)
{
    size_t totalSize = sizeof(RegistryHeader) + sizeof(RegistryFooter);

    // Compute the overall space required for the configurations
    for (auto& entry : configuration)
    {
        totalSize += sizeof(entry.first);
        totalSize += entry.second.sizeBytes();
    }
    return totalSize;
}

}  // namespace Boardcore