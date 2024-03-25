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
#pragma once

#include "RegistrySerializer.h"

#include <functional>
#include <numeric>

namespace Boardcore
{
/**
 * @brief Construct a new Registry Serializer object
 *
 * @param vector The reference to the vector for
 * serialization/deserialization procedures
 */
RegistrySerializer::RegistrySerializer(std::vector<uint8_t>& vector)
    : serializationVector(vector), vectorWritePosition(0)
{
}

/**
 * @brief Serializes the configuration map into a serialized uint8_t vector
 *
 * @note The vector is resized if not of the exact size for serialization
 * @param configuration The configuration from which we read the
 * current entries to be serialized
 * @return OK If the de-serialization was successful and the entries where
 * added into the map
 * @return MALFORMED_SERIALIZED_VECTOR if the vector not have the
 * appropriate length for the header, footer and configuration
 * @return CRC_FAIL In case the saved CRC/Checksum not corresponds with the
 * one recomputed from the serialized configuration
 * @return NO_SUCH_TYPE In case the type id not corresponds to any defined
 * data type for the configuration
 * @return CANNOT_INSERT In case could not insert into the configuration the
 * de-serialized element
 */
RegistryError RegistrySerializer::serializeConfiguration(
    RegistryConfiguration& configuration)
{
    RegistryHeader header;
    RegistryFooter footer;
    size_t vecSize      = size(configuration);
    RegistryError error = RegistryError::OK;
    bool success        = true;
    vectorWritePosition = 0;

    // Resizes the serialization vector
    serializationVector.resize(vecSize);

    // Write the header
    header.startBytes = 1;
    header.vecLen     = serializationVector.size();
    header.nrEntries  = configuration.size();
    error             = writeHeader(header);
    if (error != RegistryError::OK)
        return error;

    vectorWritePosition = sizeof(header);
    ConfigurationId id  = 0;
    TypesEnum type;

    // Add the configuration entries one after the other
    for (auto& entry : configuration)
    {
        // Appends the entry ID
        success &= (write(entry.first) == RegistryError::OK);
        // Appends the configuration entry
        type = entry.second.getType();
        success &= (write(type) == RegistryError::OK);
        if (!success)
            return RegistryError::WRONG_WRITES_SIZE;

        switch (entry.second.getType())
        {
            case TypesEnum::COORDINATES:
            {
                Coordinates coordinate{0, 0};
                entry.second.get(coordinate);
                success &= (write(coordinate) == RegistryError::OK);
                break;
            }

            case TypesEnum::UINT32:
            {
                uint32_t uint32Value = 0;
                entry.second.get(uint32Value);
                success &= (write(uint32Value) == RegistryError::OK);
                break;
            }

            case TypesEnum::FLOAT:
            {
                float floatValue = 0;
                entry.second.get(floatValue);
                success &= (write(floatValue) == RegistryError::OK);
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
    footer.crc = computeCRC();

    // Add the RegistryHeader at vector head position
    return write(footer);
}

/**
 * @brief De-serializes the data from a serialized vector into the
 * configuration map. In case of malformed serialized vectors, does not
 * changes the configuration map returns an error
 *
 * @param configuration The map in which we want to insert the entries
 * from the serialized vector
 * @return OK If the de-serialization was successful and the entries where
 * added into the map
 * @return MALFORMED_SERIALIZED_VECTOR if the vector not have the
 * appropriate length for the header, footer and configuration
 * @return CRC_FAIL In case the saved CRC/Checksum not corresponds with the
 * one recomputed from the serialized configuration
 * @return NO_SUCH_TYPE In case the type id not corresponds to any defined
 * data type for the configuration
 * @return CANNOT_INSERT In case could not insert into the configuration the
 * de-serialized element
 * @return WRONG_ENDIANESS In case the endianess of the loaded data not
 * corresponds
 */
RegistryError RegistrySerializer::deserializeConfiguration(
    RegistryConfiguration& configuration)
{
    bool success = true;
    RegistryHeader header;
    RegistryFooter footer;

    // Case the vector is empty/not have even the vector size
    if (serializationVector.size() < sizeof(header) + sizeof(footer))
        return RegistryError::MALFORMED_SERIALIZED_VECTOR;

    vectorWritePosition = 0;
    success &= (deserialize(header) == RegistryError::OK);

    if (header.startBytes != 1)
    {
        return RegistryError::WRONG_ENDIANESS;
    }

    uint32_t savedCRC = computeCRC();

    // Malformed or corrupted or empty configuration cases
    if (!success || serializationVector.size() == 0)
        return RegistryError::MALFORMED_SERIALIZED_VECTOR;

    vectorWritePosition = header.vecLen - sizeof(footer);
    success &= (deserialize(footer) == RegistryError::OK);

    if (footer.crc != savedCRC)
        return RegistryError::CRC_FAIL;

    // Clears the configuration for the correct insertion
    configuration.clear();

    // Set the configuration from the saved configuration
    int counter        = 0;
    ConfigurationId id = 0;
    TypesEnum typeId;
    vectorWritePosition = sizeof(header);

    while (vectorWritePosition < serializationVector.size() - sizeof(footer) &&
           counter < header.nrEntries && success)
    {
        // Gets the ID of the entry, the ID of the data type, the value
        success &= (deserialize(id) == RegistryError::OK);
        success &= (deserialize(typeId) == RegistryError::OK);
        switch (typeId)
        {
            case TypesEnum::COORDINATES:
            {
                Coordinates coordinate{0, 0};
                success &= (deserialize(coordinate) == RegistryError::OK);
                if (!success)
                    return RegistryError::MALFORMED_SERIALIZED_VECTOR;
                EntryStructsUnion entry = EntryStructsUnion::make(coordinate);
                success &=
                    configuration.insert(std::make_pair(id, entry)).second;
                break;
            }
            case TypesEnum::FLOAT:
            {
                float floatValue;
                success &= (deserialize(floatValue) == RegistryError::OK);
                if (!success)
                    return RegistryError::MALFORMED_SERIALIZED_VECTOR;
                EntryStructsUnion entry = EntryStructsUnion::make(floatValue);
                success &=
                    configuration.insert(std::make_pair(id, entry)).second;
                break;
            }
            case TypesEnum::UINT32:
            {
                uint32_t uint32Value = 0;
                success &= (deserialize(uint32Value) == RegistryError::OK);
                if (!success)
                    return RegistryError::MALFORMED_SERIALIZED_VECTOR;
                EntryStructsUnion entry = EntryStructsUnion::make(uint32Value);
                success &=
                    configuration.insert(std::make_pair(id, entry)).second;
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

/**
 * @brief Computes the CRC/checksum of the feed vector
 *
 * @param vector The vector from which extract a CRC checksum
 * @return uint32_t The computed CRC
 */
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

/**
 * @brief Writes into the pre-allocated space the header
 *
 * @param header The header to be written
 * @return OK If it could successfully write
 * @return NO_SPACE_FOR_HEADER Otherwise, in case cannot write the header
 * due to not enough space on the allocated vector
 */
RegistryError RegistrySerializer::writeHeader(RegistryHeader& header)
{
    if (serializationVector.size() < sizeof(header))
        return RegistryError::NO_SPACE_FOR_HEADER;
    // Writing on the space allocated before
    return write(header);
}

/**
 * @brief The size function to get the size of the vector that will be
 * serialized. It does take into account the header, actual configuration
 * and footer.
 *
 * @return size_t The size that the serialized vector will have after the
 * serialization process
 */
size_t RegistrySerializer::size(RegistryConfiguration& configuration)
{
    size_t configurationSize = sizeof(RegistryHeader) + sizeof(RegistryFooter);

    // Compute the overall space required for the configurations
    for (auto& entry : configuration)
    {
        configurationSize += sizeof(entry.first);
        configurationSize += entry.second.sizeBytes();
    }
    return configurationSize;
}

}  // namespace Boardcore