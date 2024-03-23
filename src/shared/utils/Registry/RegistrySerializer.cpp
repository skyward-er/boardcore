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

namespace
{
// Constexpr added for readability of the function serialize
constexpr bool HEAD            = false;
constexpr bool TAIL            = true;
constexpr bool FLOAT_IS_32_BIT = true;
}  // namespace

namespace Boardcore
{

/**
 * @brief Construct an empty new Registry Header object, initialized with
 * all fields as zeros
 *
 */
RegistryHeader::RegistryHeader() : zeroBytes(0), vecLen(0), nrEntries(0), crc(0)
{
}

/**
 * @brief Construct a new Registry Header object
 *
 * @param vectorSize  The length of the vector
 * @param nrEntries The nr of entries in the serialized vector
 * @param crcPartialVec The CRC/checksum computed on the partially serialized
 * vector
 */
RegistryHeader::RegistryHeader(uint32_t vectorSize, uint32_t nrEntries,
                               uint32_t crcPartialVec)
    : zeroBytes(0), vecLen(vectorSize), nrEntries(nrEntries), crc(crcPartialVec)
{
}

/**
 * @brief Computes and returns the size in nr. of uint8_t of the whole registry
 * header structure
 *
 * @return uint32_t The size of all the attributes of the header
 */
uint32_t RegistryHeader::size()
{
    return (sizeof(zeroBytes) + sizeof(vecLen) + sizeof(nrEntries) +
            sizeof(crc));
}

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
 * @param vector The vector where we serialize the configuration
 * @param configuration The configuration from which we read the
 * current entries to be serialized
 * @return true If the configuration was successfully serialized and
 * inserted into the serialized data vector
 * @return false Otherwise
 */
bool RegistrySerializer::serializeConfiguration(
    std::unordered_map<ConfigurationId, EntryStructsUnion>& configuration)
{
    RegistryHeader header;
    size_t configurationSize = 0;
    bool success             = true;
    vectorWritePosition      = 0;

    // Compute the overall space required for the configurations
    for (auto& entry : configuration)
    {
        configurationSize += sizeof(entry.first);
        configurationSize += entry.second.sizeBytes();
    }

    // Resizes the serialization vector if the size is incorrect
    if (serializationVector.size() != header.size() + configurationSize)
    {
        serializationVector.resize(header.size() + configurationSize);
    }

    vectorWritePosition  = header.size();
    uint32_t uint32Value = 0;
    ConfigurationId id   = 0;
    Coordinates coordinate(0, 0);
    float floatValue = 0;

    // Add the configuration entries one after the other
    for (auto& entry : configuration)
    {
        // Appends the entry ID
        write(entry.first);
        // Appends the configuration entry
        write(entry.second.type);

        switch (entry.second.type)
        {
            case TypesEnum::COORDINATES:
                entry.second.getFromUnion(coordinate);
                write(coordinate);
                break;

            case TypesEnum::UINT32:
                entry.second.getFromUnion(uint32Value);
                write(uint32Value);
                break;

            case TypesEnum::FLOAT:
                entry.second.getFromUnion(floatValue);
                write(floatValue);
                break;

            default:
                return false;
                break;
        }
    }

    // Compute the CRC of the serialized configuration
    vectorWritePosition = 0;
    header.zeroBytes    = 0;
    header.vecLen       = serializationVector.size();
    header.nrEntries    = configuration.size();
    auto iterator       = serializationVector.begin() + header.size();
    header.crc          = computeCRC(iterator);

    // Add the RegistryHeader at vector head position
    writeHeader(header);
    return success;
}

/**
 * @brief De-serializes the data from a serialized vector into the
 * configuration map. In case of malformed serialized vectors, does not
 * changes the configuration map and returns false
 *
 * @param serializedVector The vector from which we load the configuration
 * @param configurationToLoad The map in which we want to insert the entries
 * from the serialized vector
 * @return true If the de-serialization was successful and the entries where
 * added into the map
 * @return false Otherwise, e.g. in case of malformed or even corrupted byte
 * vectors
 */
bool RegistrySerializer::deserializeConfiguration(
    std::unordered_map<ConfigurationId, EntryStructsUnion>& configuration)
{
    bool success = true;
    RegistryHeader header;

    // Case the vector is empty/not have even the vector size
    if (serializationVector.size() < header.size())
        return false;

    vectorWritePosition = 0;
    success &= deserializeHeader(header);

    auto iterator     = serializationVector.begin() + header.size();
    uint32_t savedCRC = computeCRC(iterator);

    // Malformed or corrupted or empty configuration cases
    if (!success || serializationVector.size() == 0 || header.crc != savedCRC)
        return false;

    // Clears the configuration for the correct insertion
    configuration.clear();

    // Set the configuration from the saved configuration
    iterator             = serializationVector.begin() + header.size();
    int counter          = 0;
    uint32_t uint32Value = 0;
    ConfigurationId id   = 0;
    TypesEnum typeId;
    Coordinates coordinate(0, 0);
    float floatValue;

    while (vectorWritePosition < serializationVector.size() &&
           counter < header.nrEntries && success)
    {
        // Gets the ID of the entry, the ID of the data type, the value
        success &= deserialize(id);
        success &= deserialize(typeId);  // TODO: si sminchia... 8...
        switch (typeId)
        {
            case TypesEnum::COORDINATES:
            {
                success &= deserialize(coordinate);
                if (!success)
                    return false;
                EntryStructsUnion entry = EntryStructsUnion::make(coordinate);
                // WHY CANNOT DO THE INSERT FROM SECOND PROBLEMS!!!
                success &=
                    configuration.insert(std::make_pair(id, entry)).second;
                break;
            }
            case TypesEnum::FLOAT:
            {
                success &= deserialize(floatValue);
                if (!success)
                    return false;
                EntryStructsUnion entry = EntryStructsUnion::make(floatValue);
                // WHY CANNOT DO THE INSERT????
                success &=
                    configuration.insert(std::make_pair(id, entry)).second;
                break;
            }
            case TypesEnum::UINT32:
            {
                success &= deserialize(uint32Value);
                if (!success)
                    return false;
                EntryStructsUnion entry = EntryStructsUnion::make(uint32Value);
                // WHY CANNOT DO THE INSERT????
                success &=
                    configuration.insert(std::make_pair(id, entry)).second;
                break;
            }
            default:
            {
                success = false;  //< WHY DOES ENTER IN HERE, 2nd
                                  // configuration???? READS SHITS FOR TYPEID???
                break;
            }
        }
        if (!success)
            return false;
        counter++;
    }
    return success;
}

/**
 * @brief Computes the CRC/checksum of the feed vector
 *
 * @param vector The vector from which extract a CRC checksum
 * @return uint32_t The computed CRC
 */
uint32_t RegistrySerializer::computeCRC(std::vector<uint8_t>::iterator& it)
{
    uint32_t crc = 0, counter = 0;
    crc = std::accumulate(
        it, serializationVector.end(), 0,
        [&counter](uint32_t acc, uint8_t element)
        {
            acc ^= static_cast<uint32_t>(element >> (3 - (counter % 4)) * 8);
            counter++;
            return acc;
        });
    return crc;
}

/**
 * @brief Writes into the pre-allocated space the header
 *
 * @param header The header to be written
 * @return true If it could successfully write
 * @return false Otherwise, e.g. has no sufficient space to write
 */
bool RegistrySerializer::writeHeader(RegistryHeader& header)
{
    bool success = true;
    if (serializationVector.size() < header.size())
        return false;
    // Writing on the space allocated before
    success &= write(header.zeroBytes);
    success &= write(header.vecLen);
    success &= write(header.nrEntries);
    success &= write(header.crc);
    return success;
}

bool RegistrySerializer::writeHeader(RegistryHeader& header, uint32_t position)
{
    vectorWritePosition = position;
    return writeHeader(header);
}

/**
 * @brief Deserializes the header structure from the vector
 *
 * @param it The iterator to the current position of the vector
 * @param header The header to be retrieve
 * @return true If the header was deserialized successfully
 * @return false Otherwise, e.g. malformed/too short header
 */
bool RegistrySerializer::deserializeHeader(RegistryHeader& header,
                                           uint32_t position)
{
    vectorWritePosition = position;
    bool success        = true;
    success &= deserialize(header.zeroBytes);
    if (header.zeroBytes != 0)
        return false;
    success &= deserialize(header.vecLen);
    success &= deserialize(header.nrEntries);
    success &= deserialize(header.crc);
    return success;
}

bool RegistrySerializer::deserializeHeader(RegistryHeader& header)
{
    return deserializeHeader(header, vectorWritePosition);
}

}  // namespace Boardcore