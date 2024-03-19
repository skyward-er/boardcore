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
RegistryHeader::RegistryHeader()
    : zeroBytes(0), vecLen(0), nrEntries(0), crc(0){};

/**
 * @brief Construct a new Registry Header object
 *
 * @param lenPartiallySerializedVec  The length of the vector serialized so far
 * @param nrEntries The nr of entries in the serialized vector
 * @param crcPartialVec The CRC/checksum computed on the partially serialized
 * vector
 */
RegistryHeader::RegistryHeader(uint32_t lenPartiallySerializedVec,
                               uint32_t nrEntries, uint32_t crcPartialVec)
    : zeroBytes(0), vecLen(lenPartiallySerializedVec + size()),
      nrEntries(nrEntries), crc(crcPartialVec){};

/**
 * @brief Computes and returns the size of the whole registry header
 * structure
 *
 * @return uint32_t The size of all the attributes of the header
 */
uint32_t RegistryHeader::size()
{
    return sizeof(zeroBytes) + sizeof(vecLen) + sizeof(nrEntries) + sizeof(crc);
}

/**
 * @brief Construct a new Registry Serializer object
 *
 * @param vector The reference to the vector for
 * serialization/deserialization procedures
 */
RegistrySerializer::RegistrySerializer(std::vector<uint8_t>& vector)
    : serializationVector(vector){};

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
void RegistrySerializer::serializeConfiguration(
    std::unordered_map<ConfigurationId, EntryStructsUnion>& configuration)
{
    RegistryHeader header;
    if (serializationVector.size() < header.size())
    {
        serializationVector.clear();
        // Inserts zero bytes for allocating the space for the header
        serializationVector.insert(serializationVector.begin(), header.size(),
                                   0);
    }
    else if (serializationVector.size() > header.size())
    {
        // Keeps just the bytes for the header
        serializationVector.erase(serializationVector.begin() + header.size(),
                                  serializationVector.end());
    }

    // Add the configuration entries one after the other
    for (auto& entry : configuration)
    {
        // Appends the entry ID
        serialize(entry.first);
        // Appends the configuration entry
        serialize(entry.second);
    }

    // Compute the CRC of the serialized configuration
    header.zeroBytes = 0;
    header.vecLen    = serializationVector.size();
    header.nrEntries = configuration.size();
    auto iterator    = serializationVector.begin() + header.size();
    header.crc       = computeCRC(iterator);

    // Add the RegistryHeader at vector head position
    writeHeader(header);
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

    auto iterator = serializationVector.begin();
    success &= deserializeHeader(iterator, header);

    iterator          = serializationVector.begin() + header.size();
    uint32_t savedCRC = computeCRC(iterator);

    // Malformed or corrupted or empty configuration cases
    if (!success || serializationVector.size() == 0 ||
        header.crc != savedCRC)  // TODO: CRC ISSUE!
        return false;

    // Set the configuration from the saved configuration
    iterator    = serializationVector.begin() + header.size();
    int counter = 0;
    uint32_t uint32Value;
    ConfigurationId id;
    TypesEnum typeId;
    Coordinates coordinate;
    float floatValue;
    while (iterator != serializationVector.end() &&
           counter < header.nrEntries && success)
    {
        // Gets the ID of the entry, the ID of the data type, the value
        success &= deserialize(iterator, id);
        success &= deserialize(iterator, typeId);
        switch (typeId)
        {
            case TypesEnum::COORDINATES:
            {
                success &= deserialize(iterator, coordinate);
                EntryStructsUnion entry = EntryStructsUnion::make(coordinate);
                success &=
                    configuration.insert(std::make_pair(id, entry)).second;
                break;
            }
            case TypesEnum::FLOAT:
            {
                success &= deserialize(iterator, floatValue);
                EntryStructsUnion entry = EntryStructsUnion::make(floatValue);
                success &=
                    configuration.insert(std::make_pair(id, entry)).second;
                break;
            }
            case TypesEnum::UINT32:
            {
                success &= deserialize(iterator, uint32Value);
                EntryStructsUnion entry = EntryStructsUnion::make(uint32Value);
                success &=
                    configuration.insert(std::make_pair(id, entry)).second;
                break;
            }
            default:
            {
                success = false;
                break;
            }
        }
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
};

/**
 * @brief Adds an element to the vector in head or tail position.
 *
 * @param serializedVector The vector for which we add the serialized data
 * @param element The element to be added to the serialized vector
 */
void RegistrySerializer::serialize(uint32_t element)
{
    for (int i = 3; i >= 0; i--)
    {
        serializationVector.push_back(static_cast<uint8_t>(element >> i * 8));
    }
};

void RegistrySerializer::serialize(float element)
{
    if (FLOAT_IS_32_BIT)
    {
        // cppcheck-suppress invalidPointerCast
        serialize(*(reinterpret_cast<uint32_t*>(&element)));
    }
    else
    {
        // cppcheck-suppress invalidPointerCast
        serialize(*(reinterpret_cast<uint64_t*>(&element)));
    }
};

void RegistrySerializer::serialize(uint64_t element)
{
    for (int i = 5; i >= 0; i--)
    {
        serializationVector.push_back(static_cast<uint8_t>(element >> i * 8));
    }
};

void RegistrySerializer::serialize(Coordinates element)
{
    serialize(element.latitude);
    serialize(element.longitude);
};

bool RegistrySerializer::serialize(EntryStructsUnion element)
{
    serialize(element.type);
    switch (element.type)
    {
        case TypesEnum::COORDINATES:
            serialize(element.value.coordinates_type);
            break;
        case TypesEnum::FLOAT:
            serialize(element.value.float_type);
            break;
        case TypesEnum::UINT32:
            serialize(element.value.uint32_type);
            break;
        default:
            return false;
            break;
    }
    return true;
};

void RegistrySerializer::serialize(TypesEnum element)
{
    serialize(static_cast<uint32_t>(element));
};

/**
 * @brief Reads from the vector the element specified in sequential order.
 *
 * @param it The iterator to visit the vector, which is increased while reading
 * @param element The element we want to get from the serialized vector
 * @return true If the read was successful
 * @return false Otherwise, e.g. not enough bytes to read the element
 */
bool RegistrySerializer::deserialize(std::vector<uint8_t>::iterator& it,
                                     uint32_t& element)
{
    element = 0;
    for (int i = 3; i >= 0; i--)
    {
        // Cannot read the needed byte, malformed vector
        if (it == serializationVector.end())
            return false;
        element |= (static_cast<uint32_t>(*it) << i * 8);
        it++;
    }
    return true;
};

/**
 * @brief Reads from the vector the element specified in sequential order.
 *
 * @param it The iterator to visit the vector, which is increased while
 * reading
 * @param element The element we want to get from the serialized vector
 * @return true If the read was successful
 * @return false Otherwise, e.g. not enough bytes to read the element
 */
bool RegistrySerializer::deserialize(std::vector<uint8_t>::iterator& it,
                                     uint64_t& element)
{
    element = 0;
    for (int i = 5; i >= 0; i--)
    {
        // Cannot read the needed byte, malformed vector
        if (it == serializationVector.end())
            return false;
        element |= (static_cast<uint64_t>(*it) << i * 8);
        it++;
    }
    return true;
};

bool RegistrySerializer::deserialize(std::vector<uint8_t>::iterator& it,
                                     float& element)
{
    bool success = true;

    if (FLOAT_IS_32_BIT)
    {
        uint32_t value;

        // cppcheck-suppress invalidPointerCast
        value = *(reinterpret_cast<uint32_t*>(&element));
        success &= deserialize(it, value);
        // cppcheck-suppress invalidPointerCast
        element = *(reinterpret_cast<float*>(&value));
    }
    // 64-bit float case
    else
    {
        uint64_t value;

        // cppcheck-suppress invalidPointerCast
        value = *(reinterpret_cast<uint64_t*>(&element));
        success &= deserialize(it, value);
        // cppcheck-suppress invalidPointerCast
        element = *(reinterpret_cast<float*>(&value));
    }
    return success;
};

bool RegistrySerializer::deserialize(std::vector<uint8_t>::iterator& it,
                                     Coordinates& element)
{
    bool success = true;
    success &= deserialize(it, element.latitude);
    success &= deserialize(it, element.longitude);
    return success;
};

bool RegistrySerializer::deserialize(std::vector<uint8_t>::iterator& it,
                                     EntryStructsUnion& element)
{
    bool success = true;
    success &= deserialize(it, element.type);
    switch (element.type)
    {
        case TypesEnum::COORDINATES:
            success &= deserialize(it, element.value.coordinates_type);
            break;
        case TypesEnum::FLOAT:
            success &= deserialize(it, element.value.float_type);
            break;
        case TypesEnum::UINT32:
            success &= deserialize(it, element.value.uint32_type);
            break;
        default:
            return false;
            break;
    }
    return success;
};

bool RegistrySerializer::deserialize(std::vector<uint8_t>::iterator& it,
                                     TypesEnum& element)
{
    bool success;
    uint32_t temp;
    success = deserialize(it, temp);
    element = static_cast<TypesEnum>(temp);
    return success;
}

/**
 * @brief Writes into the pre-allocated space the header
 *
 * @param header The header to be written
 * @return true If it could successfully write
 * @return false Otherwise, e.g. has no sufficient space to write
 */
bool RegistrySerializer::writeHeader(RegistryHeader header)
{
    bool success = true;
    auto it      = serializationVector.begin();
    if (serializationVector.size() < header.size())
        return false;
    // Writing on the space allocated before
    success &= write(it, header.zeroBytes);
    success &= write(it, header.vecLen);
    success &= write(it, header.nrEntries);
    success &= write(it, header.crc);
    return success;
}

/**
 * @brief Write functions mainly used for the writeHeader
 *
 * @param initialPosition The initial position from which write the element
 * @param element The element to be written
 * @return true If it could successfully write
 * @return false Otherwise, e.g. has no sufficient space to write
 */
bool RegistrySerializer::write(std::vector<uint8_t>::iterator& it,
                               uint32_t element)
{
    for (int i = 3; i >= 0; i--)
    {
        if (it == serializationVector.end())
            return false;
        (*it) = static_cast<uint8_t>(element >> i * 8);
        it++;
    }
    return true;
};

bool RegistrySerializer::write(std::vector<uint8_t>::iterator& it,
                               uint64_t element)
{
    for (int i = 5; i >= 0; i--)
    {
        if (it == serializationVector.end())
            return false;
        (*it) = static_cast<uint8_t>(element >> i * 8);
        it++;
    }
    return true;
};

/**
 * @brief Deserializes the header structure from the vector
 *
 * @param it The iterator to the current position of the vector
 * @param header The header to be retrieve
 * @return true If the header was deserialized successfully
 * @return false Otherwise, e.g. malformed/too short header
 */
bool RegistrySerializer::deserializeHeader(std::vector<uint8_t>::iterator& it,
                                           RegistryHeader& header)
{
    bool success = true;
    success &= deserialize(it, header.zeroBytes);
    if (header.zeroBytes != 0)
        return false;
    success &= deserialize(it, header.vecLen);
    success &= deserialize(it, header.nrEntries);
    success &= deserialize(it, header.crc);
    return success;
}

}  // namespace Boardcore