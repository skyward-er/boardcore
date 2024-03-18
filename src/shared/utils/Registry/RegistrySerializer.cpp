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

namespace
{
// Constexpr added for readability of the function addToVector
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
 * @param configuration The configuration map to be serialized or to be
 * loaded
 */
RegistrySerializer::RegistrySerializer(
    std::vector<uint8_t>& vector,
    std::unordered_map<ConfigurationId, EntryStructsUnion>& configuration)
    : serializationVector(vector), configuration(configuration){};

/**
 * @brief Serializes the configuration map into a serialized uint8_t vector
 *
 * @param vector The vector where we serialize the configuration
 * @param configurationToSerialize The configuration from which we read the
 * current entries to be serialized
 * @return true If the configuration was successfully serialized and
 * inserted into the serialized data vector
 * @return false Otherwise
 */
void RegistrySerializer::serializeConfigurationToVector()
{
    // Add the configuration entries one after the other
    for (auto& entry : configuration)
    {
        // Appends the entry ID
        addToVector(entry.first, TAIL);
        // Appends the configuration entry
        addToVector(entry.second, TAIL);
    }

    // Compute the CRC of the serialized configuration
    RegistryHeader header(serializationVector.size(), configuration.size(),
                          computeCRC());

    // Add the RegistryHeader at vector head position
    addToVector(header.crc, HEAD);
    addToVector(header.nrEntries, HEAD);
    addToVector(header.vecLen, HEAD);
    addToVector(header.zeroBytes, HEAD);
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
bool RegistrySerializer::deserializeConfigurationFromVector()
{
    bool success = true;
    RegistryHeader header;

    // Case the vector is empty/not have even the vector size
    if (serializationVector.size() < header.size())
        return false;

    auto iterator = serializationVector.begin();
    success &= readFromVector(iterator, header.zeroBytes);
    if (header.zeroBytes != 0 || !success)
        return false;
    success &= readFromVector(iterator, header.vecLen);
    success &= readFromVector(iterator, header.nrEntries);
    success &= readFromVector(iterator, header.crc);

    // Remove the header to then have just the configuration into the vector
    iterator = serializationVector.begin();
    for (int i = 0; i < header.size() && iterator != serializationVector.end();
         i++)
    {
        serializationVector.erase(iterator);
        iterator = serializationVector.begin();
    }

    // Malformed or corrupted or empty configuration cases
    if (!success || header.crc != computeCRC() ||
        serializationVector.size() == 0)
        return false;

    // Set the configuration from the saved configuration
    iterator    = serializationVector.begin();
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
        success &= readFromVector(iterator, id);
        success &= readFromVector(iterator, typeId);
        switch (typeId)
        {
            case TypesEnum::COORDINATES:
                success &= readFromVector(iterator, coordinate);
                EntryStructsUnion entry = EntryStructsUnion::make(coordinate);
                success &=
                    configuration.insert(std::make_pair(id, entry)).second;
                break;
            case TypesEnum::FLOAT:
                success &= readFromVector(iterator, floatValue);
                EntryStructsUnion entry = EntryStructsUnion::make(floatValue);
                success &=
                    configuration.insert(std::make_pair(id, entry)).second;
                break;
            case TypesEnum::UINT32:
                success &= readFromVector(iterator, uint32Value);
                EntryStructsUnion entry = EntryStructsUnion::make(uint32Value);
                success &=
                    configuration.insert(std::make_pair(id, entry)).second;
                break;
            default:
                success = false;
                break;
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
uint32_t RegistrySerializer::computeCRC()
{
    uint32_t crc = 0, counter = 0;
    crc = std::accumulate(
        std::next(serializationVector.begin(), serializationVector.end()), 0,
        [&counter](uint32_t acc, uint8_t element)
        {
            acc ^= static_cast<uint32_t>(element >> (3 - (counter % 4)) * 8);
            return acc;
        });
    return crc;
};

/**
 * @brief Adds an element to the vector in head or tail position.
 *
 * @param serializedVector The vector for which we add the serialized data
 * @param element The element to be added to the serialized vector
 * @param tailAppend True if we insert at end, False if we insert to head
 */
void RegistrySerializer::addToVector(uint32_t element, bool tailAppend)
{
    if (tailAppend)
    {
        for (int i = 3; i >= 0; i--)
        {
            serializationVector.push_back(
                static_cast<uint8_t>(element >> i * 8));
        }
    }
    // Head insert
    else
    {
        for (int i = 0; i < 4; i++)
        {
            serializationVector.insert(serializationVector.begin(),
                                       static_cast<uint8_t>(element >> i * 8));
        }
    }
};

void RegistrySerializer::addToVector(float element, bool tailAppend)
{
    if (FLOAT_IS_32_BIT)
    {
        // cppcheck-suppress invalidPointerCast
        addToVector(*(reinterpret_cast<uint32_t*>(&element)), tailAppend);
    }
    else
    {
        // cppcheck-suppress invalidPointerCast
        addToVector(*(reinterpret_cast<uint64_t*>(&element)), tailAppend);
    }
};

void RegistrySerializer::addToVector(uint64_t element, bool tailAppend)
{
    if (tailAppend)
    {
        for (int i = 5; i >= 0; i--)
        {
            serializationVector.push_back(
                static_cast<uint8_t>(element >> i * 8));
        }
    }
    // Head insert
    else
    {
        for (int i = 0; i < 6; i++)
        {
            serializationVector.insert(serializationVector.begin(),
                                       static_cast<uint8_t>(element >> i * 8));
        }
    }
};

void RegistrySerializer::addToVector(Coordinates element, bool tailAppend)
{
    if (tailAppend)
    {
        addToVector(element.latitude, TAIL);
        addToVector(element.longitude, TAIL);
    }
    else
    {
        addToVector(element.longitude, HEAD);
        addToVector(element.latitude, HEAD);
    }
};

bool RegistrySerializer::addToVector(EntryStructsUnion element, bool tailAppend)
{
    if (tailAppend)
    {
        addToVector(element.type, TAIL);
    }

    switch (element.type)
    {
        case TypesEnum::COORDINATES:
            addToVector(element.value.coordinates_type, tailAppend);
            break;
        case TypesEnum::FLOAT:
            addToVector(element.value.float_type, tailAppend);
            break;
        case TypesEnum::UINT32:
            addToVector(element.value.uint32_type, tailAppend);
            break;
        default:
            return false;
            break;
    }

    if (!tailAppend)
    {
        addToVector(element.type, HEAD);
    }
    return true;
};

void RegistrySerializer::addToVector(TypesEnum element, bool tailAppend)
{
    addToVector(static_cast<uint32_t>(element), tailAppend);
};

/**
 * @brief Reads from the vector the element specified in sequential order.
 *
 * @param it The iterator to visit the vector, which is increased while reading
 * @param element The element we want to get from the serialized vector
 * @return true If the read was successful
 * @return false Otherwise, e.g. not enough bytes to read the element
 */
bool RegistrySerializer::readFromVector(std::vector<uint8_t>::iterator& it,
                                        uint32_t& element)
{
    element = 0;
    for (int i = 3; i >= 0; i--)
    {
        element |= (static_cast<uint32_t>(*it) << i * 8);
        // Cannot read the needed byte, malformed vector
        if (it != serializationVector.end() && i > 0)
            return false;
        it++;
    }
};

bool RegistrySerializer::readFromVector(std::vector<uint8_t>::iterator& it,
                                        uint64_t& element)
{
    element = 0;
    for (int i = 5; i >= 0; i--)
    {
        element |= (static_cast<uint64_t>(*it) << i * 8);
        // Cannot read the needed byte, malformed vector
        if (it != serializationVector.end() && i > 0)
            return false;
        it++;
    }
};

bool RegistrySerializer::readFromVector(std::vector<uint8_t>::iterator& it,
                                        float& element)
{
    bool success = true;

    if (FLOAT_IS_32_BIT)
    {
        uint32_t value;

        // cppcheck-suppress invalidPointerCast
        value = *(reinterpret_cast<uint32_t*>(&element));
        success &= readFromVector(it, value);
        // cppcheck-suppress invalidPointerCast
        element = *(reinterpret_cast<float*>(&value));
    }
    // 64-bit float case
    else
    {
        uint64_t value;

        // cppcheck-suppress invalidPointerCast
        value = *(reinterpret_cast<uint64_t*>(&element));
        success &= readFromVector(it, value);
        // cppcheck-suppress invalidPointerCast
        element = *(reinterpret_cast<float*>(&value));
    }
    return success;
};

bool RegistrySerializer::readFromVector(std::vector<uint8_t>::iterator& it,
                                        Coordinates& element)
{
    bool success = true;
    success &= readFromVector(it, element.latitude);
    success &= readFromVector(it, element.longitude);
    return success;
};

bool RegistrySerializer::readFromVector(std::vector<uint8_t>::iterator& it,
                                        EntryStructsUnion& element)
{
    bool success = true;
    success &= readFromVector(it, element.type);
    switch (element.type)
    {
        case TypesEnum::COORDINATES:
            success &= readFromVector(it, element.value.coordinates_type);
            break;
        case TypesEnum::FLOAT:
            success &= readFromVector(it, element.value.float_type);
            break;
        case TypesEnum::UINT32:
            success &= readFromVector(it, element.value.uint32_type);
            break;
        default:
            return false;
            break;
    }
    return success;
};
}  // namespace Boardcore