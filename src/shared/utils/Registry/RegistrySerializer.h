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

#include <utils/Debug.h>

#include <cstdint>
#include <cstring>
#include <unordered_map>
#include <vector>

#include "TypeStructures.h"

namespace Boardcore
{

struct RegistryHeader
{
    uint64_t zeroBytes;
    uint32_t vecLen;
    uint32_t nrEntries;
    uint32_t crc;

    /**
     * @brief Construct a new empty Registry Header object, initialized with
     * all fields as zeros
     *
     */
    RegistryHeader();

    /**
     * @brief Construct a new Registry Header object
     *
     * @param lenPartiallySerializedVec  The length of the vector serialized so
     * far
     * @param nrEntries The nr of entries in the serialized vector
     * @param crcPartialVec The CRC/checksum computed on the partially
     * serialized vector
     */
    RegistryHeader(uint32_t lenPartiallySerializedVec, uint32_t nrEntries,
                   uint32_t crcPartialVec);

    /**
     * @brief Computes and returns the size of the whole registry header
     * structure
     *
     * @return uint32_t The size of all the attributes of the header
     */
    uint32_t size();
};

class RegistrySerializer
{
public:
    /**
     * @brief Construct a new Registry Serializer object
     *
     * @param vector The reference to the vector for
     * serialization/deserialization procedures
     */
    explicit RegistrySerializer(std::vector<uint8_t>& vector);

    /**
     * @brief Serializes the configuration map into a serialized uint8_t vector
     *
     * @param configuration The configuration from which we read the
     * current entries to be serialized
     * * @return true If the configuration was successfully serialized and
     * inserted into the serialized data vector
     * @return false Otherwise
     */
    bool serializeConfiguration(
        std::unordered_map<ConfigurationId, EntryStructsUnion>& configuration);

    /**
     * @brief De-serializes the data from a serialized vector into the
     * configuration map. In case of malformed serialized vectors, does not
     * changes the configuration map and returns false
     *
     * @param configuration The map in which we want to insert the entries
     * from the serialized vector
     * @return true If the de-serialization was successful and the entries where
     * added into the map
     * @return false Otherwise, e.g. in case of malformed or even corrupted byte
     * vectors
     */
    bool deserializeConfiguration(
        std::unordered_map<ConfigurationId, EntryStructsUnion>& configuration);

private:
    std::vector<uint8_t>& serializationVector;
    uint32_t vectorWritePosition;

    /**
     * @brief Computes the CRC/checksum of the feed vector
     *
     * @param vector The vector from which extract a CRC checksum
     * @return uint32_t The computed CRC
     */
    uint32_t computeCRC(std::vector<uint8_t>::iterator& it);

    /**
     * @brief Reads from the vector the element specified in sequential order.
     *
     * @param it The iterator to visit the vector, which is increased while
     * reading
     * @tparam element The element we want to get from the serialized vector
     * @return true If the read was successful
     * @return false Otherwise, e.g. not enough bytes to read the element
     */
    template <typename T>
    bool deserialize(T& element)
    {
        std::size_t size = sizeof(T);

        if (serializationVector.size() < vectorWritePosition + size)
            return false;

        /*alignas(T)*/ uint8_t buffer[size];

        for (int count = 0; count < size; count++)
            buffer[count] = serializationVector.at(vectorWritePosition + count);
        std::memcpy(&element, buffer, size);

        vectorWritePosition += size;
        return true;
    }

    template <typename T>
    bool deserialize(T& element, uint32_t position)
    {
        vectorWritePosition = position;
        return deserialize(element);
    }

    /**
     * @brief Deserializes the header structure from the vector
     *
     * @param it The iterator to the current position of the vector
     * @param header The header to be retrieve
     * @return true If the header was deserialized successfully
     * @return false Otherwise, e.g. malformed/too short header
     */
    bool deserializeHeader(RegistryHeader& header, uint32_t position);
    bool deserializeHeader(RegistryHeader& header);

    /**
     * @brief Writes into the pre-allocated space the header
     *
     * @param header The header to be written
     * @return true If it could successfully write
     * @return false Otherwise, e.g. has no sufficient space to write
     */
    bool writeHeader(RegistryHeader& header, uint32_t position);
    bool writeHeader(RegistryHeader& header);

    /**
     * @brief Write functions writes to the vector the elements to serialize. It
     * does such task using memcpy and using the positional attribute to know
     * the current position where to write and updates such attribute.
     *
     * @tparam element The element to be written in the serialized vector
     * @param position: The position in which we want to write the vector, by
     * default keeps the vectorWritePosition as position
     * @return true If it could successfully write
     * @return false Otherwise, e.g. has no sufficient space to write
     */
    template <typename T>
    bool write(T& element)
    {
        std::size_t size = sizeof(T);

        if (serializationVector.size() < vectorWritePosition + size)
            return false;

        /*alignas(T)*/ uint8_t buffer[size];
        std::memcpy(buffer, &element, size);

        for (int count = 0; count < size; count++)
            serializationVector.at(vectorWritePosition + count) = buffer[count];

        vectorWritePosition += size;
        return true;
    }

    template <typename T>
    bool write(T& element, uint32_t position)
    {
        vectorWritePosition = position;
        return write(element);
    }
};

}  // namespace Boardcore