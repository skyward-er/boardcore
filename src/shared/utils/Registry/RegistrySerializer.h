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
     */
    void serializeConfiguration(
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

    /**
     * @brief Computes the CRC/checksum of the feed vector
     *
     * @param vector The vector from which extract a CRC checksum
     * @return uint32_t The computed CRC
     */
    uint32_t computeCRC(std::vector<uint8_t>::iterator& it);

    /**
     * @brief Adds an element to the vector in head or tail position.
     *
     * @param serializedVector The vector for which we add the serialized data
     * @param element The element to be added to the serialized vector
     */
    void serialize(uint32_t element);
    void serialize(float element);
    void serialize(uint64_t element);
    void serialize(Coordinates element);
    bool serialize(EntryStructsUnion element);
    void serialize(TypesEnum element);

    /**
     * @brief Reads from the vector the element specified in sequential order.
     *
     * @param it The iterator to visit the vector, which is increased while
     * reading
     * @param element The element we want to get from the serialized vector
     * @return true If the read was successful
     * @return false Otherwise, e.g. not enough bytes to read the element
     */
    bool deserialize(std::vector<uint8_t>::iterator& it, uint32_t& element);
    bool deserialize(std::vector<uint8_t>::iterator& it, uint64_t& element);
    bool deserialize(std::vector<uint8_t>::iterator& it, float& element);
    bool deserialize(std::vector<uint8_t>::iterator& it, Coordinates& element);
    bool deserialize(std::vector<uint8_t>::iterator& it,
                     EntryStructsUnion& element);
    bool deserialize(std::vector<uint8_t>::iterator& it, TypesEnum& element);

    /**
     * @brief Deserializes the header structure from the vector
     *
     * @param it The iterator to the current position of the vector
     * @param header The header to be retrieve
     * @return true If the header was deserialized successfully
     * @return false Otherwise, e.g. malformed/too short header
     */
    bool deserializeHeader(std::vector<uint8_t>::iterator& it,
                           RegistryHeader& header);

    /**
     * @brief Writes into the pre-allocated space the header
     *
     * @param header The header to be written
     * @return true If it could successfully write
     * @return false Otherwise, e.g. has no sufficient space to write
     */
    bool writeHeader(RegistryHeader header);

    /**
     * @brief Write functions mainly used for the writeHeader
     *
     * @param initialPosition The initial position from which write the element
     * @param element The element to be written
     * @return true If it could successfully write
     * @return false Otherwise, e.g. has no sufficient space to write
     */
    bool write(std::vector<uint8_t>::iterator& it, uint32_t element);
    bool write(std::vector<uint8_t>::iterator& it, uint64_t element);
};
}  // namespace Boardcore