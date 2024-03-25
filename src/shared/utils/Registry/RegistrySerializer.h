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

#include "RegistryTypes.h"

namespace Boardcore
{

using RegistryConfiguration =
    std::unordered_map<ConfigurationId, EntryStructsUnion>;

/**
 * @brief Serialization header, with useful information about the serialized
 * vector. Header to the actually serialized configuration
 */
struct RegistryHeader
{
    uint64_t startBytes;
    uint32_t vecLen;
    uint32_t nrEntries;
};

/**
 * @brief Registry Footer, with CRC checksum at end of the actually serialized
 * configuration vector
 */
struct RegistryFooter
{
    uint32_t crc;
};

/**
 * @brief Serialization and de-serialization class for the registry. It does
 * serialize and deserialize the configuration to the specified vector.
 */
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
     * @return OK If the configuration was successfully serialized and
     * inserted into the serialized data vector
     * @return WRONG_WRITES_SIZE If the write was unsuccessful
     * @return NO_SUCH_TYPE In case there is an unspecified type id to be
     * serialized
     */
    RegistryError serializeConfiguration(RegistryConfiguration& configuration);

    /**
     * @brief De-serializes the data from a serialized vector into the
     * configuration map. In case of malformed serialized vectors, does not
     * changes the configuration map and returns an error
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
    RegistryError deserializeConfiguration(
        RegistryConfiguration& configuration);

private:
    std::vector<uint8_t>& serializationVector;
    uint32_t vectorWritePosition;

    /**
     * @brief Computes the CRC/checksum of the feed vector
     *
     * @return uint32_t The computed CRC
     */
    uint32_t computeCRC();

    /**
     * @brief Reads from the vector the element specified in sequential order.
     *
     * @param it The iterator to visit the vector, which is increased while
     * reading
     * @tparam element The element we want to get from the serialized vector
     * @return OK If the read was successful
     * @return MALFORMED_SERIALIZED_VECTOR Otherwise, in case the vector is not
     * long enough to read the element
     */
    template <typename T>
    RegistryError deserialize(T& element)
    {
        size_t elSize = sizeof(T);

        if (serializationVector.size() < vectorWritePosition + elSize)
            return RegistryError::MALFORMED_SERIALIZED_VECTOR;

        std::memcpy(&element, serializationVector.data() + vectorWritePosition,
                    elSize);

        vectorWritePosition += elSize;
        return RegistryError::OK;
    }

    /**
     * @brief Writes into the pre-allocated space the header
     *
     * @param header The header to be written
     * @return OK If it could successfully write
     * @return NO_SPACE_FOR_HEADER Otherwise, in case cannot write the header
     * due to not enough space on the allocated vector
     */
    RegistryError writeHeader(RegistryHeader& header);

    /**
     * @brief Write functions writes to the vector the elements to serialize. It
     * does such task using memcpy and using the positional attribute to know
     * the current position where to write and updates such attribute.
     *
     * @tparam element The element to be written in the serialized vector
     * @return OK If it could successfully write
     * @return WRONG_WRITES_SIZE Otherwise, in case could not write due to not
     * enough space in the vector
     */
    template <typename T>
    RegistryError write(T& element)
    {
        size_t elSize = sizeof(T);

        if (serializationVector.size() < vectorWritePosition + elSize)
            return RegistryError::WRONG_WRITES_SIZE;

        std::memcpy(serializationVector.data() + vectorWritePosition, &element,
                    elSize);

        vectorWritePosition += elSize;
        return RegistryError::OK;
    }

    /**
     * @brief The size function to get the size of the vector that will be
     * serialized. It does take into account the header, actual configuration
     * and footer.
     *
     * @return size_t The size that the serialized vector will have after the
     * serialization process
     */
    size_t size(RegistryConfiguration& configuration);
};

}  // namespace Boardcore