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
#include <functional>
#include <unordered_map>
#include <vector>

#include "RegistryTypes.h"

namespace Boardcore
{

using RegistryConfiguration =
    std::unordered_map<ConfigurationId, EntryStructsUnion>;

/**
 * @brief Serialization header, with useful information about the serialized
 * data. Header to the actually serialized data.
 */
struct RegistryHeader
{
    uint64_t startBytes;  ///< Bytes at start, initialized as 1
    uint32_t totalSize;   ///< Total size of serialized data in bytes
    uint32_t nrEntries;   ///< Nr of configuration entries
};

/**
 * @brief Registry Footer, with checksum of the configuration data (not whole
 * data). Placed at the end of the actually serialized data
 */
struct RegistryFooter
{
    uint32_t checksum;
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
     * @brief Serializes the configuration map into the uint8_t vector for
     * serialized data
     *
     * @note In case an error is returned no guarantees are made about the
     * contents of the vector
     * @note The vector is resized if not of the exact size for serialization
     * @param configuration The configuration from which we read the
     * current entries to be serialized
     * @return OK If the de-serialization was successful and the entries where
     * added into the map
     * @return MALFORMED_SERIALIZED_DATA if the vector not have the
     * appropriate length for the header, footer and configuration
     * @return CHECKSUM_FAIL In case the saved Checksum not corresponds with the
     * one recomputed from the serialized configuration
     * @return NO_SUCH_TYPE In case the type id not corresponds to any defined
     * data type for the configuration
     */
    RegistryError serializeConfiguration(RegistryConfiguration& configuration);

    /**
     * @brief De-serializes the data from a serialized vector into the
     * configuration map. In case of malformed serialized vectors, does not
     * changes the configuration map and returns an error
     *
     * @param configuration The map in which we want to insert the entries
     * from the serialized vector
     * @note The deserialization adds/overwrites configuration entries. The
     * already present entries, if in the deserialized data, are overriden.
     * @note Pre-existing configuration entries are not removed (but might be
     * overwritten)
     * @return OK If the de-serialization was successful and the entries where
     * added into the map
     * @return MALFORMED_SERIALIZED_DATA if the vector not have the
     * appropriate length for the header, footer and configuration data
     * @return CHECKSUM_FAIL In case the saved Checksum not corresponds with the
     * one recomputed from the serialized configuration
     * @return NO_SUCH_TYPE In case the type id not corresponds to any defined
     * data type for the configuration
     * @return WRONG_ENDIANESS In case the endianess of the loaded data not
     * corresponds
     */
    RegistryError deserializeConfiguration(
        RegistryConfiguration& configuration);

private:
    std::vector<uint8_t>& serializationVector;
    uint32_t vectorWritePosition;

    /**
     * @brief Computes a custom checksum of the serialized configuration data
     *
     * @return uint32_t The computed custom checksum
     */
    uint32_t computeChecksum();

    /**
     *
     * @brief Deserializes from the vector the data in sequential order and
     * inserts/overwrites entries into the configuration.
     *
     * @tparam T the element data type
     * @param element The element we want to get from the serialized vector
     * @return OK If the read was successful
     * @return MALFORMED_SERIALIZED_DATA Otherwise, in case the vector is not
     * long enough to read the element
     */
    template <typename T>
    RegistryError deserialize(T& element)
    {
        size_t elSize = sizeof(T);

        if (serializationVector.size() < vectorWritePosition + elSize)
            return RegistryError::MALFORMED_SERIALIZED_DATA;

        std::memcpy(&element, serializationVector.data() + vectorWritePosition,
                    elSize);

        vectorWritePosition += elSize;
        return RegistryError::OK;
    }

    /**
     * @brief serialize functions writes to the vector the elements to
     * serialize. It does such task using memcpy and using the positional
     * attribute to know the current position where to write and updates such
     * attribute.
     *
     * @tparam T the element data type
     * @param element The element to be written in the serialized vector
     * @return OK If it could successfully write
     * @return WRONG_WRITES_SIZE Otherwise, in case could not write due to not
     * enough space in the vector
     */
    template <typename T>
    RegistryError serialize(T& element)
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