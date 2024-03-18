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
     * @param configuration The configuration map to be serialized or to be
     * loaded
     */
    RegistrySerializer(
        std::vector<uint8_t>& vector,
        std::unordered_map<ConfigurationId, EntryStructsUnion>& configuration);

    /**
     * @brief Serializes the configuration map into a serialized uint8_t vector
     *
     * @param vector The vector where we serialize the configuration
     * @param configurationToSerialize The configuration from which we read the
     * current entries to be serialized
     */
    void serializeConfigurationToVector();

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
    bool deserializeConfigurationFromVector();

private:
    std::vector<uint8_t>& serializationVector;
    std::unordered_map<ConfigurationId, EntryStructsUnion> configuration;

    /**
     * @brief Computes the CRC/checksum of the feed vector
     *
     * @param vector The vector from which extract a CRC checksum
     * @return uint32_t The computed CRC
     */
    uint32_t computeCRC();

    /**
     * @brief Adds an element to the vector in head or tail position.
     *
     * @param serializedVector The vector for which we add the serialized data
     * @param element The element to be added to the serialized vector
     * @param tailAppend True if we insert at end, False if we insert to head
     */
    void addToVector(uint32_t element, bool tailAppend);
    void addToVector(float element, bool tailAppend);
    void addToVector(uint64_t element, bool tailAppend);
    void addToVector(Coordinates element, bool tailAppend);
    bool addToVector(EntryStructsUnion element, bool tailAppend);
    void addToVector(TypesEnum element, bool tailAppend);

    /**
     * @brief Reads from the vector the element specified in sequential order.
     *
     * @param it The iterator to visit the vector, which is increased while
     * reading
     * @param element The element we want to get from the serialized vector
     * @return true If the read was successful
     * @return false Otherwise, e.g. not enough bytes to read the element
     */
    bool readFromVector(std::vector<uint8_t>::iterator& it, uint32_t& element);
    bool readFromVector(std::vector<uint8_t>::iterator& it, uint64_t& element);
    bool readFromVector(std::vector<uint8_t>::iterator& it, float& element);
    bool readFromVector(std::vector<uint8_t>::iterator& it,
                        Coordinates& element);
    bool readFromVector(std::vector<uint8_t>::iterator& it,
                        EntryStructsUnion& element);
    bool readFromVector(std::vector<uint8_t>::iterator& it, TypesEnum& element);
};
}  // namespace Boardcore