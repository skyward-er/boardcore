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
#include "RegistryFrontend.h"

#include <utils/Debug.h>

#include <bitset>
#include <mutex>

#include "TypeStructures.h"

namespace
{
constexpr uint32_t VECTOR_NR_ENTRIES_RESERVE = 40;
constexpr uint32_t NR_BYTES_ENTRY_ID =
    4;  //< Nr. bytes allocated in the vector for the entryId
constexpr uint32_t NR_BYTES_TYPE_ID =
    4;  //< Nr. bytes allocated in the vector for the typeid
constexpr uint32_t NR_BYTES_PER_ENTRY =
    6;  //< For now assuming 1B ID, 1B type ID, 4B values
constexpr uint32_t VECTOR_ZERO_OFFSET =
    8;  //< 8 zeroed bytes offset before real vector data
constexpr uint32_t CONFIGURATIONS_START_OFFSET =
    VECTOR_ZERO_OFFSET + 12; /*< Nr. bytes from start to the base where
                          configuration starts (zero B, Nr. entries (4B),
                          Len_vector (4B), checksum (4B))*/
}  // namespace
namespace Boardcore
{

/**
 * @brief Registry front end constructor. Initializes the configuration from
 * the backend.
 */
RegistryFrontend::RegistryFrontend()
{
    serializationVector.reserve(VECTOR_NR_ENTRIES_RESERVE * NR_BYTES_PER_ENTRY);
    elementVector.reserve(NR_BYTES_ENTRY_ID + NR_BYTES_PER_ENTRY +
                          sizeof(TypeUnion));
    configuration.reserve(VECTOR_NR_ENTRIES_RESERVE * NR_BYTES_PER_ENTRY);
    // middleware.init(); //< Initializes with the backend
    // TODO: Re-add it when the middleware is integrated again
    // middleware.start();
    /*
     * TODO: The registry will load from the backend the saved configuration
     * and initialize configuration, after initialize properly the middleware
     * and backend */
};

/**
 * @brief Registry front end destructor. Saves configuration to
 * the backend.
 */
RegistryFrontend::~RegistryFrontend()
{
    /* TODO: The registry will save the configurations and also use the
     * proper destructors if needed*/
    saveConfiguration();
}

/**
 * @brief Disables the memory registry set and allocations.
 * To be use when the rocket itself is armed and during flight.
 */
void RegistryFrontend::arm()
{
    const std::lock_guard<std::recursive_mutex> lock(mutexForRegistry);
    isArmed = true;
}

/**
 * @brief Enable set methods and memory allocations.
 * To be used when the rocket is NOT in an "armed" state and while on
 * ground.
 */
void RegistryFrontend::disarm()
{
    const std::lock_guard<std::recursive_mutex> lock(mutexForRegistry);
    isArmed = false;
}

/**
 * @brief Visits the configuration applying the callback with the id and
 * EntryStructsUnion union as parameter for each configured entry in the
 * configuration.
 */
void RegistryFrontend::visitConfiguration(
    std::function<void(ConfigurationId, EntryStructsUnion&)> callback)
{
    const std::lock_guard<std::recursive_mutex> lock(mutexForRegistry);
    for (auto& it : configuration)
    {
        callback(it.first, it.second);
    }
}

/**
 * @brief Loads from the backend the configuration
 * @return True if the configuration exists in memory and is not corrupted,
 * False if not.
 */
bool RegistryFrontend::loadConfiguration()
{
    /* TODO: get from the backend the vector, verify the checksum, load entry by
     * entry
     */
    // TODO: Method that will call the backend and initializes the vector
    // TODO: if(!middleware.load(serializationVector)) return false;
    const std::lock_guard<std::recursive_mutex> lock(mutexForRegistry);
    uint32_t id = 0, typeId, checksum = 0, savedChecksum = 0, nrEntries = 0,
             vectorLen = 0, counter = 0, uint32Value;
    Coordinates coordinate;
    float floatValue;

    bool success = true;
    configuration.clear();

    /* Vector structure
     *
     * [ 0|0|0|0|0|0|0|0
     * | nr_entries 31-24 | nr_e 23-16 | nr_e 15-8 | nr_e 7-0
     * | vector_length 31-24 | v_len 23-16 | v_len 15-8 | v_len 7-0
     * | crc/checksum 31-24 | crc 23-16 | crc 15-8 | crc 7-0
     * | Entry_ID 31-24 | E_ID 23-16 | E_ID 15-8 | E_ID 7-0
     * | Type_ID 31-24 | Type_ID 23-16 | Type_ID 15-8 | TYpe_ID 7-0
     * | Value(s) ...
     */

    auto it = serializationVector.begin() + VECTOR_ZERO_OFFSET;
    EntryStructsUnion::getFromSerializedVector(vectorLen, it,
                                               serializationVector.end());
    it = serializationVector.begin() + VECTOR_ZERO_OFFSET + 4;
    EntryStructsUnion::getFromSerializedVector(nrEntries, it,
                                               serializationVector.end());
    savedChecksum |= serializationVector.at(VECTOR_ZERO_OFFSET + 8) << 24;
    savedChecksum |= serializationVector.at(VECTOR_ZERO_OFFSET + 9) << 16;
    savedChecksum |= serializationVector.at(VECTOR_ZERO_OFFSET + 10) << 8;
    savedChecksum |= serializationVector.at(VECTOR_ZERO_OFFSET + 11);
    assert(vectorLen == serializationVector.size());

    for (auto iterator =
             serializationVector.begin() + CONFIGURATIONS_START_OFFSET;
         iterator != serializationVector.end(); iterator++)
    {
        checksum ^= *iterator << (3 - (counter % 4)) * 8;
        counter++;
    }
    if (checksum != savedChecksum)
    {
        LOG_ERR(logger, "Corrupted saved configuration");
        return false;
    }
    it      = serializationVector.begin() + CONFIGURATIONS_START_OFFSET;
    counter = 0;
    while (it != serializationVector.end() && success && counter < nrEntries)
    {
        // Gets the ID of the entry, the ID of the data type, the value
        EntryStructsUnion::getFromSerializedVector(id, it,
                                                   serializationVector.end());
        EntryStructsUnion::getFromSerializedVector(typeId, it,
                                                   serializationVector.end());
        TypesEnum type = static_cast<TypesEnum>(typeId);
        switch (type)
        {
            case TypesEnum::COORDINATES:
                EntryStructsUnion::getFromSerializedVector(
                    coordinate, it, serializationVector.end());
                success &= setConfigurationUnsafeInternal(id, coordinate);
                break;
            case TypesEnum::FLOAT:
                EntryStructsUnion::getFromSerializedVector(
                    floatValue, it, serializationVector.end());
                success &= setConfigurationUnsafeInternal(id, floatValue);
                break;
            case TypesEnum::UINT32:
                EntryStructsUnion::getFromSerializedVector(
                    uint32Value, it, serializationVector.end());
                success &= setConfigurationUnsafeInternal(id, uint32Value);
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
 * @brief Verify if there is an existing entry given its enum entry.
 * @param configurationIndex The configuration entry ID for which we verify
 * the entry is configured.
 * @return True if such configuration entry exists in the configuration
 * otherwise False.
 */
bool RegistryFrontend::isEntryConfigured(
    const ConfigurationId configurationIndex)
{
    const std::lock_guard<std::recursive_mutex> lock(mutexForRegistry);
    auto iterator = configuration.find(configurationIndex);
    return !(iterator == configuration.end());
}

/**
 * @brief Verify that the configuration is empty or exists some setted
 * entries
 * @return True if the configuration has no entries. False otherwise
 */
bool RegistryFrontend::isConfigurationEmpty()
{
    const std::lock_guard<std::recursive_mutex> lock(mutexForRegistry);
    return configuration.empty();
};

/**
 * @brief Updates the Serialized bytes vector of the configuration actually
 * saved in the frontend with the actual configuration
 *
 */
void RegistryFrontend::updateSerializedConfiguration()
{
    const std::lock_guard<std::recursive_mutex> lock(mutexForRegistry);
    uint32_t checksum = 0;
    int counter       = 0;
    uint32_t len      = 0;
    serializationVector.clear();
    for (auto& it : configuration)
    {
        // Insert configurationID, TypeID, value for each entry
        EntryStructsUnion::insertUint32ToVector(static_cast<uint32_t>(it.first),
                                                serializationVector);
        EntryStructsUnion::insertUint32ToVector(it.second.type,
                                                serializationVector);
        it.second.appendSerializedFromUnion(serializationVector);
    }
    for (auto& it : serializationVector)
    {
        checksum ^= it << (3 - (counter % 4)) * 8;
        counter++;
    }
    /* Vector structure
     *
     * [ 0|0|0|0|0|0|0|0
     * | nr_entries 31-24 | nr_e 23-16 | nr_e 15-8 | nr_e 7-0
     * | vector_length 31-24 | v_len 23-16 | v_len 15-8 | v_len 7-0
     * | crc/checksum 31-24 | crc 23-16 | crc 15-8 | crc 7-0
     * | Entry_ID 31-24 | E_ID 23-16 | E_ID 15-8 | E_ID 7-0
     * | Type_ID 31-24 | Type_ID 23-16 | Type_ID 15-8 | TYpe_ID 7-0
     * | Value(s) ...
     */

    /* Inserts nr. configurations, length vector, checksum and then the count
     * of elements after the 8B zeroed*/
    serializationVector.insert(serializationVector.begin(),
                               static_cast<uint8_t>(checksum));
    serializationVector.insert(serializationVector.begin(),
                               static_cast<uint8_t>(checksum >> 8));
    serializationVector.insert(serializationVector.begin(),
                               static_cast<uint8_t>(checksum >> 16));
    serializationVector.insert(serializationVector.begin(),
                               static_cast<uint8_t>(checksum >> 24));

    // Insert the size of the saved configuration as 4B UINT32
    serializationVector.insert(serializationVector.begin(),
                               (configuration.size()));
    serializationVector.insert(serializationVector.begin(),
                               (configuration.size() >> 8));
    serializationVector.insert(serializationVector.begin(),
                               (configuration.size() >> 16));
    serializationVector.insert(serializationVector.begin(),
                               (configuration.size() >> 24));

    // Insert 4B size placeholder for size attribute
    serializationVector.insert(serializationVector.begin(), 4, 0);

    // Adds at the beginning 8 zeros bytes
    serializationVector.insert(serializationVector.begin(), VECTOR_ZERO_OFFSET,
                               0);
    // Write the actual vector size
    len                        = serializationVector.size();
    serializationVector.at(8)  = static_cast<uint8_t>(len >> 24);
    serializationVector.at(9)  = static_cast<uint8_t>(len >> 16);
    serializationVector.at(10) = static_cast<uint8_t>(len >> 8);
    serializationVector.at(11) = static_cast<uint8_t>(len);
}

void RegistryFrontend::saveConfiguration()
{
    const std::lock_guard<std::recursive_mutex> lock(mutexForRegistry);
    // In case the registry is armed inhibit the saving
    if (isArmed)
        return;
    updateSerializedConfiguration();
    // TODO: Re-add it when the middleware is integrated again
    // middleware.write(serializationVector);
}

/**
 * @brief Clear the configuration actually saved, resetting to empty
 * configuration. Does affect also the underlying backend.
 * @attention It does delete also the backend saved copies
 */
void RegistryFrontend::clear()
{
    const std::lock_guard<std::recursive_mutex> lock(mutexForRegistry);
    serializationVector.clear();
    configuration.clear();
    // TODO: Re-add it when the middleware is integrated again
    // middleware.clear();
}

};  // namespace Boardcore