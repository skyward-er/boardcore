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

constexpr uint32_t vectorNrEntriesReserve = 40;
constexpr uint32_t nrBytesEntryId =
    1;  //< Nr. bytes allocated in the vector for the entryId
constexpr uint32_t nrBytesTypeId =
    1;  //< Nr. bytes allocated in the vector for the typeid
constexpr uint32_t nrBytesPerEntry =
    6;  //< For now assuming 1B ID, 1B type ID, 4B values
constexpr uint32_t vectorZeroOffset =
    8;  //< 8 zeroed bytes offset before real vector data
constexpr uint32_t configurationsStartOffset =
    vectorZeroOffset +
    6; /*< Nr. bytes from start to the base where
       configuration starts (zero B, Nr. entries, Len_vector, checksum)*/
namespace Boardcore
{

/**
 * @brief Registry front end constructor. Initializes the configuration from
 * the backend.
 */
RegistryFrontend::RegistryFrontend()
{
    serializationVector.reserve(vectorNrEntriesReserve * nrBytesPerEntry);
    elementVector.reserve(nrBytesEntryId + nrBytesPerEntry + sizeof(TypeUnion));
    configuration.reserve(vectorNrEntriesReserve * nrBytesPerEntry);
    setConfigurations.reserve(vectorNrEntriesReserve);
    /**
     * TODO: The registry will get from the backend the saved configuration
     * and initialize configuration and setConfigurations */
}

/**
 * @brief Registry front end destructor. Saves configuration to
 * the backend.
 */
RegistryFrontend::~RegistryFrontend()
{
    /**
     * TODO: The registry will save the configurations and also use the
     * proper destructors */
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
 * @brief Returns the already existing entries of the configurations as a
 * set.
 * @return Returns an un-order set with the indexes of the configuration
 * entries.
 */
auto RegistryFrontend::getConfiguredEntries()
    -> std::unordered_set<ConfigurationId>
{
    std::unordered_set<ConfigurationId> configurationSetToReturn;
    const std::lock_guard<std::recursive_mutex> lock(mutexForRegistry);
    configurationSetToReturn = setConfigurations;
    return configurationSetToReturn;
}

/**
 * @brief Loads from the backend the configuration
 * @return True if the configuration exists in memory and is not corrupted,
 * False if not.
 */
bool RegistryFrontend::loadConfiguration()
{
    /**TODO: get from the backend the vector, verify the checksum, load entry by
     * entry
     */
    /*! TODO: Method that will call the backend and initializes the vector */
    uint32_t id = 0, typeId, checksum = 0, savedChecksum = 0, len = 0,
             uint32Value;
    Coordinates coordinate;
    float floatValue;
    uint8_t nrEntries, counter = 0;
    bool success = true;
    const std::lock_guard<std::recursive_mutex> lock(mutexForRegistry);

    configuration.clear();
    setConfigurations.clear();
    //[8 0s | nr | len | s_c | s_c | s_c | s_c]
    nrEntries = serializationVector.at(vectorZeroOffset);
    len       = serializationVector.at(vectorZeroOffset + 1);
    savedChecksum |= serializationVector.at(vectorZeroOffset + 2) << 24;
    savedChecksum |= serializationVector.at(vectorZeroOffset + 3) << 16;
    savedChecksum |= serializationVector.at(vectorZeroOffset + 4) << 8;
    savedChecksum |= serializationVector.at(vectorZeroOffset + 5);

    for (auto iterator =
             serializationVector.begin() + configurationsStartOffset;
         iterator != serializationVector.end(); iterator++)
    {
        checksum ^= *iterator << (3 - (counter % 4)) * 8;
        counter++;
    }
    /*! There is an issue with the checksum!!*/
    if (checksum != savedChecksum)
    {
        return false;
    }
    auto iterator = serializationVector.begin() + configurationsStartOffset;
    while (iterator != serializationVector.end() && success)
    {
        /*! Gets the ID of the entry, the ID of the data type, the value*/
        EntryStructsUnion::getFromSerializedVector(id, iterator,
                                                   serializationVector.end());
        EntryStructsUnion::getFromSerializedVector(typeId, iterator,
                                                   serializationVector.end());
        TypesEnum type = static_cast<TypesEnum>(typeId);
        switch (type)
        {
            case TypesEnum::COORDINATES:
                EntryStructsUnion::getFromSerializedVector(
                    coordinate, iterator, serializationVector.end());
                success &= setConfigurationUnsafe(id, coordinate);
                break;
            case TypesEnum::FLOAT:
                EntryStructsUnion::getFromSerializedVector(
                    floatValue, iterator, serializationVector.end());
                success &= setConfigurationUnsafe(id, floatValue);
                break;
            case TypesEnum::UINT32_T:
                EntryStructsUnion::getFromSerializedVector(
                    uint32Value, iterator, serializationVector.end());
                success &= setConfigurationUnsafe(id, uint32Value);
                break;
            default:
                success = false;
                break;
        }
    }
    return success;
}

/**
 * @brief Verify if there is an existing entry given its enum entry.
 * @param configurationIndex The configuration entry to verify.
 * @return True if such configuration entry exists in the configuration
 * otherwise False.
 */
auto RegistryFrontend::isEntryConfigured(
    const ConfigurationId configurationIndex) -> bool
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
auto RegistryFrontend::isConfigurationEmpty() -> bool
{
    const std::lock_guard<std::recursive_mutex> lock(mutexForRegistry);
    return configuration.empty();
};

/**
 * @brief Get the Serialized bytes vector of the configuration actually
 * saved in the frontend
 *
 * @return std::vector<byte> The serialized data of the configuration
 */
std::vector<uint8_t>& RegistryFrontend::getSerializedConfiguration()
{
    const std::lock_guard<std::recursive_mutex> lock(mutexForRegistry);
    serializationVector.clear();
    uint32_t checksum = 0;
    int counter       = 0;
    for (auto it = configuration.begin(); it != configuration.end(); it++)
    {
        /*! Insert configurationID, TypeID, value for each entry */
        EntryStructsUnion::insertUint32ToVector(
            static_cast<uint32_t>(it->first), serializationVector);
        EntryStructsUnion::insertUint32ToVector(it->second.type,
                                                serializationVector);
        it->second.appendSerializedFromUnion(serializationVector);
    }
    for (auto iterator = serializationVector.begin();
         iterator != serializationVector.end(); iterator++)
    {
        checksum ^= *iterator << (3 - (counter % 4)) * 8;
        counter++;
    }
    //[8 0s | nr | len | s_c 31-24 | s_c 23-16 | s_c 15-8 | s_c 7-0]
    /*! Inserts nr. configurations, length vector, checksum and then the count
     * of elements after the 8B zeroed*/
    serializationVector.insert(serializationVector.begin(),
                               static_cast<uint8_t>(checksum));
    serializationVector.insert(serializationVector.begin(),
                               static_cast<uint8_t>(checksum >> 8));
    serializationVector.insert(serializationVector.begin(),
                               static_cast<uint8_t>(checksum >> 16));
    serializationVector.insert(serializationVector.begin(),
                               static_cast<uint8_t>(checksum >> 24));
    serializationVector.insert(
        serializationVector.begin(),
        serializationVector.size() + 2 + vectorZeroOffset);
    serializationVector.insert(serializationVector.begin(),
                               configuration.size());
    /*! Adds at the beginning 8 zeros bytes*/
    serializationVector.insert(serializationVector.begin(), vectorZeroOffset,
                               0);
    return serializationVector;
}

bool RegistryFrontend::saveConfiguration()
{
    /*! TODO: Will trigger the saving / send the vector to the backend.*/
    getSerializedConfiguration();
    return false;
}
};  // namespace Boardcore