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

namespace
{
constexpr uint32_t VECTOR_NR_ENTRIES_RESERVE = 40;
constexpr uint32_t NR_BYTES_ENTRY_ID =
    4;  //< Nr. bytes allocated in the vector for the entryId
constexpr uint32_t NR_BYTES_HEADER =
    20;  //< Nr. bytes allocated in the vector for the typeid
constexpr uint32_t NR_BYTES_PER_ENTRY =
    6;  //< For now assuming 1B ID, 1B type ID, 4B values
}  // namespace
namespace Boardcore
{

/**
 * @brief Registry front end constructor. Initializes the configuration from
 * the backend.
 */
RegistryFrontend::RegistryFrontend() : serializer(serializationVector)
{
    serializationVector.reserve(VECTOR_NR_ENTRIES_RESERVE * NR_BYTES_PER_ENTRY +
                                NR_BYTES_HEADER);
    elementVector.reserve(NR_BYTES_ENTRY_ID + NR_BYTES_PER_ENTRY +
                          sizeof(TypeUnion));
    configuration.reserve(VECTOR_NR_ENTRIES_RESERVE);
    // middleware.init(); //< Initializes with the backend
    // TODO: Re-add it when the middleware is integrated again
    // middleware.start();
    /*
     * TODO: The registry will load from the backend the saved configuration
     * and initialize configuration, after initialize properly the middleware
     * and backend */
}

/**
 * @brief Registry front end destructor. Saves configuration to
 * the backend.
 */
RegistryFrontend::~RegistryFrontend()
{
    /* TODO: The registry will save the configurations and also use the
     * proper destructors if needed?*/
    // saveConfiguration();
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
    const std::lock_guard<std::recursive_mutex> lock(mutexForRegistry);
    if (isArmed)
        return false;
    /* TODO: get from the backend the vector, verify the checksum, load entry by
     * entry
     */
    // TODO: if(!middleware.load(serializationVector)) return false;
    return serializer.deserializeConfiguration(configuration);
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
}

/**
 * @brief Saves the configuration to the backend
 *
 * @attention: The save will be inhibited in case of "armed" state in order
 * to avoid unwanted allocations to the serializationVector during flight.
 */
bool RegistryFrontend::saveConfiguration()
{
    const std::lock_guard<std::recursive_mutex> lock(mutexForRegistry);
    // In case the registry is armed inhibit the saving
    if (isArmed)
        return false;
    return serializer.serializeConfiguration(configuration);
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
};

}  // namespace Boardcore