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

namespace Boardcore
{

/**
 * @brief Registry front end constructor. Initializes the configuration of
 * the underlying objects and reserves 1KB for the vectors and map data
 * structures.
 */
RegistryFrontend::RegistryFrontend() : serializer(serializationVector)
{
    serializationVector.reserve(1024);
    configuration.reserve(
        1024 / sizeof(std::pair<ConfigurationId, EntryStructsUnion>));
}

/**
 * @brief Start function to start frontend and other objects, such as
 * ActiveObjects to write to backend and the backend itself
 */
void RegistryFrontend::start()
{
    // TODO: Will start the appropriate objects
}

/**
 * @brief Disables the memory registry set and allocations.
 * To be use when the rocket itself is armed and during flight.
 */
void RegistryFrontend::arm()
{
    const std::lock_guard<std::mutex> lock(mutexForRegistry);
    isArmed = true;
}

/**
 * @brief Enable set methods and memory allocations.
 * To be used when the rocket is NOT in an "armed" state and while on
 * ground.
 */
void RegistryFrontend::disarm()
{
    const std::lock_guard<std::mutex> lock(mutexForRegistry);
    isArmed = false;
}

/**
 * @brief Visits the configuration applying the callback with the id and
 * EntryStructsUnion union as parameter for each configured entry in the
 * configuration.
 */
void RegistryFrontend::visit(
    std::function<void(ConfigurationId, EntryStructsUnion&)> callback)
{
    const std::lock_guard<std::mutex> lock(mutexForRegistry);
    for (auto& it : configuration)
    {
        callback(it.first, it.second);
    }
}

/**
 * @brief Loads from the backend the configuration
 *
 * @return OK if the configuration exists in memory and is not
 * corrupted
 * @return MALFORMED_SERIALIZED_VECTOR if the vector has a bad format (see
 * serializer)
 * @return CRC_FAIL In case the saved CRC/Checksum is incorrect (see
 * serializer)
 * @return NO_SUCH_TYPE In case there are unspecified type ids (see
 * serializer)
 * @return CANNOT_INSERT In case could not insert into the configuration the
 * de-serialized element
 * @return WRONG_ENDIANESS In case the endianess of serialization not
 * corresponds (see serializer)
 */
RegistryError RegistryFrontend::load()
{
    const std::lock_guard<std::mutex> lock(mutexForRegistry);
    if (isArmed)
        return RegistryError::ARMED;
    /* TODO: get from the backend the vector*/
    // TODO: if(!middleware.load(serializationVector)) return false;
    return serializer.deserializeConfiguration(configuration);
}

/**
 * @brief Verify if there is an existing entry given its enum entry.
 *
 * @param configurationIndex The configuration entry ID for which we verify
 * the entry is configured.
 * @return True if such configuration entry exists in the configuration
 * otherwise False.
 */
bool RegistryFrontend::isEntryConfigured(
    const ConfigurationId configurationIndex)
{
    const std::lock_guard<std::mutex> lock(mutexForRegistry);
    auto iterator = configuration.find(configurationIndex);
    return !(iterator == configuration.end());
}

/**
 * @brief Verify that the configuration is empty or exists some setted
 * entries
 *
 * @return True if the configuration has no entries. False otherwise
 */
bool RegistryFrontend::isEmpty()
{
    const std::lock_guard<std::mutex> lock(mutexForRegistry);
    return configuration.empty();
}

/**
 * @brief Saves the configuration to the backend
 *
 * @attention: The save will be inhibited in case of "armed" state in order
 * to avoid unwanted allocations to the serializationVector during flight.
 *
 * @return OK if could save correctly
 * @return MALFORMED_SERIALIZED_VECTOR if the vector not have the
 * appropriate length (see serializer)
 * @return CRC_FAIL In case the saved CRC/Checksum not corresponds (see
 * serializer)
 * @return NO_SUCH_TYPE In case there are unspecified type ids (see
 * serializer)
 * @return CANNOT_INSERT In case could not insert into the configuration the
 * loaded element
 */
RegistryError RegistryFrontend::save()
{
    const std::lock_guard<std::mutex> lock(mutexForRegistry);
    // In case the registry is armed inhibit the saving
    if (isArmed)
        return RegistryError::ARMED;
    return serializer.serializeConfiguration(configuration);
    // TODO: Re-add it when the middleware is integrated again
    // middleware.write(serializationVector);
}

/**
 * @brief Clear the configuration actually saved, resetting to empty
 * configuration. Does affect also the underlying backend.
 *
 * @attention It does not directly delete the backend saved copy, save should be
 * used to trigger such action
 */
void RegistryFrontend::clear()
{
    const std::lock_guard<std::mutex> lock(mutexForRegistry);
    serializationVector.clear();
    configuration.clear();
    // TODO: Re-add it when the middleware is integrated again
    // middleware.clear();
};

}  // namespace Boardcore