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

#include <diagnostic/PrintLogger.h>
#include <stdint.h>
#include <utils/Debug.h>

#include <condition_variable>
#include <functional>
#include <mutex>
#include <unordered_map>
#include <vector>

// TODO: Re-add it when the middleware is integrated again
// #include "RegistryMiddleware.h"
#include "RegistrySerializer.h"
#include "TypeStructures.h"

namespace Boardcore
{

/**
 * This is the front-end for the registry in case of
 * type unsafe and safe methods for type safeness.
 * It does check the data types but its job is mainly the one of
 * getting and setting given the uint32_t or ConfigurationEnum parameter, the
 * value for such configuration entry. Also it does expose methods for change
 * into a "safe" state the registry during flight as for methods for exploring
 * the current configuration
 */
class RegistryFrontend
{
public:
    std::recursive_mutex mutexForRegistry;

    RegistryFrontend();
    ~RegistryFrontend();

    /**
     * @brief Disables the memory registry set and allocations.
     * To be use when the rocket itself is armed and during flight.
     */
    void arm();

    /**
     * @brief Enable set methods and memory allocations.
     * To be used when the rocket is NOT in an "armed" state and while on
     * ground.
     */
    void disarm();

    /**
     * @brief Visits the configuration applying the callback with the id and
     * EntryStructsUnion union as parameter for each configured entry in the
     * configuration.
     */
    void visitConfiguration(
        std::function<void(ConfigurationId, EntryStructsUnion&)> callback);

    /**
     * @brief Verify if there is an existing entry given its enum entry.
     * @param configurationIndex The configuration entry ID for which we verify
     * the entry is configured.
     * @return True if such configuration entry exists in the configuration
     * otherwise False.
     */
    bool isEntryConfigured(const ConfigurationId configurationIndex);

    /**
     * @brief Verify that the configuration is empty or exists some setted
     * entries
     * @return True if the configuration has no entries. False otherwise
     */
    bool isConfigurationEmpty();

    // TYPE UNSAFE INTERFACE METHODS

    /**
     * Method to get the value for a given configuration entry.
     * It does change the given variable with the correct value if existing
     * @tparam T The configuration struct for such configuration entry
     * @param configurationIndex Identifies the configuration entry with its
     * enumeration value
     * @param value The value to be insert for the specified configuration
     * entry
     * @return True in case of successful insertion. False otherwise (memory
     * limits or "armed" memory)
     */
    template <typename T>
    bool getConfigurationUnsafe(const ConfigurationId configurationIndex,
                                T& value)
    {
        std::lock_guard<std::recursive_mutex> lock(mutexForRegistry);
        auto iterator = configuration.find(configurationIndex);
        /** Checks that the value type corresponds to the set type and finds
         * the entry*/
        return !(iterator == configuration.end()) &&
               (iterator->second.getFromUnion(value));
    }

    /**
     * @brief Gets the value for a specified configuration entry. Otherwise
     * returns and try to set the default value
     * @tparam T The value data type to be returned and eventually set.
     * @param configurationIndex Identifies the configuration entry with its
     * enumeration value
     * @param defaultValue The default value to be returned and set
     * (eventually) in case of non-existing configuration entry
     * @return The value saved for the configuration entry in the
     * configuration or the default value if there is no such entry in the
     * configuration
     */
    template <typename T>
    T getOrSetDefaultConfigurationUnsafe(
        const ConfigurationId configurationIndex, T defaultValue)
    {
        std::lock_guard<std::recursive_mutex> lock(mutexForRegistry);
        T returnValue;
        if (getConfigurationUnsafe(configuration, &returnValue))
        {
            return returnValue;
        }
        if (!setConfigurationUnsafe(configurationIndex, defaultValue))
            TRACE("Registry - Could not insert the default configuration");
        return defaultValue;
    }

    /**
     * @brief Sets the value for the configuration entry with the specified
     * enum
     * This method calls the underlying internal method and also saves the
     * configuration
     *
     * @tparam T The configuration struct datatype
     * @param configurationIndex The ID of the configuration entry to set
     * @param value The value to be set for the specified configuration
     * entry
     * @return True if it was possible to set the configurationEntry. False
     * otherwise, e.g. in case of allocation issues or "armed" memory
     */
    template <typename T>
    bool setConfigurationUnsafe(ConfigurationId configurationIndex, T value)
    {
        std::lock_guard<std::recursive_mutex> lock(mutexForRegistry);
        if (setConfigurationUnsafeInternal(configurationIndex, value))
        {
            saveConfiguration();
            return true;
        }
        return false;
    }

    // TYPE SAFE INTERFACE METHODS

    /**
     * @brief Gets the saved configuration entry for such index type-safely.
     *
     * @tparam T The configuration entry value data type.
     * @param value The returned configuration entry with its current value.
     * @return True if the configuration has such entry in memory. False
     * otherwise.
     */
    template <typename T>
    bool getConfiguration(T& value)
    {
        std::lock_guard<std::recursive_mutex> lock(mutexForRegistry);
        auto iterator = configuration.find(value.index);
        if (iterator == configuration.end())
        {
            TRACE(
                "Registry - getConfiguration - Get configuration not "
                "found");
            return false;
        }
        value.value = iterator->second.value;
        return true;
    };

    /**
     * @brief Sets the configuration entry in the registry configuration
     * using the given configuration entry struct.
     * This method calls the underlying internal method and also saves the
     * configuration
     *
     * @tparam T The configuration entry struct
     * @param configurationEntry The configuration entry initialized and set
     * struct to be saved in the configuration.
     * @return True if the entry is correctly saved in the registry. False
     * otherwise, e.g. in case of allocation issues or "armed" memory
     */
    template <typename T>
    bool setConfiguration(T configurationEntry)
    {
        std::lock_guard<std::recursive_mutex> lock(mutexForRegistry);
        if (setInternallyConfiguration(configurationEntry))
        {
            saveConfiguration();
            return true;
        }
        return false;
    }

    // DATA SERIALIZATION TO BYTES FOR BACKEND LOAD AND SAVE

    /**
     * @brief Loads from the backend the configuration
     * @return True if the configuration exists in memory and is not
     * corrupted, False if not.
     */
    bool loadConfiguration();

    /**
     * @brief Saves the configuration to the backend
     *
     * @attention: The save will be inhibited in case of "armed" state in order
     * to avoid unwanted allocations to the serializationVector during flight.
     *
     * @return true If the saving was successful
     * @return false Otherwise
     */
    void saveConfiguration();

    /**
     * @brief Clear the configuration actually saved, resetting to empty
     * configuration. Does affect also the underlying backend.
     * @attention It does delete also the backend saved copies
     */
    void clear();

private:
    std::unordered_map<ConfigurationId, EntryStructsUnion> configuration;
    bool isArmed = false;
    std::vector<uint8_t> serializationVector;
    std::vector<uint8_t> elementVector;
    RegistrySerializer serializer;
    // TODO: Re-add it when the middleware is integrated again
    // RegistryMiddlewareFlash middleware;
    PrintLogger logger = Logging::getLogger("registry-frontend");

    /**
     * @brief Sets the value for the configuration entry with the specified
     * enum. This method is internally used both by the public method and from
     * the load.
     * @tparam T The configuration struct datatype
     * @param configurationIndex The configuration entry ID
     * @param value The value to be set for the specified configuration
     * entry
     * @return True if it was possible to set the configurationEntry. False
     * otherwise, e.g. in case of allocation issues or "armed" memory
     */
    template <typename T>
    bool setConfigurationUnsafeInternal(
        const ConfigurationId configurationIndex, T value)
    {
        std::lock_guard<std::recursive_mutex> lock(mutexForRegistry);
        /* In case that the configuration is in an armed state it cannot be
         * modified */
        {
            if (isArmed)
                return false;
            EntryStructsUnion entry = EntryStructsUnion::make(value);
            bool success =
                configuration.insert(std::make_pair(configurationIndex, entry))
                    .second;
            if (!success)
            {
                TRACE(
                    "Registry - setConfigurationUnsafe - Could not insert the "
                    "configuration entry");
                LOG_ERR(logger, "Could not insert the entry");
                return false;
            }
            return true;
        }
    }

    /**
     * @brief Sets the configuration entry in the registry configuration
     * using the given configuration entry struct. This method is internally
     * used both by the public method and from the load
     *
     * @tparam T The configuration entry struct
     * @param configurationEntry The configuration entry initialized and set
     * struct to be saved in the configuration.
     * @return True if the entry is correctly saved in the registry. False
     * otherwise, e.g. in case of allocation issues or "armed" memory
     */
    template <typename T>
    bool setInternallyConfiguration(T configurationEntry)
    {
        std::lock_guard<std::recursive_mutex> lock(mutexForRegistry);
        /* In case that the configuration is in an armed state it cannot be
         * modified */
        if (isArmed)
            return false;
        EntryStructsUnion entryToSet = EntryStructsUnion(configurationEntry);
        bool success                 = configuration
                           .insert(std::make_pair(configurationEntry.enumVal,
                                                  entryToSet.value))
                           .second;
        if (!success)
        {
            TRACE(
                "Registry - setConfiguration - Could not insert the "
                "configuration entry");
            LOG_ERR(logger, "Could not insert the entry");
            return false;
        }
        saveConfiguration();
        return success;
    }

    /**
     * @brief Updates the Serialized bytes vector of the configuration actually
     * saved in the frontend with the actual configuration
     */
    void updateSerializedConfiguration();
};
}  // namespace Boardcore
