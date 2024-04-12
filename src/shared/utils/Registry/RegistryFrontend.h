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
#include <utils/Debug.h>

#include <cstdint>
#include <mutex>
#include <unordered_map>
#include <vector>

#include "RegistryBackend.h"
#include "RegistrySerializer.h"
#include "RegistryTypes.h"

namespace Boardcore
{

/**
 * @brief This is the front-end for the registry to store and load the
 * configuration. Its methods are type unsafe since the type is determined by
 * the entry setted. It does check the data types but its job is mainly the one
 * of get and set for the given ConfigurationId, the value of the entry. It also
 * exposes methods for go into a "safe" state during armed state / flight.
 * Finally there are methods to visit the entire configuration (forEach).
 */
class RegistryFrontend
{
public:
    using EntryFunc = std::function<void(ConfigurationId, EntryStructsUnion&)>;

    /**
     * @brief Registry front end constructor. Initializes the configuration of
     * the underlying objects and reserves 1KB for the vectors and map data
     * structures.
     */
    RegistryFrontend(std::unique_ptr<RegistryBackend> backend =
                         std::make_unique<DummyBackend>());

    /**
     * @brief Start function to start frontend and other objects, such as
     * ActiveObjects, needed to write to backend, and the backend itself
     */
    [[nodiscard]] RegistryError start();

    /**
     * @brief Disables the memory registry set and allocations.
     * To be use when the rocket itself is armed and during flight.
     */
    void arm();

    /**
     * @brief Enable set methods and memory allocations.
     *
     * @note To be used when the rocket is NOT in an "armed" state and while on
     * ground.
     */
    void disarm();

    /**
     * @brief Executes immediately the predicate for each to the configuration
     * applying the callback with the id and EntryStructsUnion union as
     * parameter for each configured entry in the configuration.
     *
     * @param predicate The predicate function to execute for each configuration
     * entry
     */
    void forEach(const EntryFunc& predicate);

    /**
     * @brief Verify if there is an existing entry given its enum entry.
     *
     * @param configurationIndex The configuration entry ID for which we
     * verify the entry is configured.
     * @return True if such configuration entry exists in the configuration
     * otherwise False.
     */
    bool isEntryConfigured(const ConfigurationId configurationIndex);

    /**
     * @brief Verify that the configuration is empty or exists some setted
     * entries
     *
     * @return True if the configuration has no entries. False otherwise
     */
    bool isEmpty();

    // TYPE UNSAFE INTERFACE METHODS

    /**
     * @brief Gets the value for a given configuration entry.
     *
     * @note It does change the given variable with the correct value if
     * existing
     * @tparam T The value data type for such configuration entry
     * @param configurationIndex Identifies the configuration entry with its
     * enumeration value
     * @param outValue the specified configuration entry value
     * @return OK in case of successful insertion.
     * @return ENTRY_NOT_FOUND In case the entry is not found in the
     * configuration
     * @return INCORRECT_TYPE If the setted data type not corresponds with the
     * given value data type
     */
    template <typename T>
    RegistryError getUnsafe(const ConfigurationId configurationIndex,
                            T& outValue)
    {
        std::lock_guard<std::recursive_mutex> lock(mutexForRegistry);
        auto iterator = configuration.find(configurationIndex);
        /** Checks that the value type corresponds to the set type and finds
         * the entry*/
        if (iterator == configuration.end())
            return RegistryError::ENTRY_NOT_FOUND;
        if (!iterator->second.get(outValue))
            return RegistryError::INCORRECT_TYPE;
        return RegistryError::OK;
    }

    /**
     * @brief Gets the value for a specified configuration entry. Otherwise
     * returns default and try to set the default value
     *
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
    T getOrSetDefaultUnsafe(const ConfigurationId configurationIndex,
                            T defaultValue)
    {
        std::lock_guard<std::recursive_mutex> lock(mutexForRegistry);
        T returnValue;
        if (getUnsafe(configurationIndex, returnValue) == RegistryError::OK)
        {
            return returnValue;
        }
        if (setUnsafe(configurationIndex, defaultValue) != RegistryError::OK)
            LOG_ERR(logger,
                    "Registry - Could not insert the default configuration");
        return defaultValue;
    }

    /**
     * @brief Sets the value for the configuration entry with the specified
     * enum
     *
     * @note The set applies only to frontend, the change does not apply to
     * backend, save should be triggered manually to do so
     * @tparam T The configuration value datatype
     * @param configurationIndex The ID of the configuration entry to set
     * @param value The value to be set for the specified configuration
     * entry
     * @return OK if it was possible to set the configurationEntry.
     * @return ARMED If the registry is in an armed state, therefore setting a
     * configuration is forbidden
     */
    template <typename T>
    RegistryError setUnsafe(ConfigurationId configurationIndex, T value)
    {
        std::lock_guard<std::recursive_mutex> lock(mutexForRegistry);
        /* In case that the configuration is in an armed state it cannot be
         * modified */
        if (isArmed)
            return RegistryError::ARMED;
        EntryStructsUnion entry = EntryStructsUnion::make(value);
        auto insert = configuration.insert({configurationIndex, entry});
        if (!insert.second)
            insert.first->second = entry;
        return RegistryError::OK;
    }

    // LOAD AND SAVE TO BACKEND

    /**
     * @brief Loads from the backend the configuration
     *
     * @note The operation overrides configurations but maintains the ones that
     * are not present in the backend configuration
     * @return OK if the configuration exists in memory and is not
     * corrupted
     * @return MALFORMED_SERIALIZED_VECTOR if the vector has a bad format (see
     * serializer)
     * @return CHECKSUM_FAIL In case the saved Checksum is incorrect (see
     * serializer)
     * @return NO_SUCH_TYPE In case there are unspecified type ids (see
     * serializer)
     * @return WRONG_ENDIANESS In case the endianess of serialization not
     * corresponds (see serializer)
     */
    RegistryError load();

    /**
     * @brief Saves the configuration to the backend
     *
     * @attention: The save will be inhibited in case of "armed" state in order
     * to avoid unwanted allocations to the serializationVector during flight.
     *
     * @return OK if could save correctly
     * @return MALFORMED_SERIALIZED_DATA if the vector not have the
     * appropriate length (see serializer)
     * @return CHECKSUM_FAIL In case the saved Checksum not corresponds (see
     * serializer)
     * @return NO_SUCH_TYPE In case there are unspecified type ids (see
     * serializer)
     */
    RegistryError save();

    /**
     * @brief Clear the configuration actually saved, resetting to empty
     * configuration.
     *
     * @note Does affect the underlying backend.
     *
     * @attention It does not directly delete the backend saved copy, save
     * should be used to trigger such action
     */
    void clear();

private:
    std::recursive_mutex mutexForRegistry;
    std::unordered_map<ConfigurationId, EntryStructsUnion> configuration;
    bool isArmed = false;
    std::vector<uint8_t> serializationVector;
    std::unique_ptr<RegistryBackend> backend;
    PrintLogger logger = Logging::getLogger("registry-frontend");
};

}  // namespace Boardcore
