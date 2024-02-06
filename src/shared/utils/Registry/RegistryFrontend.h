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

#include <stdint.h>
#include <utils/Debug.h>
#include <utils/Registry/RegistryStructures.h>
#include <utils/Registry/TypeStructures.h>

#include <mutex>
#include <unordered_map>
#include <unordered_set>

#include "RegistryStructures.h"
#include "TypeStructures.h"

namespace Boardcore
{

/*! The Configuration ID for the configuration entries */
typedef uint32_t ConfigurationId;

/*! Types used for the union */
enum TypesEnum
{
    UINT32_T,
    FLOAT,
    COORDINATES
};

/**
 * Union data struct to be stored in the map. It does contain the enumeration
 * index and the value of such configuration entry
 */
struct EntryStructsUnion
{
    TypeUnion value;
    TypesEnum type;
    /**
     * @brief Constructor for the struct. It does take the right configuration
     * struct for such entry configuration
     */
    template <typename T>
    EntryStructsUnion(T configurationStructUnion, TypesEnum typeIndexToSet)
        : value(configurationStructUnion), type(typeIndexToSet)
    {
    }
    /**
     * @brief Gets from the TypeUnion the float value and returns it.
     */
    static void getFromUnion(const TypeUnion unionValue, float* value)
    {
        *value = unionValue.float_type;
    }

    /**
     * @brief Get from Union object the unsigned integer 8b  value.
     *
     * @param unionType the union value from which take the integer.
     * @param value the uint8_t value saved into the union type
     */
    static void getFromUnion(const TypeUnion unionValue, uint8_t* value)
    {
        // TODO: Just check is all correct...
        *value = static_cast<uint8_t>(unionValue.uint32_type);
    }

    /**
     * @brief Get from Union object the unsigned integer 32b value.
     *
     * @param unionType the union value from which take the integer.
     * @param value the uint32_t value saved into the union type

     */
    static void getFromUnion(const TypeUnion unionValue, uint32_t* value)
    {
        *value = unionValue.uint32_type;
    }

    /**
    * @brief Get from Union object the coordinates value.
    *
    * @param unionType the union value from which take the integer.
    * @param value the coordinates value saved into the union type

    */
    static void getFromUnion(const TypeUnion unionValue, Coordinates* value)
    {
        *value = unionValue.coordinates_type;
    }

    /**
     * @brief Set the Union object with its float value
     *
     * @param value The value to be set into the union type
     * @return TypeUnion the returned created type union with its float value
     * set.
     */
    static TypeUnion setUnion(float value)
    {
        TypeUnion returnValue;
        returnValue.float_type = value;
        return returnValue;
    }

    /**
     * @brief Set the Union object with its unsigned 8bit integer value
     *
     * @param value The value to be set into the union type
     * @return TypeUnion the returned created type union with its int value
     * set.
     */
    static TypeUnion setUnion(uint8_t value)
    {
        TypeUnion returnValue;
        returnValue.uint32_type = static_cast<uint32_t>(value);
        return returnValue;
    }

    /**
     * @brief Set the Union object with its unsigned 32bit integer value
     *
     * @param value The value to be set into the union type
     * @return TypeUnion the returned created type union with its int value
     * set.
     */
    static TypeUnion setUnion(uint32_t value)
    {
        TypeUnion returnValue;
        returnValue.uint32_type = value;
        return returnValue;
    }

    /**
     * @brief Get the enumeration index
     *
     * @param value the coordinates value saved into the union type
     */
    static TypesEnum getTypeIndex(uint32_t value)
    {
        return TypesEnum::UINT32_T;
    }

    static TypesEnum getTypeIndex(float value)
    {
        return TypesEnum::FLOAT;
    }

    static TypesEnum getTypeIndex(Coordinates value)
    {
        return TypesEnum::COORDINATES;
    }

    static TypesEnum getTypeIndex(uint8_t value)
    {
        return TypesEnum::UINT32_T;  //< Since static conversion to uint32_t
    }
};

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
private:
    std::unordered_map<ConfigurationId, EntryStructsUnion> configuration;
    std::recursive_mutex mutexForRegistry;
    std::unordered_set<ConfigurationId> setConfigurations;
    bool isArmed = false;

    /*! HELPER FUNCTIONS TO SET/GET UNION TYPE */

public:
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
     * @brief Returns the already existing entries of the configurations as a
     * set.
     * @return Returns an unorder set with the indexes of the configuration
     * entries.
     */
    std::unordered_set<ConfigurationId> getConfiguredEntries();

    /**
     * @brief Loads from the backend the configuration
     * @return True if the configuration exists in memory and is not corrupted,
     * False if not.
     */
    auto loadConfiguration() -> bool;

    /**
     * @brief Verify if there is an existing entry given its enum entry.
     * @param configurationIndex The configuration entry to verify.
     * @return True if such configuration entry exists in the configuration
     * otherwise False.
     */
    auto isEntryConfigured(const ConfigurationId configurationIndex) -> bool;

    /**
     * @brief Verify that the configuration is empty or exists some setted
     * entries
     * @return True if the configuration has no entries. False otherwise
     */
    auto isConfigurationEmpty() -> bool;

    /*! TYPE UNSAFE INTERFACE METHODS */

    /**
     * Method to get the value for a given configuration entry.
     * It does change the given variable with the correct value if existing
     * @tparam T The configuration struct for such configuration entry
     * @param configurationIndex Identifies the configuration entry with its
     * enumeration value
     * @param value The value to be insert for the specified configuration entry
     * @return True in case of successful insertion. False otherwise (memory
     * limits or "armed" memory)
     */
    template <typename T>
    auto getConfigurationUnsafe(const ConfigurationId configurationIndex,
                                T* value) -> bool
    {
        std::lock_guard<std::recursive_mutex> lock(mutexForRegistry);
        auto iterator = configuration.find(configurationIndex);
        if (iterator == configuration.end())
            return false;
        EntryStructsUnion::getFromUnion(iterator->second.value, value);
        // TODO: Check type enum and real type?
        return true;
    };
    /**
     * @brief Gets the value for a specified configuration entry. Otherwise
     * returns and try to set the default value
     * @tparam T The value data type to be returned and eventually set.
     * @param configurationIndex Identifies the configuration entry with its
     * enumeration value
     * @param defaultValue The default value to be returned and set (eventually)
     * in case of non-existing configuration entry
     * @return The value saved for the configuration entry in the configuration
     * or the default value if there is no such entry in the configuration
     */
    template <typename T>
    auto getOrSetDefaultConfigurationUnsafe(
        const ConfigurationId configurationIndex, T defaultValue) -> T
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
     * @brief Sets the value for the configuration entry with the specified enum
     * @tparam T The configuration struct datatype
     * @param configurationIndex The initialized configuration structure to be
     * set as configuration entry
     * @param value The value to be set for the specified configuration entry
     * @return True if it was possible to set the configurationEntry. False
     * otherwise, e.g. in case of allocation issues or "armed" memory
     */
    template <typename T>
    auto setConfigurationUnsafe(ConfigurationId configurationIndex,
                                T value) -> bool
    {
        std::lock_guard<std::recursive_mutex> lock(mutexForRegistry);
        /*! In case that the configuration is in an armed state it cannot be
         * modified */
        if (isArmed)
            return false;
        EntryStructsUnion entry(EntryStructsUnion::setUnion(value),
                                EntryStructsUnion::getTypeIndex(value));
        bool success =
            configuration.insert(std::make_pair(configurationIndex, entry))
                .second;
        if (!success)
        {
            TRACE(
                "Registry - setConfigurationUnsafe - Could not insert the "
                "configuration entry");
            return false;
        }
        success = setConfigurations.insert(configurationIndex).second;
        if (success)
            return true;
        TRACE(
            "Registry - setConfigurationUnsafe - Could not insert the "
            "configuration entry");
        return false;
    }

    /*! TYPE SAFE INTERFACE METHODS */

    /**
     * @brief Gets the saved configuration entry for such index type-safely.
     *
     * @tparam T The configuration entry value data type.
     * @param value The returned configuration entry with its current value.
     * @return True if the configuration has such entry in memory. False
     * otherwise.
     */
    template <typename T>
    auto getConfiguration(T* value) -> bool
    {
        std::lock_guard<std::recursive_mutex> lock(mutexForRegistry);
        auto iterator = configuration.find((*value).index);
        if (iterator == configuration.end())
        {
            TRACE("Registry - getConfiguration - Get configuration not found");
            return false;
        }
        // TODO: Check type enum and real type?
        (*value).value = iterator->second.value;
        return true;
    };

    /**
     * @brief Sets the configuration entry in the registry configuration using
     * the given configuration entry struct.
     *
     * @tparam T The configuration entry struct
     * @param configurationEntry The configuration entry initialized and set
     * struct to be saved in the configuration.
     * @return True if the entry is correctly saved in the registry. False
     * otherwise, e.g. in case of allocation issues or "armed" memory
     */
    template <typename T>
    auto setConfiguration(T configurationEntry) -> bool
    {
        std::lock_guard<std::mutex> lock(mutexForRegistry);
        EntryStructsUnion entryToSet = EntryStructsUnion(
            configurationEntry,
            EntryStructsUnion::getTypeIndex(configurationEntry.val));
        bool success = configuration
                           .insert(std::make_pair(configurationEntry.enumVal,
                                                  entryToSet.value))
                           .second;
        if (!success)
        {
            TRACE(
                "Registry - setConfiguration - Could not insert the "
                "configuration entry");
            return false;
        }
        return setConfigurations.insert(configurationEntry.index).second;
    }
};

}  // namespace Boardcore
