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

#include <mutex>

#include "RegistryStructures.h"
#include "TypeStructures.h"
namespace Boardcore
{
/**
 * Union data struct to be stored in the map. It does contain the enumeration
 * index and the value of such configuration entry
 */
struct EntryStructsUnion
{
    TypeUnion value;
    ConfigurationEnum enumVal;
    /**
     * @brief Constructor for the struct. It does take the right configuration
     * struct for such entry configuration
     */
    template <typename T>
    EntryStructsUnion(T configurationStruct,
                      ConfigurationEnum configurationEnumIndex)
        : EntryStructsUnion{configurationStruct.getUnion(),
                            configurationEnumIndex}
    {
    }
};

/**
 * @brief Registry front end class allows to perform actions on the registry and
 * its saved configuration.
 */

class RegistryFrontEnd : RegistryFrontEndInterface
{
private:
    std::unordered_map<ConfigurationEnum, EntryStructsUnion> configuration;
    std::recursive_mutex mutexForRegistry;
    std::unordered_set<ConfigurationEnum> setConfigurations;
    bool isArmed = false;

    /*! HELPER FUNCTIONS TO SET/GET UNION TYPE */

    /**
     * @brief Gets from the TypeUnion the float value and returns it.
     */
    void getFromUnion(TypeUnion unionValue, float* value)
    {
        *value = unionValue.float_type;
    }

    /**
     * @brief Get from Union object the unsigned integer 8b  value.
     *
     * @param unionType the union value from which take the integer.
     * @param value the uint8_t value saved into the union type
     */
    void getFromUnion(TypeUnion unionValue, uint8_t* value)
    {
        *value = unionValue.uint8_type;
    }

    /**
     * @brief Get from Union object the unsigned integer 32b value.
     *
     * @param unionType the union value from which take the integer.
     * @return uint32_t value saved into the union type
     */
    void getFromUnion(TypeUnion unionValue, uint32_t* value)
    {
        *value = unionValue.uint32_type;
    }

    /**
     * @brief Set the Union object with its float value
     *
     * @param value The value to be set into the union type
     * @return TypeUnion the returned created type union with its float value
     * set.
     */
    TypeUnion setUnion(const float value)
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
    TypeUnion setUnion(const uint8_t value)
    {
        TypeUnion returnValue;
        returnValue.uint8_type = value;
        return returnValue;
    }

    /**
     * @brief Set the Union object with its unsigned 32bit integer value
     *
     * @param value The value to be set into the union type
     * @return TypeUnion the returned created type union with its int value
     * set.
     */
    TypeUnion setUnion(const uint32_t value)
    {
        TypeUnion returnValue;
        returnValue.uint32_type = value;
        return returnValue;
    }

public:
    /**
     * @brief Registry front end constructor. Initializes the configuration from
     * the backend.
     */
    RegistryFrontEnd()
    {
        /**
         * TODO: The registry will get from the backend the saved configuration
         * and initialize configuration and setConfigurations */
    }

    /**
     * @brief Disables the memory registry set and allocations.
     * To be use when the rocket itself is armed and during flight.
     */
    void arm()
    {
        std::lock_guard<std::recursive_mutex> lock(mutexForRegistry);
        isArmed = true;
    }

    /**
     * @brief Enable set methods and memory allocations.
     * To be used when the rocket is NOT in an "armed" state and while on
     * ground.
     */
    void disarm()
    {
        std::lock_guard<std::recursive_mutex> lock(mutexForRegistry);
        isArmed = false;
    }

    /**
     * @brief Returns the already existing entries of the configurations as a
     * set.
     * @return Returns an un-order set with the indexes of the configuration
     * entries.
     */
    std::unordered_set<ConfigurationEnum> getConfiguredEntries()
    {
        std::unordered_set<ConfigurationEnum> configurationSetToReturn;
        std::lock_guard<std::recursive_mutex> lock(mutexForRegistry);
        configurationSetToReturn = setConfigurations;
        return configurationSetToReturn;
    }

    /**
     * @brief Verify if the configuration exists already in memory
     * @return True if the configuration exists in memory, False if not.
     */
    bool isConfigured();

    /**
     * @brief Verify if there is an existing entry given its enum entry.
     * @param configurationIndex The configuration entry to verify.
     * @return True if such configuration entry exists in the configuration
     * otherwise False.
     */
    bool isEntryConfigured(const ConfigurationEnum configurationIndex)
    {
        std::lock_guard<std::recursive_mutex> lock(mutexForRegistry);
        auto iterator = configuration.find(configurationIndex);
        if (iterator == configuration.end())
            return false;
        return true;
    }

    /**
     * @brief Verify that the configuration is empty or exists some setted
     * entries
     * @return True if the configuration has no entries. False otherwise
     */
    bool isConfigurationEmpty()
    {
        std::lock_guard<std::recursive_mutex> lock(mutexForRegistry);
        return configuration.empty();
    };

    /**
     * @brief Verifies the integrity of the configuration saved.
     * @returns True if the configuration saved in memory has corrupted data.
     * False otherwise.
     */
    bool isConfigurationCorrupted();

    /*! TYPE UNSAFE INTERFACE METHODS */

    /**
     * Method to get the value for a given configuration entry.
     * It does change the given variable with the correct value if existing
     * @tparam T The value to be get from the configuration for such entry.
     * @param configurationIndex Identifies the configuration entry with its
     * enumeration value
     * @param value The value to be insert for the specified configuration entry
     * @return True in case of successful insertion. False otherwise (memory
     * limits or "armed" memory)
     */
    template <typename T>
    bool const getConfigurationUnsafe(
        const ConfigurationEnum configurationIndex, T* value)
    {
        std::lock_guard<std::recursive_mutex> lock(mutexForRegistry);
        auto iterator = configuration.find(configurationIndex);
        if (iterator == configuration.end)
            return false;
        getFromUnion(iterator->second, value);
        return true;
    }
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
    T getOrSetDefaultConfigurationUnsafe(
        const ConfigurationEnum configurationIndex, T defaultValue)
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
     * @brief Sets the specified configuration entry using its specific data
     * structure
     * @tparam T The particular configuration struct for such
     * configuration entry
     * @param configurationEntry The initialized configuration structure to be
     * set as configuration entry
     * @return True if it was possible to set the configurationEntry. False
     * otherwise, e.g. in case of allocation issues or "armed" memory
     */
    template <typename T>
    bool setConfigurationUnsafe(const T configurationEntry)
    {
        std::lock_guard<std::recursive_mutex> lock(mutexForRegistry);
        /*! In case that the configuration is in an armed state it cannot be
         * modified */
        if (isArmed)
            return false;
        EntryStructsUnion entry(configurationEntry.getUnion(configurationEntry),
                                configurationEntry.index);
        const auto [_, success] = configuration.insert(entry.enumVal, entry);
        if (!success)
        {
            TRACE(
                "Registry - setConfigurationUnsafe - Could not insert the "
                "configuration entry")
            return false;
        }
        const auto [_, success] =
            setConfigurations.insert(configurationEntry.index);
        if (success)
            return true;
        TRACE(
            "Registry - setConfigurationUnsafe - Could not insert the "
            "configuration entry");
        return false;
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
    bool setConfigurationUnsafe(const ConfigurationEnum configurationIndex,
                                T value)
    {
        std::lock_guard<std::recursive_mutex> lock(mutexForRegistry);
        /*! In case that the configuration is in an armed state it cannot be
         * modified */
        if (isArmed)
            return false;
        EntryStructsUnion entry(createUnion(value), configurationIndex);
        auto const [_, success] = configuration.insert(entry.enumVal, entry);
        if (!success)
        {
            TRACE(
                "Registry - setConfigurationUnsafe - Could not insert the "
                "configuration entry");
            return false;
        }
        auto const [_, success] = setConfigurations.insert(configurationIndex);
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
    bool getConfiguration(T* value)
    {
        std::lock_guard<std::recursive_mutex> lock(mutexForRegistry);
        auto iterator = configuration.find((*value).index);
        if (iterator == configuration.end())
        {
            TRACE("Registry - getConfiguration - Get configuration not found");
            return false;
        }
        (*value).value = iterator->second;
        return true;
    };

    /**
     * @brief Gets the saved configuration entry for such index type-safely.
     * If such element does not exists in the configuration, returns the default
     * value given as parameter
     *
     * @tparam T The configuration entry value data type.
     * @param defaultValue The struct for such entry with the default value set.
     * @return The configuration structure from the current configuration or the
     * default one.
     */
    template <typename T>
    T getOrSetDefaultConfiguration(const T defaultValueStruct)
    {
        std::lock_guard<std::recursive_mutex> lock(mutexForRegistry);
        auto iterator = configuration.find(defaultValueStruct.index);
        if (iterator == configuration.end())
        {
            TRACE(
                "Registry - getConfigurationOrDefault - Get configuration not "
                "found, setting a default one");
            if (armed)
            {
                TRACE(
                    "Registry - getConfigurationOrDefault - "
                    "Could not configure the default value due to armed "
                    "registry");
                return defaultValueStruct;
            }
            EntryStructsUnion entryToAdd =
                EntryStructsUnion(defaultValueStruct);
            configuration.insert(entryToAdd);
            return defaultValueStruct;
        }
        EntryStructsUnion entryToReturn = EntryStructsUnion(iterator->second);
        return entryToReturn;
    }

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
    bool setConfiguration(const T configurationEntry)
    {
        std::lock_guard<std::mutex> lock(mutexForRegistry);
        EntryStructsUnion entryToSet =
            EntryStructsUnion(configurationEntry, configurationEntry.enumVal);
        auto const [_, success] =
            configuration.insert(entryToSet.enumVal, entryToSet);
        if (!success)
        {
            TRACE(
                "Registry - setConfiguration - Could not insert the "
                "configuration entry")
            return false;
        }
        auto const [_, success] =
            setConfigurations.insert(configurationEntry.index);
        if (success)
            return true;
        return false;
    }
};
}  // namespace Boardcore