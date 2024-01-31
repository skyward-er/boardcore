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

#include <unordered_map>
#include <unordered_set>

#pragma once

#include <utils/Registry/RegistryStructures.h>
#include <utils/Registry/TypeStructures.h>

namespace Boardcore
{

/**
 * This is the front-end interface for the registry in case of
 * type unsafe methods. It does not ensure type safeness.
 * It does check the data types but its job is mainly the one of
 * getting and setting given the ConfigurationEnum parameter, the value
 * for such configuration entry.
 * Also it does expose methods for change into a "safe" state the
 * registry during flight as for methods for exploring the current configuration
 */
class RegistryFrontEndInterface
{
public:
    RegistryFrontEndInterface(){};
    ~RegistryFrontEndInterface(){};

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
    std::unordered_set<ConfigurationEnum> getConfiguredEntries();

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
    bool isEntryConfigured(const ConfigurationEnum configurationIndex);

    /**
     * @brief Verify that the configuration is empty or exists some setted
     * entries
     * @return True if the configuration has no entries. False otherwise
     */
    const bool isConfigurationEmpty();

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
     * @tparam T The configuration struct for such configuration entry
     * @param configurationIndex Identifies the configuration entry with its
     * enumeration value
     * @param value The value to be insert for the specified configuration entry
     * @return True in case of successful insertion. False otherwise (memory
     * limits or "armed" memory)
     */
    template <typename T>
    bool getConfigurationUnsafe(const ConfigurationEnum configurationIndex,
                                T* value);
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
        const ConfigurationEnum configurationIndex, T defaultValue);

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
    bool setConfigurationUnsafe(const T configurationEntry);

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
                                T value);

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
    bool getConfiguration(T value);

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
    T getOrSetDefaultConfiguration(const T defaultValueStruct);

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
    bool setConfiguration(const T configurationEntry);
};

/*class RegistryFrontEnd : public RegistryFrontEndInterface
{
public:
    RegistryFrontEnd();
    ~RegistryFrontEnd();
};*/

}  // namespace Boardcore
