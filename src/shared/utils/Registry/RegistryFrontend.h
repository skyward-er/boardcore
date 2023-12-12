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

#pragma once

#include <utils/Registry/TypeStructures.h>

namespace Boardcore
{
/**
 * Configuration enum, includes all the possible configurations accepted by the
 * front-end for saving.
 * Each own has its data structure, that inherits from a datatype structure,
 * in the case of the type safe interface
 * In the type unsafe interface it just refers to the index into the map.
 * */
enum ConfigurationEnum
{
    IGNITION,
};

/**
 * Ignition struct
 * Struct for the ignition timing parameter
 */
struct Ignition : FloatType<ConfigurationEnum>
{
    const static ConfigurationEnum index = ConfigurationEnum::IGNITION;
};

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
    virtual ~RegistryFrontEndInterface(){};

    /**
     * Disables the memory registry set and allocations.
     * To be use when the rocket itself is armed and during flight.
     * @return True if the memory is "armed" correctly. False otherwise.
     */
    bool arm();

    /**
     * Enable set methods and memory allocations.
     * To be used when the rocket is NOT in an "armed" state and while on
     * ground.
     * @return True if the memory is "disarmed" correctly. False otherwise.
     */
    bool disarm();

    /*! TYPE UNSAFE INTERFACE METHODS */

    /**
     * Method to get the value for a given configuration entry.
     * It does change the given variable with the correct value if existing
     * @tparam T The configuration struct for such configuration entry
     * @param configuration Identifies the configuration entry with its
     * enumeration value
     * @param value The value to be insert for the specified configuration entry
     * @return True in case of successful insertion. False otherwise (memory
     * limits or "armed" memory)
     */
    template <typename T>
    bool getConfiguration(ConfigurationEnum configuration,
                          typename T::type* value);
    /**
     * Gets the value for a specified configuration entry. Otherwise returns and
     * try to set the default value
     * @tparam T The value data type to be returned and eventually set.
     * @param configuration Identifies the configuration entry with its
     * enumeration value
     * @param defaultValue The default value to be returned and set (eventually)
     * in case of non-existing configuration entry
     * @return The value saved for the configuration entry in the configuration
     * or the default value if there is no such entry in the configuration
     */
    template <typename T>
    T getOrSetDefaultConfiguration(ConfigurationEnum configuration,
                                   T defaultValue);

    /**
     * Sets the specified configuration entry using its specific data structure
     * @tparam T The particular configuration struct for such
     * configuration entry
     * @param configurationEntry The initialized configuration structure to be
     * set as configuration entry
     * @return True if it was possible to set the configurationEntry. False
     * otherwise, e.g. in case of allocation issues or "armed" memory
     */
    template <typename T>
    bool setConfiguration(T configurationEntry);

    /**
     * Sets the value for the configuration entry with the specified enum
     * @tparam T The configuration struct datatype
     * @param configuration The initialized configuration structure to be set as
     * configuration entry
     * @param value The value to be set for the specified configuration entry
     * @return True if it was possible to set the configurationEntry. False
     * otherwise, e.g. in case of allocation issues or "armed" memory
     */
    template <typename T>
    bool setConfiguration(ConfigurationEnum configuration,
                          typename T::type value);

    /*! TYPE SAFE INTERFACE METHODS */

    /**
     *  TODO: NOTE - the configuration in case of no entry configured what
     * returns? An empty datastructure? A struct not enabled? */

    /**
     * Gets the configuration for a
     * @tparam T The configuration entry value data type.
     * @param configuration The initialized configuration structure to be set as
     * configuration entry
     * @param value
     * @return
     */
    template <typename T>
    bool getConfigurationSafe(ConfigurationEnum configuration, T* value);
    /**
     * Sets the configuration entry in the registry configuration using the
     * given configuration entry struct.
     * @tparam T The configuration entry struct
     * @param configurationEntry The configuration entry initialized and set
     * struct to be saved in the configuration.
     * @return True if the entry is correctly saved in the registry. False
     * otherwise, e.g. in case of allocation issues or "armed" memory
     */
    template <typename T>
    bool setConfigurationSafe(T configurationEntry);
};

/**
 * This is the front-end of the registry with type safe methods.
 * It does consist on using proper data structures to ensure that
 * the data type is coherent to the configuration index given.
 */
class RegistryFrontEndInterfaceSafe
{
public:
};

}  // namespace Boardcore
