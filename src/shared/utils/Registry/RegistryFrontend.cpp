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

#include <stdint.h>
#include <utils/Debug.h>

#include <mutex>

#include "RegistryStructures.h"
#include "TypeStructures.h"
namespace Boardcore
{

/**
 * @brief Registry front end constructor. Initializes the configuration from
 * the backend.
 */
RegistryFrontend::RegistryFrontend()
{
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
auto RegistryFrontend::loadConfiguration() -> bool
{
    // TODO: Return if there is a valid configuration loaded from the backend
    return true;
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

};  // namespace Boardcore