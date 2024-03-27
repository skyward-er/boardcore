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
RegistryFrontend::RegistryFrontend()
{
    serializationVector.reserve(1024);
    configuration.reserve(
        1024 / sizeof(std::pair<ConfigurationId, EntryStructsUnion>));
}

void RegistryFrontend::start()
{
    // TODO: Will start the appropriate objects
}

void RegistryFrontend::arm()
{
    const std::lock_guard<std::mutex> lock(mutexForRegistry);
    isArmed = true;
}

void RegistryFrontend::disarm()
{
    const std::lock_guard<std::mutex> lock(mutexForRegistry);
    isArmed = false;
}

void RegistryFrontend::forEach(const EntryFunc& predicate)
{
    const std::lock_guard<std::mutex> lock(mutexForRegistry);
    for (auto& it : configuration)
    {
        predicate(it.first, it.second);
    }
}

RegistryError RegistryFrontend::load()
{
    const std::lock_guard<std::mutex> lock(mutexForRegistry);
    if (isArmed)
        return RegistryError::ARMED;
    // TODO: get from the backend the vector
    RegistrySerializer serializer(serializationVector);
    return serializer.deserializeConfiguration(configuration);
}

bool RegistryFrontend::isEntryConfigured(
    const ConfigurationId configurationIndex)
{
    const std::lock_guard<std::mutex> lock(mutexForRegistry);
    auto iterator = configuration.find(configurationIndex);
    return !(iterator == configuration.end());
}

bool RegistryFrontend::isEmpty()
{
    const std::lock_guard<std::mutex> lock(mutexForRegistry);
    return configuration.empty();
}

RegistryError RegistryFrontend::save()
{
    const std::lock_guard<std::mutex> lock(mutexForRegistry);
    // In case the registry is armed inhibit the saving
    if (isArmed)
        return RegistryError::ARMED;
    RegistrySerializer serializer(serializationVector);
    return serializer.serializeConfiguration(configuration);
    // TODO: Write to backend
}

void RegistryFrontend::clear()
{
    const std::lock_guard<std::mutex> lock(mutexForRegistry);
    serializationVector.clear();
    configuration.clear();
    // TODO: clear the backend
};

}  // namespace Boardcore