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

#include "RegistryFrontend.h"

namespace Boardcore
{
RegistryFrontend::RegistryFrontend(std::unique_ptr<RegistryBackend> backend)
    : backend{std::move(backend)}
{
    serializationVector.reserve(1024);
    configuration.reserve(
        1024 / sizeof(std::pair<ConfigurationId, EntryStructsUnion>));
}

RegistryError RegistryFrontend::start()
{
    const std::lock_guard<std::recursive_mutex> lock(mutexForRegistry);
    if (!backend->start())
        return RegistryError::BACKEND_START_FAIL;

    return RegistryError::OK;
}

void RegistryFrontend::arm()
{
    const std::lock_guard<std::recursive_mutex> lock(mutexForRegistry);
    isArmed = true;
}

void RegistryFrontend::disarm()
{
    const std::lock_guard<std::recursive_mutex> lock(mutexForRegistry);
    isArmed = false;
}

void RegistryFrontend::forEach(const EntryFunc& predicate)
{
    const std::lock_guard<std::recursive_mutex> lock(mutexForRegistry);
    for (auto& it : configuration)
        predicate(it.first, it.second);
}

RegistryError RegistryFrontend::load()
{
    const std::lock_guard<std::recursive_mutex> lock(mutexForRegistry);
    if (isArmed)
        return RegistryError::ARMED;

    if (!backend->load(serializationVector))
        return RegistryError::BACKEND_LOAD_FAIL;

    RegistrySerializer serializer(serializationVector);
    return serializer.deserializeConfiguration(configuration);
}

bool RegistryFrontend::isEntryConfigured(
    const ConfigurationId configurationIndex)
{
    const std::lock_guard<std::recursive_mutex> lock(mutexForRegistry);
    auto iterator = configuration.find(configurationIndex);
    return !(iterator == configuration.end());
}

bool RegistryFrontend::isEmpty()
{
    const std::lock_guard<std::recursive_mutex> lock(mutexForRegistry);
    return configuration.empty();
}

RegistryError RegistryFrontend::save()
{
    const std::lock_guard<std::recursive_mutex> lock(mutexForRegistry);
    // In case the registry is armed inhibit the saving
    if (isArmed)
        return RegistryError::ARMED;

    RegistrySerializer serializer(serializationVector);
    RegistryError error = serializer.serializeConfiguration(configuration);
    if (error != RegistryError::OK)
        return error;

    if (!backend->save(serializationVector))
        return RegistryError::BACKEND_SAVE_FAIL;

    return RegistryError::OK;
}

void RegistryFrontend::clear()
{
    const std::lock_guard<std::recursive_mutex> lock(mutexForRegistry);
    configuration.clear();
};

}  // namespace Boardcore
