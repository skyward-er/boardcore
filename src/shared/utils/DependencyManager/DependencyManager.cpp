/* Copyright (c) 2024 Skyward Experimental Rocketry
 * Author: Davide Mor
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

#include "DependencyManager.h"

#include <cxxabi.h>
#include <fmt/format.h>

#include <atomic>

using namespace Boardcore;

int32_t Boardcore::getNextDependencyId()
{
    static std::atomic<int32_t> NEXT_ID{0};

    int32_t next_id = NEXT_ID;
    while (next_id <= 256)
        if (NEXT_ID.compare_exchange_weak(next_id, next_id + 1))
            return next_id;

    return -1;
}

std::string demangleName(const char* name)
{
    char* demangled = abi::__cxa_demangle(name, nullptr, nullptr, nullptr);
    std::string demangled2{demangled};
    std::free(demangled);

    return demangled2;
}

void DependencyManager::graphviz(std::ostream& os)
{
    os << "digraph {" << std::endl;

    // First print out all of the nodes
    for (auto& module : modules)
        os << fmt::format("  \"{}\"", module.second.name) << std::endl;

    // Then print out the arcs
    for (auto& module : modules)
    {
        for (auto& dep : module.second.deps)
        {
            os << fmt::format("  \"{}\" -> \"{}\"", module.second.name,
                              modules[dep].name)
               << std::endl;
        }
    }

    os << "}" << std::endl;
}

bool DependencyManager::inject()
{
    load_success = true;

    for (auto& module : modules)
    {
        LOG_DEBUG(logger, "Configuring [{}]...", module.second.name);
        DependencyInjector injector{*this, module.second};
        module.second.injectable->inject(injector);
    }

    if (load_success)
        LOG_INFO(logger, "Configuring successful!");
    else
        LOG_ERR(logger, "Failed to inject modules!");

    return load_success;
}

bool DependencyManager::insertImpl(int32_t id, void* raw,
                                   Injectable* injectable, const char* name)
{
    // Check that the ID is valid
    if (id < 0)
        return false;

    return modules
        .insert({id, ModuleInfo{demangleName(name), raw, injectable, {}}})
        .second;
}

void* DependencyManager::getImpl(int32_t id)
{
    auto iter = modules.find(id);
    if (iter == modules.end())
        return nullptr;
    else
        return iter->second.raw;
}

void* DependencyInjector::getImpl(int32_t id)
{
    void* ptr = manager.getImpl(id);
    if (ptr == nullptr)
    {
        // Get failed, signal inject failure and log it
        manager.load_success = false;
        LOG_ERR(logger, "[{}] requires a modules which doesn't exist",
                info.name);
        return nullptr;
    }
    else
    {
        // Get successful, add it to the dependencies
        info.deps.push_back(id);
        return ptr;
    }
}
