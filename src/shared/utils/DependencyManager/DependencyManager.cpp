/* Copyright (c) 2024 Skyward Experimental Rocketry
 * Authors: Davide Mor
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

#include <fmt/format.h>

using namespace Boardcore;

void DependencyManager::graphviz(std::ostream& os)
{
    os << "digraph {" << std::endl;

    // First print out all of the nodes
    for (auto& module : modules)
    {
        os << fmt::format("  \"{}\"", module.first) << std::endl;
    }

    // Then print out the arcs
    for (auto& module : modules)
    {
        for (auto& dep : module.second.deps)
        {
            os << fmt::format("  \"{}\" -> \"{}\"", module.first, dep)
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
        LOG_INFO(logger, "Configuring [{}]...", module.first);
        DependencyInjector injector{*this, module};
        module.second.injectable->inject(injector);
    }

    if (load_success)
    {
        LOG_INFO(logger, "Configuring successful!");
    }
    else
    {
        LOG_ERR(logger, "Failed to inject modules!");
    }

    return load_success;
}

bool DependencyManager::insertImpl(void* raw, Injectable* injectable,
                                   std::string name)
{
    return modules.insert({std::move(name), ModuleInfo{raw, injectable, {}}})
        .second;
}

void* DependencyManager::getImpl(const std::string& name)
{
    auto iter = modules.find(name);
    if (iter == modules.end())
    {
        return nullptr;
    }
    else
    {
        return iter->second.raw;
    }
}

void* DependencyInjector::getImpl(const std::string& name)
{
    void* ptr = manager.getImpl(name);
    if (ptr == nullptr)
    {
        // Get failed, signal inject failure and log it
        manager.load_success = false;
        LOG_ERR(logger, "[{}] requires [{}], which doesn't exist", info.first,
                name);
        return nullptr;
    }
    else
    {
        // Get successful, add it to the dependencies
        info.second.deps.push_back(name);
        return ptr;
    }
}
