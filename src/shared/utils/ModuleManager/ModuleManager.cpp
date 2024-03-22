/* Copyright (c) 2022 Skyward Experimental Rocketry
 * Authors: Matteo Pignataro, Davide Mor
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

#include "ModuleManager.h"

#include <cxxabi.h>
#include <fmt/format.h>

using namespace Boardcore;

std::string type_name_demangled(const std::type_info& info)
{
    char* demangled =
        abi::__cxa_demangle(info.name(), nullptr, nullptr, nullptr);
    std::string demangled2{demangled};
    std::free(demangled);

    return demangled2;
}

void ModuleManager::graphviz(std::ostream& os)
{
    os << "digraph {" << std::endl;

    for (auto& module : modules)
    {
        for (auto& dep : module.second.deps)
        {
            os << fmt::format("  \"{}({})\" -> \"{}({})\"", module.second.name,
                              module.second.impl, modules[dep].name,
                              modules[dep].impl)
               << std::endl;
        }
    }

    os << "}" << std::endl;
}

bool ModuleManager::inject()
{
    load_success = true;

    for (auto& module : modules)
    {
        LOG_INFO(logger, "Configuring [{}]...", module.second.name);
        ModuleInjector injector{this, &module.second};
        module.second.ptr->inject(injector);
    }

    if (load_success)
    {
        LOG_INFO(logger, "Configuring succesful!");
    }
    else
    {
        LOG_ERR(logger, "Failed to inject modules!");
    }

    return load_success;
}

bool ModuleManager::insertImpl(Module* ptr, const std::type_info& module_info,
                               const std::type_info& impl_info)
{
    auto idx         = std::type_index{module_info};
    auto module_name = type_name_demangled(module_info);
    auto impl_name   = type_name_demangled(impl_info);

    return modules.insert({idx, ModuleInfo{ptr, module_name, impl_name, {}}})
        .second;
}

Module* ModuleInjector::getImpl(const std::type_info& module_info)
{
    auto idx  = std::type_index{module_info};
    auto iter = manager->modules.find(idx);
    if (iter == manager->modules.end())
    {
        manager->load_success = false;

        std::string module_name = type_name_demangled(module_info);
        LOG_ERR(logger, "[{}] requires [{}], but the latter is not present",
                info->name, module_name);

        return nullptr;
    }

    info->deps.push_back(idx);
    return iter->second.ptr;
}