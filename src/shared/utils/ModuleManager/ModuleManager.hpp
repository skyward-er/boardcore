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
#pragma once

#include <typeinfo>
#include <typeindex>
#include <vector>
#include <string>
#include <unordered_map>
#include <cxxabi.h>

namespace Boardcore
{

std::string type_name_demangled(const std::type_info &info) {
    char *demangled = abi::__cxa_demangle(info.name(), nullptr, nullptr, nullptr);
    std::string demangled2{demangled};
    std::free(demangled);

    return demangled2;
}

class ModuleInjector;
class ModuleManager;

class Module
{
public:
    virtual ~Module() = default;

    virtual void inject(ModuleInjector &injector) = 0;
};

class ModuleManager {
    friend class ModuleInjector;
private:
    struct ModuleInfo {
        Module *ptr;
        std::string name;
        std::string impl;
        std::vector<std::type_index> deps;
    };

public:
    ModuleManager() {}

    template<typename T>
    [[nodiscard]] bool insert(T* module) {
        auto idx = std::type_index{typeid(T)};

        bool ok = modules.insert({idx, ModuleInfo {
            dynamic_cast<Module*>(module),
            type_name_demangled(typeid(T)),
            type_name_demangled(typeid(*module)),
            {}
        }}).second;

        return ok;
    }

    void graphviz() {
        printf("digraph {\n");

        for(auto &slot : modules) {
            for(auto &dep : slot.second.deps) {
                printf(
                    "  \"%s(%s)\" -> \"%s(%s)\"\n", 
                    slot.second.name.c_str(), 
                    slot.second.impl.c_str(), 
                    modules[dep].name.c_str(), 
                    modules[dep].impl.c_str()
                );
            }
        }

        printf("}\n");
    }

    [[nodiscard]] bool inject();

private:
    bool load_success = true;
    std::unordered_map<std::type_index, ModuleInfo> modules;
};

class ModuleInjector {
    friend class ModuleManager;
private:
    ModuleInjector(
        ModuleManager *manager,
        ModuleManager::ModuleInfo *info
    ) : manager(manager), info(info) {}

public:
    template<typename T>
    T *get() {
        auto idx = std::type_index{typeid(T)};

        auto iter = manager->modules.find(idx);
        if(iter == manager->modules.end()) {
            manager->load_success = false;
            
            std::string name = type_name_demangled(typeid(T));
            printf(
                "[%s] requires [%s], but the latter is not present\n",
                info->name.c_str(),
                name.c_str()
            );

            return nullptr;
        }

        info->deps.push_back(idx);
        return dynamic_cast<T*>(iter->second.ptr);
    }

private:
    ModuleManager *manager;
    ModuleManager::ModuleInfo *info;
};

bool ModuleManager::inject() {
    for(auto &slot : modules) {
        ModuleInjector injector(this, &slot.second);
        slot.second.ptr->inject(injector);
    }

    return load_success;
}

}  // namespace Boardcore
