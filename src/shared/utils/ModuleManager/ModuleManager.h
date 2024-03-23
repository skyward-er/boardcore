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

#include <diagnostic/PrintLogger.h>

#include <ostream>
#include <string>
#include <typeindex>
#include <typeinfo>
#include <unordered_map>
#include <vector>

namespace Boardcore
{

class ModuleInjector;
class ModuleManager;

/**
 * @brief Interface for an injectable module.
 */
class Module
{
public:
    virtual ~Module() = default;

    /**
     * @brief Invoked by the ModuleManager to inject modules.
     *
     * @param injector Proxy class used to obtain modules.
     */
    virtual void inject(ModuleInjector &injector) = 0;
};

/**
 * @brief Main ModuleManager class.
 */
class ModuleManager
{
    friend class ModuleInjector;

private:
    struct ModuleInfo
    {
        Module *ptr;
        std::string name;
        std::string impl;
        std::vector<std::type_index> deps;
    };

public:
    ModuleManager() {}

    /**
     * @brief Insert a new module.
     *
     * @param module Module to insert in the ModuleManager.
     * @returns True if succesfull, false otherwise.
     */
    template <typename T>
    [[nodiscard]] bool insert(T *module)
    {
        return insertImpl(dynamic_cast<Module *>(module), typeid(T),
                          typeid(*module));
    }

    /**
     * @brief Generate a gaphviz compatible output showing module dependencies.
     * Needs to be called after inject.
     *
     * @param os Output stream to write to.
     */
    void graphviz(std::ostream &os);

    /**
     * @brief Inject all dependencies into all inserted modules.
     *
     * @returns True if succesfull, false otherwise.
     */
    [[nodiscard]] bool inject();

private:
    [[nodiscard]] bool insertImpl(Module *ptr,
                                  const std::type_info &module_info,
                                  const std::type_info &impl_info);

    Boardcore::PrintLogger logger =
        Boardcore::Logging::getLogger("ModuleManager");

    bool load_success = true;
    std::unordered_map<std::type_index, ModuleInfo> modules;
};

/**
 * @brief Proxy class used to obtain modules.
 */
class ModuleInjector
{
    friend class ModuleManager;

private:
    ModuleInjector(ModuleManager *manager, ModuleManager::ModuleInfo *info)
        : manager(manager), info(info)
    {
    }

public:
    /**
     * @brief Retrieve a specific module, recording dependencies and tracking
     * unsatisfied dependencies.
     *
     * @returns The requested module or nullptr if not found.
     */
    template <typename T>
    T *get()
    {
        return dynamic_cast<T *>(getImpl(typeid(T)));
    }

private:
    Module *getImpl(const std::type_info &module_info);

    Boardcore::PrintLogger logger =
        Boardcore::Logging::getLogger("ModuleManager");

    ModuleManager *manager;
    ModuleManager::ModuleInfo *info;
};

}  // namespace Boardcore
