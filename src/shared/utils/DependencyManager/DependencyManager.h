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

class DependencyInjector;
class DependencyManager;

/**
 * @brief Interface for an injectable dependency.
 */
class Injectable
{
public:
    virtual ~Injectable() = default;

    /**
     * @brief Invoked by the DependencyManager to inject dependencies.
     * Override this method to retrieve dependencies from the injector via
     * `DependencyInjector::get()`.
     *
     * @param injector Proxy class used to obtain dependencies.
     */
    virtual void inject(DependencyInjector &injector) {}
};

/**
 * @brief Main DependencyManager class.
 */
class DependencyManager
{
    friend class DependencyInjector;

private:
    struct ModuleInfo
    {
        Injectable *ptr;
        // Name of the module interface
        std::string name;
        // Name of the actual concrete implementation of this module interface
        std::string impl;
        std::vector<std::type_index> deps;
    };

public:
    DependencyManager() {}

    /**
     * @brief Insert a new dependency.
     *
     * @param dependency Injectable to insert in the DependencyManager.
     * @returns True if successful, false otherwise.
     */
    template <typename T>
    [[nodiscard]] bool insert(T *dependency)
    {
        return insertImpl(dynamic_cast<Injectable *>(dependency), typeid(T),
                          typeid(*dependency));
    }

    /**
     * @brief Generate a gaphviz compatible output showing dependencies.
     * Needs to be called after inject.
     *
     * @param os Output stream to write to.
     */
    void graphviz(std::ostream &os);

    /**
     * @brief Inject all dependencies into all inserted .
     *
     * @returns True if successful, false otherwise.
     */
    [[nodiscard]] bool inject();

private:
    [[nodiscard]] bool insertImpl(Injectable *ptr,
                                  const std::type_info &module_info,
                                  const std::type_info &impl_info);

    Boardcore::PrintLogger logger =
        Boardcore::Logging::getLogger("DependencyManager");

    bool load_success = true;
    std::unordered_map<std::type_index, ModuleInfo> modules;
};

/**
 * @brief Proxy class used to obtain dependencies.
 */
class DependencyInjector
{
    friend class DependencyManager;

private:
    DependencyInjector(DependencyManager &manager,
                       DependencyManager::ModuleInfo &info)
        : manager(manager), info(info)
    {
    }

public:
    /**
     * @brief Retrieve a specific dependencies, recording it and tracking
     * unsatisfied dependencies.
     *
     * @returns The requested dependency or nullptr if not found.
     */
    template <typename T>
    T *get()
    {
        return dynamic_cast<T *>(getImpl(typeid(T)));
    }

private:
    Injectable *getImpl(const std::type_info &module_info);

    Boardcore::PrintLogger logger =
        Boardcore::Logging::getLogger("DependencyManager");

    DependencyManager &manager;
    DependencyManager::ModuleInfo &info;
};

namespace DependencyManagerDetails
{

// Storage implementation
template <typename... Types>
struct Storage
{
    // No-op, base case
    void inject(DependencyInjector &injector) {}

    // No-op, dummy get (this should never be reached)
    template <typename T>
    T *get()
    {
        return nullptr;
    }
};

template <typename Type, typename... Types>
struct Storage<Type, Types...> : public Storage<Types...>
{
    using Super = Storage<Types...>;

    // Recursive implementation
    Type *item = nullptr;

    void inject(DependencyInjector &injector)
    {
        item = injector.get<Type>();
        // Call parent function
        Super::inject(injector);
    }

    template <typename T>
    typename std::enable_if_t<std::is_same<T, Type>::value, T *> get()
    {
        return item;
    }

    template <typename T>
    typename std::enable_if_t<!std::is_same<T, Type>::value, T *> get()
    {
        return Super::template get<T>();
    }
};

// Find type in list implementation
template <typename T, typename... Types>
struct Contains : std::false_type
{
};

template <typename T, typename Type, typename... Types>
struct Contains<T, Type, Types...>
    : std::integral_constant<bool, std::is_same<T, Type>::value ||
                                       Contains<T, Types...>::value>
{
};

}  // namespace DependencyManagerDetails

template <typename... Types>
class InjectableWithDeps : public Injectable
{
public:
    virtual void inject(DependencyInjector &injector) override
    {
        storage.inject(injector);
    }

    template <typename T>
    T *getModule()
    {
        static_assert(DependencyManagerDetails::Contains<T, Types...>::value,
                      "Dependency T is not present in the dependencies");

        return storage.template get<T>();
    }

private:
    DependencyManagerDetails::Storage<Types...> storage;
};

}  // namespace Boardcore
