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

#pragma once

#include <diagnostic/PrintLogger.h>

#include <map>
#include <numeric>
#include <ostream>
#include <string>
#include <typeindex>
#include <typeinfo>
#include <vector>

namespace Boardcore
{

/**
 * @brief Returns the next available id.
 *
 * @note THIS IS ONLY USED INTERNALLY BY getDependencyId().
 */
int32_t getNextDependencyId();

/**
 * @brief Get the ID associated with the given T type.
 */
template <typename T>
int32_t getDependencyId()
{
    static int32_t ID = getNextDependencyId();
    return ID;
}

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
 *
 * This utility is meant to be used as a dependency injector for Skyward OBSW.
 *
 * Dependencies and dependents should inherit from `Injectable`. Normally though
 * you should extend from `InjectableWithDeps` in case of dependencies.
 *
 * Here's a quick example (for more examples look at
 * src/tests/catch/test-dependencymanager.cpp):
 * @code{.cpp}
 *
 * // A simple direct dependency
 * class MyDependency1 : public Injectable {};
 *
 * // Abstracting direct dependencies with a common interface
 * class MyDependency2Iface : public Injectable {};
 * class MyDependency2 : public
 * InjectableWithDeps<InjectableBase<MyDependency2Iface>> {};
 *
 * // A simple dependant (which can become a dependency itself)
 * class MyDependant : public InjectableWithDeps<MyDependency1,
 * MyDependency2Iface> {};
 *
 * DependencyManager dependency_mgr;
 *
 * // Initialize the dependencies
 * dependency_mgr.insert<MyDependency1>(new MyDependency1());
 * dependency_mgr.insert<MyDependency2Iface>(new MyDependency2());
 * dependency_mgr.insert<MyDependant>(new MyDependant());
 *
 * // Inject and resolve all dependencies!
 * dependency_mgr.inject();
 *
 * // Optionally, print the resulting graph
 * dependency_mgr.graphviz(std::cout);
 * @endcode
 */
class DependencyManager
{
    friend class DependencyInjector;

private:
    struct ModuleInfo
    {
        std::string name;  //< Name of the type.
        void *raw;  ///< Pointer to the dependency's concrete type, returned
                    ///< when retrieving this dependency
        Injectable *injectable;  ///< Pointer to the dependency as an
                                 ///< Injectable, needed for dynamic dispatching
                                 ///< of the inject method
        std::vector<int32_t> deps;  ///< List of dependencies
    };

public:
    DependencyManager() {}

    /**
     * @brief Insert a new dependency.
     *
     * @note If T is not Injectable the compiler will fail to find this method!
     *
     * @param dependency Injectable to insert in the DependencyManager.
     * @returns True if successful, false otherwise.
     */
    template <typename T, typename = std::enable_if_t<
                              std::is_base_of<Injectable, T>::value>>
    [[nodiscard]] bool insert(T *dependency)
    {
        return insertImpl(
            getDependencyId<T>(), reinterpret_cast<void *>(dependency),
            static_cast<Injectable *>(dependency), typeid(T).name());
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
    [[nodiscard]] bool insertImpl(int32_t id, void *raw, Injectable *injectable,
                                  const char *name);

    void *getImpl(int32_t id);

    Boardcore::PrintLogger logger =
        Boardcore::Logging::getLogger("DependencyManager");

    bool load_success = true;
    // Maps from interface type name to ModuleInfo
    std::map<int32_t, ModuleInfo> modules;
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
        return reinterpret_cast<T *>(getImpl(getDependencyId<T>()));
    }

private:
    void *getImpl(int32_t id);

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

template <typename T>
struct InjectableBase
{
};

/**
 * @brief Base class for an Injectable with dependencies.
 */
template <typename... Types>
class InjectableWithDeps : public Injectable
{
protected:
    /**
     * Alias of the super class, to be used in derived classes in the
     * constructor or when overriding methods
     */
    using Super = InjectableWithDeps<Types...>;

public:
    virtual void inject(DependencyInjector &injector) override
    {
        storage.inject(injector);
    }

    /**
     * @brief Get one of the modules in Types.
     *
     * @note If T is not inside Types... the compiler will fail to find this
     * method!
     */
    template <typename T,
              typename = std::enable_if_t<
                  DependencyManagerDetails::Contains<T, Types...>::value>>
    T *getModule()
    {
        return storage.template get<T>();
    }

private:
    DependencyManagerDetails::Storage<Types...> storage;
};

/**
 * @brief Base class for an Injectable with dependencies and an Injectable
 * superclass.
 */
template <typename Base, typename... Types>
class InjectableWithDeps<InjectableBase<Base>, Types...> : public Base
{
protected:
    /**
     * Alias of the super class, to be used in derived classes in the
     * constructor or when overriding methods
     */
    using Super = InjectableWithDeps<InjectableBase<Base>, Types...>;

public:
    using InjectableSuper = Base;
    using Base::Base;  ///< Inherit constructors from Base

    static_assert(std::is_base_of<Injectable, Base>::value,
                  "Base must be Injectable");

    virtual void inject(DependencyInjector &injector) override
    {
        Base::inject(injector);
        storage.inject(injector);
    }

    /**
     * @brief Get one of the modules in Types.
     *
     * @note If T is not inside Types... the compiler will fail to find this
     * method!
     */
    template <typename T,
              typename = std::enable_if_t<
                  DependencyManagerDetails::Contains<T, Types...>::value>>
    T *getModule()
    {
        return storage.template get<T>();
    }

private:
    DependencyManagerDetails::Storage<Types...> storage;
};

}  // namespace Boardcore
