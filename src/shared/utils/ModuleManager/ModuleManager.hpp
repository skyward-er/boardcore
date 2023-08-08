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

#include <Singleton.h>
#include <assert.h>
#include <stdint.h>
#include <utils/Debug.h>

#include <array>
#include <atomic>

namespace Boardcore
{
class Module
{
public:
    virtual ~Module() = default;
};

/**
 * @brief The module manager is a singleton object, so it can be instantiated
 * only once. It contains all the active software modules which can be
 * accessed in a centralized way.
 *
 * @note Because modules are identified by their type, only one module per type
 * can be inserted into the module manager. This means that every module
 * conceptually behaves like a singleton.
 *
 * Example:
 * @code{.cpp}
 * class SensorsModule : public Module {...};
 * class Sensors : public SensorsModule {...};
 *
 * ModuleManager::getInstance().insert<SensorsModule>(new Sensors(args..));
 *
 * // The user
 * ModuleManager::getInstance().get<SensorsModule>();
 *
 * // This way substituting the instance below, the user doesn't actually know
 * // the difference as far as the upper interface is respected.
 * @endcode
 */
class ModuleManager : public Singleton<ModuleManager>
{
    friend class Singleton<ModuleManager>;

public:
    ModuleManager() {}

    ~ModuleManager()
    {
        // Delete all the modules to avoid memory leaks
        for (size_t i = 0; i < MODULES_NUMBER; i++)
        {
            // It is okay to have nullptr in delete
            delete modules[i];
        }
    }

    /**
     * @brief Inserts the module inside the array.
     *
     * @param element Module to be added. T must be subclass of Module.
     *
     * @returns false in case an object of the same class has already been
     * inserted or the maximum number of modules has been reached.
     *
     * @note Further insertions of modules after the first 'get()' call are not
     * allowed. Please notice also that the module manager from this point
     * handles completely the objects. Therefore at the destruction of the
     * module manager, all the modules will be deleted.
     */
    template <typename T>
    [[nodiscard]] bool insert(T *element)
    {
        // Verify that T is a subclass of module
        static_assert(std::is_base_of<Module, T>(),
                      "Class must be subclass of Module");
        static_assert((std::is_same<Module, T>() == false),
                      "Class must be subclass of Module and not Module itself");

        if (!insertionAcceptance)
        {
            assert(false &&
                   "Cannot insert any other module after first get() call");
            return false;
        }

        // Take the module type id
        size_t id = getId<T>();

        // This is the case in which the last slot is being occupied, so a
        // failure is returned
        if (id == MODULES_NUMBER)
        {
            return false;
        }

        // The module is added if only a module of the same subclass hasn't
        // already been added
        if (modules[id] == nullptr)
        {
            modules[id] = element;
            return true;
        }
        return false;
    }

    /**
     * @brief Get the Module object if present.
     * @returns T Software module.
     * @returns nullptr in case of a non existing software module.
     *
     * @note After the get call, no further insertion is allowed.
     */
    template <class T>
    T *get()
    {
        // Verify that T is a subclass of module but not actually a strict
        // Module
        static_assert(std::is_base_of<Module, T>(),
                      "Class must be subclass of Module");
        static_assert((std::is_same<Module, T>() == false),
                      "Class must be subclass of Module and not Module itself");

        // Inhibit further insertions
        insertionAcceptance = false;

        // Retrieve the module type id
        size_t id = getId<T>();

        // If the module is actually present, returns it by downcasting the
        // object. It can be done because at every type, a unique id is assigned
        if (modules[id] != nullptr)
        {
            return dynamic_cast<T *>(modules[id]);
        }

        // Fail if the module hasn't been added before
        assert(false && "Get of a non previously inserted module");
        return nullptr;
    }

private:
    static constexpr size_t MODULES_NUMBER = 256;

    /** @brief Array that contains all the possible modules */
    std::array<Module *, MODULES_NUMBER> modules = {nullptr};

    /**
     * @brief This boolean flag just enables the user to insert software modules
     * at the beginning but not after the first get.
     *
     * @note It enforces the fact that after the first get call no further
     * insertions are allowed.
     */
    std::atomic<bool> insertionAcceptance{true};

    /**
     * @brief Get the next id with respect to the current one.
     * @returns size_t incremented currentID
     *
     * @note This is not a thread safe function.
     */
    size_t getNextId()
    {
        // Static variable, initialized only the first time
        static size_t currentId = 0;

        if (currentId == MODULES_NUMBER)
        {
            return MODULES_NUMBER;
        }
        currentId++;
        return currentId;
    }

    /**
     * @brief This function "assigns" to every type a unique sequential id
     * based on the already assigned ones.
     *
     * @returns size_t A unique ID for the type T
     *
     * @note This is a thread safe function. It leverages on the cxa_guard of
     * miosix around a static variable initialization that yields
     * every thread that tries to initialize the variable concurrently. So
     * getNextId is executed atomically.
     *
     * Reference of cxa_guard function:
     * miosix/stdlib_integration/libstdcpp_integration.cpp
     */
    template <typename T>
    size_t getId()
    {
        // This thing works because a new static variable newId is created for
        // every type T and the initial assignment is "called" only when the
        // static variable is created
        static size_t newId = getNextId();
        return newId;
    }
};
}  // namespace Boardcore
