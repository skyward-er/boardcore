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

class ModuleManager : public Singleton<ModuleManager>
{
    friend class Singleton<ModuleManager>;

public:
    /**
     * @brief Construct a new Module Manager object and initialize the current
     * id and vector
     */
    ModuleManager()
    {
        // Init the first ID
        currentId = 0;
    }

    /**
     * @brief Destroy the Module Manager object and delete all the modules
     */
    ~ModuleManager()
    {
        // Delete all the modules to avoid memory leak
        for (size_t i = 0; i < MODULES_NUMBER; i++)
        {
            // Delete the object created with a new
            delete modules[i];
        }
    }

    /**
     * @brief Inserts the module inside the array if not already present. The
     * module manager also doesn't allow the user to insert further modules
     * after the first get is performed. PLEASE NOTICE THAT THE MODULE MANAGER
     * FROM THIS POINT HANDLES COMPLETELY THE OBJECTS. SO, AT THE END OF THE
     * MODULE MANAGER EXISTENCE, ALL THE MODULES WILL BE DELETED.
     */
    template <typename T>
    bool insert(T *element)
    {
        // Verify that T is a subclass of module
        static_assert(std::is_base_of<Module, T>(),
                      "Class must be subclass of Module");
        static_assert((std::is_same<Module, T>() == false),
                      "Class must be subclass of Module and not Module itself");

        // This assertion enforces the fact that no software module should be
        // added after the first get
        assert(insertionAcceptance.load());

        // Take the module type
        size_t id = getId<T>();

        // This is the case in which the last slot is being occupied, so i
        // return a failure
        if (id == MODULES_NUMBER - 1)
        {
            return false;
        }

        // Only if the module isn't already present i add it casting to the
        // module interface.
        if (modules[id] == nullptr)
        {
            modules[id] = static_cast<Module *>(element);
            return true;
        }

        // This is the case when someone tries to insert an already existing
        // module
        return false;
    }

    /**
     * @brief Get the Module object if present. Otherwise it creates one. PLEASE
     * NOTE THAT AFTER THE FIRST GET, NO FURTHER INSERTION IS ALLOWED
     * @return Software module
     */
    template <class T>
    T *get()
    {
        // Verify that T is a subclass of module
        static_assert(std::is_base_of<Module, T>(),
                      "Class must be subclass of Module");
        static_assert((std::is_same<Module, T>() == false),
                      "Class must be subclass of Module and not Module itself");

        // Set the insertion acceptance to false (inhibit the insertion after
        // the first get)
        insertionAcceptance = false;

        // Retrieve the module type
        size_t id = getId<T>();

        // If the module is actually present i return it by downcasting the
        // object. It can be done because at every type, a unique id is assigned
        if (modules[id] != nullptr)
        {
            // If the types are the same return the casted one
            return static_cast<T *>(modules[id]);
        }

        // The fact that a user is trying to access to a non previously added
        // module should not be considered properly working, so there is an
        // assert
        assert(modules[id] == nullptr);

        // The nullptr is returned because after the assert it is pretty clear
        // whether it would crash the software or not in debug mode. Also,
        // trying to instantiate the module may result in an unwanted behaviour
        return nullptr;
    }

private:
    static constexpr size_t MODULES_NUMBER = 256;

    /**
     * @brief Array that contains all the possible modules created with the
     * maximum number of modules
     */
    std::array<Module *, MODULES_NUMBER> modules = {nullptr};

    /**
     * @brief Id that stores the maximum id assigned so far (atomic)
     */
    std::atomic<size_t> currentId;

    /**
     * @brief This boolean flag just enables the user to insert software modules
     * at the beginning but not after the first get. With this we enforce the
     * fact that AFTER A GET, THERE SHOULDN'T BE ANY INSERTIONS
     */
    std::atomic<bool> insertionAcceptance{true};

    /**
     * @brief This function "assigns" to every type a unique sequential id
     * based on the already assigned ones
     */
    template <typename T>
    size_t getId()
    {
        // This thing works because a new static variable newId is created for
        // every type T and the initial assignment is "called" only when the
        // static variable is created
        static size_t newId =
            currentId == MODULES_NUMBER ? MODULES_NUMBER : currentId++;
        return newId;
    }
};
}  // namespace Boardcore
