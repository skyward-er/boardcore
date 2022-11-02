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

#include <assert.h>
#include <stdint.h>
#include <utils/Debug.h>

namespace Boardcore
{
namespace ModuleManagerVariables
{
/**
 * @brief Unique id sequence for classes
 */
extern uint8_t currentId;
}  // namespace ModuleManagerVariables

class Module
{
public:
    virtual ~Module() = default;
};

class ModuleManager
{
private:
    /**
     * @brief Array that contains all the possible modules created with the
     * maximum number of modules
     */
    Module *modules[256] = {nullptr};

    /**
     * @brief This function "assigns" to every type a unique sequential id
     * based on the already assigned ones
     */
    template <typename T>
    uint8_t getId()
    {
        // This thing works because a new static variable newId is created for
        // every type T and the initial assignment is "called" only when the
        // static variable is created
        static uint8_t newId = ModuleManagerVariables::currentId == 255
                                   ? 255
                                   : ModuleManagerVariables::currentId++;
        return newId;
    }

public:
    /**
     * @brief Inserts the module inside the array if not already present
     */
    template <typename T>
    void insert(T *element)
    {
        // Verify that T is a subclass of module
        static_assert(std::is_base_of<Module, T>(),
                      "Class must be subclass of Module");

        // Take the module type
        uint8_t id = getId<T>();

        // Only if the module isn't already present i add it
        if (modules[id] == nullptr)
            modules[id] = static_cast<Module *>(element);
    }

    /**
     * @brief Removes a particular module inside the array if present
     */
    template <typename T>
    void remove()
    {
        // Verify that T is a subclass of module
        static_assert(std::is_base_of<Module, T>(),
                      "Class must be subclass of Module");

        // Take the module type
        uint8_t id = getId<T>();

        // Only if the module is actually present i remove it
        if (modules[id] != nullptr)
        {
            // Call the destruction method
            delete modules[id];

            // Then remove the entry from the array
            modules[id] = nullptr;
        }
    }

    /**
     * @brief Get the Module object if present. Otherwise it creates one
     * @return Software module
     */
    template <class T>
    T *get()
    {
        // Verify that T is a subclass of module
        static_assert(std::is_base_of<Module, T>(),
                      "Class must be subclass of Module");

        // Retrieve the module type
        uint8_t id = getId<T>();

        // If the module is actually present
        if (modules[id] != nullptr)
        {
            // If the types are the same return the casted one
            return static_cast<T *>(modules[id]);
        }

        // This this part should not be considered properly working, so there is
        // an assert
        D(assert(modules[id] == nullptr));

        // I don't have any module with that type in the array so i instantiate
        // one. IF YOU ENCOUNTER A COMPILATION ERROR HERE IS PROBABLY BECAUSE
        // THE UPPER INTERFACE COULD NOT BE INSTANCED
        insert<T>(new T());
        return static_cast<T *>(modules[id]);
    }
};
}  // namespace Boardcore
