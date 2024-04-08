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

#include <type_traits>

#include "ModuleManager.h"

namespace Boardcore
{

namespace ModuleHolsterDetails
{

// Storage implementation
template <typename... Types>
struct Storage
{
    // No-op, base case
    void inject(ModuleInjector &injector) {}

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
    Type *item;

    void inject(ModuleInjector &injector)
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

}  // namespace ModuleHolsterDetails

template <typename... Types>
class ModuleHolster : public Module
{
public:
    virtual void inject(ModuleInjector &injector) override
    {
        storage.inject(injector);
    }

    template <typename T>
    T *getModule()
    {
        static_assert(ModuleHolsterDetails::Contains<T, Types...>::value,
                      "Module T is not present in the ModuleHolster");

        return storage.template get<T>();
    }

private:
    ModuleHolsterDetails::Storage<Types...> storage;
};

}  // namespace Boardcore