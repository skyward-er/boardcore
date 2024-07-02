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

#include <string>

namespace Boardcore
{

namespace TypeNameDetails
{

// This needs to be in its own namespace so that types in Boardcore:: don't get
// the namespace cut off

template <typename T>
std::string typeName()
{

    // taken from
    // https://github.com/Manu343726/ctti/blob/master/include/ctti/detail/pretty_function.hpp

#if defined(__clang__)
// clang required slightly different logic, but should be supported, keep it
// like this for now
#error "Clang does not yet support typeName"
#elif defined(__GNUC__) || defined(__GNUG__)
#define SKYWARD_PRETTY_FUNCTION __PRETTY_FUNCTION__
#define SKYWARD_PRETTY_FUNCTION_PREFIX \
    "std::string Boardcore::TypeNameDetails::typeName() [with T = "
#define SKYWARD_PRETTY_FUNCTION_SUFFIX \
    "; std::string = std::__cxx11::basic_string<char>]"
#else
#error "Compiler does not support typeName"
#endif

#define SKYWARD_PRETTY_FUNCTION_PREFIX_LEN \
    (sizeof(SKYWARD_PRETTY_FUNCTION_PREFIX) - 1)
#define SKYWARD_PRETTY_FUNCTION_SUFFIX_LEN \
    (sizeof(SKYWARD_PRETTY_FUNCTION_SUFFIX) - 1)

    std::string pretty_function{SKYWARD_PRETTY_FUNCTION};
    return std::string{
        pretty_function.begin() + SKYWARD_PRETTY_FUNCTION_PREFIX_LEN,
        pretty_function.end() - SKYWARD_PRETTY_FUNCTION_SUFFIX_LEN};

#undef SKYWARD_PRETTY_FUNCTION
#undef SKYWARD_PRETTY_FUNCTION_PREFIX
#undef SKYWARD_PRETTY_FUNCTION_SUFFIX
#undef SKYWARD_PRETTY_FUNCTION_PREFIX_LEN
#undef SKYWARD_PRETTY_FUNCTION_SUFFIX_LEN
}

}  // namespace TypeNameDetails

template <typename T>
std::string typeName()
{
    return TypeNameDetails::typeName<T>();
}

}  // namespace Boardcore