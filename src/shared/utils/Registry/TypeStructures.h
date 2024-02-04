/* Copyright (c) 2023 Skyward Experimental Rocketry
 * Author: Nicolò Caruso
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
namespace Boardcore
{

/**
 * @brief This is an empty structure from which the datatype structures inherit
 * from. It does define getting and setting methods for exploring its state.
 * This methods will be redefined by the actual data struct.
 * @tparam T_VAL Datatype for the values stored in the data structure.
 * @tparam T_ENUM Enumerator used as index for such data structure if any is
 * used.
 */
template <typename T_VAL, typename T_ENUM>
struct RootTypeStructure
{
    T_VAL value;
    T_ENUM index;
    explicit RootTypeStructure(T_VAL setValue, T_ENUM enumIndex)
        : index(enumIndex), value(setValue)
    {
    }
};

/**
 * @brief Struct for store values with data type of floats
 * @tparam T_ENUM Enumerator used as index for such data structure if any is
 * used.
 */
template <typename T_ENUM>
struct FloatType : RootTypeStructure<float, T_ENUM>
{
    explicit FloatType(const float val, const T_ENUM index)
        : RootTypeStructure<float, T_ENUM>(val, index)
    {
    }
};

/**
 * @brief Struct for store values with data type of uint32_t and upcasts uint8_t
 * @tparam T_ENUM Enumerator used as index for such data structure if any is
 * used.
 */
template <typename T_ENUM>
struct UInt32Type : RootTypeStructure<uint32_t, T_ENUM>
{
    explicit UInt32Type(const uint32_t val, const T_ENUM index)
        : RootTypeStructure<uint32_t, T_ENUM>(val, index)
    {
    }
};

};  // namespace Boardcore