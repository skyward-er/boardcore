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

#include <cstdint>

namespace Boardcore
{
/**
 * @brief Coordinates struct with latitude [degree], longitude [degree]
 */
struct Coordinates
{
    float latitude;
    float longitude;
};

// The Configuration ID for the configuration entries
using ConfigurationId = uint32_t;

// Types used for the union
enum class TypesEnum
{
    UINT32,
    FLOAT,
    COORDINATES
};

/**
 * @brief Union type used for the underlying saving mechanism for the
 * configuration values
 */
union TypeUnion
{
    float float_type;
    uint32_t
        uint32_type;  ///< used for both uint32_t and uint8_t type with upcast
    Coordinates coordinates_type;
};

static_assert(
    sizeof(TypeUnion) == 0x8,
    "TypeUnion size has changed, make sure to update code relying on it.");

/**
 * @brief RegistryError enumeration as return type.
 */
enum class RegistryError
{
    OK,                         ///< Correct condition
    MALFORMED_SERIALIZED_DATA,  ///< Malformed data while deserializing
    CHECKSUM_FAIL,              ///< The custom checksum check fails
    INCORRECT_TYPE,             ///< The typeId and value type not correspond
    WRONG_WRITES_SIZE,          ///< Cannot write due to wrong data size
    NO_SPACE_FOR_HEADER,  ///< There is not enough space to write the header
    NO_SUCH_TYPE,         ///< There is no such type in TypeEnum
    ARMED,            ///< The registry is armed, the operation is not allowed
    ENTRY_NOT_FOUND,  ///< Not found such entry
    WRONG_ENDIANESS,  ///< The endianess not corresponds
    BACKEND_START_FAIL,  //< Backend failed to start
    BACKEND_LOAD_FAIL,   //< Backend failed to load data
    BACKEND_SAVE_FAIL,   //< Backend failed to save data
};

/**
 * @brief Union data struct to be stored in the map. It does contain the
 * enumeration index and the value of such configuration entry
 */
struct EntryStructsUnion
{

    /**
     * @brief Default constructor to construct a new Entry Structs Union object
     */
    EntryStructsUnion() = default;

    /**
     * @brief Returns the size in byte of the type + value
     *
     * @return size_t The type + value size in Bytes
     */
    size_t sizeBytes()
    {
        size_t returnVal = sizeof(TypesEnum);
        switch (type)
        {
            case TypesEnum::COORDINATES:
                returnVal += sizeof(Coordinates);
                break;
            case TypesEnum::UINT32:
                returnVal += sizeof(uint32_t);
                break;
            case TypesEnum::FLOAT:
                returnVal += sizeof(float);
                break;
            default:
                break;
        }

        return returnVal;
    }

    /**
     * @brief Gets from the TypeUnion the float value and returns it.
     *
     * @param outValue the float value saved into the union type
     * @return True if the data type is correct w.r.t. the saved one
     */
    bool get(float& outValue)
    {
        if (type != TypesEnum::FLOAT)
            return false;
        outValue = value.float_type;
        return true;
    }

    /**
     * @brief Get from Union object the unsigned integer 32b value.
     *
     * @param outValue the uint32_t value saved into the union type
     * @return True if the data type is correct w.r.t. the saved one
     */
    bool get(uint32_t& outValue)
    {
        if (type != TypesEnum::UINT32)
            return false;
        outValue = value.uint32_type;
        return true;
    }

    /**
     * @brief Get from Union object the coordinates value.
     *
     * @param outValue the coordinates value saved into the union type
     * @return True if the data type is correct w.r.t. the saved one
     */
    bool get(Coordinates& outValue)
    {
        if (type != TypesEnum::COORDINATES)
            return false;
        outValue = value.coordinates_type;
        return true;
    }

    /**
     * @brief Set the Union object with its float value
     *
     * @param value The value to be set into the union type
     * @return The created EntryStructUnion instance for such value and type
     */
    static EntryStructsUnion make(float value)
    {
        TypeUnion returnValue;
        returnValue.float_type = value;
        return EntryStructsUnion(returnValue, TypesEnum::FLOAT);
    }

    /**
     * @brief Set the Union object with its unsigned 32bit integer value
     *
     * @param value The value to be set into the union type
     * @return The created EntryStructUnion instance for such value and type
     */
    static EntryStructsUnion make(uint32_t value)
    {
        TypeUnion returnValue;
        returnValue.uint32_type = value;
        return EntryStructsUnion(returnValue, TypesEnum::UINT32);
    }

    /**
     * @brief Set the Union object with its Coordinates value
     *
     * @param value The value to be set into the union type
     * @return The created EntryStructUnion instance for such value and type
     */
    static EntryStructsUnion make(Coordinates value)
    {
        TypeUnion returnValue;
        returnValue.coordinates_type = value;
        return EntryStructsUnion(returnValue, TypesEnum::COORDINATES);
    }

    /**
     * @brief Get the Value object
     *
     * @return TypeUnion the type of this entry
     */
    TypeUnion getValue() { return value; }

    /**
     * @brief Get the Type object
     *
     * @return TypeUnion the type of this entry
     */
    TypesEnum getType() { return type; }

private:
    TypeUnion value;
    TypesEnum type;
    /**
     * @brief Constructor for the struct. It does take the right configuration
     * struct for such entry configuration
     */
    EntryStructsUnion(TypeUnion value, TypesEnum typeId)
        : value(value), type(typeId)
    {
    }
};

}  // namespace Boardcore