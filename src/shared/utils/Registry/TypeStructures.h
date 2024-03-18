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

#include <diagnostic/PrintLogger.h>
#include <stdint.h>
#include <utils/Debug.h>

#include <condition_variable>
#include <functional>
#include <mutex>
#include <unordered_map>
#include <vector>
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
        : value(setValue), index(enumIndex)
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

/**
 * @brief Coordinates struct with latitude [degree], longitude [degree]
 */
struct Coordinates
{
    float latitude;
    float longitude;

    explicit Coordinates(const float setLatitude, const float setLongitude)
        : latitude(setLatitude), longitude(setLongitude)
    {
    }
    Coordinates() {}
};

/**
 * @brief Struct for store values with data type of uint32_t and upcasts uint8_t
 * @tparam T_ENUM Enumerator used as index for such data structure if any is
 * used.
 */
template <typename T_ENUM>
struct CoordinatesType : RootTypeStructure<Coordinates, T_ENUM>
{
    explicit CoordinatesType(const uint32_t latitude, const uint32_t longitude,
                             const T_ENUM index)
        : RootTypeStructure<Coordinates, T_ENUM>(
              Coordinates(latitude, longitude), index)
    {
    }
};

// The Configuration ID for the configuration entries
using ConfigurationId = uint32_t;

// Types used for the union
enum TypesEnum
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
        uint32_type;  //< used for both uint32_t and uint8_t type with upcast
    Coordinates coordinates_type;

    TypeUnion() {}
};

static_assert(
    sizeof(TypeUnion) == 0x8,
    "TypeUnion size has changed, make sure to update code relying on it.");

/**
 * Union data struct to be stored in the map. It does contain the enumeration
 * index and the value of such configuration entry
 */
struct EntryStructsUnion
{
    TypeUnion value;
    TypesEnum type;

    /**
     * @brief Constructor for the struct. It does take the right configuration
     * struct for such entry configuration
     */
    template <typename T>
    EntryStructsUnion(T configurationStructUnion, TypesEnum typeIndexToSet)
        : value(configurationStructUnion), type(typeIndexToSet)
    {
    }

    /**
     * @brief Gets from the TypeUnion the float value and returns it.
     *
     * @param unionType the union value from which take the integer.
     * @param valueToGet the uint8_t value saved into the union type
     *
     * @return True if the type is correct w.r.t. the saved one
     */
    bool getFromUnion(TypeUnion unionValue, float& valueToGet)
    {
        if (type != TypesEnum::FLOAT)
            return false;
        valueToGet = unionValue.float_type;
        return true;
    }

    /**
     * @brief Get from Union object the unsigned integer 8b  value.
     *
     * @param unionType the union value from which take the integer.
     * @param valueToGet the uint8_t value saved into the union type
     */
    bool getFromUnion(TypeUnion unionValue, uint8_t& valueToGet)
    {
        if (type != TypesEnum::UINT32)
            return false;
        valueToGet = static_cast<uint8_t>(unionValue.uint32_type);
        return true;
    }

    /**
     * @brief Get from Union object the unsigned integer 32b value.
     *
     * @param unionType the union value from which take the integer.
     * @param valueToGet the uint32_t value saved into the union type
     */
    bool getFromUnion(uint32_t& valueToGet)
    {
        if (type != TypesEnum::UINT32)
            return false;
        valueToGet = value.uint32_type;
        return true;
    }

    /**
     * @brief Get from Union object the unsigned integer 32b value.
     *
     * @param unionType the union value from which take the integer.
     * @param valueToGet the uint32_t value saved into the union type
     */
    bool getFromUnion(uint8_t& valueToGet)
    {
        if (type != TypesEnum::UINT32)
            return false;
        valueToGet = static_cast<uint8_t>(value.uint32_type);
        return true;
    }

    /**
     * @brief Get from Union object the coordinates value.
     *
     * @param unionType the union value from which take the integer.
     * @param valueToGet the coordinates value saved into the union type
     *
     * @return True if the type is coherent with the one saved into the
     * EntryStructUnion
     */
    bool getFromUnion(Coordinates& valueToGet)
    {
        if (type != TypesEnum::COORDINATES)
            return false;
        valueToGet = value.coordinates_type;
        return true;
    }

    /**
     * @brief Get from Union object the float value.
     *
     * @param unionType the union value from which take the integer.
     * @param valueToGet the float value saved into the union type
     *
     * @return True if the type is coherent with the one saved into the
     * EntryStructUnion
     */
    bool getFromUnion(float& valueToGet)
    {
        if (type != TypesEnum::FLOAT)
            return false;
        valueToGet = value.float_type;
        return true;
    }

    /**
     * @brief Set the Union object with its float value
     *
     * @param valueToSet The value to be set into the union type
     * @return TypeUnion the returned created type union with its float value
     * set.
     */
    static EntryStructsUnion make(float valueToSet)
    {
        TypeUnion returnValue;
        returnValue.float_type = valueToSet;
        return EntryStructsUnion(returnValue, TypesEnum::FLOAT);
    }

    /**
     * @brief Set the Union object with its unsigned 8bit integer value
     *
     * @param valueToSet The value to be set into the union type
     * @return TypeUnion the returned created type union with its int value
     * set.
     */
    static EntryStructsUnion make(uint8_t valueToSet)
    {
        TypeUnion returnValue;
        returnValue.uint32_type = static_cast<uint32_t>(valueToSet);
        return EntryStructsUnion(returnValue, TypesEnum::UINT32);
    }

    /**
     * @brief Set the Union object with its unsigned 32bit integer value
     *
     * @param valueToSet The value to be set into the union type
     * @return TypeUnion the returned created type union with its int value
     * set.
     */
    static EntryStructsUnion make(uint32_t valueToSet)
    {
        TypeUnion returnValue;
        returnValue.uint32_type = valueToSet;
        return EntryStructsUnion(returnValue, TypesEnum::UINT32);
    }

    /**
     * @brief Set the Union object with its Coordinates value
     *
     * @param valueToSet The value to be set into the union type
     * @return TypeUnion the returned created type union with its int value
     * set.
     */
    static EntryStructsUnion make(Coordinates valueToSet)
    {
        TypeUnion returnValue;
        returnValue.coordinates_type = valueToSet;
        return EntryStructsUnion(returnValue, TypesEnum::COORDINATES);
    }

    /**
     * @brief Inserts into the vector the uint32_t data as serialized data of
     * uint8_t elements, without touching the previous part of the vector
     *
     * @param serializationVector the vector where to insert the serialized data
     */
    static void insertUint32ToVector(uint32_t tempUint32,
                                     std::vector<uint8_t>& serializationVector)
    {
        serializationVector.push_back(static_cast<uint8_t>(tempUint32 >> 24));
        serializationVector.push_back(static_cast<uint8_t>(tempUint32 >> 16));
        serializationVector.push_back(static_cast<uint8_t>(tempUint32 >> 8));
        serializationVector.push_back(static_cast<uint8_t>(tempUint32));
    }

    /**
     * @brief Get the value From Vector object with the serialized value and
     * advances the iterator to the next field after the value
     *
     * @param iterator the iterator at the current vector position where the
     * value is.
     * @param end the end of the vector, to check not to exceed vector.
     */
    static void getFromSerializedVector(
        uint32_t& value, std::vector<uint8_t>::iterator& iterator,
        const std::vector<uint8_t>::iterator end)
    {
        value    = 0;
        int step = 3;
        while (iterator != end && step >= 0)
        {
            value |= (*iterator) << (8 * step);
            iterator++;
            step--;
        }
    }

    static void getFromSerializedVector(
        float& value, std::vector<uint8_t>::iterator& iterator,
        const std::vector<uint8_t>::iterator end)
    {
        uint32_t valueFromVector;
        EntryStructsUnion::getFromSerializedVector(valueFromVector, iterator,
                                                   end);
        // Needed cast for bitwise conversion for serialization
        // cppcheck-suppress invalidPointerCast
        value = *(reinterpret_cast<float*>(&valueFromVector));
    }

    static void getFromSerializedVector(
        Coordinates& value, std::vector<uint8_t>::iterator& iterator,
        const std::vector<uint8_t>::iterator end)
    {
        getFromSerializedVector(value.latitude, iterator, end);
        getFromSerializedVector(value.longitude, iterator, end);
    }

    /**
     * @brief Appends to the vector its value serialized, without touching the
     * previous part of the vector
     *
     * @param serializationVector The vector to which append the serialized
     * value
     * @return True if the append and serialization process went correctly,
     * False otherwise
     */
    bool appendSerializedFromUnion(std::vector<uint8_t>& serializationVector)
    {
        uint32_t tempUint32;
        switch (this->type)
        {
            case TypesEnum::UINT32:
                if (this->getFromUnion(tempUint32))
                {
                    insertUint32ToVector(tempUint32, serializationVector);
                    return true;
                }
                break;
            case TypesEnum::FLOAT:
                float tempFloat;
                if (this->getFromUnion(tempFloat))
                {
                    // Needed cast for bitwise conversion for serialization
                    // cppcheck-suppress invalidPointerCast
                    tempUint32 = *(reinterpret_cast<uint32_t*>(&tempFloat));

                    insertUint32ToVector(tempUint32, serializationVector);
                    return true;
                }
                break;
            case TypesEnum::COORDINATES:
                Coordinates tempCoord;
                if (this->getFromUnion(tempCoord))
                {
                    tempUint32 = *(
                        // cppcheck-suppress invalidPointerCast
                        reinterpret_cast<uint32_t*>(&tempCoord.latitude));

                    insertUint32ToVector(tempUint32, serializationVector);

                    tempUint32 = *(
                        // cppcheck-suppress invalidPointerCast
                        reinterpret_cast<uint32_t*>(&tempCoord.longitude));

                    insertUint32ToVector(tempUint32, serializationVector);
                    return true;
                }
                break;
        }
        return false;
    }
};

};  // namespace Boardcore