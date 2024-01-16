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
#include <utils/Registry/TypeStructures.h>
namespace Boardcore
{

/**
 * @brief Configuration enum, includes all the possible configurations accepted
 * by the front-end for saving. Each own has its data structure, that inherits
 * from a datatype structure, in the case of the type safe interface In the type
 * unsafe interface it just refers to the index into the map.
 * */
enum ConfigurationEnum
{
    IGNITION,
    DEPLOYMENT_ALTITUDE
};

/**
 * @brief Union type used for the underlying saving mechanism for the
 * configuration values
 */
union TypeUnion
{
    float float_type;
    uint32_t uint32_type;
    uint8_t uint8_type;
};

/*! UNION WRAPPER STRUCTS */

/**
 * @brief Struct to "wrap" the correct union type used by the float data
 * structures. The data structures using the TypeUnion union and the float type
 * will inherit from this.
 */
template <typename T>
struct UnionWrapFloatType : FloatType<T>
{
    UnionWrapFloatType() {}
    /**
     * @brief  Get the the correct float value from an Union object
     *
     * @param unionValue the union object from which get the union
     * @return float the value to be returned from such union
     */
    static float getFromUnion(TypeUnion unionValue)
    {
        return unionValue.float_type;
    }
    /**
     * @brief Creates an Union object from the float type value
     *
     * @param valueToSet The value to be set into the returned union
     * @return TypeUnion the created union type.
     */
    static TypeUnion createUnion(float valueToSet)
    {
        TypeUnion toReturn;
        toReturn.float_type = valueToSet;
        return toReturn;
    }
    /**
     * @brief Get the Union object with the correct float value of the instance
     *
     * @return TypeUnion the union with the value for such Float type structure
     */
    TypeUnion getUnion()
    {
        TypeUnion toReturn;
        toReturn.float_type = value;
        return toReturn;
    }
};

/**
 * @brief Struct to "wrap" the correct union type used by the uint8_t data
 * structures. The data structures using the TypeUnion union and the float type
 * will inherit from this.
 */
template <typename T>
struct UnionWrapUInt8Type : UInt8Type<T>
{
    /**
     * @brief Get the the correct value from an Union object
     *
     * @param unionValue the union object from which get the union
     * @return uint8_t the value get from such union
     */
    static uint8_t getFromUnion(TypeUnion unionValue)
    {
        return unionValue.uint8_type;
    }
    /**
     * @brief Creates an Union object from the float type value
     *
     * @param valueToSet The value to be set into the returned union
     * @return TypeUnion the created union type.
     */
    static TypeUnion createUnion(uint8_t valueToSet)
    {
        TypeUnion toReturn;
        toReturn.uint8_type = valueToSet;
        return toReturn;
    }
    /**
     * @brief Get the Union object with the correct float value of the instance
     *
     * @return TypeUnion the union with the value for such Float type structure
     */
    TypeUnion getUnion()
    {
        TypeUnion toReturn;
        toReturn.uint8_type = value;
        return toReturn;
    }
};

/**
 * @brief Struct to "wrap" the correct union type used by the uint32_t data
 * structures. The data structures using the TypeUnion union and the float type
 * will inherit from this.
 */
template <typename T>
struct UnionWrapUInt32Type : UInt8Type<T>
{
    UnionWrapUInt32Type() {}
    /**
     * @brief Get the the correct uint32_t value from an Union object
     *
     * @param unionValue the union object from which get the union
     * @return uint32_t the value get from such union
     */
    static uint32_t getFromUnion(TypeUnion unionValue)
    {
        return unionValue.uint32_type;
    }
    /**
     * @brief Creates an Union object from the float type value
     *
     * @param valueToSet The value to be set into the returned union
     * @return TypeUnion the created union type.
     */
    static TypeUnion createUnion(uint32_t valueToSet)
    {
        TypeUnion toReturn;
        toReturn.uint32_type = valueToSet;
        return toReturn;
    }
    /**
     * @brief Get the Union object with the correct float value of the instance
     *
     * @return TypeUnion the union with the value for such Float type structure
     */
    TypeUnion getUnion()
    {
        TypeUnion toReturn;
        toReturn.uint32_type = value;
        return toReturn;
    }
};

/*! CONFIGURATION STRUCTS */

/**
 * @brief Ignition struct,
 * Struct for the ignition timing parameter
 */
struct Ignition : UnionWrapUInt32Type<ConfigurationEnum>
{
    // const static ConfigurationEnum
    Ignition() : UnionWrapUInt32Type<ConfigurationEnum>()
    {
        index = ConfigurationEnum::IGNITION;
    }
};

/**
 * @brief Deployment altitude stuct
 * Struct for the deployment altitude
 */
struct DeploymentAltitude : UnionWrapFloatType<ConfigurationEnum>
{
    // const static ConfigurationEnum index =
    //     ConfigurationEnum::DEPLOYMENT_ALTITUDE;
    DeploymentAltitude() : UnionWrapFloatType<ConfigurationEnum>()
    {
        index = ConfigurationEnum::DEPLOYMENT_ALTITUDE;
    }
};

};  // namespace Boardcore