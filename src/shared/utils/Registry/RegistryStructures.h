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
#include <memory.h>
#include <utils/Registry/TypeStructures.h>
namespace Boardcore
{

/**
 * @brief Configuration enum, includes all the possible configurations accepted
 * by the front-end for saving. Each own has its data structure, that inherits
 * from a datatype structure, in the case of the type safe interface In the type
 * unsafe interface it just refers to the index into the map.
 * For reference, see the lib/mavlink-skyward-lib/message_definitions/lyra.xml
 * file on the <!-- FROM GROUND TO ROCKET --> messages section
 */
enum ConfigurationEnum
{
    REFERENCE_ALTITUDE,
    REFERENCE_TEMPERATURE,
    ORIENTATION,
    COORDINATES,
    DEPLOYMENT_ALTITUDE,
    TARGET_COORDINATES,
    ALGORITHM,
    MAIN_VALVE_ATOMIC_TIMING,
    VENTING_VALVE_ATOMIC_TIMING,
    RELEASE_VALVE_ATOMIC_TIMING,
    FILLING_VALVE_ATOMIC_TIMING,
    MAIN_VALVE_MAX_APERTURE,
    VENTING_VALVE_MAX_APERTURE,
    RELEASE_VALVE_MAX_APERTURE,
    FILLING_VALVE_MAX_APERTURE,
    IGNITION_TIME,
    AIR_BRAKES_SERVO,
    EXPULSION_SERVO,
    PARAFOIL_LEFT_SERVO,
    PARAFOIL_RIGHT_SERVO,
    MAIN_VALVE_SERVO,
    VENTING_VALVE_SERVO,
    RELEASE_VALVE_SERVO,
    FILLING_VALVE_SERVO,
    DISCONNECT_SERVO,
};

/**
 * @brief Coordinates struct.
 *
 */
struct Coordinates
{
    uint32_t x;
    uint32_t y;

    explicit Coordinates(const uint32_t setX, const uint32_t setY)
        : x(setX), y(setY)
    {
    }
    Coordinates() {}
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
    Coordinates coordinates_type;

    TypeUnion() {}
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
    UnionWrapFloatType(const float val, const T index)
        : FloatType<T>(val, index)
    {
    }
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
        toReturn.float_type = this->value;
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
    UnionWrapUInt8Type(const uint8_t val, const T index)
        : UInt8Type<T>(val, index)
    {
    }
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
     * @return TypeUnion the union with the value for such uint8_t type
     * structure
     */
    TypeUnion getUnion()
    {
        TypeUnion toReturn;
        toReturn.uint8_type = this->value;
        return toReturn;
    }
};

/**
 * @brief Struct to "wrap" the correct union type used by the uint32_t data
 * structures. The data structures using the TypeUnion union and the float type
 * will inherit from this.
 */
template <typename T>
struct UnionWrapUInt32Type : UInt32Type<T>
{
    UnionWrapUInt32Type(const int32_t val, const T index)
        : UInt32Type<T>(val, index)
    {
    }
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
     * @return TypeUnion the union with the value for such uint32_t type
     * structure
     */
    TypeUnion getUnion()
    {
        TypeUnion toReturn;
        toReturn.uint32_type = this->value;
        return toReturn;
    }
};

// TODO: CHANGE COMMENTS!
/**
 * @brief Struct to "wrap" the correct union type used by the pair uint32_t data
 * structures. The data structures using the TypeUnion union and the float type
 * will inherit from this.
 */
template <typename T>
struct UnionWrapUInt32Coordinates : RootTypeStructure<Coordinates, T>
{
    UnionWrapUInt32Coordinates(const int32_t x, const int32_t y, const T index)
        : RootTypeStructure<Coordinates, T>(Coordinates(x, y), index)
    {
    }
    /**
     * @brief Get the the uint32_t x value from an Union object
     *
     * @param unionValue the union object from which get the union
     * @return the x value get from such union
     */
    static uint32_t getXFromUnion(TypeUnion unionValue)
    {
        return unionValue.coordinates_type.x;
    }
    /**
     * @brief Get the the uint32_t y value from an Union object
     *
     * @param unionValue the union object from which get the union
     * @return uint32_t the y value get from such union
     */
    static uint32_t getYFromUnion(TypeUnion unionValue)
    {
        return unionValue.coordinates_type.y;
    }
    /**
     * @brief Creates an Union object from the x,y uint32_t type values
     *
     * @param valueToSet The value to be set into the returned union
     * @return TypeUnion the created union type.
     */
    static TypeUnion createUnion(uint32_t xToSet, uint32_t yToSet)
    {
        TypeUnion toReturn;
        toReturn.coordinates_type.x = xToSet;
        toReturn.coordinates_type.y = yToSet;
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
        toReturn.coordinates_type = this->value;
        return toReturn;
    }
};

/*! CONFIGURATION STRUCTS */

/**
 * @brief IgnitionTime struct for the [ms] time for ignition
 * Struct for the ignition timing parameter
 */
struct IgnitionTime : UnionWrapUInt32Type<ConfigurationEnum>
{
    explicit IgnitionTime(const uint32_t val)
        : UnionWrapUInt32Type<ConfigurationEnum>(
              val, ConfigurationEnum::IGNITION_TIME)
    {
    }
};

/**
 * @brief Deployment altitude stuct
 * Struct for the deployment altitude
 */
struct DeploymentAltitude : UnionWrapFloatType<ConfigurationEnum>
{
    explicit DeploymentAltitude(const float val)
        : UnionWrapFloatType<ConfigurationEnum>(
              val, ConfigurationEnum::DEPLOYMENT_ALTITUDE)
    {
    }
};

/**
 * @brief Target coordinates struct
 * Struct for the x,y target coordinates
 */
struct TargetCoordinates : UnionWrapUInt32Coordinates<ConfigurationEnum>
{
    explicit TargetCoordinates(const uint32_t x, const uint32_t y)
        : UnionWrapUInt32Coordinates<ConfigurationEnum>(
              x, y, ConfigurationEnum::TARGET_COORDINATES)
    {
    }
};

/**
 * @brief Algorithm registry struct for the algorithm number for parafoil
 * guidance and GSE tars
 *
 */
struct Algorithm : UnionWrapUInt8Type<ConfigurationEnum>
{
    explicit Algorithm(const uint8_t val)
        : UnionWrapUInt8Type<ConfigurationEnum>(val,
                                                ConfigurationEnum::ALGORITHM)
    {
    }
};

/**
 * @brief Main valve atomic timing registry struct for the [ms] timing of main
 * valve
 *
 */
struct MainValveAtomicTiming : UnionWrapUInt32Type<ConfigurationEnum>
{
    explicit MainValveAtomicTiming(const uint32_t val)
        : UnionWrapUInt32Type<ConfigurationEnum>(
              val, ConfigurationEnum::MAIN_VALVE_ATOMIC_TIMING)
    {
    }
};

/**
 * @brief Venting valve atomic timing registry struct for the [ms] timing of
 * venting valve
 *
 */
struct VentingValveAtomicTiming : UnionWrapUInt32Type<ConfigurationEnum>
{
    explicit VentingValveAtomicTiming(const uint32_t val)
        : UnionWrapUInt32Type<ConfigurationEnum>(
              val, ConfigurationEnum::VENTING_VALVE_ATOMIC_TIMING)
    {
    }
};

/**
 * @brief Release valve atomic timing registry struct for the [ms] timing of
 * release valve
 *
 */
struct ReleaseValveAtomicTiming : UnionWrapUInt32Type<ConfigurationEnum>
{
    explicit ReleaseValveAtomicTiming(const uint32_t val)
        : UnionWrapUInt32Type<ConfigurationEnum>(
              val, ConfigurationEnum::RELEASE_VALVE_ATOMIC_TIMING)
    {
    }
};

/**
 * @brief Filling valve atomic timing registry struct for the [ms] timing of
 * filling valve
 *
 */
struct FillingValveAtomicTiming : UnionWrapUInt32Type<ConfigurationEnum>
{
    explicit FillingValveAtomicTiming(const uint32_t val)
        : UnionWrapUInt32Type<ConfigurationEnum>(
              val, ConfigurationEnum::FILLING_VALVE_ATOMIC_TIMING)
    {
    }
};

/**
 * @brief Main valve atomic timing registry struct for the float maximum
 * aperture of main value
 *
 */
struct MainValveMaxAperture : UnionWrapFloatType<ConfigurationEnum>
{
    explicit MainValveMaxAperture(const float val)
        : UnionWrapFloatType<ConfigurationEnum>(
              val, ConfigurationEnum::MAIN_VALVE_MAX_APERTURE)
    {
    }
};

/**
 * @brief Venting valve maximum aperture registry struct for the float maximum
 * venting aperture value
 *
 */
struct VentingValveMaxAperture : UnionWrapFloatType<ConfigurationEnum>
{
    explicit VentingValveMaxAperture(const float val)
        : UnionWrapFloatType<ConfigurationEnum>(
              val, ConfigurationEnum::VENTING_VALVE_MAX_APERTURE)
    {
    }
};

/**
 * @brief Release valve maximum aperture registry struct for the float maximum
 * aperture value
 *
 */
struct ReleaseValveMaxAperture : UnionWrapFloatType<ConfigurationEnum>
{
    explicit ReleaseValveMaxAperture(const float val)
        : UnionWrapFloatType<ConfigurationEnum>(
              val, ConfigurationEnum::RELEASE_VALVE_MAX_APERTURE)
    {
    }
};

/**
 * @brief Filling valve maximum aperture registry struct for the float maximum
 * aperture value
 *
 */
struct FillingValveMaxAperture : UnionWrapFloatType<ConfigurationEnum>
{
    explicit FillingValveMaxAperture(const float val)
        : UnionWrapFloatType<ConfigurationEnum>(
              val, ConfigurationEnum::FILLING_VALVE_MAX_APERTURE)
    {
    }
};

/** TODO:
 * REFERENCE_ALTITUDE,
    REFERENCE_TEMPERATURE,
    ORIENTATION,
    COORDINATES,
    DEPLOYMENT_ALTITUDE,
    TARGET_COORDINATES

    And... ALSO
    SET_SERVO_ANGLES from:
    typedef enum ServosList
{
AIR_BRAKES_SERVO=1,
EXPULSION_SERVO=2,
PARAFOIL_LEFT_SERVO=3,
PARAFOIL_RIGHT_SERVO=4,
MAIN_VALVE=5,
VENTING_VALVE=6,
RELEASE_VALVE=7,
FILLING_VALVE=8,
DISCONNECT_SERVO=9,
ServosList_ENUM_END=10,
} ServosList;
*/

};  // namespace Boardcore