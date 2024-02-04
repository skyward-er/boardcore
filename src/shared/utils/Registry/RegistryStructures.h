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
    CURRENT_ORIENTATION,  // TODO: keep it?
    CURRENT_COORDINATES,  // TODO: keep it?
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
    AIR_BRAKES_SERVO_ANGLE,
    EXPULSION_SERVO_ANGLE,
    PARAFOIL_LEFT_SERVO_ANGLE,
    PARAFOIL_RIGHT_SERVO_ANGLE,
    MAIN_VALVE_SERVO_ANGLE,
    VENTING_VALVE_SERVO_ANGLE,
    RELEASE_VALVE_SERVO_ANGLE,
    FILLING_VALVE_SERVO_ANGLE,
    // DISCONNECT_SERVO_ANGLE, // TODO: is it just a command?
};

/**
 * @brief Coordinates struct with latitude [degree], longitude [degree]
 */
struct Coordinates
{
    uint32_t latitude;
    uint32_t longitude;

    explicit Coordinates(const uint32_t setLatitude,
                         const uint32_t setLongitude)
        : latitude(setLatitude), longitude(setLongitude)
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
    uint32_t
        uint32_type;  //< used for both uint32_t and uint8_t type with upcast
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
     * @brief Creates an Union object from the float type value
     *
     * @param valueToSet The value to be set into the returned union
     * @return TypeUnion the created union type.
     */
    static TypeUnion createUnion(uint8_t valueToSet)
    {
        TypeUnion toReturn;
        toReturn.uint32_type = static_cast<uint32_t>(valueToSet);
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

/**
 * @brief Struct to "wrap" the correct union type used by the coordinates
 * latitude, longitude uint32_t data structures. The data structures using the
 * TypeUnion union and the float type will inherit from this.
 */
template <typename T>
struct UnionWrapUInt32Coordinates : RootTypeStructure<Coordinates, T>
{
    UnionWrapUInt32Coordinates(const int32_t latitude, const int32_t longitude,
                               const T index)
        : RootTypeStructure<Coordinates, T>(Coordinates(latitude, longitude),
                                            index)
    {
    }
    /**
     * @brief Get the the uint32_t latitude value from an Union object
     *
     * @param unionValue the union object from which get the union
     * @return the latitude value get from such union
     */
    static uint32_t getLatitudeFromUnion(TypeUnion unionValue)
    {
        return unionValue.coordinates_type.latitude;
    }
    /**
     * @brief Get the the uint32_t longitude value from an Union object
     *
     * @param unionValue the union object from which get the union
     * @return uint32_t the longitude value get from such union
     */
    static uint32_t getLongitudeFromUnion(TypeUnion unionValue)
    {
        return unionValue.coordinates_type.longitude;
    }
    /**
     * @brief Creates an Union object from the latitude, longitude uint32_t type
     * values
     *
     * @param valueToSet The value to be set into the returned union
     * @return TypeUnion the created union type.
     */
    static TypeUnion createUnion(uint32_t latitudeToSet,
                                 uint32_t longitudeToSet)
    {
        TypeUnion toReturn;
        toReturn.coordinates_type.latitude  = latitudeToSet;
        toReturn.coordinates_type.longitude = longitudeToSet;
        return toReturn;
    }
    /**
     * @brief Get the Union object with the correct coordinates value of the
     * instance
     *
     * @return TypeUnion the union with the value for such Coordinates type
     * structure
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
 * @brief Algorithm struct for set the parafoil, Guidance and GSE tars algorithm
 */
struct Algorithm : UnionWrapUInt32Type<ConfigurationEnum>
{
    explicit Algorithm(const uint32_t val)
        : UnionWrapUInt32Type<ConfigurationEnum>(val,
                                                 ConfigurationEnum::ALGORITHM)
    {
    }

    explicit Algorithm(const uint8_t val)
        : UnionWrapUInt32Type<ConfigurationEnum>(static_cast<uint32_t>(val),
                                                 ConfigurationEnum::ALGORITHM)
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
 * Struct for the latitude, longitude target coordinates
 */
struct TargetCoordinates : UnionWrapUInt32Coordinates<ConfigurationEnum>
{
    explicit TargetCoordinates(const uint32_t latitude,
                               const uint32_t longitude)
        : UnionWrapUInt32Coordinates<ConfigurationEnum>(
              latitude, longitude, ConfigurationEnum::TARGET_COORDINATES)
    {
    }
};

/**
 * @brief Main valve atomic timing registry struct for the [ms] timing of main
 * valve
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
 */
struct FillingValveMaxAperture : UnionWrapFloatType<ConfigurationEnum>
{
    explicit FillingValveMaxAperture(const float val)
        : UnionWrapFloatType<ConfigurationEnum>(
              val, ConfigurationEnum::FILLING_VALVE_MAX_APERTURE)
    {
    }
};

/**
 * @brief Reference altitude with [m] value
 *
 */
struct ReferenceAltitude : UnionWrapFloatType<ConfigurationEnum>
{
    explicit ReferenceAltitude(const float val)
        : UnionWrapFloatType<ConfigurationEnum>(
              val, ConfigurationEnum::REFERENCE_ALTITUDE)
    {
    }
};

/**
 * @brief Reference temperature with [degC] value
 */
struct ReferenceTemperature : UnionWrapFloatType<ConfigurationEnum>
{
    explicit ReferenceTemperature(const float val)
        : UnionWrapFloatType<ConfigurationEnum>(
              val, ConfigurationEnum::REFERENCE_TEMPERATURE)
    {
    }
};

/*! SERVO ANGLES CONFIGURATION STRUCTS */
// TODO: Angles... Yes but which measure unit?

/**
 * @brief Expulsion servo angle with its normalized [0,1] value
 */
struct AirBrakesServoAngle : UnionWrapFloatType<ConfigurationEnum>
{
    explicit AirBrakesServoAngle(const float val)
        : UnionWrapFloatType<ConfigurationEnum>(
              val, ConfigurationEnum::AIR_BRAKES_SERVO_ANGLE)
    {
    }
};

/**
 * @brief Expulsion servo angle with its normalized [0,1] value
 */
struct ExpulsionServoAngle : UnionWrapFloatType<ConfigurationEnum>
{
    explicit ExpulsionServoAngle(const float val)
        : UnionWrapFloatType<ConfigurationEnum>(
              val, ConfigurationEnum::EXPULSION_SERVO_ANGLE)
    {
    }
};

/**
 * @brief Parafoil left servo angle with its normalized [0,1] value
 */
struct ParafoilLeftServoAngle : UnionWrapFloatType<ConfigurationEnum>
{
    explicit ParafoilLeftServoAngle(const float val)
        : UnionWrapFloatType<ConfigurationEnum>(
              val, ConfigurationEnum::PARAFOIL_LEFT_SERVO_ANGLE)
    {
    }
};

/**
 * @brief Parafoil right servo angle with its normalized [0,1] value
 */
struct ParafoilRightServoAngle : UnionWrapFloatType<ConfigurationEnum>
{
    explicit ParafoilRightServoAngle(const float val)
        : UnionWrapFloatType<ConfigurationEnum>(
              val, ConfigurationEnum::PARAFOIL_RIGHT_SERVO_ANGLE)
    {
    }
};

/**
 * @brief Venting valve servo angle with its normalized [0,1] value
 */
struct MainValveServoAngle : UnionWrapFloatType<ConfigurationEnum>
{
    explicit MainValveServoAngle(const float val)
        : UnionWrapFloatType<ConfigurationEnum>(
              val, ConfigurationEnum::MAIN_VALVE_SERVO_ANGLE)
    {
    }
};

/**
 * @brief Venting valve servo angle with its normalized [0,1] value
 */
struct VentingValveServoAngle : UnionWrapFloatType<ConfigurationEnum>
{
    explicit VentingValveServoAngle(const float val)
        : UnionWrapFloatType<ConfigurationEnum>(
              val, ConfigurationEnum::VENTING_VALVE_SERVO_ANGLE)
    {
    }
};

/**
 * @brief Release valve servo angle with its normalized [0,1] value
 */
struct ReleaseValveServoAngle : UnionWrapFloatType<ConfigurationEnum>
{
    explicit ReleaseValveServoAngle(const float val)
        : UnionWrapFloatType<ConfigurationEnum>(
              val, ConfigurationEnum::RELEASE_VALVE_SERVO_ANGLE)
    {
    }
};

/**
 * @brief Filling valve servo angle with its normalized [0,1] value
 */
struct FillingValveServoAngle : UnionWrapFloatType<ConfigurationEnum>
{
    explicit FillingValveServoAngle(const float val)
        : UnionWrapFloatType<ConfigurationEnum>(
              val, ConfigurationEnum::FILLING_VALVE_SERVO_ANGLE)
    {
    }
};

};  // namespace Boardcore