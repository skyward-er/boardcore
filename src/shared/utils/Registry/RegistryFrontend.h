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

#include <stdint.h>
#include <utils/Debug.h>
#include <utils/Registry/TypeStructures.h>

#include <functional>
#include <mutex>
#include <unordered_map>
#include <vector>

#include "TypeStructures.h"

namespace Boardcore
{

/*! The Configuration ID for the configuration entries */
typedef uint32_t ConfigurationId;

/*! Types used for the union */
enum TypesEnum
{
    UINT32_T,
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
        if (type != TypesEnum::UINT32_T)
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
        if (type != TypesEnum::UINT32_T)
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
        if (type != TypesEnum::UINT32_T)
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
    static EntryStructsUnion setUnion(float valueToSet)
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
    static EntryStructsUnion setUnion(uint8_t valueToSet)
    {
        TypeUnion returnValue;
        returnValue.uint32_type = static_cast<uint32_t>(valueToSet);
        return EntryStructsUnion(returnValue, TypesEnum::UINT32_T);
    }

    /**
     * @brief Set the Union object with its unsigned 32bit integer value
     *
     * @param valueToSet The value to be set into the union type
     * @return TypeUnion the returned created type union with its int value
     * set.
     */
    static EntryStructsUnion setUnion(uint32_t valueToSet)
    {
        TypeUnion returnValue;
        returnValue.uint32_type = valueToSet;
        return EntryStructsUnion(returnValue, TypesEnum::UINT32_T);
    }

    /**
     * @brief Set the Union object with its Coordinates value
     *
     * @param valueToSet The value to be set into the union type
     * @return TypeUnion the returned created type union with its int value
     * set.
     */
    static EntryStructsUnion setUnion(Coordinates valueToSet)
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
        serializationVector.insert(serializationVector.end(),
                                   static_cast<uint8_t>(tempUint32 >> 24));
        serializationVector.insert(serializationVector.end(),
                                   static_cast<uint8_t>(tempUint32 >> 16));
        serializationVector.insert(serializationVector.end(),
                                   static_cast<uint8_t>(tempUint32 >> 8));
        serializationVector.insert(serializationVector.end(),
                                   static_cast<uint8_t>(tempUint32));
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
        /*! Needed cast for bitwise conversion for serialization */
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
    // TODO: Can be templated this?
    bool appendSerializedFromUnion(std::vector<uint8_t>& serializationVector)
    {
        uint32_t tempUint32;
        switch (this->type)
        {
            case TypesEnum::UINT32_T:
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
                    /*! Needed cast for bitwise conversion for serialization */
                    // cppcheck-suppress invalidPointerCast
                    tempUint32 = *(reinterpret_cast<uint32_t*>(&tempFloat));

                    insertUint32ToVector(tempUint32, serializationVector);
                    return true;
                }
                break;
            case TypesEnum::COORDINATES:
                Coordinates tempCoordinates;
                if (this->getFromUnion(tempCoordinates))
                {
                    tempUint32 = *(
                        // cppcheck-suppress invalidPointerCast
                        reinterpret_cast<uint32_t*>(&tempCoordinates.latitude));

                    insertUint32ToVector(tempUint32, serializationVector);

                    tempUint32 = *(
                        // cppcheck-suppress invalidPointerCast
                        reinterpret_cast<uint32_t*>(
                            &tempCoordinates.longitude));

                    insertUint32ToVector(tempUint32, serializationVector);
                    return true;
                }
                break;
        }
        return false;
    }
};

/**
 * @brief Write buffer structs wraps an std::vector<uint8_t> with also its mutex
 * and a flag for specify if it changed from last write.
 */
struct WriteBuffer
{
    std::vector<uint8_t> vector; /*< vector with serialized data*/
    std::recursive_mutex mutex;
    bool needsWrite; /*< True if it is needed a write to the backend */
};

/**
 * This is the front-end for the registry in case of
 * type unsafe and safe methods for type safeness.
 * It does check the data types but its job is mainly the one of
 * getting and setting given the uint32_t or ConfigurationEnum parameter, the
 * value for such configuration entry. Also it does expose methods for change
 * into a "safe" state the registry during flight as for methods for exploring
 * the current configuration
 */
class RegistryFrontend
{
private:
    std::unordered_map<ConfigurationId, EntryStructsUnion> configuration;
    std::recursive_mutex mutexForRegistry;
    bool isArmed = false;
    WriteBuffer mainBuffer;
    WriteBuffer secondaryBuffer; /*< Used in case the main one is locked */
    std::vector<uint8_t> elementVector;
    std::recursive_mutex buffersMutex;
    bool bufferMainToWrite; /*< 0: write main, modify secondary, 1: write
                               secondary modify main*/

    /**
     * @brief Given a buffer, it updates it with the serialized vector of the
     * current configuration.
     *
     * @param bufferToUpdate The buffer it needs to update
     * @return WriteBuffer& The buffer now updated
     */
    WriteBuffer& updateBuffer(WriteBuffer& bufferToUpdate);

public:
    RegistryFrontend();
    ~RegistryFrontend();

    /**
     * @brief Disables the memory registry set and allocations.
     * To be use when the rocket itself is armed and during flight.
     */
    void arm();

    /**
     * @brief Enable set methods and memory allocations.
     * To be used when the rocket is NOT in an "armed" state and while on
     * ground.
     */
    void disarm();

    /**
     * @brief Visits the configuration applying the callback with the id and
     * EntryStructsUnion union as parameter for each configured entry in the
     * configuration.
     */
    void visitConfiguration(
        std::function<void(ConfigurationId, EntryStructsUnion&)> callback);

    /**
     * @brief Loads from the backend the configuration
     * @return True if the configuration exists in memory and is not corrupted,
     * False if not.
     */
    bool loadConfiguration();

    /**
     * @brief Saves the configuration to the backend
     *
     * @return true If the saving was successful
     * @return false Otherwise
     */
    bool saveConfiguration();

    /**
     * @brief Verify if there is an existing entry given its enum entry.
     * @param configurationIndex The configuration entry to verify.
     * @return True if such configuration entry exists in the configuration
     * otherwise False.
     */
    bool isEntryConfigured(const ConfigurationId configurationIndex);

    /**
     * @brief Verify that the configuration is empty or exists some setted
     * entries
     * @return True if the configuration has no entries. False otherwise
     */
    bool isConfigurationEmpty();

    /*! TYPE UNSAFE INTERFACE METHODS */

    /**
     * Method to get the value for a given configuration entry.
     * It does change the given variable with the correct value if existing
     * @tparam T The configuration struct for such configuration entry
     * @param configurationIndex Identifies the configuration entry with its
     * enumeration value
     * @param value The value to be insert for the specified configuration entry
     * @return True in case of successful insertion. False otherwise (memory
     * limits or "armed" memory)
     */
    template <typename T>
    bool getConfigurationUnsafe(const ConfigurationId configurationIndex,
                                T& value)
    {
        std::lock_guard<std::recursive_mutex> lock(mutexForRegistry);
        auto iterator = configuration.find(configurationIndex);
        /** Checks that the value type corresponds to the set type and finds the
         * entry*/
        return !(iterator == configuration.end()) &&
               (iterator->second.getFromUnion(value));
    }
    /**
     * @brief Gets the value for a specified configuration entry. Otherwise
     * returns and try to set the default value
     * @tparam T The value data type to be returned and eventually set.
     * @param configurationIndex Identifies the configuration entry with its
     * enumeration value
     * @param defaultValue The default value to be returned and set
     * (eventually) in case of non-existing configuration entry
     * @return The value saved for the configuration entry in the
     * configuration or the default value if there is no such entry in the
     * configuration
     */
    template <typename T>
    T getOrSetDefaultConfigurationUnsafe(
        const ConfigurationId configurationIndex, T defaultValue)
    {
        std::lock_guard<std::recursive_mutex> lock(mutexForRegistry);
        T returnValue;
        if (getConfigurationUnsafe(configuration, &returnValue))
        {
            return returnValue;
        }
        if (!setConfigurationUnsafe(configurationIndex, defaultValue))
            TRACE("Registry - Could not insert the default configuration");
        return defaultValue;
    }

    /**
     * @brief Sets the value for the configuration entry with the specified
     * enum
     * @tparam T The configuration struct datatype
     * @param configurationIndex The initialized configuration structure to
     * be set as configuration entry
     * @param value The value to be set for the specified configuration
     * entry
     * @return True if it was possible to set the configurationEntry. False
     * otherwise, e.g. in case of allocation issues or "armed" memory
     */
    template <typename T>
    bool setConfigurationUnsafe(ConfigurationId configurationIndex, T value)
    {
        std::lock_guard<std::recursive_mutex> lock(mutexForRegistry);
        /*! In case that the configuration is in an armed state it cannot be
         * modified */
        if (isArmed)
            return false;
        EntryStructsUnion entry = EntryStructsUnion::setUnion(value);
        bool success =
            configuration.insert(std::make_pair(configurationIndex, entry))
                .second;
        if (!success)
        {
            TRACE(
                "Registry - setConfigurationUnsafe - Could not insert the "
                "configuration entry");
            return false;
        }
        else
        {
            getSerializedConfiguration();
        }
        return success;
    }

    /*! TYPE SAFE INTERFACE METHODS */

    /**
     * @brief Gets the saved configuration entry for such index type-safely.
     *
     * @tparam T The configuration entry value data type.
     * @param value The returned configuration entry with its current value.
     * @return True if the configuration has such entry in memory. False
     * otherwise.
     */
    template <typename T>
    bool getConfiguration(T& value)
    {
        std::lock_guard<std::recursive_mutex> lock(mutexForRegistry);
        auto iterator = configuration.find(value.index);
        if (iterator == configuration.end())
        {
            TRACE(
                "Registry - getConfiguration - Get configuration not "
                "found");
            return false;
        }
        value.value = iterator->second.value;
        return true;
    };

    /**
     * @brief Sets the configuration entry in the registry configuration
     * using the given configuration entry struct.
     *
     * @tparam T The configuration entry struct
     * @param configurationEntry The configuration entry initialized and set
     * struct to be saved in the configuration.
     * @return True if the entry is correctly saved in the registry. False
     * otherwise, e.g. in case of allocation issues or "armed" memory
     */
    template <typename T>
    bool setConfiguration(T configurationEntry)
    {
        std::lock_guard<std::recursive_mutex> lock(mutexForRegistry);
        /*! In case that the configuration is in an armed state it cannot be
         * modified */
        if (isArmed)
            return false;
        EntryStructsUnion entryToSet = EntryStructsUnion(configurationEntry);
        bool success                 = configuration
                           .insert(std::make_pair(configurationEntry.enumVal,
                                                  entryToSet.value))
                           .second;
        if (!success)
        {
            TRACE(
                "Registry - setConfiguration - Could not insert the "
                "configuration entry");
            return false;
        }
        else
        {
            getSerializedConfiguration();
        }
        return success;
    }

    /*! DATA SERIALIZATION TO BYTES FOR BACKEND LOAD AND SAVE */

    /**
     * @brief Get the Serialized bytes vector of the configuration actually
     * saved in the frontend
     *
     * @return WriteBuffer The write buffer wrapping the vector of the
     * configuration
     */
    WriteBuffer& getSerializedConfiguration();

    /**
     * @brief Clear the configuration actually saved, resetting to empty
     * configuration. Does affect also the underlying backend.
     * @attention It does delete also the backend saved copies
     */
    void clear();
};
}  // namespace Boardcore
