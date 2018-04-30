/***************************************************************************
 *   Copyright (C) 2018 by Terraneo Federico                               *
 *                                                                         *
 *   This program is free software; you can redistribute it and/or modify  *
 *   it under the terms of the GNU General Public License as published by  *
 *   the Free Software Foundation; either version 2 of the License, or     *
 *   (at your option) any later version.                                   *
 *                                                                         *
 *   This program is distributed in the hope that it will be useful,       *
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of        *
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the         *
 *   GNU General Public License for more details.                          *
 *                                                                         *
 *   As a special exception, if other files instantiate templates or use   *
 *   macros or inline functions from this file, or you compile this file   *
 *   and link it with other works to produce a work based on this file,    *
 *   this file does not by itself cause the resulting work to be covered   *
 *   by the GNU General Public License. However the source code for this   *
 *   file must still be made available in accordance with the GNU General  *
 *   Public License. This exception does not invalidate any other reasons  *
 *   why a work based on this file might be covered by the GNU General     *
 *   Public License.                                                       *
 *                                                                         *
 *   You should have received a copy of the GNU General Public License     *
 *   along with this program; if not, see <http://www.gnu.org/licenses/>   *
 ***************************************************************************/ 

#pragma once

/**
 * Select serialization implementation for the logger by
 * commenting/uncommenting USE_CEREAL
 * 
 * There are two options:
 * 
 * - use the cereal C++ serialization library.
 *   PROS: allows to log classes that contain pointers, stl containers, etc...
 *   CONS: slower, more difficult to use (read comments in LogBase class)
 * 
 * - use a custom binary serializer that first prints the typeid of the object
 *   and then dumps the bytes of the class
 *   PROS: fast, easy
 *   CONS: only classes without pointers, stl containers, virtual functions,
 *   virtual base classes can be logged. Basically just structs with member
 *   functions and constructors
 */
// #define USE_CEREAL

#include <ostream>

#ifdef USE_CEREAL
#include <cereal/archives/portable_binary.hpp>
#include <cereal/types/polymorphic.hpp>

/**
 * Base class from which every class that is loggable has to derive.
 *
 * To be loggable, a class must:
 * - derive from LogBase
 * - have a default constructor
 * - have a serialize member function which should serialize the base class
 *   as well as every field
 * - have a print member function that first calls print on the base class and
 *   then prints all its fields. This is not used by Miosix, but is used by
 *   the program that decodes the logged data after the flight
 * - define the macro CEREAL_REGISTER_TYPE with the correct type name in some
 *   cpp file (usually the one with the same name of the .h where the class is
 *   defined)
 *
 * An example loggable class is this:
 * \code
 * class LoggableClass : public LogBase
 * {
 * public:
 *     LoggableClass() {}                  // Default constructor
 *
 *     LoggableClass(int x) : x(x) {}      // Other constructors (optional)
 *
 *     template<typename Archive>
 *     void serialize(Archive& ar)         // Serialize member function
 *     {
 *         ar(cereal::base_class<LogBase>(this),x);
 *     }
 *
 *     void print(std::ostream& os) const  // Print member funtion
 *     {
 *         LogBase::print(os);
 *         os<<"x="<<x<<' ';
 *     }
 *
 * private:
 *    int x;
 * };
 *
 * CEREAL_REGISTER_TYPE(LoggableClass);    // This must be put is some cpp file
 * \endcode
 */
class LogBase
{
public:
    /**
     * Constructor
     * Initializes the timestamp to the current time
     */
    LogBase();

    /**
     * Set timestamp for this class
     * \param timestamp timestamp
     */
    void setTimestamp(long long timestamp) { this->timestamp = timestamp; }

    /**
     * Used by cereal to serialize the class
     * \param ar a cereal archive
     */
    template <typename Archive>
    void serialize(Archive& ar)
    {
        ar(timestamp);
    }

    /**
     * Print the class fields to an ostream.
     * Used by the program that decodes the logged data after the flight.
     * \param os ostream where to print the class fields
     */
    virtual void print(std::ostream& os) const = 0;

    virtual ~LogBase() = 0;

protected:
    long long timestamp;  ///< Timestamp when the class was created
};

/**
 * Write a LogBase derived class to an ostream
 */
inline std::ostream& operator<<(std::ostream& os, const LogBase& lb)
{
    lb.print(os);
    return os;
}

/**
 * Statistics for the logger
 */
class LogStats : public LogBase
{
public:
    /**
     * Constructor
     */
    LogStats();

    /**
     * Used by cereal to serialize the class
     * \param ar a cereal archive
     */
    template <typename Archive>
    void serialize(Archive& ar)
    {
        ar(cereal::base_class<LogBase>(this), statTooLargeSamples,
           statDroppedSamples, statQueuedSamples, statBufferFilled,
           statBufferWritten, statWriteFailed, statWriteTime, statMaxWriteTime);
    }

    /**
     * Print the class fields to an ostream.
     * Used by the program that decodes the logged data after the flight.
     * \param os ostream where to print the class fields
     */
    void print(std::ostream& os) const;

    int statTooLargeSamples =
        0;  ///< Number of dropped samples because too large
    int statDroppedSamples = 0;  ///< Number of dropped samples due to fifo full
    int statQueuedSamples  = 0;  ///< Number of samples written to buffer
    int statBufferFilled   = 0;  ///< Number of buffers filled
    int statBufferWritten  = 0;  ///< Number of buffers written to disk
    int statWriteFailed    = 0;  ///< Number of fwrite() that failed
    int statWriteTime      = 0;  ///< Time to perform an fwrite() of a buffer
    int statMaxWriteTime = 0;  ///< Max time to perform an fwrite() of a buffer
};

#else //USE_CEREAL

/**
 * Statistics for the logger
 */
class LogStats
{
public:
    /**
     * Constructor
     */
    LogStats();
    
    /**
     * Set timestamp for this class
     * \param timestamp timestamp
     */
    void setTimestamp(long long timestamp) { this->timestamp = timestamp; }

    /**
     * Print the class fields to an ostream.
     * Used by the program that decodes the logged data after the flight.
     * \param os ostream where to print the class fields
     */
    void print(std::ostream& os) const;

    long long timestamp; ///< Timestamp
    int statTooLargeSamples =
        0;  ///< Number of dropped samples because too large
    int statDroppedSamples = 0;  ///< Number of dropped samples due to fifo full
    int statQueuedSamples  = 0;  ///< Number of samples written to buffer
    int statBufferFilled   = 0;  ///< Number of buffers filled
    int statBufferWritten  = 0;  ///< Number of buffers written to disk
    int statWriteFailed    = 0;  ///< Number of fwrite() that failed
    int statWriteTime      = 0;  ///< Time to perform an fwrite() of a buffer
    int statMaxWriteTime = 0;  ///< Max time to perform an fwrite() of a buffer
};

#endif //USE_CEREAL
