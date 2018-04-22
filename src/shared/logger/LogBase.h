
#pragma once

#include <ostream>
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
     * Used by cereal to serialize the class
     * \param ar a cereal archive
     */
    template<typename Archive>
    void serialize(Archive& ar)
    {
        ar(timestamp);
    }
    
    /**
     * Print the class fields to an ostream.
     * Used by the program that decodes the logged data after the flight.
     * \param os ostream where to print the class fields
     */
    virtual void print(std::ostream& os) const=0;
    
    virtual ~LogBase()=0;
    
protected:
    long long timestamp; ///< Timestamp when the class was created
};

/**
 * Write a LogBase derived class to an ostream
 */
inline std::ostream& operator<<(std::ostream& os, const LogBase& lb)
{
    lb.print(os);
    return os;
}
