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

#include <cxxabi.h>
#include <type_traits>
#include <map>
#include <functional>
#include <istream>

/**
 * Unserializer for the serialization format used by the logger
 */
class Unserializer
{
public:
    
    /**
     * All types that are serialized by the logger have to be registered before
     * a serialized file can be decoded
     * \param T the class type to be registered. Has to be trivially copyable
     * and have a print member function accepting an ostream
     */
    template<typename T>
    void registerType()
    {
        static_assert(std::is_trivially_copyable<T>::value,"");
        types[typeid(T).name()]=[](std::ostream& out, std::istream& in) {
            T object;
            in.read(reinterpret_cast<char*>(&object),sizeof(T));
            object.print(out);
        };
    }
    
    /**
     * Unserialize a logged file
     * \param out where to print unserialized data
     * \param in input stream to unserialize
     */
    void unserialize(std::ostream& out, std::istream& in)
    {
        for(;;)
        {
            std::string name=readName(in);
            auto u=types.find(name);
            if(u==types.end())
            {
                out<<"Don't know how to unserialize "<<demangle(name)<<'\n';
                return;
            }
            out<<"type="<<demangle(name)<<' ';
            u->second(out,in);
            out<<'\n';
        }
    }
    
private:
    /**
     * Demangle a C++ name
     * \param name name to demangle
     * \return the demangled name
     */
    std::string demangle(const std::string& name)
    {
        std::string result=name;
        int status;
        char* demangled = abi::__cxa_demangle(name.c_str(), NULL, 0, &status);
        if (status == 0 && demangled)
            result = demangled;
        if (demangled)
            free(demangled);
        return result;
    }

    /**
     * Read a class name from an input stream
     * \param in input stream
     * \return class name
     */
    std::string readName(std::istream& in)
    {
        std::string result;
        while(char c=in.get()) result+=c; //Not very optimized
        return result;
    }

    std::map<std::string,std::function<void (std::ostream&,std::istream&)>> types;
};
