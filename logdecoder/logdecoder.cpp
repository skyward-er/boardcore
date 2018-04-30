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

/*
 * This is a stub program for the program that will decode the logged data.
 * Fill in the TODO to make it work. 
 */

#include <iostream>
#include <fstream>
#include <string>
#include <map>
#include <functional>
#include <stdexcept>
#include <cxxabi.h>
#include "../src/shared/logger/LogBase.h"
#include "../src/entrypoints/test_logger.h"

//TODO: add here include files of serialized classes

using namespace std;

string demangle(const string& name)
{
    string result=name;
    int status;
    char* demangled = abi::__cxa_demangle(name.c_str(), NULL, 0, &status);
    if (status == 0 && demangled)
        result = demangled;
    if (demangled)
        free(demangled);
    return result;
}

string readName(ifstream& in)
{
    string result;
    while(char c=in.get()) result+=c; //Not very optimized
    return result;
}

map<string,function<void (ostream&,istream&)>> unserializers;

#define REGISTER_UNSERIALIZER(x)                               \
unserializers[typeid(x).name()]=[](ostream& out, istream& in){ \
    x object;                                                  \
    in.read(reinterpret_cast<char*>(&object),sizeof(x));       \
    object.print(out);                                         \
}

int main(int argc, char *argv[])
try {
    if(argc!=2) return 1;
    ifstream in(argv[1]);
    in.exceptions(ios::eofbit);

    //TODO: Without cereal, you have to register the serialized types
    REGISTER_UNSERIALIZER(LogStats);
    REGISTER_UNSERIALIZER(Dummy);
    
    for(;;)
    {
        string name=readName(in);
        auto u=unserializers.find(name);
        if(u==unserializers.end())
        {
            cerr<<"Don't know how to unserialize "<<demangle(name)<<endl;
            return 1;
        }
        cout<<"type="<<demangle(name)<<' ';
        u->second(cout,in);
        cout<<'\n';
    }
} catch(exception&) {
    return 0;
}
