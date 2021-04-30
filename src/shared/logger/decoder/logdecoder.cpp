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
 * You must first define a function
 *  "void registerTypes(Deserializer& ds);"
 * 
 * that registers all the required log data types in the deserializer and then
 * INCLUDE this cpp file in your source. finally compile and run
 */

#include <sys/stat.h>
#include <sys/types.h>
#include <tscpp/stream.h>

#include <filesystem>
#include <fstream>
#include <iostream>
#include <stdexcept>
#include <string>
#include <vector>

using namespace std;
using namespace tscpp;

namespace fs = filesystem;

void showHelp(string cmdName)
{
    std::cerr << "Usage: " << cmdName
              << " {-a [logs_diretory] | <log_file_path> | -h}"
              << "Options:\n"
              << "\t-h,--help\t\tShow help message\n"
              << "\t-a,--all [dir=\".\"] Deserialize all logs in the provided directory\n"
              << std::endl;
}

vector<fs::path> listLogFiles(fs::path dir)
{
    vector<fs::path> out;
    for (const auto& entry : fs::directory_iterator(dir))
    {
        if (entry.exists() && entry.is_regular_file())
        {
            if (entry.path().extension() == ".dat")
            {
                out.push_back(entry.path());
            }
        }
    }
    return out;
}

bool deserialize(fs::path log_path)
{
    cout << "Deserializing " << log_path << ".dat...\n";

    // remove extension
    log_path.replace_extension("");
    Deserializer d(log_path);
    registerTypes(d);

    return d.deserialize();
}

bool deserializeAll(fs::path dir = ".")
{
    vector<fs::path> logs = listLogFiles(dir);
    for (auto log : logs)
    {
        if (!deserialize(log.string()))
        {
            return false;
        }
    }

    return true;
}

int main(int argc, char* argv[])
{
    if (argc < 2)
    {
        showHelp(string(argv[0]));
        return 1;
    }

    bool success = false;
    string arg1  = string(argv[1]);
    if (arg1 == "-h" || arg1 == "--help")
    {
        showHelp(string(argv[0]));
        return 0;
    }

    if (arg1 == "-a" || arg1 == "--all")
    {
        fs::path dir = ".";
        if (argc == 3)
        {
            string arg2 = string(argv[2]);
            fs::directory_entry entry(arg2);
            if (entry.exists() && entry.is_directory())
            {
                dir = arg2;
            }
            else
            {
                cout << "Second argument after '-a' or '--all' must be a "
                        "directory\n";
                showHelp(string(argv[0]));
                return 1;
            }
        }
        cout << "Deserializing all logs...\n";
        success = deserializeAll(dir);
    }
    else if (arg1[0] == '-')
    {
        cerr << "Unknown option\n";
        return 1;
    }
    else
    {
        fs::directory_entry entry(arg1);
        if (entry.exists() && entry.is_regular_file())
        {
            success = deserialize(arg1);
        }
        else
        {
            showHelp(string(argv[0]));
            return 1;
        }
    }

    if (success)
    {
        cout << "Deserialization completed successfully.\n";
    }
    else
    {
        cout << "Deserialization ended with errors.\n";
    }
}
