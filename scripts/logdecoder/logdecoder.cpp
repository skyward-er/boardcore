/* Copyright (c) 2018-2022 Skyward Experimental Rocketry
 * Authors: Terrane Federico
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

#include <logger/Deserializer.h>
#include <logger/LogTypes.h>
#include <tscpp/stream.h>

#include <fstream>
#include <iostream>
#include <stdexcept>
#include <string>

/**
 * @brief Binary log files decoder.
 *
 * This program is to compile for you computer and decodes binary log files
 * through the tscpp library.
 *
 * In LogTypes.h there should be included all the classes you want to
 * deserialize.
 */

using namespace tscpp;
using namespace Boardcore;

void showUsage(string commandName);

/**
 * @brief Deserialize a file.
 *
 * @param fileName File name complete with extension.
 * @return Whether the deserialization was successful.
 */
bool deserialize(string fileName);

/**
 * @brief Deserialize all log file in the directory. Assumes the log files named
 * as logXX.dat.
 *
 * Scans for all the 100 possible log files and decode the ones found.
 *
 * @return False if an error was encountered.
 */
bool deserializeAll();

int main(int argc, char* argv[])
{
    if (argc < 2)
    {
        showUsage(string(argv[0]));
        return 1;  // Error
    }

    bool success = false;
    string arg1  = string(argv[1]);

    // Help message
    if (arg1 == "-h" || arg1 == "--help")
    {
        showUsage(string(argv[0]));
        return 0;
    }

    // File deserialization
    if (arg1 == "-a" || arg1 == "--all")
        success = deserializeAll();
    else
        success = deserialize(arg1);

    // End
    if (success)
        std::cout << "Deserialization completed successfully\n";
    else
        std::cout << "Deserialization ended with errors\n";
    return 0;
}

void showUsage(string commandName)
{
    std::cerr << "Usage: " << commandName << " {-a | <log_file_name> | -h}"
              << "Options:\n"
              << "\t-h,--help\t\tShow this help message\n"
              << "\t-a,--all Deserialize all logs in the current directory "
                 "named as logXX.dat\n"
              << std::endl;
}

bool deserialize(string fileName)
{
    std::cout << "Deserializing " << fileName << "...\n";

    Deserializer d(fileName);
    LogTypes::registerTypes(d);

    return d.deserialize();
}

bool deserializeAll()
{
    std::cout << "Deserializing all logs in the current directory...\n";

    for (int i = 0; i < 100; i++)
    {
        char fileName[11];
        sprintf(fileName, "log%02d.dat", i);
        struct stat st;

        // Check if the current logfile exists
        if (stat(fileName, &st) != 0)
            continue;

        // File found
        if (!deserialize(string(fileName)))
            return false;

        std::cout << fileName << " deserialized successfully";
    }

    return true;
}
