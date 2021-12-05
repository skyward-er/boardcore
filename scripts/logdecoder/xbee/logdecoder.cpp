/* Copyright (c) 2018 Skyward Experimental Rocketry
 * Author: Federico Terraneo
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

#include "LogTypes.h"

using namespace std;
using namespace tscpp;

namespace fs = filesystem;

void showHelp(string cmdName)
{
    std::cerr << "Usage: " << cmdName
              << " {-a [logs_diretory] | <log_file_path> | -h}"
              << "Options:\n"
              << "\t-h,--help\t\tShow help message\n"
              << "\t-a,--all [dir=\".\"] Deserialize all logs in the provided "
                 "directory\n"
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
