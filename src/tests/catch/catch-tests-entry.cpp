/*
 * Copyright (c) 2019 Skyward Experimental Rocketry
 * Authors: Luca Erbetta
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
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.  IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 */

/*
 * Entrypoint for all Catch1-based tests. You should not need to modify this
 * file.
 *
 * To add a test, just create a new source file and include <catch.hpp>, then
 * add test cases as you wish. Do not add a main() function in your test
 * sources.
 * Once the test is written, compile the test source file(s) + this entrypoint
 * in sbs.conf. Run it and all your test will be automatically executed.
 *
 * You can also include this file in your test source if you want to run it as a
 * standalone test, and not togheter with all the others. See the
 * skyward-boardcore wiki at: <link here> for more information.
 *
 * You can specify command line options such as "-s" to display messages for
 * succesfull tests (and not only messages for the ones that fail), plus others.
 * You can also add tags in order to specify which tests you want to be
 * executed. For a list of the command line arguments, take a look at:
 * https://github.com/catchorg/Catch2/blob/Catch1.x/docs/command-line.md
 *
 * To specify the command line options, add, in the definition of the entrypoint
 * in sbs.conf, add the CATCH1_CL_OPTIONS define.
 * Example: -DCATCH1_CL_OPTIONS="\"[tag1][tag2] -s\""
 * Remember to correctly escape the quotation marks, as shown
 * above.
 *
 * Further information at:
 * https://git.skywarder.eu/r2a/skyward-boardcore/wikis/Testing
 */

#define CATCH_CONFIG_RUNNER
#define CATCH_CONFIG_NO_POSIX_SIGNALS

#include <miosix.h>
#include <utils/catch.hpp>
#include <cstring>
#include <string>
#include <vector>

using miosix::Thread;
using std::string;
using std::vector;

// No tags or command line options as default.
#ifndef CATCH1_CL_OPTIONS
#define CATCH1_CL_OPTIONS ""
#endif

/**
 * @brief Splits a string around spaces
 * Eg: "This is a string" -> {"This", "is", "a", "string"}
 *
 * @param str String to split
 * @return vector<string>
 */
vector<string> splitSpaces(string str)
{
    unsigned int p = 0, p2;
    bool end       = false;
    vector<string> out;

    do
    {
        p2 = str.find(" ", p);

        if (p2 == string::npos)  // No match
        {
            p2  = str.length();
            end = true;
        }

        size_t len = p2 - p;  // Length of the substring

        if (len > 0)
        {
            out.push_back(str.substr(p, len));
        }
        p = p2 + 1;
    } while (!end);

    return out;
}

/**
 * @brief Entrypoint for the tests. Parses the options and tags
 * provided in sbs.conf and runs a Catch1 session.
 */
int main()
{
    // Parse command line arguments from #defines in sbs.conf

    string skw{"Skyward-tests"};

    // CATCH1_CL_OPTIONS defined in sbs.conf
    string options_str{CATCH1_CL_OPTIONS};

    vector<string> options = splitSpaces(options_str);
    vector<string> args{skw};

    if (options.size() > 0)
        args.insert(args.end(), options.begin(), options.end());

    // Convert vector of strings to array of c-strings
    size_t argc = args.size();

    char** argv =
        new char*[argc];  // Array of c strings, aka array of char pointers
    for (size_t i = 0; i < argc; i++)
    {
        string s    = args.at(i);
        char* c_arg = new char[s.length() + 1];
        strcpy(c_arg, s.c_str());
        argv[i] = c_arg;
    }

    // Run tests with the provided arguments
    int result = Catch::Session().run(argc, argv);

    // Clear memory
    for (size_t i = 0; i < argc; i++)
    {
        delete[] argv[i];
    }
    delete[] argv;

    printf("End.\n");
    // Infinite loop to avoid board reset each time we return
    for (;;)
    {
        Thread::sleep(10000);
    }
}