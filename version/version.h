/* Copyright (c) 2025 Skyward Experimental Rocketry
 * Author: Niccolò Betto
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

/**
 * @file version.h
 * @brief Generated version information header
 *
 * This file contains forward declarations for version information variables.
 * CMake generates and populates these variables during the build process.
 * Refer to version.cpp.in for the format and content of the generated file.
 *
 * @example Usage:
 *
 * #include <version.h>
 *
 * std::cout << "Version: " << GIT_VERSION_STRING << std::endl;
 */

#pragma once

extern const char* GIT_REV;  //< Short commit hash + dirty suffix
extern const char* GIT_TAG;  //< Tag name if current commit is tagged, or empty
extern const char* GIT_BRANCH;  //< Current branch name

extern const char* GIT_VERSION_STRING;  //< branch-rev[-dirty] or tag[-dirty]

extern const char* TARGET_NAME;  //< CMake target name
extern const char* BUILD_TYPE;   //< CMake build type (e.g. Debug, Release)

extern const char* SKYWARD_VERSION_STRING;  //< Skyward version string
