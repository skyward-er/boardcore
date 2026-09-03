# Copyright (c) 2021 Skyward Experimental Rocketry
# Author: Damiano Amatruda
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in
# all copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.  IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
# THE SOFTWARE.

cmake_minimum_required(VERSION 3.16)
project(MiosixHost C CXX ASM)

add_library(miosix-host STATIC
    interfaces/delays.cpp
    interfaces/gpio.cpp
    interfaces/portability.cpp
    kernel/scheduler/priority/priority_scheduler_types.cpp
    kernel/error.cpp
    kernel/kernel.cpp
    kernel/sync.cpp
    util/util.cpp
    util/version.cpp
)
add_library(Miosix::Miosix::host ALIAS miosix-host)
target_include_directories(miosix-host PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})
target_compile_definitions(miosix-host PUBLIC COMPILE_FOR_HOST)
target_compile_features(miosix-host PUBLIC cxx_std_14)
