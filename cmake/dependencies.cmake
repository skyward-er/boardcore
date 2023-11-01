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

# Miosix Kernel
include(${SBS_BASE}/libs/miosix-kernel/miosix/cmake/miosix.cmake EXCLUDE_FROM_ALL)

# Miosix Host
add_subdirectory(${SBS_BASE}/libs/miosix-host EXCLUDE_FROM_ALL)

# MxGui graphical library
include(${SBS_BASE}/libs/mxgui/cmake/mxgui.cmake)

# Serialization library
add_subdirectory(${SBS_BASE}/libs/tscpp EXCLUDE_FROM_ALL)

# Eigen library
set(CMAKE_BUILD_WITH_INSTALL_RPATH ON)
set(EIGEN_TEST_NOQT ON CACHE BOOL "Disable Qt support in unit tests")
set(CMAKE_Fortran_COMPILER NOTFOUND)
add_subdirectory(${SBS_BASE}/libs/eigen EXCLUDE_FROM_ALL)
target_compile_definitions(eigen INTERFACE EIGEN_MAX_ALIGN_BYTES=0)

# Format library
add_subdirectory(${SBS_BASE}/libs/fmt EXCLUDE_FROM_ALL)
target_compile_definitions(fmt-header-only INTERFACE _GLIBCXX_USE_WCHAR_T FMT_UNICODE=0 FMT_STATIC_THOUSANDS_SEPARATOR=0)
target_compile_options(fmt-header-only INTERFACE -fno-math-errno)

# Catch2 library
add_subdirectory(${SBS_BASE}/libs/Catch2 EXCLUDE_FROM_ALL)
list(APPEND CMAKE_MODULE_PATH ${SBS_BASE}/libs/Catch2/contrib)
include(Catch)

# MavLink library
add_subdirectory(${SBS_BASE}/libs/mavlink-skyward-lib EXCLUDE_FROM_ALL)
