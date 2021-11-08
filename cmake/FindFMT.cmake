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

find_path(FMT_INCLUDE_DIR
    NAMES fmt/format.h
    PATHS ${SBS_BASE}/libs/fmt/include
)
find_library(FMT_LIBRARY
    NAMES fmt
    PATHS ${SBS_BASE}/libs/fmt
)
set(FMT_CFLAGS FMT_STATIC_THOUSANDS_SEPARATOR _GLIBCXX_USE_WCHAR_T)

include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(FMT
    FOUND_VAR FMT_FOUND
    REQUIRED_VARS FMT_INCLUDE_DIR FMT_LIBRARY
)

if(FMT_FOUND)
    set(FMT_INCLUDE_DIRS ${FMT_INCLUDE_DIR})
    set(FMT_LIBRARIES ${FMT_LIBRARY})
    set(FMT_DEFINITIONS ${FMT_CFLAGS})
endif()

mark_as_advanced(FMT_INCLUDE_DIR FMT_LIBRARY)
