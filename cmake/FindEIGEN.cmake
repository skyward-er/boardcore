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

find_path(EIGEN_INCLUDE_DIR
    NAMES signature_of_eigen3_matrix_library
    PATHS ${SBS_BASE}/libs/eigen
)
set(EIGEN_CFLAGS EIGEN_MAX_ALIGN_BYTES=0)

include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(EIGEN
    FOUND_VAR EIGEN_FOUND
    REQUIRED_VARS EIGEN_INCLUDE_DIR
)

if(EIGEN_FOUND)
    set(EIGEN_INCLUDE_DIRS ${EIGEN_INCLUDE_DIR})
    set(EIGEN_DEFINITIONS ${EIGEN_CFLAGS})
endif()

mark_as_advanced(EIGEN_INCLUDE_DIR)
