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

find_path(MXGUI_INCLUDE_DIR1
    NAMES mxgui/display.h
    PATHS ${SBS_BASE}/libs
)

find_path(MXGUI_INCLUDE_DIR2
    NAMES config/mxgui_settings.h
    PATHS ${SBS_BASE}/libs/mxgui
)

include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(MXGUI
    FOUND_VAR MXGUI_FOUND
    REQUIRED_VARS MXGUI_INCLUDE_DIR1 MXGUI_INCLUDE_DIR2
)

if(MXGUI_FOUND)
    set(MXGUI_INCLUDE_DIRS ${MXGUI_INCLUDE_DIR1} ${MXGUI_INCLUDE_DIR2})
endif()

mark_as_advanced(MXGUI_INCLUDE_DIR1 MXGUI_INCLUDE_DIR2)
