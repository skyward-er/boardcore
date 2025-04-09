# Copyright (c) 2025 Skyward Experimental Rocketry
# Author: Niccolò Betto
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

# --- Version Information File Generation ---

# Find Git executable
find_package(Git REQUIRED QUIET)

set(GIT_WORKING_DIR ${CMAKE_SOURCE_DIR})

# Determine dirty status
execute_process(
    COMMAND ${GIT_EXECUTABLE} describe --dirty --always --broken
    WORKING_DIRECTORY ${GIT_WORKING_DIR}
    OUTPUT_VARIABLE GIT_DESCRIBE_OUTPUT # We don't use the output directly, just for the dirty check
    OUTPUT_STRIP_TRAILING_WHITESPACE
    RESULT_VARIABLE GIT_DESCRIBE_RESULT
    ERROR_QUIET # Ignore errors
)
set(GIT_DIRTY_SUFFIX "")
# Check if the command succeeded AND if its output ends with "-dirty"
if(GIT_DESCRIBE_RESULT EQUAL 0 AND GIT_DESCRIBE_OUTPUT MATCHES "-dirty$")
    set(GIT_DIRTY_SUFFIX "-dirty")
endif()

# Get *annotated* tag name
execute_process(
    COMMAND ${GIT_EXECUTABLE} describe --exact-match HEAD
    WORKING_DIRECTORY ${GIT_WORKING_DIR}
    OUTPUT_VARIABLE GIT_TAG # Captures the annotated tag name if successful
    OUTPUT_STRIP_TRAILING_WHITESPACE
    RESULT_VARIABLE GIT_EXACT_TAG_RESULT # Will be 0 on success, non-zero on failure
    ERROR_QUIET # Ignore errors, we expect this to fail when not on an annotated tag
)

# Get branch name
execute_process(
    COMMAND ${GIT_EXECUTABLE} rev-parse --abbrev-ref HEAD
    WORKING_DIRECTORY ${GIT_WORKING_DIR}
    OUTPUT_VARIABLE GIT_BRANCH
    OUTPUT_STRIP_TRAILING_WHITESPACE
    # No ERROR_QUIET/RESULT_VARIABLE -> CMake halts on git command failure
)

# Get short commit hash
execute_process(
    COMMAND ${GIT_EXECUTABLE} rev-parse --short HEAD
    WORKING_DIRECTORY ${GIT_WORKING_DIR}
    OUTPUT_VARIABLE GIT_REV
    OUTPUT_STRIP_TRAILING_WHITESPACE
    # No ERROR_QUIET/RESULT_VARIABLE -> CMake halts on git command failure
)

# Determine the version info based on tag match result
set(GIT_VERSION_INFO "")
if(GIT_EXACT_TAG_RESULT EQUAL 0)
    # Use the tag name when exactly on an annotated tag
    set(GIT_VERSION_INFO "${GIT_TAG}")
else()
    # Use branch-rev when not on a tag
    set(GIT_VERSION_INFO "${GIT_BRANCH}-${GIT_REV}")
endif()

# Construct the final version string
# Format: <annotated_tag>[-dirty] OR <branch>-<hash>[-dirty]
set(GIT_VERSION_STRING "${GIT_VERSION_INFO}${GIT_DIRTY_SUFFIX}")

# Additional variables set by the called of this script via command line args:
# - TARGET_NAME
# - CMAKE_BUILD_TYPE

configure_file(
  "${BOARDCORE_PATH}/version/version.cpp.in"
  "${OUT_DIR}/version.cpp"
)
