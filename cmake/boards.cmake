# Copyright (c) 2023 Skyward Experimental Rocketry
# Authors: Alberto Nidasio
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

# Boardcore custom boards registry (Miosix 3 layout)
#
# Miosix 3.0 has no external-board hook: custom boards must live in the
# kernel tree (miosix/arch/board/<board>/) and be listed in MIOSIX_BOARDS in
# miosix/arch/CMakeLists.txt. Once a Boardcore BSP is ported to the Miosix 3
# layout, add its name here so that `sbs list boards` and target validation
# know about it. The old board_options.cmake contract is obsolete and was
# removed together with the pre-Miosix-3 src/bsps tree.
set(BOARDCORE_BOARDS "")
