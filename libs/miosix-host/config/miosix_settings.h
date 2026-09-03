/* Copyright (c) 2021 Skyward Experimental Rocketry
 * Author: Damiano Amatruda
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

#pragma once

#ifndef COMPILE_FOR_HOST
#error "Unsupported"
#endif

#define MIOSIX_SETTINGS_VERSION 100

namespace miosix
{

#define SCHED_TYPE_PRIORITY

#define WITH_FILESYSTEM

#define WITH_DEVFS

#define SYNC_AFTER_WRITE

const unsigned char MAX_OPEN_FILES = 8;

#define WITH_BOOTLOG

#define WITH_ERRLOG

#define JTAG_DISABLE_SLEEP

const unsigned int STACK_MIN = 256;

const unsigned int STACK_IDLE = 256;

const unsigned int STACK_DEFAULT_FOR_PTHREAD = 2048;

const unsigned int MAX_PROCESS_IMAGE_SIZE = 64 * 1024;

const unsigned int MIN_PROCESS_STACK_SIZE = STACK_MIN;

const unsigned int SYSTEM_MODE_PROCESS_STACK_SIZE = 2 * 1024;

const short int PRIORITY_MAX = 4;

const unsigned char MAIN_PRIORITY = 1;

const unsigned int WATERMARK_LEN = 16;

const unsigned int WATERMARK_FILL = 0xaaaaaaaa;

const unsigned int STACK_FILL = 0xbbbbbbbb;

}  // namespace miosix
