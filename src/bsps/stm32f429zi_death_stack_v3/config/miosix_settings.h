/* Copyright (c) 2022 Skyward Experimental Rocketry
 * Authors: Emilio Corigliano, Alberto Nidasio
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

// Before you can compile the kernel you have to configure it by editing this
// file. After that, comment out this line to disable the reminder error.
// The PARSING_FROM_IDE is because Netbeans gets confused by this, it is never
// defined when compiling the code.
#ifndef PARSING_FROM_IDE
//#error This error is a reminder that you have not edited miosix_settings.h
// yet.
#endif  // PARSING_FROM_IDE

/**
 * \file miosix_settings.h
 * NOTE: this file contains ONLY configuration options that are not dependent
 * on architecture specific details. The other options are in the following
 * files which are included here:
 * miosix/arch/architecture name/common/arch_settings.h
 * miosix/config/arch/architecture name/board name/board_settings.h
 */
#include "arch_settings.h"
#include "board_settings.h"
#include "util/version.h"

/**
 * \internal
 * Versioning for miosix_settings.h for out of git tree projects
 */
#define MIOSIX_SETTINGS_VERSION 300

namespace miosix
{

/**
 * \addtogroup Settings
 * \{
 */

//
// Scheduler options
//

/// \def SCHED_TYPE_PRIORITY
/// If uncommented selects the priority scheduler
/// \def SCHED_TYPE_CONTROL_BASED
/// If uncommented selects the control based scheduler
/// \def SCHED_TYPE_EDF
/// If uncommented selects the EDF scheduler
// Uncomment only *one* of those

#define SCHED_TYPE_PRIORITY
//#define SCHED_TYPE_CONTROL_BASED
//#define SCHED_TYPE_EDF

/// \def WITH_CPU_TIME_COUNTER
/// Allows to enable/disable CPUTimeCounter to save code size and remove its
/// overhead from the scheduling process. By default it is not defined
/// (CPUTimeCounter is disabled).
//#define WITH_CPU_TIME_COUNTER

//
// Filesystem options
//

/// \def WITH_FILESYSTEM
/// Allows to enable/disable filesystem support to save code size
/// By default it is defined (filesystem support is enabled)
#define WITH_FILESYSTEM

/// \def WITH_DEVFS
/// Allows to enable/disable DevFs support to save code size
/// By default it is defined (DevFs is enabled)
#define WITH_DEVFS

/// \def SYNC_AFTER_WRITE
/// Increases filesystem write robustness. After each write operation the
/// filesystem is synced so that a power failure happens data is not lost
/// (unless power failure happens exactly between the write and the sync)
/// Unfortunately write latency and throughput becomes twice as worse
/// By default it is defined (slow but safe)
#define SYNC_AFTER_WRITE

/// Maximum number of open files. Trying to open more will fail.
/// Cannot be lower than 3, as the first three are stdin, stdout, stderr
const unsigned char MAX_OPEN_FILES = 8;

/// \def WITH_PROCESSES
/// If uncommented enables support for processes as well as threads.
/// This enables the dynamic loader to load elf programs, the extended system
/// call service and, if the hardware supports it, the MPU to provide memory
/// isolation of processes
//#define WITH_PROCESSES

#if defined(WITH_PROCESSES) && defined(__NO_EXCEPTIONS)
#error Processes require C++ exception support
#endif  // defined(WITH_PROCESSES) && defined(__NO_EXCEPTIONS)

#if defined(WITH_PROCESSES) && !defined(WITH_FILESYSTEM)
#error Processes require filesystem support
#endif  // defined(WITH_PROCESSES) && !defined(WITH_FILESYSTEM)

#if defined(WITH_PROCESSES) && !defined(WITH_DEVFS)
#error Processes require devfs support
#endif  // defined(WITH_PROCESSES) && !defined(WITH_DEVFS)

//
// C/C++ standard library I/O (stdin, stdout and stderr related)
//

/// \def WITH_BOOTLOG
/// Uncomment to print bootlogs on stdout.
/// By default it is defined (bootlogs are printed)
#define WITH_BOOTLOG

/// \def WITH_ERRLOG
/// Uncomment for debug information on stdout.
/// By default it is defined (error information is printed)
#define WITH_ERRLOG

//
// Kernel related options (stack sizes, priorities)
//

/// \def WITH_DEEP_SLEEP
/// Adds interfaces and required variables to support deep sleep state switch
/// automatically when peripherals are not required
//#define WITH_DEEP_SLEEP

/**
 * \def JTAG_DISABLE_SLEEP
 * JTAG debuggers lose communication with the device if it enters sleep
 * mode, so to use debugging it is necessary to disable sleep in the idle
 * thread. By default it is not defined (idle thread calls sleep).
 */
//#define JTAG_DISABLE_SLEEP

#if defined(WITH_DEEP_SLEEP) && defined(JTAG_DISABLE_SLEEP)
#error Deep sleep cannot work together with jtag
#endif  // defined(WITH_PROCESSES) && !defined(WITH_DEVFS)

/// Minimum stack size (MUST be divisible by 4)
const unsigned int STACK_MIN = 256;

/// \internal Size of idle thread stack.
/// Should be >=STACK_MIN (MUST be divisible by 4)
const unsigned int STACK_IDLE = 256;

/// Default stack size for pthread_create.
/// The chosen value is enough to call C standard library functions
/// such as printf/fopen which are stack-heavy
const unsigned int STACK_DEFAULT_FOR_PTHREAD = 2048;

/// Maximum size of the RAM image of a process. If a program requires more
/// the kernel will not run it (MUST be divisible by 4)
const unsigned int MAX_PROCESS_IMAGE_SIZE = 64 * 1024;

/// Minimum size of the stack for a process. If a program specifies a lower
/// size the kernel will not run it (MUST be divisible by 4)
const unsigned int MIN_PROCESS_STACK_SIZE = STACK_MIN;

/// Every userspace thread has two stacks, one for when it is running in
/// userspace and one for when it is running in kernelspace (that is, while it
/// is executing system calls). This is the size of the stack for when the
/// thread is running in kernelspace (MUST be divisible by 4)
const unsigned int SYSTEM_MODE_PROCESS_STACK_SIZE = 2 * 1024;

/// Number of priorities (MUST be >1)
/// PRIORITY_MAX-1 is the highest priority, 0 is the lowest. -1 is reserved as
/// the priority of the idle thread.
/// The meaning of a thread's priority depends on the chosen scheduler.
#ifdef SCHED_TYPE_PRIORITY
// Can be modified, but a high value makes context switches more expensive
const short int PRIORITY_MAX = 4;
#elif defined(SCHED_TYPE_CONTROL_BASED)
// Don't touch, the limit is due to the fixed point implementation
// It's not needed for if floating point is selected, but kept for consistency
const short int PRIORITY_MAX = 64;
#else  // SCHED_TYPE_EDF
// Doesn't exist for this kind of scheduler
#endif

/// Priority of main()
/// The meaning of a thread's priority depends on the chosen scheduler.
const unsigned char MAIN_PRIORITY = 1;

#ifdef SCHED_TYPE_PRIORITY
/// Maximum thread time slice in nanoseconds, after which preemption occurs
const unsigned int MAX_TIME_SLICE = 1000000;
#endif  // SCHED_TYPE_PRIORITY

//
// Other low level kernel options. There is usually no need to modify these.
//

/// \internal Length of wartermark (in bytes) to check stack overflow.
/// MUST be divisible by 4 and can also be zero.
/// A high value increases context switch time.
const unsigned int WATERMARK_LEN = 16;

/// \internal Used to fill watermark
const unsigned int WATERMARK_FILL = 0xaaaaaaaa;

/// \internal Used to fill stack (for checking stack usage)
const unsigned int STACK_FILL = 0xbbbbbbbb;

// Compiler version checks
#if !defined(_MIOSIX_GCC_PATCH_MAJOR) || _MIOSIX_GCC_PATCH_MAJOR < 3
#error \
    "You are using a too old or unsupported compiler. Get the latest one from https://miosix.org/wiki/index.php?title=Miosix_Toolchain"
#endif
#if _MIOSIX_GCC_PATCH_MAJOR > 3
#warning "You are using a too new compiler, which may not be supported"
#endif

/**
 * \}
 */

}  // namespace miosix
