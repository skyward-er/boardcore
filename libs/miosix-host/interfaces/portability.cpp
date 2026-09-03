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

#include "portability.h"

namespace miosix_private
{

void IRQsystemReboot() {}

inline void doYield() {}

void initCtxsave(unsigned int *ctxsave, void *(*pc)(void *), unsigned int *sp,
                 void *argv)
{
}

SyscallParameters::SyscallParameters(unsigned int *context) {}

int SyscallParameters::getSyscallId() const { return 0; }

unsigned int SyscallParameters::getFirstParameter() const { return 0; }

unsigned int SyscallParameters::getSecondParameter() const { return 0; }

unsigned int SyscallParameters::getThirdParameter() const { return 0; }

void SyscallParameters::setReturnValue(unsigned int ret) {}

FaultData::FaultData() {}

FaultData::FaultData(int id, unsigned int pc, unsigned int arg) {}

bool FaultData::faultHappened() const { return false; }

void FaultData::print() const {}

void initCtxsave(unsigned int *ctxsave, void *(*pc)(void *), unsigned int *sp,
                 void *argv, unsigned int *gotBase)
{
}

inline void portableSwitchToUserspace() {}

void IRQstackOverflowCheck() {}

void IRQportableStartKernel() {}

inline void doDisableInterrupts() {}

inline void doEnableInterrupts() {}

inline bool checkAreInterruptsEnabled() { return true; }

void sleepCpu() {}

}  // namespace miosix_private
