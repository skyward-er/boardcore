/* Copyright (c) 2015-2017 Skyward Experimental Rocketry
 * Author: Alain Carlucci
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

#include <Common.h>
#include <Singleton.h>
#include <diagnostic/FaultCounterData.h>

namespace Boardcore
{

class FaultCounterMgr : public Singleton<FaultCounterMgr>
{
    friend class Singleton<FaultCounterMgr>;

public:
    ~FaultCounterMgr() {}

    void Increment(const Fault id)
    {
        using FaultCounterData::FaultToCategory;

        miosix::FastInterruptDisableLock dLock;
        const uint32_t numId = static_cast<uint32_t>(id);
        const uint32_t catId = FaultToCategory[numId];
        mCounters[numId]++;
        mCategories[catId]++;

        mFaultTriggered[numId]    = 1;
        mCategoryTriggered[catId] = 1;
    }

    inline std::pair<const uint8_t *, size_t> GetFaultCounters() const
    {
        return std::make_pair(mCounters, Fault_SIZE);
    }

    inline std::pair<const uint8_t *, size_t> GetCategoryCounters() const
    {
        using FaultCounterData::FaultCategory_SIZE;
        return std::make_pair(mCategories, FaultCategory_SIZE);
    }

private:
    uint8_t mCounters[Fault_SIZE];
    uint8_t mCategories[FaultCounterData::FaultCategory_SIZE];

    std::vector<bool> mFaultTriggered;
    std::vector<bool> mCategoryTriggered;

    FaultCounterMgr()
    {
        memset(mCounters, 0, sizeof(mCounters));
        memset(mCategories, 0, sizeof(mCategories));

        // TODO: static bitmap?
        mFaultTriggered.resize(Fault_SIZE);
        mCategoryTriggered.resize(FaultCounterData::FaultCategory_SIZE);
    }
};

#define sFaultCounterMgr FaultCounterMgr::getInstance()

}  // namespace Boardcore
