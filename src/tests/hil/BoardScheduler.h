/* Copyright (c) 2023 Skyward Experimental Rocketry
 * Author: Matteo Pignataro
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

#include <scheduler/TaskScheduler.h>

#include <utils/ModuleManager/ModuleManager.hpp>

namespace HILTest
{
/**
 * @brief Class that wraps the 4 main task schedulers of the entire OBSW.
 * There is a task scheduler for every miosix priority
 */
class BoardScheduler : public Boardcore::Module
{
public:
    BoardScheduler();

    /**
     * @brief Get the Scheduler object relative to the requested priority
     *
     * @param priority The task scheduler priority
     * @return Boardcore::TaskScheduler& Reference to the requested task
     * scheduler.
     * @note Min priority scheduler is returned in case of non valid priority.
     */
    Boardcore::TaskScheduler* getScheduler(miosix::Priority priority);

    [[nodiscard]] bool start();

    /**
     * @brief Returns if all the schedulers are up and running
     */
    bool isStarted();

private:
    Boardcore::TaskScheduler* scheduler1;
    Boardcore::TaskScheduler* scheduler2;
    Boardcore::TaskScheduler* scheduler3;
    Boardcore::TaskScheduler* scheduler4;
};
}  // namespace HILTest
