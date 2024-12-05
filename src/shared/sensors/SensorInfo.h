/* Copyright (c) 2020 Skyward Experimental Rocketry
 * Author: Luca Conterio
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

#include <units/Frequency.h>
#include <utils/KernelTime.h>

#include <chrono>
#include <functional>

namespace Boardcore
{

/**
 * @brief Sensors information struct needed by the SensorManager.
 *
 * This structure contains the sampling period of a sensor,
 * the function to be called after the sampling (callback) and
 * one boolean indicating if the sensor has to be sampled (is enabled).
 */
struct SensorInfo
{
    std::string id;
    std::chrono::nanoseconds period;
    std::function<void()> callback;
    bool isEnabled;
    bool isInitialized;

    SensorInfo(
        // cppcheck-suppress passedByValue
        const std::string id = "", uint32_t period = 0,
        std::function<void()> callback = []() {}, bool isEnabled = true)
        : SensorInfo(id, std::chrono::milliseconds{period}, std::move(callback),
                     isEnabled)
    {
    }

    SensorInfo(
        // cppcheck-suppress passedByValue
        const std::string id, Units::Frequency::Hertz frequency,
        std::function<void()> callback = []() {}, bool isEnabled = true)
        : SensorInfo(id,
                     std::chrono::nanoseconds{
                         static_cast<int64_t>(sToNs(1) / frequency.value())},
                     std::move(callback), isEnabled)
    {
    }

    SensorInfo(
        // cppcheck-suppress passedByValue
        const std::string id, std::chrono::nanoseconds period,
        std::function<void()> callback = []() {}, bool isEnabled = true)
        : id(id), period(period), callback(std::move(callback)),
          isEnabled(isEnabled), isInitialized(false)
    {
    }

    SensorInfo& operator=(const SensorInfo& info)
    {
        id            = info.id;
        period        = info.period;
        callback      = info.callback;
        isEnabled     = info.isEnabled;
        isInitialized = info.isInitialized;

        return *this;
    }

    bool operator==(const SensorInfo& info) const
    {
        return id == info.id && period == info.period &&
               isEnabled == info.isEnabled &&
               isInitialized == info.isInitialized &&
               callback.target_type() == info.callback.target_type() &&
               callback.target<void()>() == info.callback.target<void()>();
    };
};

}  // namespace Boardcore
