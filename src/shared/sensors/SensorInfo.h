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

#include <atomic>
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
    uint32_t period;  ///< Period in ms
    std::function<void()> callback;
    bool isEnabled;
    bool isInitialized;

    SensorInfo(
        const std::string id = "", uint32_t period = 0,
        std::function<void()> callback = []() {}, bool isEnabled = true);

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

private:
    SensorInfo(const std::string id, uint32_t period,
               std::function<void()> callback, bool isEnabled,
               bool isInitialized);
};

// cppcheck-suppress passedByValue
inline SensorInfo::SensorInfo(const std::string id, uint32_t period,
                              std::function<void()> callback, bool isEnabled)
    : SensorInfo(id, period, callback, isEnabled, false)
{
}

// cppcheck-suppress passedByValue
inline SensorInfo::SensorInfo(const std::string id, uint32_t period,
                              std::function<void()> callback, bool isEnabled,
                              bool isInitialized)
    : id(id), period(period), callback(callback), isEnabled(isEnabled),
      isInitialized(isInitialized)
{
}

}  // namespace Boardcore
