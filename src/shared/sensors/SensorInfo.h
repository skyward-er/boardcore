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

#include <utils/KernelTime.h>

#include <chrono>
#include <functional>

namespace Boardcore
{

/**
 * @brief Represents the status of a sensor
 * handled by the sensor manager.
 */
enum class SensorStatus : uint8_t
{
    ENABLED,   // Initialized and enabled.
    DISABLED,  // Initialized but disabled.
    NOT_INIT,  // Not initialized, so it cannot be enabled.
};

/**
 * @brief Struct used to retrieve information of a sensor from the
 * sensor manager.
 *
 * It contains the id and group of the sensor, the sampling period
 * and the current status.
 */
struct SensorInfo
{
    std::string id;
    std::chrono::nanoseconds period;
    uint8_t groupID;
    SensorStatus status;

    SensorInfo(
        // cppcheck-suppress passedByValue
        const std::string id = "", uint32_t period = 0,
        std::function<void()> callback = []() {}, uint8_t groupID = 0,
        SensorStatus status = SensorStatus::NOT_INIT)
        : id(id), period(period), groupID(groupID), status(status)
    {
    }

    SensorInfo& operator=(const SensorInfo& info)
    {
        id      = info.id;
        period  = info.period;
        groupID = info.groupID;
        status  = info.status;

        return *this;
    }

    bool operator==(const SensorInfo& info) const
    {
        return id == info.id && period == info.period &&
               groupID == info.groupID && status == info.status;
    };

    std::string toString()
    {
        std::string ret = id + ", " + std::to_string(groupID) + ", " +
                          std::to_string(period.count()) + "ns, ";
        switch (status)
        {
            case SensorStatus::ENABLED:
                ret += "enabled";
                break;
            case SensorStatus::DISABLED:
                ret += "disabled";
                break;
            case SensorStatus::NOT_INIT:
                ret += "not init";
                break;
            default:
                break;
        }

        return ret;
    }
};

}  // namespace Boardcore
