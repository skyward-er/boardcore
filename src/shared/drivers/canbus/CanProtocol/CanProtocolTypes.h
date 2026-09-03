/* Copyright (c) 2022 Skyward Experimental Rocketry
 * Author: Alberto Nidasio
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

#include <actuators/Servo/ServoData.h>
#include <sensors/SensorData.h>
#include <sensors/analog/BatteryVoltageSensorData.h>
#include <sensors/analog/Pitot/PitotData.h>

#include <cstring>
#include <reflect.hpp>

#include "CanProtocolData.h"

namespace Boardcore
{

inline uint32_t floatToInt32(float val)
{
    uint32_t val2 = 0;
    std::memcpy(&val2, &val, sizeof(float));
    return val2;
}

inline float int32ToFloat(uint32_t val)
{
    float val2 = 0;
    std::memcpy(&val2, &val, sizeof(uint32_t));
    return val2;
}

struct CanPitotData : PitotData
{
    uint8_t secondaryType = 0;
    uint8_t source        = 0;

    static constexpr auto reflect()
    {
        return STRUCT_DEF(CanPitotData, EXTEND_DEF(PitotData) FIELD_DEF(
                                            secondaryType) FIELD_DEF(source));
    }
};

struct CanPressureData : PressureData
{
    uint8_t secondaryType = 0;
    uint8_t source        = 0;

    static constexpr auto reflect()
    {
        return STRUCT_DEF(CanPressureData,
                          EXTEND_DEF(PressureData) FIELD_DEF(secondaryType)
                              FIELD_DEF(source));
    }
};

struct CanTemperatureData : TemperatureData
{
    uint8_t secondaryType = 0;
    uint8_t source        = 0;

    static constexpr auto reflect()
    {
        return STRUCT_DEF(CanTemperatureData,
                          EXTEND_DEF(TemperatureData) FIELD_DEF(secondaryType)
                              FIELD_DEF(source));
    }
};

struct CanCurrentData : CurrentData
{
    uint8_t secondaryType = 0;
    uint8_t source        = 0;

    static constexpr auto reflect()
    {
        return STRUCT_DEF(CanCurrentData, EXTEND_DEF(CurrentData) FIELD_DEF(
                                              secondaryType) FIELD_DEF(source));
    }
};

struct CanServoData : ServoData
{
    uint8_t secondaryType = 0;
    uint8_t source        = 0;

    static constexpr auto reflect()
    {
        return STRUCT_DEF(CanServoData, EXTEND_DEF(ServoData) FIELD_DEF(
                                            secondaryType) FIELD_DEF(source));
    }
};

struct CanBatteryVoltageSensorData : BatteryVoltageSensorData
{
    uint8_t secondaryType = 0;
    uint8_t source        = 0;

    static constexpr auto reflect()
    {
        return STRUCT_DEF(CanBatteryVoltageSensorData,
                          EXTEND_DEF(BatteryVoltageSensorData)
                              FIELD_DEF(secondaryType) FIELD_DEF(source));
    }
};

struct CanVoltageData : VoltageData
{
    uint8_t secondaryType = 0;
    uint8_t source        = 0;

    static constexpr auto reflect()
    {
        return STRUCT_DEF(CanVoltageData, EXTEND_DEF(VoltageData) FIELD_DEF(
                                              secondaryType) FIELD_DEF(source));
    }
};

struct DeviceStatus
{
    uint64_t timestamp = 0;
    int16_t logNumber  = 0;
    uint8_t state      = 0;

    bool armed   = false;
    bool hil     = false;
    bool logGood = false;

    static constexpr auto reflect()
    {
        return STRUCT_DEF(DeviceStatus,
                          FIELD_DEF(timestamp) FIELD_DEF(logNumber)
                              FIELD_DEF(state) FIELD_DEF(armed) FIELD_DEF(hil)
                                  FIELD_DEF(logGood));
    }
};

struct CanDeviceStatus : DeviceStatus
{
    uint8_t secondaryType = 0;
    uint8_t source        = 0;

    static constexpr auto reflect()
    {
        return STRUCT_DEF(CanDeviceStatus,
                          EXTEND_DEF(DeviceStatus) FIELD_DEF(secondaryType)
                              FIELD_DEF(source));
    }
};

struct ServoCommand
{
    uint64_t timestamp   = 0;
    uint32_t openingTime = 0;

    static constexpr auto reflect()
    {
        return STRUCT_DEF(ServoCommand,
                          FIELD_DEF(timestamp) FIELD_DEF(openingTime));
    }
};

struct CanServoCommand : ServoCommand
{
    uint8_t secondaryType = 0;
    uint8_t source        = 0;

    static constexpr auto reflect()
    {
        return STRUCT_DEF(CanServoCommand,
                          EXTEND_DEF(ServoCommand) FIELD_DEF(secondaryType)
                              FIELD_DEF(source));
    }
};

struct ServoFeedback
{
    uint64_t timestamp = 0;
    float aperture     = 0;
    bool open          = false;

    static constexpr auto reflect()
    {
        return STRUCT_DEF(ServoFeedback, FIELD_DEF(timestamp) FIELD_DEF(
                                             aperture) FIELD_DEF(open));
    }
};

struct CanServoFeedback : ServoFeedback
{
    uint8_t secondaryType = 0;
    uint8_t source        = 0;

    static constexpr auto reflect()
    {
        return STRUCT_DEF(CanServoFeedback,
                          EXTEND_DEF(ServoFeedback) FIELD_DEF(secondaryType)
                              FIELD_DEF(source));
    }
};

struct CanEvent
{
    uint64_t timestamp;
    uint8_t source = 0;
    uint8_t target = 0;
    uint8_t event  = 0;

    static constexpr auto reflect()
    {
        return STRUCT_DEF(CanEvent, FIELD_DEF(timestamp) FIELD_DEF(source)
                                        FIELD_DEF(target) FIELD_DEF(event));
    }
};

inline Canbus::CanMessage toCanMessage(const PitotData& data)
{
    Canbus::CanMessage message;

    message.id         = -1;
    message.length     = 1;
    message.payload[0] = (data.timestamp & ~0x3) << 30;
    message.payload[0] |= floatToInt32(data.airspeed);

    return message;
}

inline Canbus::CanMessage toCanMessage(const PressureData& data)
{
    Canbus::CanMessage message;

    message.id         = -1;
    message.length     = 1;
    message.payload[0] = (data.pressureTimestamp & ~0x3) << 30;
    message.payload[0] |= floatToInt32(data.pressure);

    return message;
}

inline Canbus::CanMessage toCanMessage(const TemperatureData& data)
{
    Canbus::CanMessage message;

    message.id         = -1;
    message.length     = 1;
    message.payload[0] = (data.temperatureTimestamp & ~0x3) << 30;
    message.payload[0] |= floatToInt32(data.temperature);

    return message;
}

inline Canbus::CanMessage toCanMessage(const CurrentData& data)
{
    Canbus::CanMessage message;

    message.id         = -1;
    message.length     = 1;
    message.payload[0] = (data.currentTimestamp & ~0x3) << 30;
    message.payload[0] |= floatToInt32(data.current);

    return message;
}

inline Canbus::CanMessage toCanMessage(const ServoData& data)
{
    Canbus::CanMessage message;

    // Denormalize the position in 8 bit
    uint16_t position = static_cast<uint16_t>(data.position * 65535);

    // The position is approximated into a 16 bit integer.
    // Packet: TIMESTAMP (32bit) | POSITION | CHANNEL | TIMER
    message.id         = -1;
    message.length     = 1;
    message.payload[0] = (data.timestamp & ~0x3) << 30;
    message.payload[0] |= static_cast<uint32_t>(position) << 16;
    message.payload[0] |= static_cast<uint16_t>(data.channel) << 8;
    message.payload[0] |= data.timer;

    return message;
}

inline Canbus::CanMessage toCanMessage(const BatteryVoltageSensorData& data)
{
    Canbus::CanMessage message;

    message.id         = -1;
    message.length     = 1;
    message.payload[0] = (data.voltageTimestamp & ~0x3) << 30;
    message.payload[0] |= floatToInt32(data.batVoltage);

    return message;
}

inline Canbus::CanMessage toCanMessage(const VoltageData& data)
{
    Canbus::CanMessage message;

    message.id         = -1;
    message.length     = 1;
    message.payload[0] = (data.voltageTimestamp & ~0x3) << 30;
    message.payload[0] |= floatToInt32(data.voltage);

    return message;
}

inline Canbus::CanMessage toCanMessage(const DeviceStatus& data)
{
    Canbus::CanMessage message;

    message.id         = -1;
    message.length     = 1;
    message.payload[0] = (data.timestamp & ~0x3) << 30;
    message.payload[0] |= data.state;
    message.payload[0] |= static_cast<uint16_t>(data.logNumber) << 8;
    message.payload[0] |= (data.armed ? 1 : 0) << 24;
    message.payload[0] |= (data.hil ? 1 : 0) << 25;
    message.payload[0] |= (data.logGood ? 1 : 0) << 26;

    return message;
}

inline Canbus::CanMessage toCanMessage(const ServoCommand& data)
{
    Canbus::CanMessage message;

    message.id         = -1;
    message.length     = 1;
    message.payload[0] = (data.timestamp & ~0x3) << 30;
    message.payload[0] |= data.openingTime;

    return message;
}

inline Canbus::CanMessage toCanMessage(const ServoFeedback& data)
{
    Canbus::CanMessage message;

    message.id         = -1;
    message.length     = 1;
    message.payload[0] = (data.timestamp & ~0x3) << 30;
    message.payload[0] |= static_cast<uint16_t>(data.aperture * 65535);
    message.payload[0] |= (data.open ? 1 : 0) << 16;

    return message;
}

inline CanPitotData pitotDataFromCanMessage(const Canbus::CanMessage& msg)
{
    CanPitotData data;

    data.deltaP        = 0.0;  // put to 0 to avoid undefined behaviour
    data.airspeed      = floatToInt32(msg.payload[0]);
    data.timestamp     = (msg.payload[0] >> 30) & ~0x3;
    data.secondaryType = msg.getSecondaryType();
    data.source        = msg.getSource();

    return data;
}

inline CanPressureData pressureDataFromCanMessage(const Canbus::CanMessage& msg)
{
    CanPressureData data;

    data.pressureTimestamp = (msg.payload[0] >> 30) & ~0x3;
    data.pressure          = int32ToFloat(msg.payload[0]);
    data.secondaryType     = msg.getSecondaryType();
    data.source            = msg.getSource();

    return data;
}

inline CanTemperatureData temperatureDataFromCanMessage(
    const Canbus::CanMessage& msg)
{
    CanTemperatureData data;

    data.temperatureTimestamp = (msg.payload[0] >> 30) & ~0x3;
    data.temperature          = int32ToFloat(msg.payload[0]);
    data.secondaryType        = msg.getSecondaryType();
    data.source               = msg.getSource();

    return data;
}

inline CanCurrentData currentDataFromCanMessage(const Canbus::CanMessage& msg)
{
    CanCurrentData data;

    data.currentTimestamp = (msg.payload[0] >> 30) & ~0x3;
    data.current          = int32ToFloat(msg.payload[0]);
    data.secondaryType    = msg.getSecondaryType();
    data.source           = msg.getSource();

    return data;
}

inline CanServoData servoDataFromCanMessage(const Canbus::CanMessage& msg)
{
    CanServoData data;

    data.timestamp     = (msg.payload[0] >> 30) & ~0x3;
    data.position      = static_cast<uint16_t>(msg.payload[0] >> 16) / 65535.f;
    data.channel       = static_cast<uint8_t>(msg.payload[0] >> 8);
    data.timer         = static_cast<uint8_t>(msg.payload[0]);
    data.secondaryType = msg.getSecondaryType();
    data.source        = msg.getSource();

    return data;
}

inline CanBatteryVoltageSensorData batteryVoltageDataFromCanMessage(
    const Canbus::CanMessage& msg)
{
    CanBatteryVoltageSensorData data;

    data.voltageTimestamp = (msg.payload[0] >> 30) & ~0x3;
    data.batVoltage       = int32ToFloat(msg.payload[0]);
    data.secondaryType    = msg.getSecondaryType();
    data.source           = msg.getSource();

    return data;
}

inline CanVoltageData voltageDataFromCanMessage(const Canbus::CanMessage& msg)
{
    CanVoltageData data;

    data.voltageTimestamp = (msg.payload[0] >> 30) & ~0x3;
    data.voltage          = int32ToFloat(msg.payload[0]);
    data.secondaryType    = msg.getSecondaryType();
    data.source           = msg.getSource();

    return data;
}

inline CanDeviceStatus deviceStatusFromCanMessage(const Canbus::CanMessage& msg)
{
    CanDeviceStatus data;

    data.timestamp     = (msg.payload[0] >> 30) & ~0x3;
    data.state         = static_cast<uint8_t>(msg.payload[0]);
    data.logNumber     = static_cast<int16_t>(msg.payload[0] >> 8);
    data.armed         = ((msg.payload[0] >> 24) & 1) != 0;
    data.hil           = ((msg.payload[0] >> 25) & 1) != 0;
    data.logGood       = ((msg.payload[0] >> 26) & 1) != 0;
    data.secondaryType = msg.getSecondaryType();
    data.source        = msg.getSource();

    return data;
}

inline CanServoCommand servoCommandFromCanMessage(const Canbus::CanMessage& msg)
{
    CanServoCommand data;

    data.timestamp     = (msg.payload[0] >> 30) & ~0x3;
    data.openingTime   = static_cast<uint32_t>(msg.payload[0]);
    data.secondaryType = msg.getSecondaryType();
    data.source        = msg.getSource();

    return data;
}

inline CanServoFeedback servoFeedbackFromCanMessage(
    const Canbus::CanMessage& msg)
{
    CanServoFeedback data;

    data.timestamp     = (msg.payload[0] >> 30) & ~0x3;
    data.aperture      = static_cast<uint16_t>(msg.payload[0]) / 65535.f;
    data.open          = ((msg.payload[0] >> 16) & 1) != 0;
    data.secondaryType = msg.getSecondaryType();
    data.source        = msg.getSource();

    return data;
}

}  // namespace Boardcore
