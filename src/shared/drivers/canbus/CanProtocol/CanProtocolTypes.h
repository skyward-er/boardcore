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

    static std::string header()
    {
        return "timestamp,deltaP,airspeed,secondaryType,source\n";
    }

    void print(std::ostream& os) const
    {
        os << timestamp << "," << deltaP << "," << airspeed << ","
           << static_cast<int>(secondaryType) << "," << static_cast<int>(source)
           << "\n";
    }
};

struct CanPressureData : PressureData
{
    uint8_t secondaryType = 0;
    uint8_t source        = 0;

    static std::string header()
    {
        return "timestamp,pressure,secondaryType,source\n";
    }

    void print(std::ostream& os) const
    {
        os << pressureTimestamp << "," << pressure << ","
           << static_cast<int>(secondaryType) << "," << static_cast<int>(source)
           << "\n";
    }
};

struct CanTemperatureData : TemperatureData
{
    uint8_t secondaryType = 0;
    uint8_t source        = 0;

    static std::string header()
    {
        return "timestamp,temperature,secondaryType,source\n";
    }

    void print(std::ostream& os) const
    {
        os << temperatureTimestamp << "," << temperature << ","
           << static_cast<int>(secondaryType) << "," << static_cast<int>(source)
           << "\n";
    }
};

struct CanCurrentData : CurrentData
{
    uint8_t secondaryType = 0;
    uint8_t source        = 0;

    static std::string header()
    {
        return "timestamp,current,secondaryType,source\n";
    }

    void print(std::ostream& os) const
    {
        os << currentTimestamp << "," << current << ","
           << static_cast<int>(secondaryType) << "," << static_cast<int>(source)
           << "\n";
    }
};

struct CanServoData : ServoData
{
    uint8_t secondaryType = 0;
    uint8_t source        = 0;

    static std::string header()
    {
        return "timestamp,timer,channel,position,secondaryType,source\n";
    }

    void print(std::ostream& os) const
    {
        os << timestamp << "," << static_cast<int>(timer) << ","
           << static_cast<int>(channel) << "," << position << ","
           << static_cast<int>(secondaryType) << "," << static_cast<int>(source)
           << "\n";
    }
};

struct CanBatteryVoltageSensorData : BatteryVoltageSensorData
{
    uint8_t secondaryType = 0;
    uint8_t source        = 0;

    static std::string header()
    {
        return "voltageTimestamp,channelId,voltage,batVoltage,secondaryType,"
               "source\n";
    }

    void print(std::ostream& os) const
    {
        os << voltageTimestamp << "," << static_cast<int>(channelId) << ","
           << voltage << "," << batVoltage << ","
           << static_cast<int>(secondaryType) << "," << static_cast<int>(source)
           << "\n";
    }
};

struct CanVoltageData : VoltageData
{
    uint8_t secondaryType = 0;
    uint8_t source        = 0;

    static std::string header()
    {
        return "timestamp,voltage,secondaryType,source\n";
    }

    void print(std::ostream& os) const
    {
        os << voltageTimestamp << "," << voltage << ","
           << static_cast<int>(secondaryType) << "," << static_cast<int>(source)
           << "\n";
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

    static std::string header()
    {
        return "timestamp,state,logNumber,armed,hil,logGood\n";
    }

    void print(std::ostream& os) const
    {
        os << timestamp << "," << static_cast<int>(state) << "," << logNumber
           << "," << (armed ? 1 : 0) << "," << (hil ? 1 : 0) << ","
           << (logGood ? 1 : 0) << "\n";
    }
};

struct CanDeviceStatus : DeviceStatus
{
    uint8_t secondaryType = 0;
    uint8_t source        = 0;

    static std::string header()
    {
        return "timestamp,state,logNumber,armed,hil,logGood,secondaryType,"
               "source\n";
    }

    void print(std::ostream& os) const
    {
        os << timestamp << "," << static_cast<int>(state) << "," << logNumber
           << "," << (armed ? 1 : 0) << "," << (hil ? 1 : 0) << ","
           << (logGood ? 1 : 0) << "," << static_cast<int>(secondaryType) << ","
           << static_cast<int>(source) << "\n";
    }
};

struct ServoCommand
{
    uint64_t timestamp   = 0;
    uint32_t openingTime = 0;

    static std::string header() { return "timestamp,openingTime\n"; }

    void print(std::ostream& os) const
    {
        os << timestamp << "," << openingTime << "\n";
    }
};

struct CanServoCommand : ServoCommand
{
    uint8_t secondaryType = 0;
    uint8_t source        = 0;

    static std::string header()
    {
        return "timestamp,openingTime,secondaryType,source\n";
    }

    void print(std::ostream& os) const
    {
        os << timestamp << "," << openingTime << ","
           << static_cast<int>(secondaryType) << "," << static_cast<int>(source)
           << "\n";
    }
};

struct ServoFeedback
{
    uint64_t timestamp = 0;
    float aperture     = 0;
    bool open          = false;

    static std::string header() { return "timestamp,aperture,open\n"; }

    void print(std::ostream& os) const
    {
        os << timestamp << "," << aperture << "," << (open ? 1 : 0) << "\n";
    }
};

struct CanServoFeedback : ServoFeedback
{
    uint8_t secondaryType = 0;
    uint8_t source        = 0;

    static std::string header()
    {
        return "timestamp,aperture,open,secondaryType,source\n";
    }

    void print(std::ostream& os) const
    {
        os << timestamp << "," << aperture << "," << (open ? 1 : 0) << ","
           << static_cast<int>(secondaryType) << "," << static_cast<int>(source)
           << "\n";
    }
};

struct CanEvent
{
    uint64_t timestamp;
    uint8_t source = 0;
    uint8_t target = 0;
    uint8_t event  = 0;

    static std::string header() { return "timestamp,source,target,event"; }

    void print(std::ostream& os) const
    {
        os << timestamp << "," << source << "," << target << "," << event
           << "\n";
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
