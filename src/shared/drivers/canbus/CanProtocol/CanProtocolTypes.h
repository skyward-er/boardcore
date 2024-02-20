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

inline Canbus::CanMessage toCanMessage(const PitotData& data)
{
    Canbus::CanMessage message;

    uint32_t airspeed;
    memcpy(&airspeed, &(data.airspeed), sizeof(airspeed));

    message.id         = -1;
    message.length     = 1;
    message.payload[0] = (data.timestamp & ~0x3) << 30;
    message.payload[0] |= airspeed;

    return message;
}

inline Canbus::CanMessage toCanMessage(const PressureData& data)
{
    Canbus::CanMessage message;

    uint32_t pressure;
    memcpy(&pressure, &(data.pressure), sizeof(pressure));

    message.id         = -1;
    message.length     = 1;
    message.payload[0] = (data.pressureTimestamp & ~0x3) << 30;
    message.payload[0] |= pressure;

    return message;
}
inline Canbus::CanMessage toCanMessage(const TemperatureData& data)
{
    Canbus::CanMessage message;

    uint32_t temperature;
    memcpy(&temperature, &(data.temperature), sizeof(temperature));

    message.id         = -1;
    message.length     = 1;
    message.payload[0] = (data.temperatureTimestamp & ~0x3) << 30;
    message.payload[0] |= temperature;

    return message;
}

inline Canbus::CanMessage toCanMessage(const CurrentData& data)
{
    Canbus::CanMessage message;

    uint32_t current;
    memcpy(&current, &(data.current), sizeof(current));

    message.id         = -1;
    message.length     = 1;
    message.payload[0] = (data.currentTimestamp & ~0x3) << 30;
    message.payload[0] |= current;

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

    uint32_t voltage;
    memcpy(&voltage, &(data.batVoltage), sizeof(voltage));

    message.id         = -1;
    message.length     = 1;
    message.payload[0] = (data.voltageTimestamp & ~0x3) << 30;
    message.payload[0] |= voltage;

    return message;
}

inline PitotData pitotDataFromCanMessage(const Canbus::CanMessage& msg)
{
    PitotData data;

    uint32_t airspeed = msg.payload[0];
    memcpy(&(data.airspeed), &airspeed, sizeof(data.airspeed));

    data.deltaP = 0.0;  // put to 0 to avoid undefined behaviour

    data.timestamp = (msg.payload[0] >> 30) & ~0x3;

    return data;
}

inline PressureData pressureDataFromCanMessage(const Canbus::CanMessage& msg)
{
    using namespace Units::Pressure;

    PressureData data;

    uint32_t pressure = msg.payload[0];
    uint32_t dest;
    memcpy(&dest, &pressure, sizeof(data.pressure));
    data.pressure = Pascal(static_cast<float>(dest));

    data.pressureTimestamp = (msg.payload[0] >> 30) & ~0x3;

    return data;
}

inline TemperatureData temperatureDataFromCanMessage(
    const Canbus::CanMessage& msg)
{
    TemperatureData data;

    uint32_t temperature = msg.payload[0];
    memcpy(&(data.temperature), &temperature, sizeof(data.temperature));

    data.temperatureTimestamp = (msg.payload[0] >> 30) & ~0x3;

    return data;
}

inline CurrentData currentDataFromCanMessage(const Canbus::CanMessage& msg)
{
    CurrentData data;

    uint32_t current = msg.payload[0];
    memcpy(&(data.current), &current, sizeof(data.current));

    data.currentTimestamp = (msg.payload[0] >> 30) & ~0x3;

    return data;
}

inline ServoData servoDataFromCanMessage(const Canbus::CanMessage& msg)
{
    ServoData data;

    data.timestamp = (msg.payload[0] >> 30) & ~0x3;
    data.position  = static_cast<uint16_t>(msg.payload[0] >> 16) / 65535.f;
    data.channel   = static_cast<uint8_t>(msg.payload[0] >> 8);
    data.timer     = static_cast<uint8_t>(msg.payload[0]);

    return data;
}

inline BatteryVoltageSensorData voltageDataFromCanMessage(
    const Canbus::CanMessage& msg)
{
    BatteryVoltageSensorData data;

    uint32_t voltage = msg.payload[0];
    memcpy(&(data.batVoltage), &voltage, sizeof(data.batVoltage));

    data.voltageTimestamp = (msg.payload[0] >> 30) & ~0x3;

    return data;
}

}  // namespace Boardcore
