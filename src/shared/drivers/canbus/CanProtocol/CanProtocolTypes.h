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
#include <algorithms/Ereg/EregConfig.h>
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

inline uint64_t int64ToUint64(int64_t val)
{
    uint64_t val2 = 0;
    std::memcpy(&val2, &val, sizeof(int64_t));
    return val2;
}

inline int64_t uint64ToInt64(uint64_t val)
{
    int64_t val2 = 0;
    std::memcpy(&val2, &val, sizeof(uint64_t));
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
    uint8_t servoId      = 0;

    static constexpr auto reflect()
    {
        return STRUCT_DEF(ServoCommand, FIELD_DEF(timestamp) FIELD_DEF(
                                            openingTime) FIELD_DEF(servoId));
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

struct MeaData
{
    float mass = 0;

    static constexpr auto reflect()
    {
        return STRUCT_DEF(MeaData, FIELD_DEF(mass));
    }
};

struct CanMeaData : MeaData
{
    uint8_t secondaryType = 0;
    uint8_t source        = 0;

    static constexpr auto reflect()
    {
        return STRUCT_DEF(CanMeaData, EXTEND_DEF(MeaData) FIELD_DEF(
                                          secondaryType) FIELD_DEF(source));
    }
};
struct ValveData
{
    uint64_t timestamp = 0;
    uint8_t idx        = 0;
    uint8_t position   = 0;
    bool open          = false;

    static constexpr auto reflect()
    {
        return STRUCT_DEF(ValveData, FIELD_DEF(timestamp) FIELD_DEF(idx)
                                         FIELD_DEF(position) FIELD_DEF(open));
    }
};

struct CanValveData : ValveData
{
    uint8_t secondaryType = 0;
    uint8_t source        = 0;

    static constexpr auto reflect()
    {
        return STRUCT_DEF(CanValveData, EXTEND_DEF(ValveData) FIELD_DEF(
                                            secondaryType) FIELD_DEF(source));
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

struct SequenceConfig
{
    uint64_t timestamp;
    uint32_t fullThrottleTime;
    uint32_t lowThrottleTime;
    uint32_t pilotLeadTime;
    float pilotOxPosition;
    float pilotFuelPosition;

    static constexpr auto reflect()

    {
        return STRUCT_DEF(
            SequenceConfig,
            FIELD_DEF(timestamp) FIELD_DEF(fullThrottleTime)
                FIELD_DEF(lowThrottleTime) FIELD_DEF(pilotLeadTime)
                    FIELD_DEF(pilotOxPosition) FIELD_DEF(pilotFuelPosition));
    }
};

struct CanSequenceConfig : SequenceConfig
{
    uint8_t secondaryType = 0;
    uint8_t source        = 0;

    static constexpr auto reflect()
    {
        return STRUCT_DEF(CanSequenceConfig,
                          EXTEND_DEF(SequenceConfig) FIELD_DEF(secondaryType)
                              FIELD_DEF(source));
    }
};

struct EregPIDSet
{
    float KpPressurization;
    float KiPressurization;
    float KdPressurization;
    float KpDischarge;
    float KiDischarge;
    float KdDischarge;
    uint8_t eregId;

    static constexpr auto reflect()
    {
        return STRUCT_DEF(
            EregPIDSet, FIELD_DEF(KpPressurization) FIELD_DEF(KiPressurization)
                            FIELD_DEF(KdPressurization) FIELD_DEF(KpDischarge)
                                FIELD_DEF(KiDischarge) FIELD_DEF(KdDischarge));
    }
};

struct CanEregPIDSet : EregPIDSet
{
    uint8_t secondaryType = 0;
    uint8_t source        = 0;

    static constexpr auto reflect()
    {
        return STRUCT_DEF(CanEregPIDSet, EXTEND_DEF(EregPIDSet) FIELD_DEF(
                                             secondaryType) FIELD_DEF(source));
    }
};
struct EregTarget
{
    uint64_t timestamp;
    float oxTarget;
    float fuelTarget;

    static constexpr auto reflect()
    {
        return STRUCT_DEF(EregTarget, FIELD_DEF(timestamp) FIELD_DEF(oxTarget)
                                          FIELD_DEF(fuelTarget));
    }
};

struct CanEregTarget : EregTarget
{
    uint8_t secondaryType = 0;
    uint8_t source        = 0;

    static constexpr auto reflect()
    {
        return STRUCT_DEF(CanEregTarget, EXTEND_DEF(EregTarget) FIELD_DEF(
                                             secondaryType) FIELD_DEF(source));
    }
};
struct IgnitionThresholds
{
    uint64_t timestamp;
    float igniterThreshold;
    float pilotThreshold;

    static constexpr auto reflect()
    {
        return STRUCT_DEF(IgnitionThresholds,
                          FIELD_DEF(timestamp) FIELD_DEF(igniterThreshold)
                              FIELD_DEF(pilotThreshold));
    }
};

struct CanIgnitionThresholds : IgnitionThresholds
{
    uint8_t secondaryType = 0;
    uint8_t source        = 0;

    static constexpr auto reflect()
    {
        return STRUCT_DEF(CanIgnitionThresholds,
                          EXTEND_DEF(IgnitionThresholds)
                              FIELD_DEF(secondaryType) FIELD_DEF(source));
    }
};

struct EregServoCoefficients
{
    uint8_t eregId                              = 0;
    float coefficients[POLY_SERVO_COEFF_NUMBER] = {};

    EregServoCoefficients() = default;

    EregServoCoefficients(uint8_t id,
                          const float coeffs[POLY_SERVO_COEFF_NUMBER])
        : eregId(id), coefficients{}
    {
        for (int i = 0; i < POLY_SERVO_COEFF_NUMBER; i++)
            coefficients[i] = coeffs[i];
    };

    static constexpr auto reflect()
    {
        return STRUCT_DEF(EregServoCoefficients,
                          FIELD_DEF(eregId) FIELD_DEF(coefficients));
    }
};

struct CanEregServoCoefficients : EregServoCoefficients
{
    uint8_t secondaryType = 0;
    uint8_t source        = 0;

    CanEregServoCoefficients() = default;

    static constexpr auto reflect()
    {
        return STRUCT_DEF(CanEregServoCoefficients,
                          EXTEND_DEF(EregServoCoefficients)
                              FIELD_DEF(secondaryType) FIELD_DEF(source));
    }
};

struct MEAInitialMass
{
    float mass = 0;

    static constexpr auto reflect()
    {
        return STRUCT_DEF(MEAInitialMass, FIELD_DEF(mass));
    }
};

struct CanMEAInitialMass : MEAInitialMass
{
    uint8_t secondaryType = 0;
    uint8_t source        = 0;

    static constexpr auto reflect()
    {
        return STRUCT_DEF(CanMEAInitialMass,
                          EXTEND_DEF(MEAInitialMass) FIELD_DEF(secondaryType)
                              FIELD_DEF(source));
    }
};

inline Canbus::CanMessage toCanMessage(const uint8_t& data)
{
    Canbus::CanMessage message;

    message.id         = -1;
    message.length     = 1;
    message.payload[0] = static_cast<uint8_t>(data);

    return message;
}

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

inline Canbus::CanMessage toCanMessage(const MeaData& data)
{
    Canbus::CanMessage message;

    message.id         = -1;
    message.length     = 1;
    message.payload[0] = floatToInt32(data.mass);

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
    message.length     = 2;
    message.payload[0] = (data.timestamp & ~0x3) << 30;
    message.payload[0] |= data.openingTime;
    message.payload[1] = static_cast<uint64_t>(data.servoId) << 56;

    return message;
}

inline Canbus::CanMessage toCanMessage(const ValveData& data)
{
    Canbus::CanMessage message;

    message.id         = -1;
    message.length     = 1;
    message.payload[0] = (data.timestamp & ~0x3) << 30;
    message.payload[0] |= static_cast<uint32_t>(data.idx) << 24;
    message.payload[0] |= data.position << 16;
    message.payload[0] |= (data.open ? 1 : 0);

    return message;
}

inline Canbus::CanMessage toCanMessage(const SequenceConfig& data)
{
    Canbus::CanMessage message;

    message.id         = -1;
    message.length     = 3;
    message.payload[0] = (data.timestamp & ~0x3) << 30;
    message.payload[0] |= data.fullThrottleTime;

    message.payload[1] = (static_cast<uint64_t>(data.lowThrottleTime) << 32);
    message.payload[1] |= data.pilotLeadTime;

    message.payload[2] =
        (static_cast<uint64_t>(floatToInt32(data.pilotOxPosition)) << 32);
    message.payload[2] |= floatToInt32(data.pilotFuelPosition);

    return message;
}

inline Canbus::CanMessage toCanMessage(const EregPIDSet& data)
{
    Canbus::CanMessage message;

    message.id     = -1;
    message.length = 4;
    message.payload[0] =
        static_cast<uint64_t>(floatToInt32(data.KpPressurization)) << 32;
    message.payload[0] |=
        static_cast<uint64_t>(floatToInt32(data.KiPressurization));

    message.payload[1] =
        static_cast<uint64_t>(floatToInt32(data.KdPressurization)) << 32;
    message.payload[1] |= static_cast<uint64_t>(floatToInt32(data.KpDischarge));

    message.payload[2] = static_cast<uint64_t>(floatToInt32(data.KiDischarge))
                         << 32;
    message.payload[2] |= static_cast<uint64_t>(floatToInt32(data.KdDischarge));

    message.payload[3] = data.eregId;

    return message;
}

inline Canbus::CanMessage toCanMessage(const EregTarget& data)
{
    Canbus::CanMessage message;

    message.id     = -1;
    message.length = 2;

    message.payload[0] = data.timestamp;
    message.payload[1] =
        (static_cast<uint64_t>(floatToInt32(data.oxTarget)) << 32);
    message.payload[1] |= floatToInt32(data.fuelTarget);

    return message;
}

inline Canbus::CanMessage toCanMessage(const IgnitionThresholds& data)
{
    Canbus::CanMessage message;

    message.id     = -1;
    message.length = 2;

    message.payload[0] = data.timestamp;
    message.payload[1] =
        (static_cast<uint64_t>(floatToInt32(data.igniterThreshold)) << 32);
    message.payload[1] |= floatToInt32(data.pilotThreshold);

    return message;
}

inline Canbus::CanMessage toCanMessage(const EregServoCoefficients& data)
{
    Canbus::CanMessage message;

    message.id     = -1;
    message.length = 4;

    message.payload[0] = data.eregId;
    message.payload[0] |=
        (static_cast<uint64_t>(floatToInt32(data.coefficients[0])) << 8);
    message.payload[1] = floatToInt32(data.coefficients[1]);
    message.payload[1] |=
        (static_cast<uint64_t>(floatToInt32(data.coefficients[2])) << 32);
    message.payload[2] = floatToInt32(data.coefficients[3]);
    message.payload[2] |=
        (static_cast<uint64_t>(floatToInt32(data.coefficients[4])) << 32);
    message.payload[3] = floatToInt32(data.coefficients[5]);

    return message;
}

inline Canbus::CanMessage toCanMessage(const MEAInitialMass& data) {

    Canbus::CanMessage message;

    message.id = -1;
    message.length = 1;

    message.payload[0] = floatToInt32(data.mass);

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

inline CanMeaData meaDataFromCanMessage(const Canbus::CanMessage& msg)
{
    CanMeaData data;

    data.secondaryType = msg.getSecondaryType();
    data.source        = msg.getSource();
    data.mass          = int32ToFloat(msg.payload[0]);

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
    data.servoId       = static_cast<uint8_t>(msg.payload[1] >> 56);
    data.secondaryType = msg.getSecondaryType();
    data.source        = msg.getSource();

    return data;
}

inline float meaMassFromCanMessage(const Canbus::CanMessage& msg)
{
    float data = static_cast<float>(msg.payload[0]);

    return data;
}

inline CanValveData valveDataFromCanMessage(const Canbus::CanMessage& msg)
{
    CanValveData data;

    data.timestamp     = (msg.payload[0] >> 30) & ~0x3;
    data.idx           = static_cast<uint8_t>(msg.payload[0] >> 24 & 0xFF);
    data.position      = static_cast<uint8_t>(msg.payload[0] >> 16 & 0xFF);
    data.open          = (msg.payload[0] & 1) != 0;
    data.secondaryType = msg.getSecondaryType();
    data.source        = msg.getSource();

    return data;
}

inline CanSequenceConfig sequenceConfigFromCanMessage(
    const Canbus::CanMessage& msg)
{
    CanSequenceConfig data;

    data.timestamp         = (msg.payload[0] >> 30) & ~0x3;
    data.fullThrottleTime  = static_cast<uint32_t>(msg.payload[0]);
    data.lowThrottleTime   = msg.payload[1] >> 32;
    data.pilotLeadTime     = static_cast<uint32_t>(msg.payload[1]);
    data.pilotOxPosition   = int32ToFloat(msg.payload[2] >> 32);
    data.pilotFuelPosition = int32ToFloat(msg.payload[2]);
    data.secondaryType     = msg.getSecondaryType();
    data.source            = msg.getSource();

    return data;
}

inline CanEregPIDSet eregPIDSetFromCanMessage(const Canbus::CanMessage& msg)
{
    CanEregPIDSet data;

    data.KpPressurization = int32ToFloat(msg.payload[0] >> 32);
    data.KiPressurization = int32ToFloat(msg.payload[0]);
    data.KdPressurization = int32ToFloat(msg.payload[1] >> 32);
    data.KpDischarge      = int32ToFloat(msg.payload[1]);
    data.KiDischarge      = int32ToFloat(msg.payload[2] >> 32);
    data.KdDischarge      = int32ToFloat(msg.payload[2]);
    data.eregId           = static_cast<uint8_t>(msg.payload[3]);
    data.secondaryType    = msg.getSecondaryType();
    data.source           = msg.getSource();

    return data;
}

inline CanEregTarget eregTargetFromCanMessage(const Canbus::CanMessage& msg)
{
    CanEregTarget data;

    data.timestamp     = msg.payload[0];
    data.oxTarget      = msg.payload[1] >> 32;
    data.fuelTarget    = msg.payload[1];
    data.secondaryType = msg.getSecondaryType();
    data.source        = msg.getSource();

    return data;
}

inline CanIgnitionThresholds ignitionThresholdsFromCanMessage(
    const Canbus::CanMessage& msg)
{
    CanIgnitionThresholds data;

    data.timestamp        = msg.payload[0];
    data.igniterThreshold = int32ToFloat(msg.payload[1] >> 32);
    data.pilotThreshold   = int32ToFloat(msg.payload[1]);
    data.secondaryType    = msg.getSecondaryType();
    data.source           = msg.getSource();

    return data;
}

inline CanEregServoCoefficients canServoCoefficientsFromCanMessage(
    const Canbus::CanMessage& msg)
{
    CanEregServoCoefficients data{};

    data.eregId          = static_cast<uint8_t>(msg.payload[0]);
    data.coefficients[0] = int32ToFloat(msg.payload[0] >> 8);
    data.coefficients[1] = int32ToFloat(msg.payload[1] >> 32);
    data.coefficients[2] = int32ToFloat(msg.payload[1]);
    data.coefficients[3] = int32ToFloat(msg.payload[2] >> 32);
    data.coefficients[4] = int32ToFloat(msg.payload[2]);
    data.coefficients[5] = int32ToFloat(msg.payload[3]);

    data.secondaryType = msg.getSecondaryType();
    data.source        = msg.getSource();

    return data;
}

inline CanMEAInitialMass CanMEAInitialMassFromCanMessage(const Canbus::CanMessage& msg) {
    CanMEAInitialMass data{};

    data.mass = int32ToFloat(static_cast<uint32_t>(msg.payload[0]));
    
    data.secondaryType = msg.getSecondaryType();
    data.source = msg.getSource();

    return data;
}

}  // namespace Boardcore
