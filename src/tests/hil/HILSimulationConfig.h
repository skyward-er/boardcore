/* Copyright (c) 2020-2024 Skyward Experimental Rocketry
 * Author: Emilio Corigliano
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

#include <drivers/timer/TimestampTimer.h>
#include <drivers/usart/USART.h>
#include <utils/Debug.h>
#include <utils/Stats/Stats.h>

#include <list>
#include <utils/ModuleManager/ModuleManager.hpp>

#include "algorithms/ADA/ADAData.h"
#include "algorithms/NAS/NAS.h"
#include "algorithms/NAS/NASState.h"
#include "sensors/SensorInfo.h"

namespace HILConfig
{

struct SensorConfig : public Boardcore::SensorInfo
{
    SensorConfig(const std::string s, const uint32_t period)
        : Boardcore::SensorInfo{s, period, []() {}, true}
    {
    }
};

/** baudrate of connection */
const int SIM_BAUDRATE = 115200;

/** Period of simulation in milliseconds */
const int SIMULATION_PERIOD = 100;

/** sample frequency of sensor (samples/second) */
const int ACCEL_FREQ = 100;
const int GYRO_FREQ  = 100;
const int MAGN_FREQ  = 100;
const int IMU_FREQ   = 100;
const int BARO_FREQ  = 20;
const int PITOT_FREQ = 20;
const int TEMP_FREQ  = 10;
const int GPS_FREQ   = 10;

// /** update frequency of airbrakes control algorithm */
// const int CONTROL_FREQ = 10;

// /** min and max values in radiants of the actuator */
// const float MinAlphaDegree = 0.0;
// const float MaxAlphaDegree = 0.84;

/** sensors configuration */
const SensorConfig accelConfig("accel", ACCEL_FREQ);
const SensorConfig gyroConfig("gyro", GYRO_FREQ);
const SensorConfig magnConfig("magn", MAGN_FREQ);
const SensorConfig imuConfig("imu", IMU_FREQ);
const SensorConfig baroConfig("baro", BARO_FREQ);
const SensorConfig pitotConfig("pitot", PITOT_FREQ);
const SensorConfig gpsConfig("gps", GPS_FREQ);
const SensorConfig tempConfig("temp", TEMP_FREQ);

/** Number of samples per sensor at each simulator iteration */
const int N_DATA_ACCEL = (ACCEL_FREQ * SIMULATION_PERIOD) / 1000;  // 10
const int N_DATA_GYRO  = (GYRO_FREQ * SIMULATION_PERIOD) / 1000;   // 10
const int N_DATA_MAGN  = (MAGN_FREQ * SIMULATION_PERIOD) / 1000;   // 10
const int N_DATA_IMU   = (IMU_FREQ * SIMULATION_PERIOD) / 1000;    // 10
const int N_DATA_BARO  = (BARO_FREQ * SIMULATION_PERIOD) / 1000;   // 2
const int N_DATA_PITOT = (PITOT_FREQ * SIMULATION_PERIOD) / 1000;  // 2
const int N_DATA_GPS   = (GPS_FREQ * SIMULATION_PERIOD) / 1000;    // 1
const int N_DATA_TEMP  = (TEMP_FREQ * SIMULATION_PERIOD) / 1000;   // 1

/**
 * @brief Data structure used by the simulator in order to directly deserialize
 * the data received
 *
 * This structure then is accessed by sensors and other components in order to
 * get the data they need
 */
struct SimulatorData
{
    struct Accelerometer
    {
        float measures[N_DATA_ACCEL][3];
    } accelerometer;

    struct Gyro
    {
        float measures[N_DATA_GYRO][3];
    } gyro;

    struct Magnetometer
    {
        float measures[N_DATA_MAGN][3];
    } magnetometer;

    struct Gps
    {
        float positionMeasures[N_DATA_GPS][3];
        float velocityMeasures[N_DATA_GPS][3];
        float fix;
        float num_satellites;
    } gps;

    struct Barometer
    {
        float measures[N_DATA_BARO];
    } barometer1, barometer2, barometer3;

    struct Pitot
    {
        float deltaP[N_DATA_PITOT];
        float staticPressure[N_DATA_PITOT];
    } pitot;

    struct Temperature
    {
        float measure;
    } temperature;

    struct Flags
    {
        float flag_flight;
        float flag_ascent;
        float flag_burning;
        float flag_airbrakes;
        float flag_para1;
        float flag_para2;
    } flags;

    void printAccelerometer()
    {
        TRACE("accel\n");
        for (int i = 0; i < N_DATA_ACCEL; i++)
            TRACE("%+.3f\t%+.3f\t%+.3f\n", accelerometer.measures[i][0],
                  accelerometer.measures[i][1], accelerometer.measures[i][2]);
    }

    void printGyro()
    {
        TRACE("gyro\n");
        for (int i = 0; i < N_DATA_GYRO; i++)
            TRACE("%+.3f\t%+.3f\t%+.3f\n", gyro.measures[i][0],
                  gyro.measures[i][1], gyro.measures[i][2]);
    }

    void printMagnetometer()
    {
        TRACE("magneto\n");
        for (int i = 0; i < N_DATA_MAGN; i++)
            TRACE("%+.3f\t%+.3f\t%+.3f\n", magnetometer.measures[i][0],
                  magnetometer.measures[i][1], magnetometer.measures[i][2]);
    }

    void printGPS()
    {
        TRACE("gps\n");
        TRACE("pos\n");
        for (int i = 0; i < N_DATA_GPS; i++)
            TRACE("%+.3f\t%+.3f\t%+.3f\n", gps.positionMeasures[i][0],
                  gps.positionMeasures[i][1], gps.positionMeasures[i][2]);

        TRACE("vel\n");
        for (int i = 0; i < N_DATA_GPS; i++)
            TRACE("%+.3f\t%+.3f\t%+.3f\n", gps.velocityMeasures[i][0],
                  gps.velocityMeasures[i][1], gps.velocityMeasures[i][2]);
        TRACE("fix:%+.3f\tnsat:%+.3f\n", gps.fix, gps.num_satellites);
    }

    void printBarometer1()
    {
        TRACE("press1\n");
        for (int i = 0; i < N_DATA_BARO; i++)
            TRACE("%+.3f\n", barometer1.measures[i]);
    }

    void printBarometer2()
    {
        TRACE("press2\n");
        for (int i = 0; i < N_DATA_BARO; i++)
            TRACE("%+.3f\n", barometer2.measures[i]);
    }

    void printBarometer3()
    {
        TRACE("press3\n");
        for (int i = 0; i < N_DATA_BARO; i++)
            TRACE("%+.3f\n", barometer3.measures[i]);
    }

    void printPitot()
    {
        TRACE("pitot\n");
        for (int i = 0; i < N_DATA_PITOT; i++)
            TRACE("%+.3f, \n", pitot.staticPressure[i], pitot.deltaP[i]);
    }

    void printTemperature()
    {
        TRACE("temp\n");
        for (int i = 0; i < N_DATA_TEMP; i++)
            TRACE("%+.3f\n", temperature.measure);
    }

    void printFlags()
    {
        TRACE("flags\n");
        TRACE(
            "flight:\t%+.3f\n"
            "ascent:\t%+.3f\n"
            "burning:\t%+.3f\n"
            "airbrakes:\t%+.3f\n"
            "para1:\t%+.3f\n"
            "para2:\t%+.3f\n",
            flags.flag_flight, flags.flag_ascent, flags.flag_burning,
            flags.flag_airbrakes, flags.flag_para1, flags.flag_para2);
    }

    void print()
    {
        printAccelerometer();
        printGyro();
        printMagnetometer();
        printGPS();
        printBarometer1();
        printBarometer2();
        printBarometer3();
        printPitot();
        printTemperature();
        printFlags();
    }
};

/**
 * @brief ADA data sent to the simulator
 */
struct ADAdataHIL
{
    uint64_t ada_timestamp;
    float aglAltitude;    // Altitude at mean sea level [m].
    float verticalSpeed;  // Vertical speed [m/s].

    ADAdataHIL& operator+=(const ADAdataHIL& x)
    {
        this->ada_timestamp += x.ada_timestamp;
        this->aglAltitude += x.aglAltitude;
        this->verticalSpeed += x.verticalSpeed;

        return *this;  // return the result by reference
    }

    ADAdataHIL operator/(int x)
    {
        return ADAdataHIL{this->ada_timestamp / x, this->aglAltitude / x,
                          this->verticalSpeed / x};
    }

    ADAdataHIL operator*(int x)
    {
        return ADAdataHIL{this->ada_timestamp * x, this->aglAltitude * x,
                          this->verticalSpeed * x};
    }

    static std::string header()
    {
        return "timestamp,aglAltitude,verticalSpeed\n";
    }

    void print(std::ostream& os) const
    {
        os << ada_timestamp << "," << aglAltitude << "," << verticalSpeed
           << "\n";
    }
};

/**
 * @brief Data structure expected by the simulator
 */
struct ActuatorData
{
    Boardcore::NASState nasState;  ///< NAS
    ADAdataHIL adaState;           ///< ADA
    float airbrakes_opening;       ///< Airbrakes opening (percentage)
    float estimated_mass;          ///< Estimated mass of the rocket
    float liftoff;                 ///< Flag for liftoff
    float burning_shutdown;        ///< Flag for engine shutdown

    void print() const
    {
        TRACE(
            "size:%u, %u, %u\n"
            "abk:%f\n"
            "tsnas:%f\n"
            "ned:%f,%f,%f\n"
            "vned:%f,%f,%f\n"
            "q:%f,%f,%f,%f\n"
            "bias:%f,%f,%f\n"
            "tsada:%f\n"
            "ada:%f,%f\n\n",
            sizeof(airbrakes_opening), sizeof(Boardcore::NASState),
            sizeof(ADAdataHIL), airbrakes_opening, nasState.timestamp,
            nasState.n, nasState.e, nasState.d, nasState.vn, nasState.ve,
            nasState.vd, nasState.qx, nasState.qy, nasState.qz, nasState.qw,
            nasState.bx, nasState.by, nasState.bz, adaState.ada_timestamp,
            adaState.aglAltitude, adaState.verticalSpeed);
    }
};

}  // namespace HILConfig
