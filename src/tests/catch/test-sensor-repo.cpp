/* Copyright (c) 2020 Skyward Experimental Rocketry
 * Authors: Luca Conterio
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rightd
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.  IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 */

#ifdef STANDALONE_CATCH1_TEST
#include "catch/catch-tests-entry.cpp"
#endif

#include "sensors/SensorDataRepository.h"

class SensorRepoFixture
{
public:
    SensorRepoFixture() {}

    ~SensorRepoFixture() {}

    SensorDataRepository sensor_repo;
};

TEST_CASE_METHOD(SensorRepoFixture,
                 "Data is correctly written and read to/from the sensor repo")
{
    SECTION("Sensor data")
    {
        SensorData data;
        data.timestamp = getTick();

        sensor_repo.addSample(data);

        SensorData data2;
        data2 = sensor_repo.readSample<SensorData>(SensorID::BASE);

        REQUIRE(data.timestamp == data2.timestamp);
        REQUIRE(data.id == data2.id);
        REQUIRE(data.id == SensorID::BASE);
    }

    SECTION("Temperature data")
    {
        TemperatureData data;
        data.timestamp   = getTick();
        data.temperature = 25.5;

        sensor_repo.addSample(data);

        TemperatureData data2;
        data2 = sensor_repo.readSample<TemperatureData>(SensorID::TEMPERATURE);

        REQUIRE(data.timestamp == data2.timestamp);
        REQUIRE(data.id == data2.id);
        REQUIRE(data.id == SensorID::TEMPERATURE);
        REQUIRE(data.temperature == data2.temperature);
    }

    SECTION("Barometer data")
    {
        BarometerData data;
        data.timestamp = getTick();
        data.pressure  = 20551.32;

        sensor_repo.addSample(data);

        BarometerData data2;
        data2 = sensor_repo.readSample<BarometerData>(SensorID::BAROMETER);

        REQUIRE(data.timestamp == data2.timestamp);
        REQUIRE(data.id == data2.id);
        REQUIRE(data.id == SensorID::BAROMETER);
        REQUIRE(data.pressure == data2.pressure);
    }

    SECTION("Motion Sensor data")
    {
        AccelerometerData data;
        data.timestamp = getTick();
        data.vector.setX(0.34);
        data.vector.setY(0.35);
        data.vector.setZ(0.31);

        sensor_repo.addSample(data);

        AccelerometerData data2;
        data2 =
            sensor_repo.readSample<AccelerometerData>(SensorID::ACCELEROMETER);

        REQUIRE(data.timestamp == data2.timestamp);
        REQUIRE(data.id == data2.id);
        REQUIRE(data.id == SensorID::ACCELEROMETER);
        REQUIRE(data.vector.getX() == data2.vector.getX());
        REQUIRE(data.vector.getY() == data2.vector.getY());
        REQUIRE(data.vector.getZ() == data2.vector.getZ());
    }

    SECTION("IMU 9 dof data")
    {
        IMU9DofData data;
        data.timestamp = getTick();
        data.accel.vector.setX(0.1);
        data.accel.vector.setY(0.2);
        data.accel.vector.setZ(0.3);
        data.gyro.vector.setX(0.4);
        data.gyro.vector.setY(0.5);
        data.gyro.vector.setZ(0.6);
        data.magneto.vector.setX(0.7);
        data.magneto.vector.setY(0.8);
        data.magneto.vector.setZ(0.9);

        sensor_repo.addSample(data);

        IMU9DofData data2;
        data2 = sensor_repo.readSample<IMU9DofData>(SensorID::IMU_9DOF);

        REQUIRE(data.timestamp == data2.timestamp);
        REQUIRE(data.id == data2.id);
        REQUIRE(data.id == SensorID::IMU_9DOF);
        REQUIRE(data.accel.vector.getX() == data2.accel.vector.getX());
        REQUIRE(data.accel.vector.getY() == data2.accel.vector.getY());
        REQUIRE(data.accel.vector.getZ() == data2.accel.vector.getZ());
        REQUIRE(data.gyro.vector.getX() == data2.gyro.vector.getX());
        REQUIRE(data.gyro.vector.getY() == data2.gyro.vector.getY());
        REQUIRE(data.gyro.vector.getZ() == data2.gyro.vector.getZ());
        REQUIRE(data.magneto.vector.getX() == data2.magneto.vector.getX());
        REQUIRE(data.magneto.vector.getY() == data2.magneto.vector.getY());
        REQUIRE(data.magneto.vector.getZ() == data2.magneto.vector.getZ());
    }

    SECTION("GPS data")
    {
        GPSData data;
        data.timestamp      = getTick();
        data.latitude       = 21.5;
        data.longitude      = 42.3;
        data.height         = 1456;
        data.velocity_north = 321.6;
        data.velocity_east  = 123.4;
        data.velocity_down  = 50.1;
        data.speed          = 244.4;
        data.num_satellites = 7;
        data.fix            = true;

        sensor_repo.addSample(data);

        GPSData data2;
        data2 = sensor_repo.readSample<GPSData>(SensorID::GPS);

        REQUIRE(data.timestamp == data2.timestamp);
        REQUIRE(data.id == data2.id);
        REQUIRE(data.id == SensorID::GPS);
        REQUIRE(data.latitude == data2.latitude);
        REQUIRE(data.longitude == data2.longitude);
        REQUIRE(data.height == data2.height);
        REQUIRE(data.velocity_north == data2.velocity_north);
        REQUIRE(data.velocity_east == data2.velocity_east);
        REQUIRE(data.velocity_down == data2.velocity_down);
        REQUIRE(data.speed == data2.speed);
        REQUIRE(data.num_satellites == data2.num_satellites);
        REQUIRE(data.fix == data2.fix);
    }
}

TEST_CASE("Sensor data types must be trivially copiable in order to be logged")
{
    REQUIRE(std::is_trivially_copyable<SensorData>::value);
    REQUIRE(std::is_trivially_copyable<TemperatureData>::value);
    REQUIRE(std::is_trivially_copyable<BarometerData>::value);
    REQUIRE(std::is_trivially_copyable<MotionSensorData>::value);
    REQUIRE(std::is_trivially_copyable<AccelerometerData>::value);
    REQUIRE(std::is_trivially_copyable<GyroscopeData>::value);
    REQUIRE(std::is_trivially_copyable<MagnetometerData>::value);
    REQUIRE(std::is_trivially_copyable<IMU9DofData>::value);
    REQUIRE(std::is_trivially_copyable<GPSData>::value);
}