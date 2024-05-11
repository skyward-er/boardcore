/* Copyright (c) 2021-2023 Skyward Experimental Rocketry
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

#include <sensors/SensorData.h>
#include <sensors/analog/Pitot/PitotData.h>

struct HILAccelerometerData : public Boardcore::AccelerometerData
{
    static std::string header()
    {
        return "timestamp,accelerationX,accelerationY,accelerationZ\n";
    }

    void print(std::ostream& os) const
    {
        os << accelerationTimestamp << "," << accelerationX << ","
           << accelerationY << "," << accelerationZ << "\n";
    }
};

struct HILGyroscopeData : public Boardcore::GyroscopeData
{
    static std::string header()
    {
        return "timestamp,angularSpeedX,angularSpeedY,angularSpeedZ\n";
    }

    void print(std::ostream& os) const
    {
        os << angularSpeedTimestamp << "," << angularSpeedX << ","
           << angularSpeedY << "," << angularSpeedZ << "\n";
    }
};

struct HILMagnetometerData : public Boardcore::MagnetometerData
{
    static std::string header()
    {
        return "timestamp,magneticFieldX,magneticFieldY,magneticFieldZ\n";
    }

    void print(std::ostream& os) const
    {
        os << magneticFieldTimestamp << "," << magneticFieldX << ","
           << magneticFieldY << "," << magneticFieldZ << "\n";
    }
};

struct HILImuData : public HILAccelerometerData,
                    public HILGyroscopeData,
                    public HILMagnetometerData
{
    static std::string header()
    {
        return "accelerationTimestamp,accelerationX,accelerationY,"
               "accelerationZ,angularSpeedTimestamp,angularSpeedX,"
               "angularSpeedY,angularSpeedZ,magneticFieldTimestamp,"
               "magneticFieldX,magneticFieldY,magneticFieldZ\n";
    }

    void print(std::ostream& os) const
    {
        os << accelerationTimestamp << "," << accelerationX << ","
           << accelerationY << "," << accelerationZ << ","
           << angularSpeedTimestamp << "," << angularSpeedX << ","
           << angularSpeedY << "," << angularSpeedZ << ","
           << magneticFieldTimestamp << "," << magneticFieldX << ","
           << magneticFieldY << "," << magneticFieldZ << "\n";
    }
};

struct HILGpsData : public Boardcore::GPSData
{
    static std::string header()
    {
        return "timestamp,latitude,longitude,height,velocityNorth,velocityEast,"
               "velocityDown,speed,track,positionDOP,satellites,fix\n";
    }

    void print(std::ostream& os) const
    {
        os << gpsTimestamp << "," << longitude << "," << latitude << ","
           << height << "," << velocityNorth << "," << velocityEast << ","
           << velocityDown << "," << speed << "," << track << "," << positionDOP
           << "," << (int)satellites << "," << (int)fix << "\n";
    }
};

struct HILBarometerData : public Boardcore::PressureData
{
    static std::string header() { return "timestamp,pressure\n"; }

    void print(std::ostream& os) const
    {
        os << pressureTimestamp << "," << pressure << "\n";
    }
};

struct HILPitotData : public Boardcore::PitotData
{
    static std::string header() { return "timestamp,deltaP,airspeed\n"; }

    void print(std::ostream& os) const
    {
        os << timestamp << "," << deltaP << "," << airspeed << "\n";
    }
};

struct HILTempData : public Boardcore::TemperatureData
{
    static std::string header() { return "timestamp,temperature\n"; }

    void print(std::ostream& os) const
    {
        os << temperatureTimestamp << "," << temperature << "\n";
    }
};
