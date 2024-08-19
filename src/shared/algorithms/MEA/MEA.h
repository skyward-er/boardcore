/* Copyright (c) 2024 Skyward Experimental Rocketry
 * Author: Davide Mor
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

#include <algorithms/MEA/MEAData.h>
#include <sensors/SensorData.h>
#include <utils/AeroUtils/AeroUtils.h>

#include <Eigen/Dense>

namespace Boardcore
{

class MEA
{
public:
    struct Config
    {
        Eigen::Matrix<float, 3, 3> F;  //< State propagation matrix
        Eigen::Matrix<float, 3, 3> Q;  //< Model variance matrix
        Eigen::Vector<float, 3> G;     //< Input vector

        Eigen::Vector<float, 3> baroH;  //< Barometer output matrix
        float baroR;                    //< Barometer measurement variance

        Eigen::Matrix<float, 3, 3> P;  //< Error covariance matrix

        float initialMass;  //< Initial mass of the rocket

        float accelThresh;  //< Minimum required acceleration to trigger accel
                            // correction.
        float speedThresh;  //< Minumum required speed to trigger accel
                            // correction.

        float Kt;     //< TODO: What is this?
        float alpha;  //< TODO: What is this?
        float c;      //< TODO: What is this?

        Aeroutils::AerodynamicCoeff coeffs;  //< Aerodynamic coefficients.
        float crossSection;                  //< Cross section of the rocket.

        float ae;  //< Efflux area
        float p0;  //< Pressure at nozzle exit

        float minMass;  //< Minimum mass used for predicted apogee
        float maxMass;  //< Maximum mass used for predicted apogee
    };

    struct Step
    {
        float mainValveOpen =
            0.0f;  //< How open is the main valve? (between 0-1)

        float ccPressure =
            0.0f;  //< What is the current combustion chamber pressure? [bar]
        bool hasCCPressure = false;  //< Is ccPressure valid?

        Eigen::Vector<float, 3> acceleration =
            Eigen::Vector<float, 3>::Zero();  //< Acceleration [m/s^2]
        bool hasAcceleration = false;         //< Is acceleration valid?

        float verticalSpeed =
            0.0f;  //< Vertical component of speed [up positive] [m/s]
        float mslAltitude = 0.0f;  //< Mean sea level altitude [m]
        bool hasSpeedAndAlt =
            false;  //< Are verticalSpeed and mslAltitude valid?

        explicit Step(float mainValveOpen);

        void withCCPressure(PressureData ccPressure);
        void withCCPressure(float ccPressure);

        void withAcceleration(AccelerometerData acceleration);
        void withAcceleration(Eigen::Vector<float, 3> acceleration);

        void withSpeedAndAlt(float verticalSpeed, float mslAltitude);
    };

    explicit MEA(const Config &config);

    void update(const Step &step);

    MEAState getState();

private:
    void predict(const Step &step);
    void computeForce(const Step &step);
    void correctBaro(const Step &step);
    void correctAccel(const Step &step);
    void computeMass();
    void computeApogee(const Step &step);
    void updateState();

    Eigen::Matrix<float, 3, 3> F;  //< State propagation matrix
    Eigen::Matrix<float, 3, 3> Q;  //< Model variance matrix

    Eigen::Vector<float, 3> G;  //< Input Vector

    Eigen::Matrix<float, 1, 3> baroH;
    float baroR;

    Eigen::Matrix<float, 3, 3> P;  //< Error covariance matrix
    Eigen::Vector<float, 3> x;     //< State vector

    float mach  = 0.0f;  //< Latest computed mach
    float cd    = 0.0f;  //< Latest computed CD
    float rho   = 0.0f;  //< Latest computed rho
    float q     = 0.0f;  //< Latest computed dynamic pressure
    float force = 0.0f;  //< Latest computed force

    float mass   = 0.0f;  //< Latest computed mass
    float apogee = 0.0f;  //< Latest computed apogee

    float accelThresh;
    float speedThresh;

    float Kt;
    float alpha;
    float c;

    Aeroutils::AerodynamicCoeff coeffs;
    float crossSection;

    float ae;
    float p0;

    float minMass;
    float maxMass;

    MEAState state;
};

}  // namespace Boardcore