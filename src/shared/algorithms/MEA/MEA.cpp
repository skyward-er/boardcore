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

#include "MEA.h"

#include <drivers/timer/TimestampTimer.h>

using namespace Boardcore;
using namespace Eigen;

MEA::Step::Step(float mainValveOpen) : mainValveOpen{mainValveOpen} {}

void MEA::Step::withCCPressure(PressureData ccPressure)
{
    withCCPressure(ccPressure.pressure);
}

void MEA::Step::withCCPressure(float ccPressure)
{
    this->ccPressure = ccPressure;
    hasCCPressure    = true;
}

void MEA::Step::withAcceleration(AccelerometerData acceleration)
{
    withAcceleration(acceleration);
}

void MEA::Step::withAcceleration(Eigen::Vector<float, 3> acceleration)
{
    this->acceleration = acceleration;
    hasAcceleration    = true;
}

void MEA::Step::withSpeedAndAlt(float verticalSpeed, float mslAltitude)
{
    this->verticalSpeed = verticalSpeed;
    this->mslAltitude   = mslAltitude;
    hasSpeedAndAlt      = true;
}

MEA::MEA(const Config &config)
    : F{config.F}, Q{config.Q}, G{config.G}, baroH{config.baroH},
      baroR{config.baroR}, P{config.P}, x{0, 0, config.initialMass},
      accelThresh{config.accelThresh}, speedThresh{config.speedThresh},
      Kt{config.Kt}, alpha{config.alpha}, c{config.c}, coeffs{config.coeffs},
      crossSection{config.crossSection}, ae{config.ae}, p0{config.p0},
      minMass{config.minMass}, maxMass{config.maxMass}
{
}

void MEA::update(const Step &step)
{
    // First run the prediction step
    predict(step);

    // Compute applied force/mach/CD/rho
    computeForce(step);

    // Run cc pressure correction
    correctBaro(step);

    // Run accelerometer correction
    correctAccel(step);

    // Compute current mass
    computeMass();

    // Run apogee prediction
    computeApogee(step);

    // Finally update state
    updateState();
}

MEAState MEA::getState() { return state; }

void MEA::predict(const Step &step)
{
    x = F * x + G * step.mainValveOpen;
    P = F * P * F.transpose() + Q;
}

void MEA::computeForce(const Step &step)
{
    if (!step.hasSpeedAndAlt)
        return;

    // NOTE: Here we assume that verticalSpeed roughly equals the total speed,
    // so that we don't depend on N/E speed components, which can be quite
    // unreliable in case of no GPS fix
    mach = Aeroutils::computeMach(-step.mslAltitude, step.verticalSpeed,
                                  Constants::MSL_TEMPERATURE);

    cd  = Aeroutils::computeCd(coeffs, mach);
    rho = Aeroutils::computeRho(-step.mslAltitude, Constants::MSL_TEMPERATURE);

    // Dynamic pressure
    q = 0.5f * rho * (step.verticalSpeed * step.verticalSpeed);

    // Aerodynamic force component
    force = q * crossSection * cd;

    if (step.mslAltitude > 800)
    {
        // At high altitudes we need to compensate for low external pressure

        // External pressure
        float p = Aeroutils::relPressure(step.mslAltitude);
        force -= (p0 - p) * ae;
    }
}

void MEA::correctBaro(const Step &step)
{
    if (!step.hasCCPressure)
        return;

    float S               = baroH * P * baroH.transpose() + baroR;
    Matrix<float, 3, 1> K = (P * baroH.transpose()) / S;

    P = (Matrix<float, 3, 3>::Identity() - K * baroH) * P;
    x = x + K * (step.ccPressure - baroH * x);
}

void MEA::correctAccel(const Step &step)
{
    if (!step.hasAcceleration || !step.hasSpeedAndAlt)
        return;

    // Do not correct at low speed/acceleration
    if (step.acceleration.norm() < accelThresh ||
        step.verticalSpeed < speedThresh)
        return;

    float y = (Kt * (float)(baroH * x) - force) / x(2);

    Matrix<float, 1, 3> accelH =
        Vector<float, 3>{Kt * baroH(0) / x(2), Kt * baroH(1) / x(2),
                         -(Kt * (float)(baroH * x) - force) / (x(2) * x(2))}
            .transpose();

    float accelR = alpha * q + c;

    float S               = accelH * P * accelH.transpose() + accelR;
    Matrix<float, 3, 1> K = (P * accelH.transpose()) / S;

    P = (Matrix<float, 3, 3>::Identity() - K * accelH) * P;
    x = x + K * (step.acceleration.x() - y);
}

void MEA::computeMass()
{
    // Clamp the used mass, so that we don't use bogus values in case the filter
    // fails BAD
    mass = std::max(std::min(x(2), maxMass), minMass);
}

void MEA::computeApogee(const Step &step)
{
    if (!step.hasSpeedAndAlt)
        return;

    // TODO: Matlab uses CD_correction_shutDown, should we?
    // Holy fuck, massive formula, split this for the love of god
    apogee = step.mslAltitude +
             1 / (2 * ((0.5f * rho * cd * crossSection) / mass)) *
                 log1p(1 + (step.verticalSpeed * step.verticalSpeed *
                            ((0.5f * rho * cd * crossSection) / mass)) /
                               Constants::g);
}

void MEA::updateState()
{
    state.timestamp = TimestampTimer::getTimestamp();

    state.x0 = x(0);
    state.x1 = x(1);
    state.x2 = x(2);

    state.estimatedMass     = mass;
    state.estimatedPressure = baroH * x;
    state.estimatedApogee   = apogee;
    state.estimatedForce    = force;
}