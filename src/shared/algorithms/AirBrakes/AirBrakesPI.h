/* Copyright (c) 2021-2022 Skyward Experimental Rocketry
 * Authors: Vincenzo Santomarco, Alberto Nidasio, Emilio Corigliano
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

#include <actuators/Servo/Servo.h>
#include <algorithms/Algorithm.h>
#include <algorithms/PIController.h>

#include <functional>
#include <vector>

#include "AirBrakesPIConfig.h"
#include "TrajectorySet.h"

namespace Boardcore
{

class AirBrakesPI : public Algorithm
{
public:
    AirBrakesPI(std::function<TimedTrajectoryPoint()> getCurrentPosition,
                const TrajectorySet &trajectorySet,
                const AirBrakesPIConfig &config,
                std::function<void(float)> setActuator);

    bool init() override;

    /**
     * @brief This method chooses the trajectory the rocket will follow and
     * starts the algorithm.
     */
    void begin();

    /**
     * @brief Looks for nearest point in the current chosen trajectory and moves
     * the airbraks according to the current rocket speed and the prediction.
     */
    void step() override;

private:
    /**
     * @brief Returns the air density at the current altitude using the basic
     * atmosphere model.
     *
     * @param z The current altitude [m].
     * @return The density of air according to current altitude [Kg/m^3]
     */
    float getRho(float z);

    /**
     * @brief Compute the necessary airbrakes surface to match the
     * given drag force from the Pid as closely as possible.
     *
     * @param currentPosition Current rocket position.
     * @param rho Air density [kg/m^2].
     * @param drag Target drag force.
     * @return AirBrakes surface [m];
     */
    float getSurface(const TimedTrajectoryPoint &currentPosition, float rho,
                     float drag);

    /**
     * @brief Computes the airbrakes extension given the desired area.
     *
     * @param surface Desired airbrakes surface [m^2].
     * @return The radial extension [m].
     */
    float getExtension(float surface);

    /**
     * @brief Returns the coefficient of drag for the airbrakes at the given
     * position and with the given extension.
     *
     * @param currentPosition Current rocket position.
     * @param extension AirBrakes extension [m].
     * @return The coefficient drag [1].
     */
    float getCD(TimedTrajectoryPoint currentPosition, float extension);

    /**
     * @brief Return the drag generated from the AirBrakes in the given
     * conditions.
     *
     * @param currentPosition Current position of the rocket.
     * @param cd Coefficient of drag [1].
     * @param rho Air density [kg/m^2].
     * @return Generated drag [N].
     */
    float getDrag(TimedTrajectoryPoint currentPosition, float cd, float rho);

    /**
     * @brief Searched all the trajectories and find the neares point to the
     * given position. The trajectory of this point is the one choosen.
     */
    void chooseTrajectory(TrajectoryPoint currentPosition);

    /**
     * @brief Searches, in the choosen trajectory, the point neares to the given
     * one. This method considers the Euclidean distance between altitude and
     * vertical speed.
     */
    TrajectoryPoint getSetpoint(TrajectoryPoint currentPosition);

    /**
     * @brief Update PI to compute the target drag froce.
     *
     * @returns Target drag force to generate [N].
     */
    float piStep(TimedTrajectoryPoint currentPosition,
                 TrajectoryPoint reference, float rho);

private:
    std::function<TimedTrajectoryPoint()> getCurrentPosition;
    const AirBrakesPIConfig &config;
    std::function<void(float)> setActuator;

    TimedTrajectoryPoint lastPosition;
    uint32_t lastSelectedPointIndex = 0;

    PIController pi;

    // Trajectory set from which the used trajectory can be choosen
    const TrajectorySet &trajectorySet;

    Trajectory *chosenTrajectory = nullptr;
};

}  // namespace Boardcore
