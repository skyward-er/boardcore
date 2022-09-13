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

#include "AirBrakes.h"
#include "AirBrakesData.h"
#include "AirBrakesPIConfig.h"
#include "TrajectorySet.h"

namespace Boardcore
{

class AirBrakesPI : public AirBrakes
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
    const AirBrakesPIConfig &config;  ///< specialized config for PI
    PIController pi;

    Trajectory *chosenTrajectory = nullptr;
};

}  // namespace Boardcore
