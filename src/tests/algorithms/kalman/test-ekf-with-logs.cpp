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

#include <algorithms/ExtendedKalman/ExtendedKalman.h>
#include <algorithms/ExtendedKalman/StateInitializer.h>
#include <sensors/BMX160/BMX160.h>
#include <utils/CSVReader/CSVReader.h>
#include <utils/SkyQuaternion/SkyQuaternion.h>

#include <iostream>

using namespace miosix;
using namespace Boardcore;
using namespace Eigen;

ExtendedKalmanConfig getEKConfig();
void setInitialOrientation();
void updateKalman(BMX160Data data);
void printState();
std::istream& operator>>(std::istream& input, BMX160Data& data);

auto milanMag = Vector3f(0.4742f, 0.025f, 0.8801f);  // Already normalized

ExtendedKalman* kalman;
bool triad = false;

int main()
{
    // Prepare the Kalman
    kalman = new ExtendedKalman(getEKConfig());
    StateInitializer initStates;
    setInitialOrientation();

    // Retrieve all the data
    auto logData = CSVParser<BMX160Data>("/sd/bmx160log2.csv").collect();
    printf("Log size: %d\n", logData.size());

    // Wait to start
    std::cin.get();

    // Loop thrpugh the logs
    for (size_t index = 0; index < logData.size(); index++)
    {
        printf("Iteration %lu\n", (unsigned long)index);
        auto iterationData = logData.at(index);

        if (triad)
        {
            triad = false;

            Vector3f acceleration(iterationData.accelerationX,
                                  iterationData.accelerationY,
                                  iterationData.accelerationZ);
            Vector3f magneticField(iterationData.magneticFieldX,
                                   iterationData.magneticFieldY,
                                   iterationData.magneticFieldZ);

            initStates.triad(acceleration, magneticField, milanMag);

            std::cout << "Triad output" << std::endl;
            std::cout << initStates.getInitX().transpose() << std::endl;
        }
        else
        {
            // Predict and correct
            updateKalman(iterationData);

            // Print results
            printState();
        }

        // Wait to continue
        // std::cin.get();
    }
}

ExtendedKalmanConfig getEKConfig()
{
    ExtendedKalmanConfig config;

    config.T              = 0.02f;
    config.SIGMA_BETA     = 0.0001f;
    config.SIGMA_W        = 0.3f;
    config.SIGMA_MAG      = 0.05f;
    config.SIGMA_GPS      = 10.0f;
    config.SIGMA_BAR      = 4.3f;
    config.SIGMA_POS      = 10.0;
    config.SIGMA_VEL      = 10.0;
    config.P_POS          = 1.0f;
    config.P_POS_VERTICAL = 10.0f;
    config.P_VEL          = 1.0f;
    config.P_VEL_VERTICAL = 10.0f;
    config.P_ATT          = 0.01f;
    config.P_BIAS         = 0.01f;
    config.SATS_NUM       = 6.0f;

    // Normalized magnetic field in Milan
    config.NED_MAG = milanMag;

    return config;
}

void setInitialOrientation()
{
    Eigen::Matrix<float, 13, 1> x = Eigen::Matrix<float, 13, 1>::Zero();

    // Set quaternions
    Eigen::Vector4f q = SkyQuaternion::eul2quat({0, 0, 0});
    x(6)              = q(0);
    x(7)              = q(1);
    x(8)              = q(2);
    x(9)              = q(3);

    std::cout << "Initial state" << std::endl;
    std::cout << x.transpose() << std::endl;
    kalman->setX(x);
}

void updateKalman(BMX160Data data)
{
    // Vector3f acceleration(data.accelerationX, data.accelerationY,
    //                       data.accelerationZ);
    // kalman->predict(acceleration);

    Vector3f angularVelocity(data.angularVelocityX, data.angularVelocityY,
                             data.angularVelocityZ);
    std::cout << "Angular velocity" << std::endl;
    std::cout << angularVelocity.transpose() << std::endl;
    kalman->predictMEKF(angularVelocity);

    Vector3f magneticField(data.magneticFieldX, data.magneticFieldY,
                           data.magneticFieldZ);
    magneticField.normalize();
    kalman->correctMEKF(magneticField);
}

void printState()
{
    auto kalmanState    = kalman->getState();
    auto kalmanRotation = SkyQuaternion::quat2eul(Vector4f(
        kalmanState(6), kalmanState(7), kalmanState(8), kalmanState(9)));

    std::cout << "Kalman state:" << std::endl;
    std::cout << kalmanState << std::endl;
    printf("Orientation: %6.4f, %6.2f, %6.4f\n", kalmanRotation(0),
           kalmanRotation(1), kalmanRotation(2));
}

std::istream& operator>>(std::istream& input, BMX160Data& data)
{
    input >> data.accelerationTimestamp;
    input.ignore(1, ',');
    input >> data.accelerationX;
    input.ignore(1, ',');
    input >> data.accelerationY;
    input.ignore(1, ',');
    input >> data.accelerationZ;
    input.ignore(1, ',');
    input >> data.angularVelocityTimestamp;
    input.ignore(1, ',');
    input >> data.angularVelocityX;
    input.ignore(1, ',');
    input >> data.angularVelocityY;
    input.ignore(1, ',');
    input >> data.angularVelocityZ;
    input.ignore(1, ',');
    input >> data.magneticFieldTimestamp;
    input.ignore(1, ',');
    input >> data.magneticFieldX;
    input.ignore(1, ',');
    input >> data.magneticFieldY;
    input.ignore(1, ',');
    input >> data.magneticFieldZ;

    return input;
}
