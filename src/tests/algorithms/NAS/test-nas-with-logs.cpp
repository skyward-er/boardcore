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

#include <algorithms/NAS/NAS.h>
#include <algorithms/NAS/StateInitializer.h>
#include <sensors/BMX160/BMX160.h>
#include <utils/CSVReader/CSVReader.h>
#include <utils/SkyQuaternion/SkyQuaternion.h>

#include <iostream>

using namespace miosix;
using namespace Boardcore;
using namespace Eigen;

NASConfig getEKConfig();
NASState readInitialState();
void updateKalman(BMX160Data data);
void printState();
std::istream& operator>>(std::istream& input, BMX160Data& data);
std::istream& operator>>(std::istream& input, NASState& data);

// Normalized NED magnetic field in Milan
Vector3f nedMag = Vector3f(0.47338841, 0.02656764, 0.88045305);

NAS* kalman;
bool triad = false;

int main()
{
    // Prepare the Kalman
    kalman = new NAS(getEKConfig());
    kalman->setState(readInitialState());

    // Retrieve all the data
    auto logData = CSVParser<BMX160Data>("/sd/imu_data.csv", true).collect();
    printf("Log size: %d\n", logData.size());

    // Wait to start
    printf("Waiting...\n");
    std::cin.get();

    // Loop through the logs
    for (size_t index = 0; index < logData.size(); index++)
    {
        printf("Iteration %lu\n", (unsigned long)index);
        auto iterationData = logData.at(index);

        // Predict and correct
        updateKalman(iterationData);

        // Print results
        printState();

        // Wait to continue
        std::cin.get();
    }
}

NASConfig getEKConfig()
{
    NASConfig config;

    config.T              = 0.02f;
    config.SIGMA_BETA     = 0.0001f;
    config.SIGMA_W        = 0.3f;
    config.SIGMA_MAG      = 0.1f;
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
    config.NED_MAG = nedMag;

    return config;
}

NASState readInitialState()
{
    auto initialState =
        CSVParser<NASState>("/sd/initial_state.csv", true).collect();

    printf("Initial state count: %d\n", initialState.size());

    if (initialState.size() == 0)
        return NASState();
    else
        return initialState.at(0);
}

void updateKalman(BMX160Data data)
{
    // Vector3f acceleration(data.accelerationX, data.accelerationY,
    //                       data.accelerationZ);
    // kalman->predict(acceleration);

    Vector3f angularVelocity(data.angularVelocityX, data.angularVelocityY,
                             data.angularVelocityZ);
    kalman->predictGyro(angularVelocity);

    Vector3f magneticField(data.magneticFieldX, data.magneticFieldY,
                           data.magneticFieldZ);
    magneticField.normalize();
    kalman->correctMag(magneticField);
}

void printState()
{
    auto kalmanState = kalman->getState().getX();
    auto kalmanRotation =
        SkyQuaternion::quat2eul(kalmanState.block<4, 1>(NAS::IDX_QUAT, 0));

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

std::istream& operator>>(std::istream& input, NASState& data)
{
    input >> data.timestamp;
    input.ignore(1, ',');
    input >> data.n;
    input.ignore(1, ',');
    input >> data.e;
    input.ignore(1, ',');
    input >> data.d;
    input.ignore(1, ',');
    input >> data.vn;
    input.ignore(1, ',');
    input >> data.ve;
    input.ignore(1, ',');
    input >> data.vd;
    input.ignore(1, ',');
    input >> data.qx;
    input.ignore(1, ',');
    input >> data.qy;
    input.ignore(1, ',');
    input >> data.qz;
    input.ignore(1, ',');
    input >> data.qw;
    input.ignore(1, ',');
    input >> data.bx;
    input.ignore(1, ',');
    input >> data.by;
    input.ignore(1, ',');
    input >> data.bz;

    return input;
}
