/* Copyright (c) 2023 Skyward Experimental Rocketry
 * Authors: Emilio Corigliano, Davide Basso
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

#include <catch2/catch.hpp>
#include <fstream>
#include <iostream>

#include "data/acc/include.h"
#include "data/baro/include.h"
#include "data/complete/include.h"
#include "data/gps/include.h"
#include "data/gyro/include.h"
#include "data/mag/include.h"
#include "data/pitot/include.h"

using namespace Boardcore;
using namespace Eigen;

enum class NASAlgorithm
{
    PredictAcceleration,
    PredictGyro,
    CorrectGPS,
    CorrectBaro,
    CorrectMag,
    CorrectPitot,
    Complete
};

enum class NASExecution
{
    Static,
    Evolving
};

std::string algorithmToString(NASAlgorithm algorithm)
{
    switch (algorithm)
    {
        case NASAlgorithm::PredictAcceleration:
            return "acc";
        case NASAlgorithm::PredictGyro:
            return "gyro";
        case NASAlgorithm::CorrectGPS:
            return "gps";
        case NASAlgorithm::CorrectBaro:
            return "baro";
        case NASAlgorithm::CorrectMag:
            return "mag";
        case NASAlgorithm::CorrectPitot:
            return "pitot";
        case NASAlgorithm::Complete:
            return "complete";
        default:
            return "unknown";
    }
}

std::string executionToString(NASExecution execution)
{
    switch (execution)
    {
        case NASExecution::Static:
            return "static";
        case NASExecution::Evolving:
            return "evolving";
        default:
            return "default";
    }
}

void testAlgorithm(NASAlgorithm algorithm, NASExecution execution,
                   uint32_t length, AccelerometerData acc[],
                   GyroscopeData gyro[], GPSData gps[], PressureData baro[],
                   MagnetometerData mag[], PitotData pitot[], NASState input[],
                   NASState output[], NASConfig nasConfig,
                   NASPredictionSteps steps[]);
bool checkStates(uint32_t i, const NASState &x_curr, const NASState &x_i);

TEST_CASE("NAS - Predict acceleration - Static")
{
    testAlgorithm(NASAlgorithm::PredictAcceleration, NASExecution::Static,
                  sizeof(acc_acc) / sizeof(acc_acc[0]), acc_acc, acc_gyro,
                  acc_gps, acc_baro, acc_mag, acc_pitot, acc_input, acc_output,
                  acc_nas_config, acc_steps);
}

TEST_CASE("NAS - Predict gyro - Static")
{
    testAlgorithm(NASAlgorithm::PredictGyro, NASExecution::Static,
                  sizeof(gyro_acc) / sizeof(gyro_acc[0]), gyro_acc, gyro_gyro,
                  gyro_gps, gyro_baro, gyro_mag, gyro_pitot, gyro_input,
                  gyro_output, gyro_nas_config, gyro_steps);
}

TEST_CASE("NAS - Correct GPS - Static")
{
    testAlgorithm(NASAlgorithm::CorrectGPS, NASExecution::Static,
                  sizeof(gps_acc) / sizeof(gps_acc[0]), gps_acc, gps_gyro,
                  gps_gps, gps_baro, gps_mag, gps_pitot, gps_input, gps_output,
                  gps_nas_config, gps_steps);
}

TEST_CASE("NAS - Correct barometer - Static")
{
    testAlgorithm(NASAlgorithm::CorrectBaro, NASExecution::Static,
                  sizeof(baro_acc) / sizeof(baro_acc[0]), baro_acc, baro_gyro,
                  baro_gps, baro_baro, baro_mag, baro_pitot, baro_input,
                  baro_output, baro_nas_config, baro_steps);
}

TEST_CASE("NAS - Correct magnetometer - Static")
{
    testAlgorithm(NASAlgorithm::CorrectMag, NASExecution::Static,
                  sizeof(mag_acc) / sizeof(mag_acc[0]), mag_acc, mag_gyro,
                  mag_gps, mag_baro, mag_mag, mag_pitot, mag_input, mag_output,
                  mag_nas_config, mag_steps);
}

TEST_CASE("NAS - Correct pitot - Static")
{
    testAlgorithm(NASAlgorithm::CorrectPitot, NASExecution::Static,
                  sizeof(pitot_acc) / sizeof(pitot_acc[0]), pitot_acc,
                  pitot_gyro, pitot_gps, pitot_baro, pitot_mag, pitot_pitot,
                  pitot_input, pitot_output, pitot_nas_config, pitot_steps);
}

TEST_CASE("NAS - Complete - Static")
{
    testAlgorithm(NASAlgorithm::Complete, NASExecution::Static,
                  sizeof(complete_acc) / sizeof(complete_acc[0]), complete_acc,
                  complete_gyro, complete_gps, complete_baro, complete_mag,
                  complete_pitot, complete_input, complete_output,
                  complete_nas_config, complete_steps);
}

TEST_CASE("NAS - Predict acceleration - Evolving")
{
    testAlgorithm(NASAlgorithm::PredictAcceleration, NASExecution::Evolving,
                  sizeof(acc_acc) / sizeof(acc_acc[0]), acc_acc, acc_gyro,
                  acc_gps, acc_baro, acc_mag, acc_pitot, acc_input, acc_output,
                  acc_nas_config, acc_steps);
}

TEST_CASE("NAS - Predict gyro - Evolving")
{
    testAlgorithm(NASAlgorithm::PredictGyro, NASExecution::Evolving,
                  sizeof(gyro_acc) / sizeof(gyro_acc[0]), gyro_acc, gyro_gyro,
                  gyro_gps, gyro_baro, gyro_mag, gyro_pitot, gyro_input,
                  gyro_output, gyro_nas_config, gyro_steps);
}

TEST_CASE("NAS - Correct GPS - Evolving")
{
    testAlgorithm(NASAlgorithm::CorrectGPS, NASExecution::Evolving,
                  sizeof(gps_acc) / sizeof(gps_acc[0]), gps_acc, gps_gyro,
                  gps_gps, gps_baro, gps_mag, gps_pitot, gps_input, gps_output,
                  gps_nas_config, gps_steps);
}

TEST_CASE("NAS - Correct barometer - Evolving")
{
    testAlgorithm(NASAlgorithm::CorrectBaro, NASExecution::Evolving,
                  sizeof(baro_acc) / sizeof(baro_acc[0]), baro_acc, baro_gyro,
                  baro_gps, baro_baro, baro_mag, baro_pitot, baro_input,
                  baro_output, baro_nas_config, baro_steps);
}

TEST_CASE("NAS - Correct magnetometer - Evolving")
{
    testAlgorithm(NASAlgorithm::CorrectMag, NASExecution::Evolving,
                  sizeof(mag_acc) / sizeof(mag_acc[0]), mag_acc, mag_gyro,
                  mag_gps, mag_baro, mag_mag, mag_pitot, mag_input, mag_output,
                  mag_nas_config, mag_steps);
}

TEST_CASE("NAS - Correct pitot - Evolving")
{
    testAlgorithm(NASAlgorithm::CorrectPitot, NASExecution::Evolving,
                  sizeof(pitot_acc) / sizeof(pitot_acc[0]), pitot_acc,
                  pitot_gyro, pitot_gps, pitot_baro, pitot_mag, pitot_pitot,
                  pitot_input, pitot_output, pitot_nas_config, pitot_steps);
}

TEST_CASE("NAS - Complete - Evolving")
{
    testAlgorithm(NASAlgorithm::Complete, NASExecution::Evolving,
                  sizeof(complete_acc) / sizeof(complete_acc[0]), complete_acc,
                  complete_gyro, complete_gps, complete_baro, complete_mag,
                  complete_pitot, complete_input, complete_output,
                  complete_nas_config, complete_steps);
}

void testAlgorithm(NASAlgorithm algorithm, NASExecution execution,
                   uint32_t length, AccelerometerData acc[],
                   GyroscopeData gyro[], GPSData gps[], PressureData baro[],
                   MagnetometerData mag[], PitotData pitot[], NASState input[],
                   NASState output[], NASConfig nasConfig,
                   NASPredictionSteps steps[])
{
    NAS nas(nasConfig);
    NASState x_curr;
    ReferenceValues r(gps[0].height, baro[0].pressure, 15, gps[0].latitude,
                      gps[0].longitude);

    bool correct = true;

    // std::ofstream outfile;
    // auto filepath = algorithmToString(algorithm) + "-" +
    //                 executionToString(execution) + ".csv";
    // using ios = std::ios;
    // outfile.open(filepath, ios::out | ios::trunc);
    // outfile << "n,e,d,vn,ve,vd,qx,qy,qz,qw,bx,by,bz\n";

    nas.setReferenceValues(r);
    nas.setX(input[0].getX());

    for (uint32_t i = 0; i < length - 1; i++)
    {
        if (execution == NASExecution::Static)
        {
            nas.setX(input[i].getX());
        }

        // Predict acceleration
        if (algorithm == NASAlgorithm::PredictAcceleration ||
            algorithm == NASAlgorithm::Complete)
        {
            nas.predictAcc(acc[i]);
        }

        // Predict gyro
        if (algorithm == NASAlgorithm::PredictGyro ||
            algorithm == NASAlgorithm::Complete)
        {
            nas.predictGyro(gyro[i]);
        }

        // Correct gps
        if (Vector3f(acc[i].accelerationX, acc[i].accelerationY,
                     acc[i].accelerationZ)
                .norm() < 34)
        {
            if (algorithm == NASAlgorithm::CorrectGPS ||
                algorithm == NASAlgorithm::Complete)
            {
                nas.correctGPS(gps[i]);
            }
        }

        // Correct barometer
        if (algorithm == NASAlgorithm::CorrectBaro ||
            algorithm == NASAlgorithm::Complete)
        {
            nas.correctBaro(baro[i].pressure);
        }

        // Correct magnetometer
        if (algorithm == NASAlgorithm::CorrectMag ||
            algorithm == NASAlgorithm::Complete)
        {
            nas.correctMag(mag[i]);
        }

        // Correct pitot
        if (algorithm == NASAlgorithm::CorrectPitot ||
            algorithm == NASAlgorithm::Complete)
        {
            nas.correctPitot(pitot[i].airspeed);
        }

        // Get the results
        x_curr  = nas.getState();
        correct = correct && checkStates(i, x_curr, output[i]);

        // outfile << x_curr.n << "," << x_curr.e << "," << x_curr.d << ","
        //         << x_curr.vn << "," << x_curr.ve << "," << x_curr.vd << ","
        //         << x_curr.qx << "," << x_curr.qy << "," << x_curr.qz << ","
        //         << x_curr.qw << "," << x_curr.bx << "," << x_curr.by << ","
        //         << x_curr.bz << "\n";
    }

    // outfile.close();

    if (!correct)
    {
        FAIL("Divergence detected in NAS algorithm");
    }
}

bool checkStates(uint32_t i, const NASState &x_curr, const NASState &x_i)
{
    static const float marginN  = 1;
    static const float marginE  = 1;
    static const float marginD  = 2;
    static const float marginVN = 0.5;
    static const float marginVE = 0.5;
    static const float marginVD = 0.5;
    static const float marginQ  = 0.05;
    static const float marginB  = 0.01;

    if (x_curr.n != Approx(x_i.n).margin(marginN))
    {
        printf("The estimated n differs from the correct one [%u]: %f != %f\n",
               i, x_curr.n, x_i.n);
        return false;
    }
    if (x_curr.e != Approx(x_i.e).margin(marginE))
    {
        printf("The estimated e differs from the correct one [%u]: %f != %f\n",
               i, x_curr.e, x_i.e);
        return false;
    }
    if (x_curr.d != Approx(x_i.d).margin(marginD))
    {
        printf("The estimated d differs from the correct one [%u]: %f != %f\n",
               i, x_curr.d, x_i.d);
        return false;
    }
    if (x_curr.vn != Approx(x_i.vn).margin(marginVN))
    {
        printf("The estimated vn differs from the correct one [%u]: %f != %f\n",
               i, x_curr.vn, x_i.vn);
        return false;
    }
    if (x_curr.ve != Approx(x_i.ve).margin(marginVE))
    {
        printf("The estimated ve differs from the correct one [%u]: %f != %f\n",
               i, x_curr.ve, x_i.ve);
        return false;
    }
    if (x_curr.vd != Approx(x_i.vd).margin(marginVD))
    {
        printf("The estimated vd differs from the correct one [%u]: %f != %f\n",
               i, x_curr.vd, x_i.vd);
        return false;
    }
    if (x_curr.qx != Approx(x_i.qx).margin(marginQ))
    {
        printf("The estimated qx differs from the correct one [%u]: %f != %f\n",
               i, x_curr.qx, x_i.qx);
        return false;
    }
    if (x_curr.qy != Approx(x_i.qy).margin(marginQ))
    {
        printf("The estimated qy differs from the correct one [%u]: %f != %f\n",
               i, x_curr.qy, x_i.qy);
        return false;
    }
    if (x_curr.qz != Approx(x_i.qz).margin(marginQ))
    {
        printf("The estimated qz differs from the correct one [%u]: %f != %f\n",
               i, x_curr.qz, x_i.qz);
        return false;
    }
    if (x_curr.qw != Approx(x_i.qw).margin(marginQ))
    {
        printf("The estimated qw differs from the correct one [%u]: %f != %f\n",
               i, x_curr.qw, x_i.qw);
        return false;
    }
    if (x_curr.bx != Approx(x_i.bx).margin(marginQ))
    {
        printf("The estimated bx differs from the correct one [%u]: %f != %f\n",
               i, x_curr.bx, x_i.bx);
        return false;
    }
    if (x_curr.by != Approx(x_i.by).margin(marginQ))
    {
        printf("The estimated by differs from the correct one [%u]: %f != %f\n",
               i, x_curr.by, x_i.by);
        return false;
    }
    if (x_curr.bz != Approx(x_i.bz).margin(marginQ))
    {
        printf("The estimated bz differs from the correct one [%u]: %f != %f\n",
               i, x_curr.bz, x_i.bz);
        return false;
    }
    return true;
}
