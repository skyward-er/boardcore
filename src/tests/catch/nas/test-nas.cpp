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

void testAlgorithm(NASAlgorithm algorithm, NASExecution execution, int length,
                   AccelerometerData acc[], GyroscopeData gyro[], GPSData gps[],
                   PressureData baro[], MagnetometerData mag[],
                   PitotData pitot[], NASState input[], NASState output[],
                   NASConfig nasConfig, NASPredictionSteps steps[]);
void checkStates(uint32_t i, const NASState &x_curr, const NASState &x_i);

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

void testAlgorithm(NASAlgorithm algorithm, NASExecution execution, int length,
                   AccelerometerData acc[], GyroscopeData gyro[], GPSData gps[],
                   PressureData baro[], MagnetometerData mag[],
                   PitotData pitot[], NASState input[], NASState output[],
                   NASConfig nasConfig, NASPredictionSteps steps[])
{
    NAS nas(nasConfig);
    NASState x_curr;
    ReferenceValues r(gps[0].height, baro[0].pressure, 15, gps[0].latitude,
                      gps[0].longitude);
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
            printf("[%d] Predicting acceleration:\n", i);
            printf("[%d] N: %.2f/%.2f\n", i, nas.getState().n, output[i].n);
            printf("[%d] E: %.2f/%.2f\n", i, nas.getState().e, output[i].e);
            printf("[%d] D: %.2f/%.2f\n", i, nas.getState().d, output[i].d);
            printf("[%d] VN: %.2f/%.2f\n", i, nas.getState().vn, output[i].vn);
            printf("[%d] VE: %.2f/%.2f\n", i, nas.getState().ve, output[i].ve);
            printf("[%d] VD: %.2f/%.2f\n", i, nas.getState().vd, output[i].vd);
        }

        // Predict gyro
        if (algorithm == NASAlgorithm::PredictGyro ||
            algorithm == NASAlgorithm::Complete)
        {
            nas.predictGyro(gyro[i]);
            printf("[%d] Predicting gyroscope:\n", i);
            printf("[%d] QX: %.2f/%.2f\n", i, nas.getState().qx, output[i].qx);
            printf("[%d] QX: %.2f/%.2f\n", i, nas.getState().qy, output[i].qy);
            printf("[%d] QX: %.2f/%.2f\n", i, nas.getState().qz, output[i].qz);
            printf("[%d] QW:%.2f/%.2f\n", i, nas.getState().qw, output[i].qw);
            printf("[%d]BX: %.2f/%.2f\n", i, nas.getState().bx, output[i].bx);
            printf("[%d] BX: %.2f/%.2f\n", i, nas.getState().by, output[i].by);
            printf("[%d] BX: %.2f/%.2f\n", i, nas.getState().bz, output[i].bz);
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
                printf("[%d] Correcting GPS:\n", i);
                printf("[%d] N: %.2f/%.2f\n", i, nas.getState().n,
                       steps[i].gps_x);
                printf("[%d] E: %.2f/%.2f\n", i, nas.getState().e,
                       steps[i].gps_y);
                printf("[%d] D: %.2f/%.2f\n", i, nas.getState().d,
                       steps[i].gps_z);
                printf("[%d] VN: %.2f/%.2f\n", i, nas.getState().vn,
                       steps[i].gps_vx);
                printf("[%d] VE: %.2f/%.2f\n", i, nas.getState().ve,
                       steps[i].gps_vy);
                printf("[%d] VD: %.2f/%.2f\n", i, nas.getState().vd,
                       steps[i].gps_vz);
            }
        }

        // Correct barometer
        if (algorithm == NASAlgorithm::CorrectBaro ||
            algorithm == NASAlgorithm::Complete)
        {
            nas.correctBaro(baro[i].pressure);
            printf("[%d] Correcting barometer:\n", i);
            printf("[%d] N: %.2f/%.2f\n", i, nas.getState().n, steps[i].baro_x);
            printf("[%d] E: %.2f/%.2f\n", i, nas.getState().e, steps[i].baro_y);
            printf("[%d] D: %.2f/%.2f\n", i, nas.getState().d, steps[i].baro_z);
            printf("[%d] VN: %.2f/%.2f\n", i, nas.getState().vn,
                   steps[i].baro_vx);
            printf("[%d] VE: %.2f/%.2f\n", i, nas.getState().ve,
                   steps[i].baro_vy);
            printf("[%d] VD: %.2f/%.2f\n", i, nas.getState().vd,
                   steps[i].baro_vz);
        }

        // Correct magnetometer
        if (algorithm == NASAlgorithm::CorrectMag ||
            algorithm == NASAlgorithm::Complete)
        {
            nas.correctMag(mag[i]);
            printf("[%d] Correcting magnetometer:\n", i);
            printf("[%d] QX: %.2f/%.2f\n", i, nas.getState().qx,
                   steps[i].mag_gx);
            printf("[%d] QY: %.2f/%.2f\n", i, nas.getState().qy,
                   steps[i].mag_gy);
            printf("[%d] QZ: %.2f/%.2f\n", i, nas.getState().qz,
                   steps[i].mag_gz);
            printf("[%d] QW:%.2f/%.2f\n", i, nas.getState().qw,
                   steps[i].mag_gw);
            printf("[%d]BX: %.2f/%.2f\n", i, nas.getState().bx,
                   steps[i].mag_gbx);
            printf("[%d] BY: %.2f/%.2f\n", i, nas.getState().by,
                   steps[i].mag_gby);
            printf("[%d] BZ: %.2f/%.2f\n", i, nas.getState().bz,
                   steps[i].mag_gbz);
        }

        // Correct pitot
        if (algorithm == NASAlgorithm::CorrectPitot ||
            algorithm == NASAlgorithm::Complete)
        {
            nas.correctPitot(pitot[i].airspeed);
            printf("[%d] Correcting pitot:\n", i);
            printf("[%d] N: %.2f/%.2f\n", i, nas.getState().n,
                   steps[i].pitot_x);
            printf("[%d] E: %.2f/%.2f\n", i, nas.getState().e,
                   steps[i].pitot_y);
            printf("[%d] D: %.2f/%.2f\n", i, nas.getState().d,
                   steps[i].pitot_z);
            printf("[%d] VN: %.2f/%.2f\n", i, nas.getState().vn,
                   steps[i].pitot_vx);
            printf("[%d] VE: %.2f/%.2f\n", i, nas.getState().ve,
                   steps[i].pitot_vy);
            printf("[%d] VD: %.2f/%.2f\n", i, nas.getState().vd,
                   steps[i].pitot_vz);
        }

        // Get the results
        x_curr = nas.getState();
        checkStates(i, x_curr, output[i]);
    }
}

void checkStates(uint32_t i, const NASState &x_curr, const NASState &x_i)
{
    static const float marginN  = 0.5;
    static const float marginE  = 0.5;
    static const float marginD  = 0.5;
    static const float marginVN = 0.1;
    static const float marginVE = 0.1;
    static const float marginVD = 0.1;
    static const float marginQ  = 0.02;
    static const float marginB  = 0.01;

    if (x_curr.n != Approx(x_i.n).margin(marginN))
    {
        FAIL("The estimated n differs from the correct one ["
             << i << "]: " << x_curr.n << " != " << x_i.n);
    }
    if (x_curr.e != Approx(x_i.e).margin(marginE))
    {
        FAIL("The estimated e differs from the correct one ["
             << i << "]: " << x_curr.e << " != " << x_i.e);
    }
    if (x_curr.d != Approx(x_i.d).margin(marginD))
    {
        FAIL("The estimated d differs from the correct one ["
             << i << "]: " << x_curr.d << " != " << x_i.d);
    }
    if (x_curr.vn != Approx(x_i.vn).margin(marginVN))
    {
        FAIL("The estimated vn differs from the correct one ["
             << i << "]: " << x_curr.vn << " != " << x_i.vn);
    }
    if (x_curr.ve != Approx(x_i.ve).margin(marginVE))
    {
        FAIL("The estimated ve differs from the correct one ["
             << i << "]: " << x_curr.ve << " != " << x_i.ve);
    }
    if (x_curr.vd != Approx(x_i.vd).margin(marginVD))
    {
        FAIL("The estimated vd differs from the correct one ["
             << i << "]: " << x_curr.vd << " != " << x_i.vd);
    }
    if (x_curr.qx != Approx(x_i.qx).margin(marginQ))
    {
        FAIL("The estimated qx differs from the correct one ["
             << i << "]: " << x_curr.qx << " != " << x_i.qx);
    }
    if (x_curr.qy != Approx(x_i.qy).margin(marginQ))
    {
        FAIL("The estimated qy differs from the correct one ["
             << i << "]: " << x_curr.qy << " != " << x_i.qy);
    }
    if (x_curr.qz != Approx(x_i.qz).margin(marginQ))
    {
        FAIL("The estimated qz differs from the correct one ["
             << i << "]: " << x_curr.qz << " != " << x_i.qz);
    }
    if (x_curr.qw != Approx(x_i.qw).margin(marginQ))
    {
        FAIL("The estimated qw differs from the correct one ["
             << i << "]: " << x_curr.qw << " != " << x_i.qw);
    }
    if (x_curr.bx != Approx(x_i.bx).margin(marginQ))
    {
        FAIL("The estimated bx differs from the correct one ["
             << i << "]: " << x_curr.bx << " != " << x_i.bx);
    }
    if (x_curr.by != Approx(x_i.by).margin(marginQ))
    {
        FAIL("The estimated by differs from the correct one ["
             << i << "]: " << x_curr.by << " != " << x_i.by);
    }
    if (x_curr.bz != Approx(x_i.bz).margin(marginQ))
    {
        FAIL("The estimated bz differs from the correct one ["
             << i << "]: " << x_curr.bz << " != " << x_i.bz);
    }
}
