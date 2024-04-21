/* Copyright (c) 2023 Skyward Experimental Rocketry
 * Author: Emilio Corigliano, Davide Basso
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

#include "nasData.h"

using namespace Boardcore;
using namespace Eigen;

void checkStates(uint32_t i, const NASState &x_curr, const NASState &x_i);

TEST_CASE("NAS complete")
{
    static_assert(sizeof(acc) / sizeof(acc[0]) ==
                  sizeof(output) / sizeof(output[0]));
    static_assert(sizeof(baro) / sizeof(baro[0]) ==
                  sizeof(output) / sizeof(output[0]));
    static_assert(sizeof(gps) / sizeof(gps[0]) ==
                  sizeof(output) / sizeof(output[0]));
    static_assert(sizeof(gyro) / sizeof(gyro[0]) ==
                  sizeof(output) / sizeof(output[0]));
    static_assert(sizeof(mag) / sizeof(mag[0]) ==
                  sizeof(output) / sizeof(output[0]));
    static_assert(sizeof(pitot) / sizeof(pitot[0]) ==
                  sizeof(output) / sizeof(output[0]));
    static_assert(sizeof(input) / sizeof(input[0]) ==
                  sizeof(output) / sizeof(output[0]));

    NAS nas(nasConfig);
    NASState x_curr;
    ReferenceValues r(gps[0].height, baro[0].pressure, 15, gps[0].latitude,
                      gps[0].longitude);
    nas.setReferenceValues(r);

    for (uint32_t i = 0; i < sizeof(output) / sizeof(output[0]) - 1; i++)
    {
        nas.setX(input[i].getX());

        // Predict acceleration
        nas.predictAcc(acc[i]);
        printf("[%d] Predicting acceleration:\n", i);
        printf("[%d] N: %f.2/%f.2\n", i, nas.getState().n, steps[i].acc_x);
        printf("[%d] E: %f.2/%f.2\n", i, nas.getState().e, steps[i].acc_y);
        printf("[%d] D: %f.2/%f.2\n", i, nas.getState().d, steps[i].acc_z);
        printf("[%d] VN: %f.2/%f.2\n", i, nas.getState().vn, steps[i].acc_vx);
        printf("[%d] VE: %f.2/%f.2\n", i, nas.getState().ve, steps[i].acc_vy);
        printf("[%d] VD: %f.2/%f.2\n", i, nas.getState().vd, steps[i].acc_vz);

        // Predict gyro
        nas.predictGyro(gyro[i]);
        printf("[%d] Predicting gyroscope:\n", i);
        printf("[%d] QX: %f.2/%f.2\n", i, nas.getState().qx, steps[i].gyro_gx);
        printf("[%d] QX: %f.2/%f.2\n", i, nas.getState().qy, steps[i].gyro_gy);
        printf("[%d] QX: %f.2/%f.2\n", i, nas.getState().qz, steps[i].gyro_gz);
        printf("[%d] QW: %f.2/%f.2\n", i, nas.getState().qw, steps[i].gyro_gw);
        printf("[%d] BX: %f.2/%f.2\n", i, nas.getState().bx, steps[i].gyro_gbx);
        printf("[%d] BX: %f.2/%f.2\n", i, nas.getState().by, steps[i].gyro_gby);
        printf("[%d] BX: %f.2/%f.2\n", i, nas.getState().bz, steps[i].gyro_gbz);

        // Correct gps
        nas.correctGPS(gps[i]);
        printf("[%d] Correcting GPS:\n", i);
        printf("[%d] N: %f.2/%f.2\n", i, nas.getState().n, steps[i].gps_x);
        printf("[%d] E: %f.2/%f.2\n", i, nas.getState().e, steps[i].gps_y);
        printf("[%d] D: %f.2/%f.2\n", i, nas.getState().d, steps[i].gps_z);
        printf("[%d] VN: %f.2/%f.2\n", i, nas.getState().vn, steps[i].gps_vx);
        printf("[%d] VE: %f.2/%f.2\n", i, nas.getState().ve, steps[i].gps_vy);
        printf("[%d] VD: %f.2/%f.2\n", i, nas.getState().vd, steps[i].gps_vz);

        // Correct barometer
        nas.correctBaro(baro[i].pressure);
        printf("[%d] Correcting barometer:\n", i);
        printf("[%d] N: %f.2/%f.2\n", i, nas.getState().n, steps[i].baro_x);
        printf("[%d] E: %f.2/%f.2\n", i, nas.getState().e, steps[i].baro_y);
        printf("[%d] D: %f.2/%f.2\n", i, nas.getState().d, steps[i].baro_z);
        printf("[%d] VN: %f.2/%f.2\n", i, nas.getState().vn, steps[i].baro_vx);
        printf("[%d] VE: %f.2/%f.2\n", i, nas.getState().ve, steps[i].baro_vy);
        printf("[%d] VD: %f.2/%f.2\n", i, nas.getState().vd, steps[i].baro_vz);

        // Correct magnetometer
        nas.correctMag(mag[i]);
        printf("[%d] Correcting magnetometer:\n", i);
        printf("[%d] QX: %f.2/%f.2\n", i, nas.getState().qx, steps[i].mag_gx);
        printf("[%d] QY: %f.2/%f.2\n", i, nas.getState().qy, steps[i].mag_gy);
        printf("[%d] QZ: %f.2/%f.2\n", i, nas.getState().qz, steps[i].mag_gz);
        printf("[%d] QW: %f.2/%f.2\n", i, nas.getState().qw, steps[i].mag_gw);
        printf("[%d] BX: %f.2/%f.2\n", i, nas.getState().bx, steps[i].mag_gbx);
        printf("[%d] BY: %f.2/%f.2\n", i, nas.getState().by, steps[i].mag_gby);
        printf("[%d] BZ: %f.2/%f.2\n", i, nas.getState().bz, steps[i].mag_gbz);

        // Correct pitot
        nas.correctPitot(pitot[i].airspeed);
        printf("[%d] Correcting pitot:\n", i);
        printf("[%d] N: %f.2/%f.2\n", i, nas.getState().n, steps[i].pitot_x);
        printf("[%d] E: %f.2/%f.2\n", i, nas.getState().e, steps[i].pitot_y);
        printf("[%d] D: %f.2/%f.2\n", i, nas.getState().d, steps[i].pitot_z);
        printf("[%d] VN: %f.2/%f.2\n", i, nas.getState().vn, steps[i].pitot_vx);
        printf("[%d] VE: %f.2/%f.2\n", i, nas.getState().ve, steps[i].pitot_vy);
        printf("[%d] VD: %f.2/%f.2\n", i, nas.getState().vd, steps[i].pitot_vz);

        // Get the results
        x_curr = NASState(i, nas.getX());
        checkStates(i, x_curr, input[i + 1]);
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
