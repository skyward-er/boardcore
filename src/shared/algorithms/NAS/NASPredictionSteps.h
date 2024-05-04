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

#pragma once

namespace Boardcore
{

struct NASPredictionSteps
{
    float acc_x;
    float acc_y;
    float acc_z;
    float acc_vx;
    float acc_vy;
    float acc_vz;
    float gyro_gx;
    float gyro_gy;
    float gyro_gz;
    float gyro_gw;
    float gyro_gbx;
    float gyro_gby;
    float gyro_gbz;
    float gps_x;
    float gps_y;
    float gps_z;
    float gps_vx;
    float gps_vy;
    float gps_vz;
    float baro_x;
    float baro_y;
    float baro_z;
    float baro_vx;
    float baro_vy;
    float baro_vz;
    float mag_gx;
    float mag_gy;
    float mag_gz;
    float mag_gw;
    float mag_gbx;
    float mag_gby;
    float mag_gbz;
    float pitot_x;
    float pitot_y;
    float pitot_z;
    float pitot_vx;
    float pitot_vy;
    float pitot_vz;
};
}  // namespace Boardcore