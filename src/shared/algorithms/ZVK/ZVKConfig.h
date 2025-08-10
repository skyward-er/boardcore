/* Copyright (c) 2025 Skyward Experimental Rocketry
 * Author: Giovanni Annaloro
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
 #include <Eigen/Core>
 #include <Eigen/Dense>

 namespace Boardcore 

 {

    struct ZVKConfig
    {
        float T;                //[s] Sample period
        float TUNE_PARAM_mu;    //[] Algorithm tunable parameter
        float TUNE_PARAM_Re;    //[] Algorithm tunable parameter
        float TUNE_PARAM_J2;    //[] Algorithm tunable parameter
        float SIGMA_GYRO;       //[rad/s] Estimated gyroscope variance
        float SIGMA_GYRO_BIAS;  //[rad/s] Estimated gyroscope bias variance
        float SIGMA_ACC;        //[m/s^2] Estimated accelerometer variance
        float SIGMA_BIAS_ACC;   //[m/s^2] Estimated accelerometer bias variance
        float SIGMA_MAG;        //[mgauss] Estimated magnetometer variance
        float BIAS_ACC;         //[m/s^2] Accelerometer bias
        float BIAS_GYRO;        //[rad/s] Gyroscope bias
        float VELOCITY_UNCERTAINTY; //[m/s] Uncertainty on velocity
        float POSITION_UNCERTAINTY; //[m] Uncertainty on position
        Eigen::Vector3f NED_MAG; // Normalized magnetic field vector in NED frame

    };

 } // namespace Boardcore
