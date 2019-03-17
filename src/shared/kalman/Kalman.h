/* Copyright (c) 2018 Skyward Experimental Rocketry
 * Authors: Luca Mozzarelli
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
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.  IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 */

#ifndef Kalman_h
#define Kalman_h
#include "Matrix.hpp"
/*!
 * \class Kalman
 * \brief A class representing a Kalman filter
 *
 * To use the filter:
 * (1) Call the initializer with the appropriate matrices
 * (2) Define the state propagation matrix and initial state
 * (3) Call the update function for each new sample acquired
 */
class Kalman
{
private:
    Matrix R; /**< Measurement variance vector */
    Matrix Q; /**< Model variance matrix */
    Matrix H; /**< Vector mapping the measurements to the state */
public:
    Matrix P;   /**< Error covariance matrix */
    Matrix X;   /**< State matrix */
    Matrix Phi; /**< State propagation matrix */

    /**
     * \brief Constructor
     * \param P_init Error covariance matrix
     * \param R_init Measurement variance vector
     * \param Q_init Model variance matrix
     * \param H_init Vector mapping the measurements to the state
     */
    Kalman(Matrix P_init, Matrix R_init, Matrix Q_init, Matrix H_init);

    /**
     * \brief Method for updating the estimate
     * \param y The measurement vector
     */
    void update(Matrix y);
};

#endif /* Kalman_h */
