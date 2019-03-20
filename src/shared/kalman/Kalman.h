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
#include "matrix.h"
/*!
 * \class Kalman
 * \brief A class representing a Kalman filter
 *
 * m: number of inputs
 * n: number of states
 * p: number of outputs
 * 
 * x(k+1) = A x(k) + B u(k) + v1
 * y(k)   = C x(k) + D u(k) + v2
 * v1 ~ WN(0, V1)
 * v2 ~ WN(0, V2)
 * 
 * To use the filter:
 * (1) Call the initializer with the appropriate matrices
 * (2) Define the state propagation matrix and initial state
 * (3) Call the update function for each new sample acquired
 */
template </* unsigned m, */unsigned n, unsigned p>
class Kalman
{
// private:
    // Matrix R; /**< Measurement variance vector */
    // Matrix Q; /**< Model variance matrix */
    // Matrix H; /**< Vector mapping the measurements to the state */
public:
    MatrixBase<float, n, n> A;      /**< State propagation matrix */
    // MatrixBase<float, n, m> B; 
    MatrixBase<float, p, n> C;      /**< Vector mapping the measurements to the state */
    // MatrixBase<float, p, m> D;     
    MatrixBase<float, n, 1> X;      /**< State matrix */
    MatrixBase<float, n, n> V1;     /**< Model variance matrix */
    MatrixBase<float, p, p> V2;     /**< Measurement variance vector */
    // MatrixBase<float, , > V12; /**< Measurement variance vector */
    MatrixBase<float, n, n > P;     /**< Error covariance matrix */

    /**
     * \brief Constructor
     */
    Kalman() {};

    /**
     * \brief Predicts k steps ahead
     */
    MatrixBase<float, n, 1> predict(int k);

    /**
    * \brief Method for updating the estimate
    * \param y The measurement vector
    */
//    template <unsigned m, unsigned n, unsigned p>
   void update(MatrixBase<float, p, 1> y)
   {
       // y is the measurement vector
       
       // Error matrix propagation
       MatrixBase<float, n, n> P_new = A * P * transpose(A) + V1;
       
       // Gain calculation
       MatrixBase<float, n, p> K = P_new * transpose(C) * inv((C * P_new * transpose(C)) + V2);
       
       // Build the identity matrix
       MatrixBase<float, n, n> I{0};
       for (int i = 0; i < n; i++) { I(i,i) = 1; }

       // Error matrix correction
       P = (I - (K*C)) * P_new;
       
       // State propagation
       MatrixBase<float, n, 1> X_new = A * X;
       // State correction
       X = X_new + (K * (y - C * X_new));
    };
};

#endif /* Kalman_h */
