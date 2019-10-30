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
#include <../libs/simple-template-matrix/matrix.h>
#include <math.h>

/*!
 * \class Kalman
 * \brief A class representing a Kalman filter
 *
 * ``` m ```: number of inputs
 * ``` n ```: number of states
 * ``` p ```: number of outputs
 *
 * ``` 
 * x(k+1) = A x(k) + B u(k) + v1
 * y(k)   = C x(k) + D u(k) + v2
 * v1 ~ WN(0, V1)
 * v2 ~ WN(0, V2)
 *  ```
 *
 * EXOGENOUS INPUTS ARE NOT YET IMPLEMENTED
 * COVARIANCE BETWEEN v1 AND v2 NOT YET IMPLEMENTED
 * 
 * To use the filter:
 * (1) Call the initializer with the appropriate matrices
 * (2) Define the state propagation matrix and initial state
 * (3) Call the update function for each new sample acquired
 */
template <unsigned n, unsigned p>
class Kalman
{
    using MatrixNN = MatrixBase<float, n, n>;
    using MatrixPN = MatrixBase<float, p, n>;
    using MatrixNP = MatrixBase<float, n, p>;
    using MatrixPP = MatrixBase<float, p, p>;
    using CVectorN = MatrixBase<float, n, 1>;
    using CVectorP = MatrixBase<float, p, 1>;
    
public:
    MatrixNN A;  /**< State propagation matrix */
    MatrixPN C;  /**< Vector mapping measurements to the state */
    CVectorN X;  /**< State matrix */
    MatrixNN V1; /**< Model variance matrix */
    MatrixPP V2; /**< Measurement variance vector */
    MatrixNN P;  /**< Error covariance matrix */

    /**
     * \brief Constructor
     */
    Kalman(const MatrixNN & A_init,
           const MatrixPN & C_init,
           const MatrixNN & V1_init,
           const MatrixPP & V2_init,
           const MatrixNN & P_init)
        : A(A_init), C(C_init), V1(V1_init),V2(V2_init), P(P_init)
        {};

    /**
     * \brief Method for updating the estimate
     * \param y The measurement vector
     */
    //    template <unsigned m, unsigned n, unsigned p>
    bool update( const CVectorP & y)
    {
        // Variable names:
        // x(k|k):      X
        // x(k|k-1):    X_new
        // P(k|k):      P
        // P(k|k-1):    P_new
        // y(k):        y


        // Error covariance matrix correction
        // P(k|k-1) = A*P(k-1|k-1)*A^T + V1
        MatrixNN P_new = A*P*transpose(A) + V1;

        // Kalman gain calculation
        // K(k) = P(k|k-1)*C^T*( C*P(k|k-1)*C^T + V2 )^-1
        MatrixPP temp = C*P_new*transpose(C) + V2;
        float d = det(temp);
        if(fabs(d) < 1e-3) {
            // Matrix ill conditioned
            return false;
        }
        MatrixNP K = P_new*transpose(C)*inv(temp, d);

        // Eye matrix
        
        MatrixNN I = MatrixNN::eye();
        
        // Error covariance matrix propagation
        // P(k|k) = ( I-K(k)*C ) * P(k|k-1)
        P = (I-K*C)*P_new;
        
        // State propagation
        // x(k|k-1) = A*x(k-1|k-1)
        CVectorN X_new = A*X;
        
        // State correction
        // x(k|k) = x(k|k-1) + K(k) * ( y(k)-C*x(k|k-1) )
        X = X_new + K*(y-C*X_new);
        return true;
    };


    /**
     * \brief Predicts k steps ahead the state
     */
    CVectorN state(int k) {
        MatrixBase<float, n, 1> X_hat = X;
        for(int i = 0; i < k; i++)
        {
            X_hat = A*X_hat;
        }
        return X_hat;
    }

    /**
     * \brief Predicts k steps ahead the output
     */
    CVectorP output(int k) {
        return C*state(k);
    }

};
#endif /* Kalman_h */
