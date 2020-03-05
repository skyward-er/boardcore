/* Copyright (c) 2019 Skyward Experimental Rocketry
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

#pragma once

#include <../libs/simple-template-matrix/matrix.h>
#include <iostream>

/*!
 * \class RLS
 * \brief A performin Recursive least squares identification
 *
 * ``` n ```: number of parameters to be estimated
 * ``` n ```: number of states
 * ``` p ```: number of outputs
 *
 * The system model is:
 * ``` 
 * y(t) = phi(t)^T * theta(t)
 *  ```
 * being y the output and theta the vector of parameters to be identified.
 * See test-rls.cpp for an example.
 */
template <unsigned n>
class RLS
{
    using CVectorN = MatrixBase<float, n, 1>;
    using MatrixNN = MatrixBase<float, n, n>;
private:
    MatrixNN V;
    CVectorN theta;
    float mu;
public:
    RLS(const MatrixNN &V_0, const CVectorN &theta_0, float mu) : V(V_0), theta(theta_0), mu(mu){};


    ~RLS(){};
    
    void update(const CVectorN & y_kp1, const CVectorN & phi)
    {
        V = V/mu - (V/mu * phi*transpose(phi) * V/mu) / (1 + transpose(phi)*V/mu* phi)(0,0);
        theta = theta + V*phi*(y_kp1 - transpose(theta)*phi);
    };

    CVectorN getEstimate(){ return theta; };
};