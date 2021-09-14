/* Copyright (c) 2019 Skyward Experimental Rocketry
 * Author: Luca Mozzarelli
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

/*
* This test simulates power control in an unknown resistor (i.e. thermal cutters ;) )
* y(t) = phi(t)^T * theta(t)
* P = V^2/R
* 
* y(t)      = P(t) = V(t)*I(t)  (power on the resistor)
* phi(t)    = V(t)^2            (square of the voltage on the resistor)
* theta(t)  = 1/R               (inverse of the resistance)
*/

#include <libs/simple-template-matrix/matrix.h>
#include <rls/RLS.h>
#include <math.h>
#include <iostream>


int main()
{
    
    float R = 0.3;      // True resistance
    float P_ref = 20;   // Power setpoint
    float V;            // Voltage applied to the resistor
    float I;            // Current in the resistor

    float R0 = 0.5;                         // Inital guess of R
    MatrixBase<float,1,1> theta0{1/R0};
    MatrixBase<float,1,1> V0{1};            // Confidence of the initial guess (variance)
    float mu = 0.7;                         // Forgetting factor
    float R_est;                            // Estimated R

    // Instantiate the RLS
    RLS<1> rls{V0,theta0,mu};

    MatrixBase<float,1,1> phi{0};
    MatrixBase<float,1,1> y{0};

    for (unsigned i = 0; i < 1000; i++)
    {
        // Get the estimate of R
        R_est    = 1/(rls.getEstimate()(0,0));

        // Compute the power on the resistor
        V        = sqrtf(P_ref*R_est);

        // Compute the mesured current
        I        = V/R;

        // Regressors vector
        phi(0,0) = V*V;

        // Measurement vector
        y(0,0)   = V*I;

        // Update the filter
        rls.update(y,phi);

        // std::cout << R << ", " << R_est << ", " << R-R_est << ", " << V*I << "\n";

        // Start a slope on R
        if (i > 500)
        {
            R = R+0.05;
        }
    }
    
}