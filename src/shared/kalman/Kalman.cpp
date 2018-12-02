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

#include "Kalman.h"
#include <iostream>

Kalman::Kalman(Matrix P_init, Matrix R_init, Matrix Q_init, Matrix H_init)
    : P(P_init), R(R_init), Q(Q_init), H(H_init), X(H_init.columns, 1),
      Phi(H.columns, H.columns)
{
}

void Kalman::update(Matrix y)
{
    // y is the measurement vector

    // Error matrix propagation
    Matrix P_new = Phi * P * Phi.transposed() + Q;

    // Gain calculation
    Matrix K =
        P_new * H.transposed() * (((H * P_new * H.transposed()) + R).inverse());

    // Error matrix correction
    P = (Matrix::eye(3) - (K * H)) * P_new;

    // State propagation
    Matrix X_new = Phi * X;

    // State correction
    X = X_new + (K * (y - H * X_new));
}
