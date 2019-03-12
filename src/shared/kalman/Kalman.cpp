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
    : R(R_init), Q(Q_init), H(H_init), P(P_init), X(H_init.columns, 1),
      Phi(H.columns, H.columns)
{
}

void Kalman::update(Matrix y)
{
    // y is the measurement vector

    // Error matrix propagation
    // Matrix P_new = Phi * P * Phi.transposed() + Q;
    
    Matrix Phi_transposed{Phi.columns, Phi.rows};
    Matrix::transpose(Phi, Phi_transposed);
    
    Matrix P_new{P.rows, P.columns};
    Matrix::multiply(Phi, P, P_new); // P_new = Phi*P
    Matrix::multiply(P_new, Phi_transposed, P_new); // Phi_new = P_new*Phi_transposed
    Matrix::sum(P_new, Q, P_new); // Phi_new = Phi_new + Q

    // Gain calculation
    // K = P_new * H.transposed() * (((H * P_new * H.transposed()) + R).inverse());
    Matrix H_transposed{H.columns, H.rows};
    Matrix::transpose(H, H_transposed);
    Matrix A{H.rows, P_new.columns};
    Matrix::multiply(H, P_new, A); // A = H*P_new
    Matrix B{H.rows, H_transposed.columns};
    Matrix::multiply(A, H_transposed, B); // B = A*H_transposed
    Matrix::sum(B, R, B); // B = B+R
    Matrix::invert(B, B); // B = B.inverse()
    Matrix C{P_new.rows, H_transposed.columns};
    Matrix::multiply(P_new, H_transposed, C); // C = P_new*H_transposed
    Matrix K{C.rows, B.columns};
    Matrix::multiply(C, B, K); // K = C*B

    // Error matrix correction
    // P = (Matrix::eye(3) - (K * H)) * P_new;
    Matrix D{K.rows, H.columns};
    Matrix::multiply(K, H, D); // D = K*H
    Matrix::subtract(Matrix::eye(3), D, D); // D = I-D
    Matrix::multiply(D, P_new, P); // P = D*P_new

    // State propagation
    // X_new = Phi * X;
    Matrix X_new{Phi.rows, X.columns};
    Matrix::multiply(Phi, X, X_new);

    // State correction
    // X = X_new + (K * (y - H * X_new));
    Matrix E{y.rows, y.columns};
    Matrix::multiply(H, X_new, E); // E = H*X_new
    Matrix::subtract(y, E, E); // E = y-E
    Matrix F{K.rows, E.columns};
    Matrix::multiply(K, E, F); // F = K*E
    Matrix::sum(X_new, F, X); // X = X_new+F
}
