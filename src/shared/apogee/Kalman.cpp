//
//  Kalman.cpp
//  Kalman
//
//  Created by Luca Mozzarelli on 06/10/2018.
//  Copyright © 2018 Luca Mozzarelli. All rights reserved.
//

#include "Kalman.h"
#include <iostream>


Kalman::Kalman(Matrix P_init, Matrix R_init, Matrix Q_init, Matrix H_init) : P(P_init), R(R_init), Q(Q_init), H(H_init), X(H_init.columns,1), Phi(H.columns, H.columns){
}

void Kalman::update(Matrix y) {
    // y is the measurement vector

    // Error matrix propagation
    Matrix P_new = Phi*P*Phi.transposed() + Q;
    
    printf("BBBBBBBB \n\n");
    // Gain calculation
    Matrix K = P_new*H.transposed()*( ((H*P_new*H.transposed()) + R).inverse() );
    
    printf("CCCCCCCC \n\n");
    // Error matrix correction
    P = (Matrix::eye(3)-(K*H))*P_new;
    
    printf("DDDDDDDDD \n\n");
    // State propagation
    Matrix X_new = Phi*X;
    
    printf("EEEEEEEEE \n\n");
    // State correction
    X = X_new + (K*(y-H*X_new));
    printf("FFFFFFFFF \n\n");
}
