//
//  Kalman.h
//  Apogee
//
//  Created by Luca Mozzarelli on 06/10/2018.
//  Copyright © 2018 Luca Mozzarelli. All rights reserved.
//

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
