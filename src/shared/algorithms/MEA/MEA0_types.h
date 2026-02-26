//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// File: MEA0_types.h
//
// Code generated for Simulink model 'MEA0'.
//
// Model version                  : 11.125
// Simulink Coder version         : 24.2 (R2024b) 21-Jun-2024
// C/C++ source code generated on : Sun Feb 22 11:31:50 2026
//
// Target selection: ert.tlc
// Embedded hardware selection: ARM Compatible->ARM Cortex-M
// Code generation objective: RAM efficiency
// Validation result: Passed (3), Warning (1), Error (1)
//
#ifndef MEA0_types_h_
#define MEA0_types_h_
#include "reflect.hpp"
#include "multiword_types.h"
#ifndef DEFINED_TYPEDEF_FOR_Bus_MEA_CC_
#define DEFINED_TYPEDEF_FOR_Bus_MEA_CC_

struct MEAState
{
    uint64_t timestamp;

    float estimatedPressure;  ///< Estimated pressure in combustion chamber [Pa]
    float estimatedMass;      ///< Estimated rocket mass [kg]
    float estimatedApogee;    ///< Estimated apogee in msl [m]
    float estimatedForce;     ///< Estimated drag force [N]

    float x0;  ///< first kalman state
    float x1;  ///< second kalman state
    float x2;

    static constexpr auto reflect()
    {
        return STRUCT_DEF(
            MEAState, FIELD_DEF(timestamp) FIELD_DEF(estimatedPressure)
                          FIELD_DEF(estimatedMass) FIELD_DEF(estimatedApogee)
                              FIELD_DEF(estimatedForce) FIELD_DEF(x0)
                                  FIELD_DEF(x1) FIELD_DEF(x2));
    }
};

struct Bus_MEA_CC
{
  float CCPressure;
  uint64m_T CCTimestamp;
};

#endif
#endif                                 // MEA0_types_h_

//
// File trailer for generated code.
//
// [EOF]
//
