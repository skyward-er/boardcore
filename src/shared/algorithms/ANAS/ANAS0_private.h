//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// File: ANAS0_private.h
//
// Code generated for Simulink model 'ANAS0'.
//
// Model version                  : 11.276
// Simulink Coder version         : 24.2 (R2024b) 21-Jun-2024
// C/C++ source code generated on : Fri May 29 18:43:15 2026
//
// Target selection: ert.tlc
// Embedded hardware selection: ARM Compatible->ARM Cortex-M
// Code generation objectives:
//    1. Execution efficiency
//    2. Debugging
//    3. RAM efficiency
// Validation result: Not run
//
#ifndef ANAS0_private_h_
#define ANAS0_private_h_
#include <stdbool.h>
#include <stdint.h>
#include "ANAS0_types.h"
#include "rtw_continuous.h"
#include "rtw_solver.h"

extern double rt_remd(double u0, double u1);
extern void rt_mrdivide_U1d6x3_U2d3x3_Yd6x3(const double u0[18], const double
  u1[9], double y[18]);
extern void rt_mrdivide_U1d6x4_U2d4x4_Yd6x4(double u0[24], const double u1[16]);
extern void rt_mrdivide_U1d6x2_U2d2x2_Yd6x2(const double u0[12], const double
  u1[4], double y[12]);

#endif                                 // ANAS0_private_h_

//
// File trailer for generated code.
//
// [EOF]
//
