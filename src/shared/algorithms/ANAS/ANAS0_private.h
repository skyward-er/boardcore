//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// File: ANAS0_private.h
//
// Code generated for Simulink model 'ANAS0'.
//
// Model version                  : 11.296
// Simulink Coder version         : 24.2 (R2024b) 21-Jun-2024
// C/C++ source code generated on : Mon Jun  8 17:58:37 2026
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
#include "dsp_rt.h"    // DSP System Toolbox general run time support functions

extern void rt_mrdividef3x3(const float u0[9], const float u1[9], float y[9]);
extern void rt_mrdivide_U1f6x2_U2f2x2_Yf6x2(const float u0[12], const float u1[4],
  float y[12]);
extern void LUf_boolfloatint32_t(float outU[], float outP[], int32_t N, bool
  outS[]);
extern void rt_mrdivide_U1f6x4_U2f4x4_Yf6x4(float u0[24], const float u1[16]);

#endif                                 // ANAS0_private_h_

//
// File trailer for generated code.
//
// [EOF]
//
