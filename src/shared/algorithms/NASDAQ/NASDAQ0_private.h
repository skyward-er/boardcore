//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// File: NASDAQ0_private.h
//
// Code generated for Simulink model 'NASDAQ0'.
//
// Model version                  : 11.240
// Simulink Coder version         : 24.2 (R2024b) 21-Jun-2024
// C/C++ source code generated on : Tue May 12 11:57:35 2026
//
// Target selection: ert.tlc
// Embedded hardware selection: STMicroelectronics->ST10/Super10
// Code generation objectives:
//    1. Execution efficiency
//    2. Debugging
//    3. RAM efficiency
// Validation result: Not run
//
#ifndef NASDAQ0_private_h_
#define NASDAQ0_private_h_
#include <stdbool.h>
#include <stdint.h>
#include "NASDAQ0_types.h"
#include "dsp_rt.h"    // DSP System Toolbox general run time support functions

extern void LUf_boolfloatint32_t(float outU[], float outP[], int32_t N, bool
  outS[]);
extern void rt_mrdivide_U1f6x4_U2f4x4_Yf6x4(float u0[24], const float u1[16]);

#endif                                 // NASDAQ0_private_h_

//
// File trailer for generated code.
//
// [EOF]
//
