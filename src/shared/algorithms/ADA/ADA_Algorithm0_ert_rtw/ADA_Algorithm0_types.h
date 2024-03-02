//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// File: ADA_Algorithm0_types.h
//
// Code generated for Simulink model 'ADA_Algorithm0'.
//
// Model version                  : 1.11
// Simulink Coder version         : 23.2 (R2023b) 01-Aug-2023
// C/C++ source code generated on : Wed Nov  1 00:43:29 2023
//
// Target selection: ert.tlc
// Embedded hardware selection: STMicroelectronics->ST10/Super10
// Code generation objectives:
//    1. Execution efficiency
//    2. Safety precaution
// Validation result: Passed (20), Warning (1), Error (0)
//
#ifndef RTW_HEADER_ADA_Algorithm0_types_h_
#define RTW_HEADER_ADA_Algorithm0_types_h_
#ifndef DEFINED_TYPEDEF_FOR_ADAState_
#define DEFINED_TYPEDEF_FOR_ADAState_

struct ADAState
{
  float timestamp;
  float mslAltitude;
  float aglAltitude;
  float verticalSpeed;
  float x0;
  float x1;
  float x2;
};

#endif
#endif                                 // RTW_HEADER_ADA_Algorithm0_types_h_

//
// File trailer for generated code.
//
// [EOF]
//
