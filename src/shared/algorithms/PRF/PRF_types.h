//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// File: PRF_types.h
//
// Code generated for Simulink model 'PRF'.
//
// Model version                  : 11.335
// Simulink Coder version         : 24.2 (R2024b) 21-Jun-2024
// C/C++ source code generated on : Wed Jul 22 13:17:22 2026
//
// Target selection: ert.tlc
// Embedded hardware selection: ARM Compatible->ARM Cortex-M
// Code generation objectives:
//    1. Execution efficiency
//    2. Debugging
//    3. RAM efficiency
// Validation result: Not run
//
#ifndef PRF_types_h_
#define PRF_types_h_
#include <stdint.h>
#ifndef DEFINED_TYPEDEF_FOR_PRFReference_
#define DEFINED_TYPEDEF_FOR_PRFReference_

struct PRFReference
{
  float WindDirection;
  float TargetPositionLLA[2];
};

#endif

#ifndef DEFINED_TYPEDEF_FOR_PRFIn_
#define DEFINED_TYPEDEF_FOR_PRFIn_

// Input Structure for PRF GNC Algorithm
struct PRFIn
{
  float NASDAQPosition[3];
  float NASDAQVelocity[3];
};

#endif

#ifndef DEFINED_TYPEDEF_FOR_PRFLogs_
#define DEFINED_TYPEDEF_FOR_PRFLogs_

// Output Structure for PRF Controller GNC Algorithm used for OBSW Logging
struct PRFLogs
{
  float Q1[2];
  float Q2[2];
  float TerminalTarget[4];
  uint8_t TargetIndex;
  float Heading;
  float Reference;
  float ServoCommands[2];
  float WindHeading;
};

#endif
#endif                                 // PRF_types_h_

//
// File trailer for generated code.
//
// [EOF]
//
