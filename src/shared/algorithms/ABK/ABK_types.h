//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// File: ABK_types.h
//
// Code generated for Simulink model 'ABK'.
//
// Model version                  : 11.280
// Simulink Coder version         : 24.2 (R2024b) 21-Jun-2024
// C/C++ source code generated on : Mon Jun  1 12:33:18 2026
//
// Target selection: ert.tlc
// Embedded hardware selection: ARM Compatible->ARM Cortex-M
// Code generation objectives:
//    1. Execution efficiency
//    2. Debugging
//    3. RAM efficiency
// Validation result: Not run
//
#ifndef ABK_types_h_
#define ABK_types_h_
#include <stdint.h>
#include <stdbool.h>
#ifndef DEFINED_TYPEDEF_FOR_ABKLogs_
#define DEFINED_TYPEDEF_FOR_ABKLogs_

// Output Structure for ABK GNC Algorithm used for OBSW Logging
struct ABKLogs
{
  float ABKCommand;
  uint64_t Timestamp;
  float FilterCoefficient;
  float PrePIDCommand;
  float PostPIDCommand;
  bool BypassActivation;
};

#endif

#ifndef DEFINED_TYPEDEF_FOR_ABKIn_
#define DEFINED_TYPEDEF_FOR_ABKIn_

// Input Structure for ABK GNC Algorithm
struct ABKIn
{
  float MEAMass;
  float ANASPosition[3];
  float ANASVelocity[3];
};

#endif
#endif                                 // ABK_types_h_

//
// File trailer for generated code.
//
// [EOF]
//
