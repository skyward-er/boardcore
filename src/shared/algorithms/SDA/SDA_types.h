//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// File: SDA_types.h
//
// Code generated for Simulink model 'SDA'.
//
// Model version                  : 11.369
// Simulink Coder version         : 24.2 (R2024b) 21-Jun-2024
// C/C++ source code generated on : Thu Jul 16 14:00:32 2026
//
// Target selection: ert.tlc
// Embedded hardware selection: ARM Compatible->ARM Cortex-M
// Code generation objectives:
//    1. Execution efficiency
//    2. Debugging
//    3. RAM efficiency
// Validation result: Not run
//
#ifndef SDA_types_h_
#define SDA_types_h_
#include <stdbool.h>
#include <stdint.h>
#ifndef DEFINED_TYPEDEF_FOR_SDALogs_
#define DEFINED_TYPEDEF_FOR_SDALogs_

// Output Structure for SDA GNC Algorithm used for OBSW Logging
struct SDALogs
{
  bool ShutdownCommand;
  uint64_t Timestamp;
  uint8_t ShutdownCounter;
  float Apogee[3];
};

#endif

#ifndef DEFINED_TYPEDEF_FOR_SDAIn_
#define DEFINED_TYPEDEF_FOR_SDAIn_

// Input Structure for SDA GNC Algorithm
struct SDAIn
{
  float MEAMass;
  float ANASPosition[3];
  float ANASVelocity[3];
};

#endif
#endif                                 // SDA_types_h_

//
// File trailer for generated code.
//
// [EOF]
//
