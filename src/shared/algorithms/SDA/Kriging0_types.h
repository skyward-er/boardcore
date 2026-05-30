//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// File: Kriging0_types.h
//
// Code generated for Simulink model 'Kriging0'.
//
// Model version                  : 11.257
// Simulink Coder version         : 24.2 (R2024b) 21-Jun-2024
// C/C++ source code generated on : Sun May 24 16:00:49 2026
//
// Target selection: ert.tlc
// Embedded hardware selection: ARM Compatible->ARM Cortex-M
// Code generation objectives:
//    1. Execution efficiency
//    2. Debugging
//    3. RAM efficiency
// Validation result: Not run
//
#ifndef Kriging0_types_h_
#define Kriging0_types_h_
#include <stdbool.h>
#include <stdint.h>
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

#ifndef DEFINED_TYPEDEF_FOR_SDALogs_
#define DEFINED_TYPEDEF_FOR_SDALogs_

// Output Structure for SDA GNC Algorithm used for OBSW Logging
struct SDALogs
{
  bool ShutdownCommand;
  uint64_t Timestamp;
  uint8_t ShutdownCounter;
  float Apogee[2];
};

#endif

#ifndef DEFINED_TYPEDEF_FOR_struct_6Rnfmp5FIUwEnZoyXFhchB_
#define DEFINED_TYPEDEF_FOR_struct_6Rnfmp5FIUwEnZoyXFhchB_

struct struct_6Rnfmp5FIUwEnZoyXFhchB
{
  float x[1600];
  float mu;
  float theta[1600];
  float p[1600];
  float psiV[320];
  float yMu;
  float ySigma;
  float isGEK;
  float dyDirs[5];
  float ranges[10];
};

#endif
#endif                                 // Kriging0_types_h_

//
// File trailer for generated code.
//
// [EOF]
//
