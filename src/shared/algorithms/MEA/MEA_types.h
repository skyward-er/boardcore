//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// File: MEA_types.h
//
// Code generated for Simulink model 'MEA'.
//
// Model version                  : 11.238
// Simulink Coder version         : 24.2 (R2024b) 21-Jun-2024
// C/C++ source code generated on : Mon May 11 01:43:03 2026
//
// Target selection: ert.tlc
// Embedded hardware selection: STMicroelectronics->ST10/Super10
// Code generation objectives:
//    1. Execution efficiency
//    2. Debugging
//    3. RAM efficiency
// Validation result: Not run
//
#ifndef MEA_types_h_
#define MEA_types_h_
#include <stdint.h>
#ifndef DEFINED_TYPEDEF_FOR_MEAIn_
#define DEFINED_TYPEDEF_FOR_MEAIn_

namespace MEA {
// Input Structure for MEA GNC Algorithm
struct MEAIn
{
  float CCPTMeasure;
  uint64_t CCPTTimestamp;
  float MainValvePosition;
  uint8_t FSMState;
};

#endif

#ifndef DEFINED_TYPEDEF_FOR_MEAOut_
#define DEFINED_TYPEDEF_FOR_MEAOut_

// Output Stucture for MEA GNC Algorithm
struct MEAOut
{
  uint64_t Timestamp;
  float Mass;
};

#endif

#ifndef DEFINED_TYPEDEF_FOR_MEALogs_
#define DEFINED_TYPEDEF_FOR_MEALogs_

// Output Structure for MEA GNC Algorithm used for OBSW Logging
struct MEALogs
{
  uint64_t Timestamp;
  float Mass;
  float States[2];
  float Pressure;
};
}
#endif
#endif                                 // MEA_types_h_

//
// File trailer for generated code.
//
// [EOF]
//
