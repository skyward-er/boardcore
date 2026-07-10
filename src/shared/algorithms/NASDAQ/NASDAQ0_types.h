//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// File: NASDAQ0_types.h
//
// Code generated for Simulink model 'NASDAQ0'.
//
// Model version                  : 11.328
// Simulink Coder version         : 24.2 (R2024b) 21-Jun-2024
// C/C++ source code generated on : Mon Jun 29 09:37:08 2026
//
// Target selection: ert.tlc
// Embedded hardware selection: ARM Compatible->ARM Cortex-M
// Code generation objectives:
//    1. Execution efficiency
//    2. Debugging
//    3. RAM efficiency
// Validation result: Not run
//
#ifndef NASDAQ0_types_h_
#define NASDAQ0_types_h_
#include <stdint.h>
#include <stdbool.h>
#ifndef DEFINED_TYPEDEF_FOR_ANAS_NASDAQ_
#define DEFINED_TYPEDEF_FOR_ANAS_NASDAQ_

// Output Structure for ANAS GNC Algorithm used for NASDAQ GNC Algorithm initialization 
struct ANAS_NASDAQ
{
  float LinearCovariance[36];
  float Position[3];
  float Velocity[3];
};

#endif

#ifndef DEFINED_TYPEDEF_FOR_NASDAQOut_
#define DEFINED_TYPEDEF_FOR_NASDAQOut_

// Output Stucture for NASDAQ GNC Algorithm
struct NASDAQOut
{
  uint64_t Timestamp;
  float Position[3];
  float Velocity[3];
};

#endif

#ifndef DEFINED_TYPEDEF_FOR_NASDAQLogs_
#define DEFINED_TYPEDEF_FOR_NASDAQLogs_

// Output Structure for NASDAQ GNC Algorithm used for OBSW Logging
struct NASDAQLogs
{
  uint64_t Timestamp;
  float Position[3];
  float Velocity[3];
  float CovMatD[6];
  bool BaroActivation;
  bool GPSAct;
  bool ADAActovation;
};

#endif

#ifndef DEFINED_TYPEDEF_FOR_NASDAQInADA_
#define DEFINED_TYPEDEF_FOR_NASDAQInADA_

// ADA Input Structure for NASDAQ GNC Algorithm
struct NASDAQInADA
{
  float VerticalSpeed;
  float VerticalSpeedCovariance;
  uint64_t Timestamp;
};

#endif

#ifndef DEFINED_TYPEDEF_FOR_NASDAQInSensors_
#define DEFINED_TYPEDEF_FOR_NASDAQInSensors_

// Sensors Input Structure for NASDAQ GNC Algorithm
struct NASDAQInSensors
{
  float BaroMeasure;
  uint64_t BaroTimestamp;
  float GPSMeasure[4];
  uint64_t GPSTimestamp;
};

#endif

#ifndef DEFINED_TYPEDEF_FOR_NASDAQReference_
#define DEFINED_TYPEDEF_FOR_NASDAQReference_

// ISA Reference Input Structure for NASDAQ GNC Algorithm
struct NASDAQReference
{
  float GroundTemperature;
  float GroundPressure;
};

#endif
#endif                                 // NASDAQ0_types_h_

//
// File trailer for generated code.
//
// [EOF]
//
