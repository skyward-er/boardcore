//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// File: ANAS0_types.h
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
#ifndef ANAS0_types_h_
#define ANAS0_types_h_
#include <stdint.h>
#include <stdbool.h>
#ifndef DEFINED_TYPEDEF_FOR_ANASReference_
#define DEFINED_TYPEDEF_FOR_ANASReference_

// ISA Reference and Initial States Setter Input Structure for NASDAQ GNC Algorithm 
struct ANASReference
{
  float GroundTemperature;
  float GroundPressure;
  float InitialPosition[3];
  float InitialVelocity[3];
  float InitialQuaternion[4];
};

#endif

#ifndef DEFINED_TYPEDEF_FOR_ANASOut_
#define DEFINED_TYPEDEF_FOR_ANASOut_

// Output Stucture for ANAS GNC Algorithm
struct ANASOut
{
  uint64_t Timestamp;
  float Position[3];
  float Velocity[3];
  float Quaternion[4];
};

#endif

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

#ifndef DEFINED_TYPEDEF_FOR_ANASLogs_
#define DEFINED_TYPEDEF_FOR_ANASLogs_

// Output Structure for ANAS GNC Algorithm used for OBSW Logging
struct ANASLogs
{
  uint64_t Timestamp;
  float Position[3];
  float Velocity[3];
  float Quaternion[4];
  float CovarianceMatrixDiagonal[6];
  uint8_t BaroPitotActivation;
  bool GPSActivation;
  bool MagActivation;
  bool AccActivation;
};

#endif

#ifndef DEFINED_TYPEDEF_FOR_ANASIn_
#define DEFINED_TYPEDEF_FOR_ANASIn_

// Input Structure for ANAS GNC Algorithm
struct ANASIn
{
  float AccMeasure[3];
  uint64_t AccTimestamp;
  float GyroMeasure[3];
  uint64_t GyroTimestamp;
  float BaroMeasure;
  uint64_t BaroTimestamp;
  float GPSMeasure[4];
  float GPSHorizontalPrecision;
  float GPSSpeedPrecision;
  uint64_t GPSTimestamp;
  float PitotMeasure[2];
  uint64_t PitotTimestamp;
  float MagMeasure[3];
  uint64_t MagTimestamp;
  float ABKCommand;
  bool FlyingState;
};

#endif
#endif                                 // ANAS0_types_h_

//
// File trailer for generated code.
//
// [EOF]
//
