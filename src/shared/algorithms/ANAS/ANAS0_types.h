//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// File: ANAS0_types.h
//
// Code generated for Simulink model 'ANAS0'.
//
// Model version                  : 11.183
// Simulink Coder version         : 24.2 (R2024b) 21-Jun-2024
// C/C++ source code generated on : Thu Apr 16 13:46:31 2026
//
// Target selection: ert.tlc
// Embedded hardware selection: STMicroelectronics->ST10/Super10
// Code generation objectives:
//    1. Execution efficiency
//    2. Debugging
//    3. RAM efficiency
// Validation result: Passed (3), Warning (1), Error (0)
//
#ifndef ANAS0_types_h_
#define ANAS0_types_h_
#include <stdbool.h>
#include <stdint.h>
#ifndef DEFINED_TYPEDEF_FOR_NASIn_
#define DEFINED_TYPEDEF_FOR_NASIn_

struct NASIn
{
    float AccMeasure[3];
    uint32_t AccTimestamp;
    float GyroMeasure[3];
    uint32_t GyroTimestamp;
    float BaroMeasure;
    uint32_t BaroTimestamp;
    float GPSMeasure[4];
    uint32_t GPSTimestamp;
    float GPSHorizAccuracy;
    float GPSVertAccuracy;
    float PitotMeasure[2];
    uint32_t PitotTimestamp;
    float MagMeasure[3];
    uint32_t MagTimestamp;
    float ABKCommand;
    bool PinDetach;
};

#endif

#ifndef DEFINED_TYPEDEF_FOR_NASOut_
#define DEFINED_TYPEDEF_FOR_NASOut_

struct NASOut
{
    float Position[3];
    float Velocity[3];
    float Quaternion[4];
    uint32_t Timestamp;
};

#endif

#ifndef DEFINED_TYPEDEF_FOR_NASLogs_
#define DEFINED_TYPEDEF_FOR_NASLogs_

struct NASLogs
{
    float Velocity[3];
    float Quaternion[4];
    float Covariance[9];
    int8_t BaroActivation;
    bool GPSActivation;
    bool MagActivation;
    bool AccActivation;
    float Position[3];
};

#endif

#ifndef DEFINED_TYPEDEF_FOR_NASFinal_
#define DEFINED_TYPEDEF_FOR_NASFinal_

struct NASFinal
{
    float LinearCovariance[36];
};

#endif
#endif  // ANAS0_types_h_

//
// File trailer for generated code.
//
// [EOF]
//
