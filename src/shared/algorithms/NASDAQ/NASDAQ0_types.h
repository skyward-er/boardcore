//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// File: NASDAQ0_types.h
//
// Code generated for Simulink model 'NASDAQ0'.
//
// Model version                  : 11.128
// Simulink Coder version         : 24.2 (R2024b) 21-Jun-2024
// C/C++ source code generated on : Tue Mar 24 12:53:07 2026
//
// Target selection: ert.tlc
// Embedded hardware selection: STMicroelectronics->ST10/Super10
// Code generation objectives:
//    1. Execution efficiency
//    2. Debugging
//    3. RAM efficiency
// Validation result: Passed (3), Warning (1), Error (0)
//
#ifndef NASDAQ0_types_h_
#define NASDAQ0_types_h_
#include <stdint.h>
#ifndef DEFINED_TYPEDEF_FOR_Bus_AdaState_
#define DEFINED_TYPEDEF_FOR_Bus_AdaState_

struct Bus_AdaState
{
  float covariance[9];
  float verticalSpeedCovariance;
  float timestamp;
  float mslAltitude;
  float aglAltitude;
  float verticalSpeed;
  float x0;
  float x1;
  float x2;
  uint8_t apogeeCounter;
  uint8_t parachuteCounter;
};

#endif

#ifndef DEFINED_TYPEDEF_FOR_Bus_Baro_
#define DEFINED_TYPEDEF_FOR_Bus_Baro_

struct Bus_Baro
{
  float Measure;
  uint64_t Timestamp;
};

#endif

#ifndef DEFINED_TYPEDEF_FOR_Bus_GPS_
#define DEFINED_TYPEDEF_FOR_Bus_GPS_

struct Bus_GPS
{
  float Measure[10];
  uint64_t Timestamp;
};

#endif

#ifndef DEFINED_TYPEDEF_FOR_struct_yJUZSwkrTKNshoCnRn6b2E_
#define DEFINED_TYPEDEF_FOR_struct_yJUZSwkrTKNshoCnRn6b2E_

struct struct_yJUZSwkrTKNshoCnRn6b2E
{
  double GPS;
  double baro;
  double ADA;
  double ADAdynamicR;
};

#endif

#ifndef DEFINED_TYPEDEF_FOR_struct_sE07iPeH2fcFuyDctof9lB_
#define DEFINED_TYPEDEF_FOR_struct_sE07iPeH2fcFuyDctof9lB_

struct struct_sE07iPeH2fcFuyDctof9lB
{
  double predictor;
  double GPS;
  double baro;
  double ADA;
};

#endif

#ifndef DEFINED_TYPEDEF_FOR_struct_7PVAQObeoH08VqIRAhlRKD_
#define DEFINED_TYPEDEF_FOR_struct_7PVAQObeoH08VqIRAhlRKD_

struct struct_7PVAQObeoH08VqIRAhlRKD
{
  double GPS[16];
  double baro;
  double ADA;
};

#endif

#ifndef DEFINED_TYPEDEF_FOR_struct_WmcIulKf1n1h4Ys2kSojGD_
#define DEFINED_TYPEDEF_FOR_struct_WmcIulKf1n1h4Ys2kSojGD_

struct struct_WmcIulKf1n1h4Ys2kSojGD
{
  double P0[36];
  double Q[36];
};

#endif

#ifndef DEFINED_TYPEDEF_FOR_struct_8YttkCbEoudiA6ENJJVZ3C_
#define DEFINED_TYPEDEF_FOR_struct_8YttkCbEoudiA6ENJJVZ3C_

struct struct_8YttkCbEoudiA6ENJJVZ3C
{
  double a;
  double b;
  double lat0;
  double lon0;
  double accLimit;
};

#endif

#ifndef DEFINED_TYPEDEF_FOR_struct_9Ht3NaUwqaCNU0KBZn3mBF_
#define DEFINED_TYPEDEF_FOR_struct_9Ht3NaUwqaCNU0KBZn3mBF_

struct struct_9Ht3NaUwqaCNU0KBZn3mBF
{
  double a;
  double n;
  double refTemp;
  double refPres;
};

#endif

#ifndef DEFINED_TYPEDEF_FOR_struct_UyXIlhvkjCvDDDjTjsnNlD_
#define DEFINED_TYPEDEF_FOR_struct_UyXIlhvkjCvDDDjTjsnNlD_

struct struct_UyXIlhvkjCvDDDjTjsnNlD
{
  struct_yJUZSwkrTKNshoCnRn6b2E flags;
  struct_sE07iPeH2fcFuyDctof9lB frequency;
  struct_7PVAQObeoH08VqIRAhlRKD sigma;
  struct_WmcIulKf1n1h4Ys2kSojGD initMatrix;
  struct_8YttkCbEoudiA6ENJJVZ3C gps;
  struct_9Ht3NaUwqaCNU0KBZn3mBF baro;
};

#endif
#endif                                 // NASDAQ0_types_h_

//
// File trailer for generated code.
//
// [EOF]
//
