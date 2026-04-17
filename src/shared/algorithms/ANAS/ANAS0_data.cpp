//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// File: ANAS0_data.cpp
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
#include "ANAS0.h"

// Block parameters (default storage)
ANAS0::P_ANAS0_T ANAS0::ANAS0_P{
  // Computed Parameter: Zero2_Value
  //  Referenced by: '<S1>/Zero2'

  {
    {
      0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F,
      0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F,
      0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F }
    // LinearCovariance
  },

  // Computed Parameter: Zero1_Value
  //  Referenced by: '<S1>/Zero1'

  {
    {
      0.0F, 0.0F, 0.0F }
    ,                                  // Velocity

    {
      0.0F, 0.0F, 0.0F, 0.0F }
    ,                                  // Quaternion

    {
      0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F }
    ,                                  // Covariance
    0,                                 // BaroActivation
    false,                             // GPSActivation
    false,                             // MagActivation
    false,                             // AccActivation

    {
      0.0F, 0.0F, 0.0F }
    // Position
  },

  // Computed Parameter: Zero_Value
  //  Referenced by: '<S1>/Zero'

  {
    {
      0.0F, 0.0F, 0.0F }
    ,                                  // Position

    {
      0.0F, 0.0F, 0.0F }
    ,                                  // Velocity

    {
      0.0F, 0.0F, 0.0F, 0.0F }
    ,                                  // Quaternion
    0U                                 // Timestamp
  }
};

//
// File trailer for generated code.
//
// [EOF]
//
