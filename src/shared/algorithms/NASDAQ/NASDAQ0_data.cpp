//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// File: NASDAQ0_data.cpp
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
#include "NASDAQ0.h"

// Block parameters (default storage)
NASDAQ0::P_NASDAQ0_T NASDAQ0::NASDAQ0_P{
  // Variable: nasdaq
  //  Referenced by:
  //    '<S8>/Constant'
  //    '<S24>/Constant'

  {
    {
      1.0,
      1.0,
      1.0,
      1.0
    },

    {
      100.0,
      10.0,
      50.0,
      50.0
    },

    {
      { 0.0447, 0.0, 0.0, 0.0, 0.0, 0.0447, 0.0, 0.0, 0.0, 0.0, 0.0183, 0.0, 0.0,
        0.0, 0.0, 0.0183 },
      100.0,
      0.5
    },

    {
      { 0.1, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.1, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
        0.1, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.1, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
        0.1, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.1 },

      { 0.006, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.006, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
        0.75, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.02, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
        0.02, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.22 }
    },

    {
      111132.95225,
      111412.87733,
      39.38928,
      -8.288044,
      34.0
    },

    {
      0.0065,
      5.2559,
      288.15,
      101325.0
    }
  },

  // Computed Parameter: Memory_InitialCondition
  //  Referenced by: '<S15>/Memory'

  (0ULL),

  // Computed Parameter: Memory_InitialCondition_b
  //  Referenced by: '<S24>/Memory'

  (0ULL),

  // Expression: nasdaq.sigma.ADA
  //  Referenced by: '<S7>/Constant'

  0.5F,

  // Computed Parameter: Constant_Value_g
  //  Referenced by: '<S13>/Constant'

  { 1.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 1.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F,
    0.0F, 1.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 1.0F, 0.0F, 0.0F, 0.0F, 0.0F,
    0.0F, 0.0F, 1.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 1.0F },

  // Computed Parameter: Gain_Gain
  //  Referenced by: '<S4>/Gain'

  -1.0F,

  // Computed Parameter: Memory_InitialCondition_l
  //  Referenced by: '<S8>/Memory'

  0.0F,

  // Computed Parameter: Constant_Value_h
  //  Referenced by: '<S21>/Constant'

  { 1.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 1.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F,
    0.0F, 1.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 1.0F, 0.0F, 0.0F, 0.0F, 0.0F,
    0.0F, 0.0F, 1.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 1.0F },

  // Computed Parameter: Constant_Value_j
  //  Referenced by: '<S18>/Constant'

  { 0.0F, 0.0F },

  // Computed Parameter: StandardAirPressureP0_Value
  //  Referenced by: '<S18>/StandardAirPressure P0'

  99417.6F,

  // Computed Parameter: Gain3_Gain
  //  Referenced by: '<S18>/Gain3'

  9.80261F,

  // Computed Parameter: Gain4_Gain
  //  Referenced by: '<S18>/Gain4'

  0.0065F,

  // Computed Parameter: StandardAirTemperatureT0_Value
  //  Referenced by: '<S18>/StandardAirTemperature T0'

  287.11F,

  // Computed Parameter: gR_Value
  //  Referenced by: '<S18>/g R'

  0.0341423526F,

  // Computed Parameter: Constant1_Value
  //  Referenced by: '<S18>/Constant1'

  { 0.0F, 0.0F, 0.0F },

  // Expression: nasdaq.sigma.baro
  //  Referenced by: '<S14>/Constant'

  100.0F,

  // Computed Parameter: StandardAirPressureP0_Value_a
  //  Referenced by: '<S22>/StandardAirPressure P0'

  99417.6F,

  // Computed Parameter: StandardAirTemperatureT0_Valu_o
  //  Referenced by: '<S22>/StandardAirTemperature T0'

  287.11F,

  // Computed Parameter: Gain_Gain_j
  //  Referenced by: '<S20>/Gain'

  -1.0F,

  // Expression: nasdaq.baro.a
  //  Referenced by: '<S22>/HeightTemperatureGradient'

  0.0065F,

  // Computed Parameter: gravity_Value
  //  Referenced by: '<S22>/gravity'

  9.80066F,

  // Computed Parameter: Rair_Value
  //  Referenced by: '<S22>/R air'

  287.052856F,

  // Computed Parameter: Constant_Value_a
  //  Referenced by: '<S30>/Constant'

  { 1.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 1.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F,
    0.0F, 1.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 1.0F, 0.0F, 0.0F, 0.0F, 0.0F,
    0.0F, 0.0F, 1.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 1.0F },

  // Computed Parameter: Constant_Value_c
  //  Referenced by: '<S27>/Constant'

  8.99823135E-6F,

  // Computed Parameter: Constant1_Value_e
  //  Referenced by: '<S27>/Constant1'

  0.0F,

  // Computed Parameter: Gain_Gain_a
  //  Referenced by: '<S27>/Gain'

  8.99823135E-6F,

  // Expression: nasdaq.gps.lat0
  //  Referenced by: '<S27>/Bias'

  39.3892784F,

  // Computed Parameter: Gain1_Gain
  //  Referenced by: '<S31>/Gain1'

  0.0174532924F,

  // Computed Parameter: Constant3_Value
  //  Referenced by: '<S27>/Constant3'

  1.23816417E+10F,

  // Computed Parameter: Constant4_Value
  //  Referenced by: '<S27>/Constant4'

  1.0F,

  // Expression: nasdaq.gps.b
  //  Referenced by: '<S27>/Gain1'

  111412.875F,

  // Computed Parameter: Gain3_Gain_b
  //  Referenced by: '<S27>/Gain3'

  1000.0F,

  // Computed Parameter: Constant5_Value
  //  Referenced by: '<S27>/Constant5'

  { 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F },

  // Computed Parameter: Constant6_Value
  //  Referenced by: '<S27>/Constant6'

  { 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 1.0F, 0.0F, 0.0F, 1.0F, 0.0F, 0.0F },

  // Expression: nasdaq.sigma.GPS
  //  Referenced by: '<S23>/Constant'

  { 0.0447F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0447F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0183F,
    0.0F, 0.0F, 0.0F, 0.0F, 0.0183F },

  // Computed Parameter: Gain3_Gain_g
  //  Referenced by: '<S29>/Gain3'

  8.99823135E-6F,

  // Expression: nasdaq.gps.lat0
  //  Referenced by: '<S29>/Bias'

  39.3892784F,

  // Computed Parameter: Gain1_Gain_g
  //  Referenced by: '<S32>/Gain1'

  0.0174532924F,

  // Expression: nasdaq.gps.b
  //  Referenced by: '<S29>/Gain4'

  111412.875F,

  // Expression: nasdaq.gps.lon0
  //  Referenced by: '<S29>/Bias1'

  -8.28804398F,

  // Computed Parameter: Gain_Gain_jx
  //  Referenced by: '<S29>/Gain'

  { 1000.0F, 1000.0F, 1.0F, 1.0F },

  // Computed Parameter: Constant1_Value_o
  //  Referenced by: '<S3>/Constant1'

  { 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F,
    0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 1.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 1.0F,
    0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 1.0F, 0.0F, 0.0F, 0.0F },

  // Computed Parameter: Gain1_Gain_g2
  //  Referenced by: '<S3>/Gain1'

  0.01F,

  // Computed Parameter: Bias_Bias_l
  //  Referenced by: '<S3>/Bias'

  { 1.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 1.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F,
    0.0F, 1.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 1.0F, 0.0F, 0.0F, 0.0F, 0.0F,
    0.0F, 0.0F, 1.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 1.0F },

  // Expression: nasdaq.initMatrix.Q
  //  Referenced by: '<S3>/Bias1'

  { 0.006F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.006F, 0.0F, 0.0F, 0.0F, 0.0F,
    0.0F, 0.0F, 0.75F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.02F, 0.0F, 0.0F,
    0.0F, 0.0F, 0.0F, 0.0F, 0.02F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.22F },

  // Computed Parameter: Gain_Gain_b
  //  Referenced by: '<S3>/Gain'

  0.01F,

  // Expression: init.NASVariance
  //  Referenced by: '<S1>/NAS Variance Interface'

  { 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F,
    0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F,
    0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F },

  // Computed Parameter: NASStateInterface_InitialCondit
  //  Referenced by: '<S1>/NAS State Interface'

  { 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F },

  // Expression: nasdaq.flags.ADAdynamicR
  //  Referenced by: '<S7>/Constant1'

  true,

  // Expression: nasdaq.flags.baro
  //  Referenced by: '<S15>/Constant'

  true,

  // Computed Parameter: HMatrix_Value
  //  Referenced by: '<S7>/H Matrix'

  { 0U, 0U, 0U, 0U, 0U, 1U }
};

//
// File trailer for generated code.
//
// [EOF]
//
