//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// File: NASDAQ0_data.cpp
//
// Code generated for Simulink model 'NASDAQ0'.
//
// Model version                  : 11.257
// Simulink Coder version         : 24.2 (R2024b) 21-Jun-2024
// C/C++ source code generated on : Sun May 24 17:30:06 2026
//
// Target selection: ert.tlc
// Embedded hardware selection: ARM Compatible->ARM Cortex-M
// Code generation objectives:
//    1. Execution efficiency
//    2. Debugging
//    3. RAM efficiency
// Validation result: Not run
//
#include "NASDAQ0.h"

// Block parameters (default storage)
NASDAQ0::P_NASDAQ0_T NASDAQ0::NASDAQ0_P{
  // Computed Parameter: UnitDelay1_InitialCondition
  //  Referenced by: '<S19>/Unit Delay1'

  {
    0.0F,                              // GroundTemperature
    0.0F                               // GroundPressure
  },

  // Expression: minCond
  //  Referenced by: '<S11>/NASCondLim'

  0.001,

  // Expression: flagADA
  //  Referenced by: '<S8>/Constant'

  1.0,

  // Expression: minCond
  //  Referenced by: '<S24>/NASCondLim'

  0.001,

  // Expression: minCond
  //  Referenced by: '<S38>/NASCondLim'

  0.001,

  // Expression: flagGPS
  //  Referenced by: '<S34>/Constant'

  1.0,

  // Computed Parameter: Memory_InitialCondition
  //  Referenced by: '<S8>/Memory'

  (0ULL),

  // Computed Parameter: Memory_InitialCondition_o
  //  Referenced by: '<S20>/Memory'

  (0ULL),

  // Computed Parameter: Memory_InitialCondition_oa
  //  Referenced by: '<S34>/Memory'

  (0ULL),

  // Computed Parameter: Constant_Value_g
  //  Referenced by: '<S16>/Constant'

  0.0F,

  // Computed Parameter: IdentityMatrix_IDMatrixData
  //  Referenced by: '<S18>/Identity Matrix'

  1.0F,

  // Computed Parameter: Constant_Value_an
  //  Referenced by: '<S7>/Constant'

  0.5F,

  // Computed Parameter: Constant_Value_f
  //  Referenced by: '<S13>/Constant'

  { 1.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 1.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F,
    0.0F, 1.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 1.0F, 0.0F, 0.0F, 0.0F, 0.0F,
    0.0F, 0.0F, 1.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 1.0F },

  // Computed Parameter: Gain_Gain
  //  Referenced by: '<S4>/Gain'

  -1.0F,

  // Computed Parameter: Constant_Value_h
  //  Referenced by: '<S29>/Constant'

  0.0F,

  // Computed Parameter: IdentityMatrix_IDMatrixData_b
  //  Referenced by: '<S31>/Identity Matrix'

  1.0F,

  // Computed Parameter: Constant_Value_c
  //  Referenced by: '<S26>/Constant'

  { 1.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 1.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F,
    0.0F, 1.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 1.0F, 0.0F, 0.0F, 0.0F, 0.0F,
    0.0F, 0.0F, 1.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 1.0F },

  // Computed Parameter: Constant_Value_n
  //  Referenced by: '<S23>/Constant'

  { 0.0F, 0.0F },

  // Computed Parameter: Gain3_Gain
  //  Referenced by: '<S23>/Gain3'

  0.0341423526F,

  // Computed Parameter: Gain4_Gain
  //  Referenced by: '<S23>/Gain4'

  0.0065F,

  // Computed Parameter: gR_Value
  //  Referenced by: '<S23>/g R'

  0.0341423526F,

  // Computed Parameter: Constant1_Value
  //  Referenced by: '<S23>/Constant1'

  { 0.0F, 0.0F, 0.0F },

  // Computed Parameter: Constant_Value_p
  //  Referenced by: '<S19>/Constant'

  100.0F,

  // Computed Parameter: Gain_Gain_d
  //  Referenced by: '<S25>/Gain'

  -1.0F,

  // Computed Parameter: HeightTemperatureGradient_Value
  //  Referenced by: '<S32>/HeightTemperatureGradient'

  0.0065F,

  // Computed Parameter: gravity_Value
  //  Referenced by: '<S32>/gravity'

  9.80066F,

  // Computed Parameter: Rair_Value
  //  Referenced by: '<S32>/R air'

  287.052856F,

  // Computed Parameter: Gain_Gain_dp
  //  Referenced by: '<S43>/Gain'

  0.0F,

  // Computed Parameter: Constant_Value_l
  //  Referenced by: '<S44>/Constant'

  0.0F,

  // Computed Parameter: IdentityMatrix_IDMatrixData_a
  //  Referenced by: '<S46>/Identity Matrix'

  { 1.0F, 0.0F, 0.0F, 0.0F, 0.0F, 1.0F, 0.0F, 0.0F, 0.0F, 0.0F, 1.0F, 0.0F, 0.0F,
    0.0F, 0.0F, 1.0F },

  // Computed Parameter: Constant_Value_p1
  //  Referenced by: '<S40>/Constant'

  { 1.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 1.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F,
    0.0F, 1.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 1.0F, 0.0F, 0.0F, 0.0F, 0.0F,
    0.0F, 0.0F, 1.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 1.0F },

  // Computed Parameter: Constant_Value_fg
  //  Referenced by: '<S37>/Constant'

  8.99823135E-6F,

  // Computed Parameter: Constant1_Value_f
  //  Referenced by: '<S37>/Constant1'

  0.0F,

  // Computed Parameter: Gain_Gain_e
  //  Referenced by: '<S37>/Gain'

  8.99823135E-6F,

  // Computed Parameter: Bias_Bias
  //  Referenced by: '<S37>/Bias'

  39.3892784F,

  // Computed Parameter: Gain1_Gain
  //  Referenced by: '<S41>/Gain1'

  0.0174532924F,

  // Computed Parameter: Constant3_Value
  //  Referenced by: '<S37>/Constant3'

  1.23816417E+10F,

  // Computed Parameter: Constant4_Value
  //  Referenced by: '<S37>/Constant4'

  1.0F,

  // Computed Parameter: Gain1_Gain_n
  //  Referenced by: '<S37>/Gain1'

  111412.875F,

  // Computed Parameter: Gain3_Gain_k
  //  Referenced by: '<S37>/Gain3'

  1000.0F,

  // Computed Parameter: Constant5_Value
  //  Referenced by: '<S37>/Constant5'

  { 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F },

  // Computed Parameter: Constant6_Value
  //  Referenced by: '<S37>/Constant6'

  { 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 1.0F, 0.0F, 0.0F, 1.0F, 0.0F, 0.0F },

  // Computed Parameter: Constant_Value_o
  //  Referenced by: '<S33>/Constant'

  { 0.0447F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0447F, 0.0F, 0.0F, 0.0F, 0.0F, 0.1F, 0.0F,
    0.0F, 0.0F, 0.0F, 0.1F },

  // Computed Parameter: Gain3_Gain_h
  //  Referenced by: '<S39>/Gain3'

  8.99823135E-6F,

  // Computed Parameter: Bias_Bias_l
  //  Referenced by: '<S39>/Bias'

  39.3892784F,

  // Computed Parameter: Gain1_Gain_d
  //  Referenced by: '<S47>/Gain1'

  0.0174532924F,

  // Computed Parameter: Gain4_Gain_l
  //  Referenced by: '<S39>/Gain4'

  111412.875F,

  // Computed Parameter: Bias1_Bias
  //  Referenced by: '<S39>/Bias1'

  -8.28804398F,

  // Computed Parameter: Gain_Gain_l
  //  Referenced by: '<S39>/Gain'

  { 1000.0F, 1000.0F, 1.0F, 1.0F },

  // Computed Parameter: Constant1_Value_j
  //  Referenced by: '<S3>/Constant1'

  { 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F,
    0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 1.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 1.0F,
    0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 1.0F, 0.0F, 0.0F, 0.0F },

  // Computed Parameter: Gain1_Gain_h
  //  Referenced by: '<S3>/Gain1'

  0.01F,

  // Computed Parameter: Bias_Bias_d
  //  Referenced by: '<S3>/Bias'

  { 1.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 1.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F,
    0.0F, 1.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 1.0F, 0.0F, 0.0F, 0.0F, 0.0F,
    0.0F, 0.0F, 1.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 1.0F },

  // Computed Parameter: Bias1_Bias_h
  //  Referenced by: '<S3>/Bias1'

  { 0.006F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.006F, 0.0F, 0.0F, 0.0F, 0.0F,
    0.0F, 0.0F, 0.75F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.02F, 0.0F, 0.0F,
    0.0F, 0.0F, 0.0F, 0.0F, 0.02F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.22F },

  // Computed Parameter: Gain_Gain_i
  //  Referenced by: '<S3>/Gain'

  0.01F,

  // Computed Parameter: UnitDelay_InitialCondition
  //  Referenced by: '<S1>/Unit Delay'

  0.0F,

  // Computed Parameter: UnitDelay1_InitialCondition_o
  //  Referenced by: '<S1>/Unit Delay1'

  0.0F,

  // Computed Parameter: Constant1_Value_f5
  //  Referenced by: '<S7>/Constant1'

  true,

  // Computed Parameter: Constant_Value_o1
  //  Referenced by: '<S20>/Constant'

  true,

  // Computed Parameter: UnitDelay2_InitialCondition
  //  Referenced by: '<S1>/Unit Delay2'

  -1,

  // Computed Parameter: Bias1_Bias_l
  //  Referenced by: '<S1>/Bias1'

  1,

  // Computed Parameter: Saturation1_UpperSat
  //  Referenced by: '<S1>/Saturation1'

  1,

  // Computed Parameter: Saturation1_LowerSat
  //  Referenced by: '<S1>/Saturation1'

  -1,

  // Computed Parameter: HMatrix_Value
  //  Referenced by: '<S7>/H Matrix'

  { 0U, 0U, 0U, 0U, 0U, 1U },

  // Computed Parameter: UnitDelay_InitialCondition_n
  //  Referenced by: '<S19>/Unit Delay'

  0U,

  // Computed Parameter: Switch_Threshold
  //  Referenced by: '<S19>/Switch'

  0U,

  // Computed Parameter: Bias_Bias_m
  //  Referenced by: '<S19>/Bias'

  1U,

  // Computed Parameter: Saturation_UpperSat
  //  Referenced by: '<S19>/Saturation'

  1U,

  // Computed Parameter: Saturation_LowerSat
  //  Referenced by: '<S19>/Saturation'

  0U,

  // Computed Parameter: UnitDelay3_InitialCondition
  //  Referenced by: '<S1>/Unit Delay3'

  0U,

  // Computed Parameter: Bias2_Bias
  //  Referenced by: '<S1>/Bias2'

  1U,

  // Computed Parameter: Saturation2_UpperSat
  //  Referenced by: '<S1>/Saturation2'

  1U,

  // Computed Parameter: Saturation2_LowerSat
  //  Referenced by: '<S1>/Saturation2'

  0U,

  // Start of '<S24>/No correction'
  {
    // Computed Parameter: Gain_Gain
    //  Referenced by: '<S28>/Gain'

    0.0F
  }
  ,

  // End of '<S24>/No correction'

  // Start of '<S11>/No correction'
  {
    // Computed Parameter: Gain_Gain
    //  Referenced by: '<S15>/Gain'

    0.0F
  }
  // End of '<S11>/No correction'
};

//
// File trailer for generated code.
//
// [EOF]
//
