//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// File: SDA_data.cpp
//
// Code generated for Simulink model 'SDA'.
//
// Model version                  : 11.329
// Simulink Coder version         : 24.2 (R2024b) 21-Jun-2024
// C/C++ source code generated on : Mon Jun 29 14:46:21 2026
//
// Target selection: ert.tlc
// Embedded hardware selection: ARM Compatible->ARM Cortex-M
// Code generation objectives:
//    1. Execution efficiency
//    2. Debugging
//    3. RAM efficiency
// Validation result: Not run
//
#include "SDA.h"

namespace SDA 
{


// Block parameters (default storage)
SDA::P_SDA_T SDA::SDA_P{
  // Mask Parameter: CompareToConstant2_const
  //  Referenced by: '<S2>/Constant'

  3100.0F,

  // Mask Parameter: CompareToConstant3_const
  //  Referenced by: '<S3>/Constant'

  5U,

  // Expression: single(zeros(predSamples,1))
  //  Referenced by: '<S9>/prediction_samples'

  { 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F,
    0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F,
    0.0F, 0.0F, 0.0F, 0.0F },

  // Expression: single(zeros(predSamples,1))
  //  Referenced by: '<S9>/time_samples'

  { 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F,
    0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F,
    0.0F, 0.0F, 0.0F, 0.0F },

  // Expression: single(0)
  //  Referenced by: '<S9>/time_samples1'

  0.0F,

  // Computed Parameter: Gain2_Gain
  //  Referenced by: '<S9>/Gain2'

  0.02F,

  // Computed Parameter: Gain1_Gain
  //  Referenced by: '<S9>/Gain1'

  30.0F,

  // Computed Parameter: Gain_Gain
  //  Referenced by: '<S9>/Gain'

  30.0F,

  // Computed Parameter: Constant15_Value
  //  Referenced by: '<S7>/Constant15'

  0.3F,

  // Computed Parameter: Constant_Value
  //  Referenced by: '<S9>/Constant'

  1.0F,

  // Computed Parameter: Zero_Value
  //  Referenced by: '<S4>/Zero'

  0.0F,

  // Computed Parameter: Constant_Value_d
  //  Referenced by: '<S1>/Constant'

  { -4560.37744F, -101.033333F, -363.592834F, -6.66133815E-16F, 3.73489189F,
    196.812302F, -0.00145337696F, 1.5361377F, 25.0F, 40.0F },

  // Computed Parameter: Gain_Gain_m
  //  Referenced by: '<S6>/Gain'

  -1.0F,

  // Computed Parameter: Bias_Bias
  //  Referenced by: '<S6>/Bias'

  0.122913599F,

  // Computed Parameter: Gain2_Gain_i
  //  Referenced by: '<S6>/Gain2'

  1011.07538F,

  // Computed Parameter: Bias1_Bias
  //  Referenced by: '<S6>/Bias1'

  2589.15894F,

  // Expression: transFlag
  //  Referenced by: '<S1>/Constant2'

  true,

  // Computed Parameter: One_Value
  //  Referenced by: '<S1>/One'

  1,

  // Computed Parameter: Gain1_Gain_g
  //  Referenced by: '<S1>/Gain1'

  -1,

  // Computed Parameter: Gain3_Gain
  //  Referenced by: '<S7>/Gain3'

  164U,

  // Computed Parameter: Constant12_Value
  //  Referenced by: '<S7>/Constant12'

  0U,

  // Computed Parameter: Constant11_Value
  //  Referenced by: '<S7>/Constant11'

  5U,

  // Computed Parameter: Switch4_Threshold
  //  Referenced by: '<S7>/Switch4'

  0U,

  // Computed Parameter: UnitDelay_InitialCondition
  //  Referenced by: '<S1>/Unit Delay'

  0U,

  // Computed Parameter: Saturation_UpperSat
  //  Referenced by: '<S1>/Saturation'

  5U,

  // Computed Parameter: Saturation_LowerSat
  //  Referenced by: '<S1>/Saturation'

  0U
};


} // fine namespace SDA

//
// File trailer for generated code.
//
// [EOF]
//
