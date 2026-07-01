//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// File: PRF_data.cpp
//
// Code generated for Simulink model 'PRF'.
//
// Model version                  : 11.328
// Simulink Coder version         : 24.2 (R2024b) 21-Jun-2024
// C/C++ source code generated on : Mon Jun 29 09:49:15 2026
//
// Target selection: ert.tlc
// Embedded hardware selection: ARM Compatible->ARM Cortex-M
// Code generation objectives:
//    1. Execution efficiency
//    2. Debugging
//    3. RAM efficiency
// Validation result: Not run
//
#include "PRF.h"

namespace PRF 
{


// Block parameters (default storage)
PRF::P_PRF_T PRF::PRF_P{
  // Mask Parameter: WrapToZero_Threshold
  //  Referenced by: '<S20>/FixPt Switch'

  360.0F,

  // Mask Parameter: Comparetoconstant_const
  //  Referenced by: '<S14>/Constant'

  15.0F,

  // Computed Parameter: Zero_Value
  //  Referenced by: '<S8>/Zero'

  0.0F,

  // Computed Parameter: Zero_Value_c
  //  Referenced by: '<S7>/Zero'

  { 0.0F, 0.0F },

  // Computed Parameter: Zero_Value_p
  //  Referenced by: '<S9>/Zero'

  0.0F,

  // Computed Parameter: Constant5_Value
  //  Referenced by: '<S5>/Constant5'

  0.05F,

  // Computed Parameter: Constant4_Value
  //  Referenced by: '<S5>/Constant4'

  0.0025F,

  // Computed Parameter: Constant1_Value
  //  Referenced by: '<S10>/Constant1'

  6.28318548F,

  // Computed Parameter: Constant_Value
  //  Referenced by: '<S10>/Constant'

  6.28318548F,

  // Computed Parameter: Bias1_Bias
  //  Referenced by: '<S5>/Bias1'

  3.14159274F,

  // Computed Parameter: Bias_Bias
  //  Referenced by: '<S5>/Bias'

  -3.14159274F,

  // Computed Parameter: Gain2_Gain
  //  Referenced by: '<S15>/Gain2'

  0.5F,

  // Computed Parameter: Gain4_Gain
  //  Referenced by: '<S15>/Gain4'

  0.4F,

  // Computed Parameter: Gain3_Gain
  //  Referenced by: '<S15>/Gain3'

  0.5F,

  // Computed Parameter: Gain1_Gain
  //  Referenced by: '<S15>/Gain1'

  0.7F,

  // Computed Parameter: Constant_Value_k
  //  Referenced by: '<S20>/Constant'

  0.0F,

  // Expression: single(1e10*ones(3, 2))
  //  Referenced by: '<S13>/Target Points'

  { 1.0E+10F, 1.0E+10F, 1.0E+10F, 1.0E+10F, 1.0E+10F, 1.0E+10F },

  // Computed Parameter: Q2_Y0
  //  Referenced by: '<S13>/Q2'

  0.0F,

  // Computed Parameter: Constant3_Value
  //  Referenced by: '<S13>/Constant3'

  { -200.0F, 200.0F },

  // Computed Parameter: Gain1_Gain_b
  //  Referenced by: '<S19>/Gain1'

  0.0174532924F,

  // Computed Parameter: Constant5_Value_b
  //  Referenced by: '<S13>/Constant5'

  100.0F,

  // Computed Parameter: glideratio_Value
  //  Referenced by: '<S13>/glide ratio'

  2.0F,

  // Computed Parameter: Gain_Gain
  //  Referenced by: '<S17>/Gain'

  -1.0F,

  // Computed Parameter: Gain1_Gain_c
  //  Referenced by: '<S17>/Gain1'

  0.5F,

  // Computed Parameter: theta_Value
  //  Referenced by: '<S13>/theta'

  -0.5F,

  // Computed Parameter: Merge_InitialOutput
  //  Referenced by: '<S13>/Merge'

  0.0F,

  // Computed Parameter: Constant_Value_a
  //  Referenced by: '<S11>/Constant'

  0.0F,

  // Computed Parameter: Constant_Value_j
  //  Referenced by: '<S12>/Constant'

  0.0F,

  // Computed Parameter: Constant1_Value_e
  //  Referenced by: '<S4>/Constant1'

  { -200.0F, 200.0F },

  // Computed Parameter: glideratio_Value_o
  //  Referenced by: '<S4>/glide ratio'

  2.0F,

  // Computed Parameter: PRFControlzThresholdGain_Gain
  //  Referenced by: '<S4>/PRFControl.zThresholdGain'

  1.0F,

  // Computed Parameter: Memory1_InitialCondition
  //  Referenced by: '<S4>/Memory1'

  40.0F,

  // Computed Parameter: Memory_InitialCondition
  //  Referenced by: '<S4>/Memory'

  40.0F,

  // Computed Parameter: _Value
  //  Referenced by: '<S4>/-'

  20.0F,

  // Computed Parameter: Memory_InitialCondition_n
  //  Referenced by: '<S5>/Memory'

  0.0F,

  // Computed Parameter: Memory2_InitialCondition
  //  Referenced by: '<S5>/Memory2'

  0.0F,

  // Computed Parameter: Switch_Threshold
  //  Referenced by: '<S5>/Switch'

  3.14159274F,

  // Computed Parameter: Switch1_Threshold
  //  Referenced by: '<S5>/Switch1'

  0.0F,

  // Computed Parameter: Constant3_Value_c
  //  Referenced by: '<S5>/Constant3'

  0.045F,

  // Computed Parameter: UnitDelay_InitialCondition
  //  Referenced by: '<S5>/Unit Delay'

  0.0F,

  // Computed Parameter: Constant2_Value
  //  Referenced by: '<S5>/Constant2'

  0.03F,

  // Computed Parameter: Constant1_Value_k
  //  Referenced by: '<S5>/Constant1'

  0.05F,

  // Computed Parameter: Saturation1_UpperSat
  //  Referenced by: '<S5>/Saturation1'

  1.0F,

  // Computed Parameter: Saturation1_LowerSat
  //  Referenced by: '<S5>/Saturation1'

  -1.0F,

  // Computed Parameter: Merge_InitialOutput_l
  //  Referenced by: '<S5>/Merge'

  0.0F,

  // Expression: uint8(0)
  //  Referenced by: '<S6>/Unit Delay1'

  0U,

  // Computed Parameter: Bias_Bias_c
  //  Referenced by: '<S6>/Bias'

  1U,

  // Computed Parameter: Bias1_Bias_b
  //  Referenced by: '<S6>/Bias1'

  1U,

  // Computed Parameter: Saturation_UpperSat
  //  Referenced by: '<S6>/Saturation'

  1U,

  // Computed Parameter: Saturation_LowerSat
  //  Referenced by: '<S6>/Saturation'

  0U
};


} // fine namespace PRF

//
// File trailer for generated code.
//
// [EOF]
//
