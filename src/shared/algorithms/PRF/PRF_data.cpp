//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// File: PRF_data.cpp
//
// Code generated for Simulink model 'PRF'.
//
// Model version                  : 11.339
// Simulink Coder version         : 24.2 (R2024b) 21-Jun-2024
// C/C++ source code generated on : Wed Jul 15 16:02:04 2026
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
  //  Referenced by: '<S23>/FixPt Switch'

  360.0F,

  // Mask Parameter: Comparetoconstant_const
  //  Referenced by: '<S17>/Constant'

  15.0F,

  // Mask Parameter: SaturationCheckUp_const
  //  Referenced by: '<S10>/Constant'

  1.0F,

  // Mask Parameter: SaturationCheckLw_const
  //  Referenced by: '<S9>/Constant'

  -1.0F,

  // Computed Parameter: PRFLogsOBSW_Y0
  //  Referenced by: '<S16>/PRF Logs OBSW'

  {
    {
      0.0F, 0.0F }
    ,                                  // Q1

    {
      0.0F, 0.0F }
    ,                                  // Q2

    {
      0.0F, 0.0F, 0.0F, 0.0F }
    ,                                  // TerminalTarget
    0U,                                // TargetIndex
    0.0F,                              // Heading
    0.0F,                              // Reference

    {
      0.0F, 0.0F }
    ,                                  // ServoCommands
    0.0F                               // WindHeading
  },

  // Computed Parameter: Zero_Value
  //  Referenced by: '<S12>/Zero'

  0.0F,

  // Computed Parameter: Zero_Value_b
  //  Referenced by: '<S8>/Zero'

  { 0.0F, 0.0F },

  // Computed Parameter: Zero_Value_f
  //  Referenced by: '<S11>/Zero'

  0.0F,

  // Computed Parameter: Constant5_Value
  //  Referenced by: '<S6>/Constant5'

  0.05F,

  // Computed Parameter: Constant4_Value
  //  Referenced by: '<S6>/Constant4'

  0.0025F,

  // Computed Parameter: Constant1_Value
  //  Referenced by: '<S13>/Constant1'

  6.28318548F,

  // Computed Parameter: Constant_Value
  //  Referenced by: '<S13>/Constant'

  6.28318548F,

  // Computed Parameter: Bias1_Bias
  //  Referenced by: '<S6>/Bias1'

  3.14159274F,

  // Computed Parameter: Bias_Bias
  //  Referenced by: '<S6>/Bias'

  -3.14159274F,

  // Computed Parameter: Gain2_Gain
  //  Referenced by: '<S18>/Gain2'

  0.5F,

  // Computed Parameter: Gain4_Gain
  //  Referenced by: '<S18>/Gain4'

  0.7F,

  // Computed Parameter: Gain3_Gain
  //  Referenced by: '<S18>/Gain3'

  0.5F,

  // Computed Parameter: Gain1_Gain
  //  Referenced by: '<S18>/Gain1'

  0.4F,

  // Computed Parameter: Constant_Value_p
  //  Referenced by: '<S23>/Constant'

  0.0F,

  // Computed Parameter: TargetPoints_Y0
  //  Referenced by: '<S16>/Target Points'

  0.0F,

  // Computed Parameter: Q2_Y0
  //  Referenced by: '<S16>/Q2'

  0.0F,

  // Computed Parameter: Gain1_Gain_g
  //  Referenced by: '<S22>/Gain1'

  0.0174532924F,

  // Computed Parameter: Constant5_Value_d
  //  Referenced by: '<S16>/Constant5'

  100.0F,

  // Computed Parameter: glideratio_Value
  //  Referenced by: '<S16>/glide ratio'

  2.0F,

  // Computed Parameter: Gain_Gain
  //  Referenced by: '<S20>/Gain'

  -1.0F,

  // Computed Parameter: Gain1_Gain_h
  //  Referenced by: '<S20>/Gain1'

  0.5F,

  // Computed Parameter: theta_Value
  //  Referenced by: '<S16>/theta'

  -0.5F,

  // Computed Parameter: Merge_InitialOutput
  //  Referenced by: '<S16>/Merge'

  0.0F,

  // Computed Parameter: Constant_Value_o
  //  Referenced by: '<S28>/Constant'

  1.0F,

  // Computed Parameter: Constant3_Value
  //  Referenced by: '<S26>/Constant3'

  1.0F,

  // Computed Parameter: Constant2_Value
  //  Referenced by: '<S26>/Constant2'

  0.00335281063F,

  // Computed Parameter: f_Value
  //  Referenced by: '<S26>/f'

  1.0F,

  // Computed Parameter: Constant1_Value_l
  //  Referenced by: '<S26>/Constant1'

  6.378137E+6F,

  // Computed Parameter: Constant_Value_i
  //  Referenced by: '<S26>/Constant'

  1.0F,

  // Computed Parameter: Zero_Value_m
  //  Referenced by: '<S24>/Zero'

  0.0F,

  // Computed Parameter: Constant_Value_ir
  //  Referenced by: '<S14>/Constant'

  0.0F,

  // Computed Parameter: Constant_Value_a
  //  Referenced by: '<S15>/Constant'

  0.0F,

  // Computed Parameter: Constant1_Value_h
  //  Referenced by: '<S1>/Constant1'

  { 39.3887253F, -8.2878418F },

  // Computed Parameter: glideratio_Value_o
  //  Referenced by: '<S5>/glide ratio'

  2.0F,

  // Computed Parameter: PRFControlzThresholdGain_Gain
  //  Referenced by: '<S5>/PRFControl.zThresholdGain'

  1.0F,

  // Computed Parameter: Memory1_InitialCondition
  //  Referenced by: '<S5>/Memory1'

  40.0F,

  // Computed Parameter: Memory_InitialCondition
  //  Referenced by: '<S5>/Memory'

  40.0F,

  // Computed Parameter: _Value
  //  Referenced by: '<S5>/-'

  20.0F,

  // Computed Parameter: Switch_Threshold
  //  Referenced by: '<S6>/Switch'

  3.14159274F,

  // Computed Parameter: Memory_InitialCondition_b
  //  Referenced by: '<S6>/Memory'

  0.0F,

  // Computed Parameter: Constant3_Value_l
  //  Referenced by: '<S6>/Constant3'

  0.045F,

  // Computed Parameter: UnitDelay_InitialCondition
  //  Referenced by: '<S6>/Unit Delay'

  0.0F,

  // Computed Parameter: Constant2_Value_c
  //  Referenced by: '<S6>/Constant2'

  0.005F,

  // Computed Parameter: Constant1_Value_p
  //  Referenced by: '<S6>/Constant1'

  0.05F,

  // Computed Parameter: Saturation1_UpperSat
  //  Referenced by: '<S6>/Saturation1'

  1.0F,

  // Computed Parameter: Saturation1_LowerSat
  //  Referenced by: '<S6>/Saturation1'

  -1.0F,

  // Computed Parameter: Merge_InitialOutput_g
  //  Referenced by: '<S6>/Merge'

  0.0F,

  // Computed Parameter: Memory2_InitialCondition
  //  Referenced by: '<S6>/Memory2'

  false,

  // Expression: uint8(0)
  //  Referenced by: '<S7>/Unit Delay1'

  0U,

  // Computed Parameter: Bias_Bias_i
  //  Referenced by: '<S7>/Bias'

  1U,

  // Computed Parameter: Bias1_Bias_n
  //  Referenced by: '<S7>/Bias1'

  1U,

  // Computed Parameter: Saturation_UpperSat
  //  Referenced by: '<S7>/Saturation'

  1U,

  // Computed Parameter: Saturation_LowerSat
  //  Referenced by: '<S7>/Saturation'

  0U
};


} // fine namespace PRF

//
// File trailer for generated code.
//
// [EOF]
//
