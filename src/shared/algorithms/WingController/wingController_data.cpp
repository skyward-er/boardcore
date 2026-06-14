//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// File: wingController_data.cpp
//
// Code generated for Simulink model 'wingController'.
//
// Model version                  : 11.284
// Simulink Coder version         : 24.2 (R2024b) 21-Jun-2024
// C/C++ source code generated on : Wed Jun  3 10:43:26 2026
//
// Target selection: ert.tlc
// Embedded hardware selection: ARM Compatible->ARM Cortex-M
// Code generation objectives:
//    1. Execution efficiency
//    2. Debugging
//    3. RAM efficiency
// Validation result: Not run
//
#include "wingController.h"

// Block parameters (default storage)
wingController::P_wingController_T wingController::wingController_P{
  // Mask Parameter: WrapToZero_Threshold
  //  Referenced by: '<S24>/FixPt Switch'

  360.0F,

  // Mask Parameter: Comparetoconstant_const
  //  Referenced by: '<S18>/Constant'

  15.0F,

  // Mask Parameter: SaturationCheckUp1_const
  //  Referenced by: '<S11>/Constant'

  0.1F,

  // Mask Parameter: SaturationCheckLw1_const
  //  Referenced by: '<S9>/Constant'

  -0.1F,

  // Mask Parameter: SaturationCheckUp_const
  //  Referenced by: '<S10>/Constant'

  2.04203534F,

  // Mask Parameter: SaturationCheckLw_const
  //  Referenced by: '<S8>/Constant'

  -2.04203534F,

  // Computed Parameter: Zero_Value
  //  Referenced by: '<S12>/Zero'

  0.0F,

  // Computed Parameter: Zero_Value_d
  //  Referenced by: '<S7>/Zero'

  { 0.0F, 0.0F },

  // Computed Parameter: Zero_Value_p
  //  Referenced by: '<S13>/Zero'

  0.0F,

  // Expression: PRFControl.subsystemSamplingTime
  //  Referenced by: '<S5>/Constant5'

  1.0F,

  // Expression: PRFControl.OBSW.ki
  //  Referenced by: '<S5>/Constant4'

  0.05F,

  // Expression: PRFControl.subsystemSamplingTime
  //  Referenced by: '<S5>/Constant8'

  1.0F,

  // Expression: PRFControl.ki
  //  Referenced by: '<S5>/Constant2'

  0.0025F,

  // Expression: PRFControl.servoRadius
  //  Referenced by: '<S5>/Constant7'

  0.0587649F,

  // Expression: PRFControl.halfWing
  //  Referenced by: '<S5>/Constant6'

  1.175F,

  // Computed Parameter: Gain_Gain
  //  Referenced by: '<S5>/Gain'

  0.5F,

  // Computed Parameter: Gain1_Gain
  //  Referenced by: '<S5>/Gain1'

  2.0F,

  // Computed Parameter: Gain3_Gain
  //  Referenced by: '<S5>/Gain3'

  0.48970753F,

  // Computed Parameter: Constant1_Value
  //  Referenced by: '<S14>/Constant1'

  6.28318548F,

  // Computed Parameter: Constant_Value
  //  Referenced by: '<S14>/Constant'

  6.28318548F,

  // Computed Parameter: Bias1_Bias
  //  Referenced by: '<S5>/Bias1'

  3.14159274F,

  // Computed Parameter: Bias_Bias
  //  Referenced by: '<S5>/Bias'

  -3.14159274F,

  // Computed Parameter: Gain2_Gain
  //  Referenced by: '<S19>/Gain2'

  0.5F,

  // Expression: PRFControl.perpendicularGain
  //  Referenced by: '<S19>/Gain4'

  0.4F,

  // Computed Parameter: Gain3_Gain_m
  //  Referenced by: '<S19>/Gain3'

  0.5F,

  // Expression: PRFControl.parallelGain
  //  Referenced by: '<S19>/Gain1'

  0.7F,

  // Computed Parameter: Constant_Value_a
  //  Referenced by: '<S24>/Constant'

  0.0F,

  // Expression: single(1e10*ones(3, 2))
  //  Referenced by: '<S17>/Target Points'

  { 1.0E+10F, 1.0E+10F, 1.0E+10F, 1.0E+10F, 1.0E+10F, 1.0E+10F },

  // Computed Parameter: Q2_Y0
  //  Referenced by: '<S17>/Q2'

  0.0F,

  // Expression: PRFControl.target
  //  Referenced by: '<S17>/Constant3'

  { -200.0F, 200.0F },

  // Expression: PRFControl.windHeading
  //  Referenced by: '<S17>/Constant1'

  200.0F,

  // Computed Parameter: Gain1_Gain_d
  //  Referenced by: '<S23>/Gain1'

  0.0174532924F,

  // Expression: PRFControl.windAlignmentRadius
  //  Referenced by: '<S17>/Constant5'

  100.0F,

  // Expression: PRFControl.glideRatio
  //  Referenced by: '<S17>/glide ratio'

  1.61818182F,

  // Computed Parameter: Gain_Gain_e
  //  Referenced by: '<S21>/Gain'

  -1.0F,

  // Computed Parameter: Gain1_Gain_dp
  //  Referenced by: '<S21>/Gain1'

  0.5F,

  // Expression: PRFControl.DPGAngle
  //  Referenced by: '<S17>/theta'

  -0.5F,

  // Computed Parameter: Merge_InitialOutput
  //  Referenced by: '<S17>/Merge'

  0.0F,

  // Computed Parameter: Constant_Value_o
  //  Referenced by: '<S15>/Constant'

  0.0F,

  // Computed Parameter: Constant_Value_oa
  //  Referenced by: '<S16>/Constant'

  0.0F,

  // Expression: PRFControl.target
  //  Referenced by: '<S4>/Constant1'

  { -200.0F, 200.0F },

  // Expression: PRFControl.glideRatio
  //  Referenced by: '<S4>/glide ratio'

  1.61818182F,

  // Expression: PRFControl.zThresholdGain
  //  Referenced by: '<S4>/PRFControl.zThresholdGain'

  1.0F,

  // Computed Parameter: Memory1_InitialCondition
  //  Referenced by: '<S4>/Memory1'

  40.0F,

  // Computed Parameter: Memory_InitialCondition
  //  Referenced by: '<S4>/Memory'

  40.0F,

  // Expression: PRFControl.QThreshold
  //  Referenced by: '<S4>/-'

  20.0F,

  // Computed Parameter: Memory_InitialCondition_d
  //  Referenced by: '<S5>/Memory'

  0.0F,

  // Computed Parameter: Switch_Threshold
  //  Referenced by: '<S5>/Switch'

  3.14159274F,

  // Expression: PRFControl.OBSW.kp
  //  Referenced by: '<S5>/Constant3'

  0.9F,

  // Expression: PRFControl.OBSW.upSaturation
  //  Referenced by: '<S5>/Saturation1'

  2.04203534F,

  // Expression: PRFControl.OBSW.lwSaturation
  //  Referenced by: '<S5>/Saturation1'

  -2.04203534F,

  // Computed Parameter: Memory1_InitialCondition_f
  //  Referenced by: '<S5>/Memory1'

  0.0F,

  // Expression: PRFControl.kp
  //  Referenced by: '<S5>/Constant1'

  0.045F,

  // Expression: PRFControl.upSaturation
  //  Referenced by: '<S5>/Saturation'

  0.1F,

  // Expression: PRFControl.lwSaturation
  //  Referenced by: '<S5>/Saturation'

  -0.1F,

  // Expression: PRFControl.scalingFactor
  //  Referenced by: '<S5>/Gain2'

  0.142857149F,

  // Computed Parameter: Merge_InitialOutput_e
  //  Referenced by: '<S5>/Merge'

  0.0F,

  // Computed Parameter: Memory2_InitialCondition
  //  Referenced by: '<S5>/Memory2'

  false,

  // Expression: PRFControl.OBSW.activation
  //  Referenced by: '<S5>/Constant9'

  true,

  // Computed Parameter: Memory3_InitialCondition
  //  Referenced by: '<S5>/Memory3'

  false,

  // Expression: uint8(0)
  //  Referenced by: '<S6>/Unit Delay1'

  0U,

  // Computed Parameter: Bias_Bias_e
  //  Referenced by: '<S6>/Bias'

  1U,

  // Computed Parameter: Bias1_Bias_b
  //  Referenced by: '<S6>/Bias1'

  1U,

  // Computed Parameter: Saturation_UpperSat_f
  //  Referenced by: '<S6>/Saturation'

  1U,

  // Computed Parameter: Saturation_LowerSat_b
  //  Referenced by: '<S6>/Saturation'

  0U
};

//
// File trailer for generated code.
//
// [EOF]
//
