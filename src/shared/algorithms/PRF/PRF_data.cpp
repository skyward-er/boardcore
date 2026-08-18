//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// File: PRF_data.cpp
//
// Code generated for Simulink model 'PRF'.
//
// Model version                  : 11.335
// Simulink Coder version         : 24.2 (R2024b) 21-Jun-2024
// C/C++ source code generated on : Wed Jul 22 13:17:22 2026
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
  // Mask Parameter: SaturationCheckUp_const
  //  Referenced by: '<S6>/Constant'

  0.7F,

  // Mask Parameter: SaturationCheckLw_const
  //  Referenced by: '<S5>/Constant'

  -0.7F,

  // Computed Parameter: Zero_Value
  //  Referenced by: '<S8>/Zero'

  0.0F,

  // Computed Parameter: Zero_Value_m
  //  Referenced by: '<S4>/Zero'

  { 0.0F, 0.0F },

  // Computed Parameter: Zero_Value_o
  //  Referenced by: '<S7>/Zero'

  0.0F,

  // Computed Parameter: Constant5_Value
  //  Referenced by: '<S2>/Constant5'

  0.05F,

  // Computed Parameter: Constant4_Value
  //  Referenced by: '<S2>/Constant4'

  0.0F,

  // Computed Parameter: Constant1_Value
  //  Referenced by: '<S9>/Constant1'

  6.28318548F,

  // Computed Parameter: Constant_Value
  //  Referenced by: '<S9>/Constant'

  6.28318548F,

  // Computed Parameter: Bias1_Bias
  //  Referenced by: '<S2>/Bias1'

  3.14159274F,

  // Computed Parameter: Bias_Bias
  //  Referenced by: '<S2>/Bias'

  -3.14159274F,

  // Computed Parameter: Constant_Value_c
  //  Referenced by: '<S16>/Constant'

  1.0F,

  // Computed Parameter: Constant3_Value
  //  Referenced by: '<S14>/Constant3'

  1.0F,

  // Computed Parameter: Constant2_Value
  //  Referenced by: '<S14>/Constant2'

  0.00335281063F,

  // Computed Parameter: f_Value
  //  Referenced by: '<S14>/f'

  1.0F,

  // Computed Parameter: Constant1_Value_a
  //  Referenced by: '<S14>/Constant1'

  6.378137E+6F,

  // Computed Parameter: Constant_Value_a
  //  Referenced by: '<S14>/Constant'

  1.0F,

  // Computed Parameter: Zero_Value_on
  //  Referenced by: '<S12>/Zero'

  0.0F,

  // Computed Parameter: Constant_Value_n
  //  Referenced by: '<S10>/Constant'

  0.0F,

  // Computed Parameter: Constant_Value_nk
  //  Referenced by: '<S11>/Constant'

  0.0F,

  // Computed Parameter: Constant1_Value_i
  //  Referenced by: '<S1>/Constant1'

  { 45.501091F, 9.15642643F },

  // Computed Parameter: Memory_InitialCondition
  //  Referenced by: '<S2>/Memory'

  0.0F,

  // Computed Parameter: Switch_Threshold
  //  Referenced by: '<S2>/Switch'

  3.14159274F,

  // Computed Parameter: Constant3_Value_p
  //  Referenced by: '<S2>/Constant3'

  0.286478907F,

  // Computed Parameter: UnitDelay_InitialCondition
  //  Referenced by: '<S2>/Unit Delay'

  0.0F,

  // Computed Parameter: Constant2_Value_a
  //  Referenced by: '<S2>/Constant2'

  0.0F,

  // Computed Parameter: Constant1_Value_p
  //  Referenced by: '<S2>/Constant1'

  0.05F,

  // Computed Parameter: Saturation1_UpperSat
  //  Referenced by: '<S2>/Saturation1'

  0.7F,

  // Computed Parameter: Saturation1_LowerSat
  //  Referenced by: '<S2>/Saturation1'

  -0.7F,

  // Computed Parameter: Merge_InitialOutput
  //  Referenced by: '<S2>/Merge'

  0.0F,

  // Computed Parameter: Memory2_InitialCondition
  //  Referenced by: '<S2>/Memory2'

  false
};


} // fine namespace PRF

//
// File trailer for generated code.
//
// [EOF]
//
