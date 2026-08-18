//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// File: PRF_data.cpp
//
// Code generated for Simulink model 'PRF'.
//
// Model version                  : 11.370
// Simulink Coder version         : 24.2 (R2024b) 21-Jun-2024
// C/C++ source code generated on : Tue Aug 18 16:18:50 2026
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
  // Mask Parameter: SaturationCheckUp1_const
  //  Referenced by: '<S6>/Constant'

  1.0,

  // Mask Parameter: SaturationCheckLw1_const
  //  Referenced by: '<S5>/Constant'

  -1.0,

  // Expression: 0
  //  Referenced by: '<S2>/Constant'

  0.0,

  // Expression: 0
  //  Referenced by: '<S2>/Unit Delay3'

  0.0,

  // Expression: upSatTE
  //  Referenced by: '<S2>/Saturation2'

  1.0,

  // Expression: lwSatTE
  //  Referenced by: '<S2>/Saturation2'

  -1.0,

  // Computed Parameter: Zero_Value
  //  Referenced by: '<S8>/Zero'

  0.0F,

  // Computed Parameter: Zero_Value_a
  //  Referenced by: '<S4>/Zero'

  { 0.0F, 0.0F },

  // Computed Parameter: Zero_Value_d
  //  Referenced by: '<S7>/Zero'

  0.0F,

  // Computed Parameter: Constant10_Value
  //  Referenced by: '<S2>/Constant10'

  0.05F,

  // Computed Parameter: Constant9_Value
  //  Referenced by: '<S2>/Constant9'

  0.0159154944F,

  // Computed Parameter: Constant1_Value
  //  Referenced by: '<S9>/Constant1'

  6.28318548F,

  // Computed Parameter: Constant_Value_j
  //  Referenced by: '<S9>/Constant'

  6.28318548F,

  // Computed Parameter: Bias1_Bias
  //  Referenced by: '<S2>/Bias1'

  3.14159274F,

  // Computed Parameter: Bias_Bias
  //  Referenced by: '<S2>/Bias'

  -3.14159274F,

  // Computed Parameter: Constant_Value_b
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

  // Computed Parameter: Constant1_Value_p
  //  Referenced by: '<S14>/Constant1'

  6.378137E+6F,

  // Computed Parameter: Constant_Value_n
  //  Referenced by: '<S14>/Constant'

  1.0F,

  // Computed Parameter: Zero_Value_c
  //  Referenced by: '<S12>/Zero'

  0.0F,

  // Computed Parameter: Constant_Value_o
  //  Referenced by: '<S10>/Constant'

  0.0F,

  // Computed Parameter: Constant_Value_e
  //  Referenced by: '<S11>/Constant'

  0.0F,

  // Computed Parameter: Constant1_Value_c
  //  Referenced by: '<S1>/Constant1'

  { 45.5646515F, 12.5745802F },

  // Computed Parameter: Switch_Threshold
  //  Referenced by: '<S2>/Switch'

  3.14159274F,

  // Computed Parameter: Constant8_Value
  //  Referenced by: '<S2>/Constant8'

  0.286478907F,

  // Computed Parameter: UnitDelay1_InitialCondition
  //  Referenced by: '<S2>/Unit Delay1'

  0.0F,

  // Computed Parameter: Constant7_Value
  //  Referenced by: '<S2>/Constant7'

  0.0F,

  // Computed Parameter: Constant6_Value
  //  Referenced by: '<S2>/Constant6'

  0.05F,

  // Computed Parameter: Merge1_InitialOutput
  //  Referenced by: '<S2>/Merge1'

  0.0F,

  // Computed Parameter: UnitDelay2_InitialCondition
  //  Referenced by: '<S2>/Unit Delay2'

  false
};


} // fine namespace PRF

//
// File trailer for generated code.
//
// [EOF]
//
