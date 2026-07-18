//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// File: MEA_data.cpp
//
// Code generated for Simulink model 'MEA'.
//
// Model version                  : 11.238
// Simulink Coder version         : 24.2 (R2024b) 21-Jun-2024
// C/C++ source code generated on : Mon May 11 01:43:03 2026
//
// Target selection: ert.tlc
// Embedded hardware selection: STMicroelectronics->ST10/Super10
// Code generation objectives:
//    1. Execution efficiency
//    2. Debugging
//    3. RAM efficiency
// Validation result: Not run
//
#include "MEA.h"
namespace MEA {
// Block parameters (default storage)
MEA::P_MEA_T MEA::MEA_P{
  // Mask Parameter: CompareToConstant_const
  //  Referenced by: '<S5>/Constant'

  0.0F,

  // Computed Parameter: UnitDelay_InitialCondition
  //  Referenced by: '<S2>/Unit Delay'

  (0ULL),

  // Computed Parameter: IdentityMatrix_IDMatrixData
  //  Referenced by: '<S4>/IdentityMatrix'

  { 1.0F, 0.0F, 0.0F, 1.0F },

  // Computed Parameter: Gain4_Gain
  //  Referenced by: '<S4>/Gain4'

  { 0.927390635F, -0.910008192F },

  // Computed Parameter: Gain1_Gain
  //  Referenced by: '<S4>/Gain1'

  { 0.927390635F, -0.910008192F },

  // Computed Parameter: Constant1_Value
  //  Referenced by: '<S4>/Constant1'

  0.36F,

  // Computed Parameter: Gain2_Gain
  //  Referenced by: '<S4>/Gain2'

  { 0.927390635F, -0.910008192F },

  // Computed Parameter: PreviousCovariance_InitialCondi
  //  Referenced by: '<S1>/Previous Covariance'

  { 0.0F, 0.0F, 0.0F, 0.0F },

  // Computed Parameter: Gain2_Gain_l
  //  Referenced by: '<S3>/Gain2'

  { 1.9035809F, 1.0F, -0.905418F, 0.0F },

  // Computed Parameter: Gain3_Gain
  //  Referenced by: '<S3>/Gain3'

  { 1.9035809F, -0.905418F, 1.0F, 0.0F },

  // Computed Parameter: Bias_Bias
  //  Referenced by: '<S3>/Bias'

  { 1.0F, 0.0F, 0.0F, 1.0F },

  // Computed Parameter: Gain1_Gain_d
  //  Referenced by: '<S2>/Gain1'

  { 0.927390635F, -0.910008192F },

  // Computed Parameter: Gain4_Gain_k
  //  Referenced by: '<S2>/Gain4'

  { 0.927390635F, -0.910008192F },

  // Computed Parameter: Bias_Bias_a
  //  Referenced by: '<S2>/Bias'

  0.36F,

  // Computed Parameter: PreviousState_InitialCondition
  //  Referenced by: '<S1>/Previous State'

  { 0.0F, 0.0F },

  // Computed Parameter: Gain1_Gain_k
  //  Referenced by: '<S3>/Gain1'

  { 1.9035809F, 1.0F, -0.905418F, 0.0F },

  // Computed Parameter: UnitDelay2_InitialCondition
  //  Referenced by: '<S1>/Unit Delay2'

  36.156456F,

  // Computed Parameter: Gain1_Gain_f
  //  Referenced by: '<S1>/Gain1'

  { 0.927390635F, -0.910008192F },

  // Computed Parameter: Gain2_Gain_n
  //  Referenced by: '<S1>/Gain2'

  0.0500489585F,

  // Computed Parameter: Gain3_Gain_c
  //  Referenced by: '<S1>/Gain3'

  0.02F,

  // Computed Parameter: Saturation_UpperSat
  //  Referenced by: '<S1>/Saturation'

  36.156456F,

  // Computed Parameter: Saturation_LowerSat
  //  Referenced by: '<S1>/Saturation'

  25.0F,

  // Computed Parameter: Gain_Gain
  //  Referenced by: '<S3>/Gain'

  { 2U, 0U }
};
}
//
// File trailer for generated code.
//
// [EOF]
//
