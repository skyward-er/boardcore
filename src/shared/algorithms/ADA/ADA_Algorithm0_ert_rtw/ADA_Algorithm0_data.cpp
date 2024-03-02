//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// File: ADA_Algorithm0_data.cpp
//
// Code generated for Simulink model 'ADA_Algorithm0'.
//
// Model version                  : 1.11
// Simulink Coder version         : 23.2 (R2023b) 01-Aug-2023
// C/C++ source code generated on : Wed Nov  1 00:43:29 2023
//
// Target selection: ert.tlc
// Embedded hardware selection: STMicroelectronics->ST10/Super10
// Code generation objectives:
//    1. Execution efficiency
//    2. Safety precaution
// Validation result: Passed (20), Warning (1), Error (0)
//
#include "ADA_Algorithm0.h"

// Block parameters (default storage)
ADA_Algorithm0::P_ADA_Algorithm0_T ADA_Algorithm0::ADA_Algorithm0_P{
  // Computed Parameter: Identity_matrix_Value
  //  Referenced by: '<S9>/Identity_matrix'

  { 1.0F, 0.0F, 0.0F, 0.0F, 1.0F, 0.0F, 0.0F, 0.0F, 1.0F },

  // Expression: H_INIT
  //  Referenced by: '<S2>/H_memory'

  { 1.0F, 0.0F, 0.0F },

  // Expression: R_INIT
  //  Referenced by: '<S2>/R_memory'

  4000.0F,

  // Expression: F_INIT
  //  Referenced by: '<S1>/F_memory'

  { 1.0F, 0.0F, 0.0F, 0.02F, 1.0F, 0.0F, 0.0002F, 0.02F, 1.0F },

  // Expression: P_INIT
  //  Referenced by: '<S1>/P_memory'

  { 1.0F, 0.0F, 0.0F, 0.0F, 1.0F, 0.0F, 0.0F, 0.0F, 1.0F },

  // Expression: Q_INIT
  //  Referenced by: '<S1>/Q_memory'

  { 30.0F, 0.0F, 0.0F, 0.0F, 10.0F, 0.0F, 0.0F, 0.0F, 2.5F },

  // Expression: x_INIT
  //  Referenced by: '<S1>/x_memory'

  { 99417.6F, 0.0F, 0.0F },

  // Computed Parameter: fake_timestamp_Value
  //  Referenced by: '<S1>/fake_timestamp'

  1.0F,

  // Expression: T_ref
  //  Referenced by: '<S6>/T_ref'

  288.15F,

  // Computed Parameter: a_Value
  //  Referenced by: '<S6>/a'

  0.0065F,

  // Computed Parameter: Constant_Value
  //  Referenced by: '<S6>/Constant'

  1.0F,

  // Expression: P_ref
  //  Referenced by: '<S6>/P_ref'

  101325.0F,

  // Computed Parameter: nInv_Value
  //  Referenced by: '<S6>/nInv'

  0.1902612F,

  // Expression: z0_ref
  //  Referenced by: '<S5>/z0_ref'

  160.0F,

  // Expression: T_ref
  //  Referenced by: '<S7>/T_ref'

  288.15F,

  // Expression: P_ref
  //  Referenced by: '<S7>/P_ref'

  101325.0F,

  // Computed Parameter: nInv_Value_g
  //  Referenced by: '<S7>/nInv'

  0.1902612F,

  // Computed Parameter: Gain_Gain
  //  Referenced by: '<S7>/Gain'

  -1.0F,

  // Computed Parameter: a_Value_e
  //  Referenced by: '<S7>/a'

  0.0065F
};

//
// File trailer for generated code.
//
// [EOF]
//
