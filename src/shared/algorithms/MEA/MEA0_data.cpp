//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// File: MEA0_data.cpp
//
// Code generated for Simulink model 'MEA0'.
//
// Model version                  : 11.125
// Simulink Coder version         : 24.2 (R2024b) 21-Jun-2024
// C/C++ source code generated on : Sun Feb 22 11:31:50 2026
//
// Target selection: ert.tlc
// Embedded hardware selection: ARM Compatible->ARM Cortex-M
// Code generation objective: RAM efficiency
// Validation result: Passed (3), Warning (1), Error (1)
//
#include "MEA0.h"

// Block parameters (default storage)
MEA0::P_MEA0_T MEA0::MEA0_P{
  // Mask Parameter: CompareToConstant_const
  //  Referenced by: '<S5>/Constant'

  0.0F,

  // Computed Parameter: Memory_InitialCondition
  //  Referenced by: '<S1>/Memory'

  { { 0UL, 0UL } },

  // Computed Parameter: IdentityMatrix_IDMatrixData
  //  Referenced by: '<S6>/IdentityMatrix'

  { 1.0F, 0.0F, 0.0F, 1.0F },

  // Expression: mea.CObservable
  //  Referenced by: '<S6>/Constant'

  { 0.927390635F, -0.910008192F },

  // Expression: mea.baroR
  //  Referenced by: '<S6>/Constant1'

  0.36F,

  // Expression: mea.CObservable
  //  Referenced by: '<S3>/Constant'

  { 0.927390635F, -0.910008192F },

  // Expression: mea.baroR
  //  Referenced by: '<S3>/Constant1'

  0.36F,

  // Expression: mea.CObservable
  //  Referenced by: '<S1>/Constant'

  { 0.927390635F, -0.910008192F },

  // Expression: mea.massCoeff
  //  Referenced by: '<S1>/Constant1'

  0.0500489585F,

  // Computed Parameter: Constant2_Value
  //  Referenced by: '<S1>/Constant2'

  0.02F,

  // Expression: mea.BObservable
  //  Referenced by: '<S4>/Constant1'

  { 2.0F, 0.0F },

  // Expression: mea.AObservable
  //  Referenced by: '<S4>/Constant'

  { 1.9035809F, 1.0F, -0.905418F, 0.0F },

  // Computed Parameter: x_memory_InitialCondition
  //  Referenced by: '<S1>/x_memory'

  { 0.0F, 0.0F },

  // Expression: mea.P0Observable
  //  Referenced by: '<S1>/P_memory'

  { 0.0F, 0.0F, 0.0F, 0.0F },

  // Expression: mea.QObservable
  //  Referenced by: '<S4>/Constant2'

  { 1.0F, 0.0F, 0.0F, 1.0F },

  // Computed Parameter: mass_memory_InitialCondition
  //  Referenced by: '<S1>/mass_memory'

  36.156456F,

  // Expression: mea.maxMass
  //  Referenced by: '<S1>/Saturation'

  36.0F,

  // Expression: mea.minMass
  //  Referenced by: '<S1>/Saturation'

  25.0F
};

//
// File trailer for generated code.
//
// [EOF]
//
