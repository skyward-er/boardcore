//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// File: ANAS0.cpp
//
// Code generated for Simulink model 'ANAS0'.
//
// Model version                  : 11.183
// Simulink Coder version         : 24.2 (R2024b) 21-Jun-2024
// C/C++ source code generated on : Thu Apr 16 13:46:31 2026
//
// Target selection: ert.tlc
// Embedded hardware selection: STMicroelectronics->ST10/Super10
// Code generation objectives:
//    1. Execution efficiency
//    2. Debugging
//    3. RAM efficiency
// Validation result: Passed (3), Warning (1), Error (0)
//
#include "ANAS0.h"

// Model step function
void ANAS0::step()
{
  // Outport: '<Root>/NASLogs' incorporates:
  //   Constant: '<S1>/Zero1'

  ANAS0_Y.NASLogs_c = ANAS0_P.Zero1_Value;

  // Outport: '<Root>/NASFinal' incorporates:
  //   Constant: '<S1>/Zero2'

  ANAS0_Y.NASFinal_n = ANAS0_P.Zero2_Value;

  // Outport: '<Root>/NASOut' incorporates:
  //   Constant: '<S1>/Zero'

  ANAS0_Y.NASOut_m = ANAS0_P.Zero_Value;
}

// Model initialize function
void ANAS0::initialize()
{
  // (no initialization code required)
}

// Model terminate function
void ANAS0::terminate()
{
  // (no terminate code required)
}

// Constructor
ANAS0::ANAS0():
  ANAS0_U(),
  ANAS0_Y()
{
  // Currently there is no constructor body generated.
}

// Destructor
// Currently there is no destructor body generated.
ANAS0::~ANAS0() = default;

//
// File trailer for generated code.
//
// [EOF]
//
