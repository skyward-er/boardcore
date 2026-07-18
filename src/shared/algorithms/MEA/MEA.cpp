//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// File: MEA.cpp
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
#include <stdint.h>
#include <stdbool.h>

namespace MEA {
// Model step function
void MEA::step()
{
  float rtb_APAQ[4];
  float rtb_APAQ_0[4];
  float rtb_KRK_0[4];
  float rtb_K_j[4];
  float rtb_K_h[2];
  float Gain2_Gain_l;
  float Gain2_Gain_l_0;
  float Gain3_Gain;
  float Gain3_Gain_0;
  float Gain3_Gain_1;
  float rtb_AxBu_idx_0;
  float rtb_AxBu_idx_1;
  float rtb_CPCR;
  float rtb_KRK_idx_1;
  float rtb_KRK_idx_2;
  float rtb_KRK_idx_3;
  int16_t i;
  int16_t rtb_K_j_tmp;
  bool tmp;

  // Outputs for Atomic SubSystem: '<Root>/MEA II - Autocoding'
  // Gain: '<S3>/Gain2' incorporates:
  //   UnitDelay: '<S1>/Previous Covariance'

  rtb_AxBu_idx_0 = MEA_DW.PreviousCovariance_DSTATE[1];
  rtb_AxBu_idx_1 = MEA_DW.PreviousCovariance_DSTATE[0];
  rtb_KRK_idx_1 = MEA_DW.PreviousCovariance_DSTATE[3];
  rtb_KRK_idx_2 = MEA_DW.PreviousCovariance_DSTATE[2];

  // Bias: '<S3>/Bias' incorporates:
  //   Gain: '<S3>/Gain3'

  rtb_KRK_idx_3 = MEA_P.Gain3_Gain[1];
  Gain3_Gain = MEA_P.Gain3_Gain[0];
  Gain3_Gain_0 = MEA_P.Gain3_Gain[3];
  Gain3_Gain_1 = MEA_P.Gain3_Gain[2];
  for (i = 0; i < 2; i++) {
    // Gain: '<S3>/Gain2'
    Gain2_Gain_l = MEA_P.Gain2_Gain_l[i + 2];
    Gain2_Gain_l_0 = MEA_P.Gain2_Gain_l[i];
    rtb_CPCR = Gain2_Gain_l * rtb_AxBu_idx_0 + Gain2_Gain_l_0 * rtb_AxBu_idx_1;
    Gain2_Gain_l = Gain2_Gain_l * rtb_KRK_idx_1 + Gain2_Gain_l_0 * rtb_KRK_idx_2;

    // Bias: '<S3>/Bias' incorporates:
    //   Gain: '<S3>/Gain3'

    rtb_APAQ[i] = (Gain2_Gain_l * rtb_KRK_idx_3 + rtb_CPCR * Gain3_Gain) +
      MEA_P.Bias_Bias[i];
    rtb_APAQ[i + 2] = (Gain2_Gain_l * Gain3_Gain_0 + rtb_CPCR * Gain3_Gain_1) +
      MEA_P.Bias_Bias[i + 2];
  }

  // Bias: '<S2>/Bias' incorporates:
  //   Bias: '<S3>/Bias'
  //   Gain: '<S2>/Gain1'
  //   Gain: '<S2>/Gain4'

  rtb_CPCR = ((MEA_P.Gain1_Gain_d[0] * rtb_APAQ[0] + MEA_P.Gain1_Gain_d[1] *
               rtb_APAQ[1]) * MEA_P.Gain4_Gain_k[0] + (MEA_P.Gain1_Gain_d[0] *
    rtb_APAQ[2] + MEA_P.Gain1_Gain_d[1] * rtb_APAQ[3]) * MEA_P.Gain4_Gain_k[1])
    + MEA_P.Bias_Bias_a;

  // DataTypeConversion: '<S1>/Cast To Boolean' incorporates:
  //   Inport: '<Root>/MEA In'

  tmp = (MEA_U.MEAIn_o.MainValvePosition != 0.0F);

  // Sum: '<S3>/Add1' incorporates:
  //   Gain: '<S3>/Gain'
  //   Gain: '<S3>/Gain1'
  //   UnitDelay: '<S1>/Previous State'

  rtb_AxBu_idx_0 = (MEA_P.Gain1_Gain_k[0] * MEA_DW.PreviousState_DSTATE[0] +
                    MEA_DW.PreviousState_DSTATE[1] * MEA_P.Gain1_Gain_k[2]) +
    static_cast<float>(tmp ? static_cast<int16_t>(MEA_P.Gain_Gain[0]) : 0);
  rtb_AxBu_idx_1 = (MEA_DW.PreviousState_DSTATE[0] * MEA_P.Gain1_Gain_k[1] +
                    MEA_DW.PreviousState_DSTATE[1] * MEA_P.Gain1_Gain_k[3]) +
    static_cast<float>(tmp ? static_cast<int16_t>(MEA_P.Gain_Gain[1]) : 0);

  // Outputs for Enabled SubSystem: '<S2>/Active PT Correction' incorporates:
  //   EnablePort: '<S4>/Enable'

  // Outputs for Enabled SubSystem: '<S2>/Non Active PT Correction' incorporates:
  //   EnablePort: '<S6>/Enable'

  // Logic: '<S2>/AND' incorporates:
  //   Constant: '<S5>/Constant'
  //   Inport: '<Root>/MEA In'
  //   RelationalOperator: '<S2>/Relational Operator'
  //   RelationalOperator: '<S5>/Compare'
  //   UnitDelay: '<S2>/Unit Delay'

  if ((MEA_DW.UnitDelay_DSTATE < MEA_U.MEAIn_o.CCPTTimestamp) && (rtb_CPCR !=
       MEA_P.CompareToConstant_const)) {
    // Product: '<S4>/Divide' incorporates:
    //   Bias: '<S3>/Bias'
    //   Gain: '<S4>/Gain4'

    rtb_CPCR = 1.0F / rtb_CPCR;
    rtb_K_h[0] = (rtb_APAQ[0] * MEA_P.Gain4_Gain[0] + MEA_P.Gain4_Gain[1] *
                  rtb_APAQ[2]) * rtb_CPCR;
    rtb_K_h[1] = (MEA_P.Gain4_Gain[0] * rtb_APAQ[1] + MEA_P.Gain4_Gain[1] *
                  rtb_APAQ[3]) * rtb_CPCR;

    // Sum: '<S4>/Add' incorporates:
    //   Gain: '<S4>/Gain1'
    //   IdentityMatrix: '<S4>/IdentityMatrix'
    //   Product: '<S4>/Divide'
    //   Product: '<S4>/Matrix Multiply4'

    rtb_CPCR = MEA_DW.I[0] - rtb_K_h[0] * MEA_P.Gain1_Gain[0];
    rtb_KRK_idx_1 = MEA_DW.I[1] - MEA_P.Gain1_Gain[0] * rtb_K_h[1];
    rtb_KRK_idx_2 = MEA_DW.I[2] - rtb_K_h[0] * MEA_P.Gain1_Gain[1];
    rtb_KRK_idx_3 = MEA_DW.I[3] - rtb_K_h[1] * MEA_P.Gain1_Gain[1];
    for (i = 0; i < 2; i++) {
      // Product: '<S4>/Matrix Multiply2' incorporates:
      //   Bias: '<S3>/Bias'
      //   Math: '<S4>/Transpose1'
      //   Product: '<S4>/Matrix Multiply4'

      Gain3_Gain = rtb_APAQ[i + 2];
      Gain3_Gain_0 = rtb_APAQ[i];
      rtb_APAQ_0[i] = Gain3_Gain * rtb_KRK_idx_2 + Gain3_Gain_0 * rtb_CPCR;

      // Product: '<S4>/Matrix Multiply4' incorporates:
      //   Constant: '<S4>/Constant1'
      //   Math: '<S4>/Transpose2'
      //   Product: '<S4>/Divide'

      Gain3_Gain_1 = MEA_P.Constant1_Value * rtb_K_h[i];
      rtb_K_j_tmp = i << 1;
      rtb_K_j[rtb_K_j_tmp] = Gain3_Gain_1 * rtb_K_h[0];

      // Product: '<S4>/Matrix Multiply2' incorporates:
      //   Math: '<S4>/Transpose1'
      //   Product: '<S4>/Matrix Multiply4'

      rtb_APAQ_0[i + 2] = Gain3_Gain * rtb_KRK_idx_3 + Gain3_Gain_0 *
        rtb_KRK_idx_1;

      // Product: '<S4>/Matrix Multiply4' incorporates:
      //   Product: '<S4>/Divide'

      rtb_K_j[rtb_K_j_tmp + 1] = Gain3_Gain_1 * rtb_K_h[1];
    }

    // Product: '<S4>/Matrix Multiply2' incorporates:
    //   Product: '<S4>/Matrix Multiply4'

    for (i = 0; i < 2; i++) {
      rtb_K_j_tmp = i << 1;
      Gain3_Gain = rtb_APAQ_0[rtb_K_j_tmp + 1];
      Gain3_Gain_0 = rtb_APAQ_0[rtb_K_j_tmp];
      rtb_KRK_0[rtb_K_j_tmp] = Gain3_Gain * rtb_KRK_idx_2 + Gain3_Gain_0 *
        rtb_CPCR;
      rtb_KRK_0[rtb_K_j_tmp + 1] = Gain3_Gain * rtb_KRK_idx_3 + Gain3_Gain_0 *
        rtb_KRK_idx_1;
    }

    // Sum: '<S4>/Add3' incorporates:
    //   Merge: '<S2>/Merge1'

    rtb_APAQ[0] = rtb_K_j[0] + rtb_KRK_0[0];
    rtb_APAQ[1] = rtb_K_j[1] + rtb_KRK_0[1];
    rtb_APAQ[2] = rtb_K_j[2] + rtb_KRK_0[2];
    rtb_APAQ[3] = rtb_K_j[3] + rtb_KRK_0[3];

    // Sum: '<S4>/Add1' incorporates:
    //   Gain: '<S4>/Gain2'
    //   Sum: '<S3>/Add1'

    rtb_CPCR = MEA_U.MEAIn_o.CCPTMeasure - (MEA_P.Gain2_Gain[0] * rtb_AxBu_idx_0
      + MEA_P.Gain2_Gain[1] * rtb_AxBu_idx_1);

    // Sum: '<S4>/Add2' incorporates:
    //   Merge: '<S2>/Merge'
    //   Product: '<S4>/Divide'
    //   Product: '<S4>/Product'
    //   Sum: '<S3>/Add1'

    rtb_AxBu_idx_0 += rtb_K_h[0] * rtb_CPCR;
    rtb_AxBu_idx_1 += rtb_K_h[1] * rtb_CPCR;
  }

  // End of Logic: '<S2>/AND'
  // End of Outputs for SubSystem: '<S2>/Non Active PT Correction'
  // End of Outputs for SubSystem: '<S2>/Active PT Correction'

  // Gain: '<S1>/Gain1' incorporates:
  //   Merge: '<S2>/Merge'

  rtb_CPCR = MEA_P.Gain1_Gain_f[0] * rtb_AxBu_idx_0 + MEA_P.Gain1_Gain_f[1] *
    rtb_AxBu_idx_1;

  // Sum: '<S1>/Subtract' incorporates:
  //   Gain: '<S1>/Gain1'
  //   Gain: '<S1>/Gain2'
  //   Gain: '<S1>/Gain3'
  //   UnitDelay: '<S1>/Unit Delay2'

  rtb_KRK_idx_1 = MEA_DW.UnitDelay2_DSTATE - MEA_P.Gain2_Gain_n * rtb_CPCR *
    MEA_P.Gain3_Gain_c;

  // Saturate: '<S1>/Saturation'
  if (rtb_KRK_idx_1 > MEA_P.Saturation_UpperSat) {
    rtb_KRK_idx_1 = MEA_P.Saturation_UpperSat;
  } else if (rtb_KRK_idx_1 < MEA_P.Saturation_LowerSat) {
    rtb_KRK_idx_1 = MEA_P.Saturation_LowerSat;
  }

  // BusCreator generated from: '<S1>/MEA Logs OBSW_BusCreator' incorporates:
  //   Gain: '<S1>/Gain1'
  //   Merge: '<S2>/Merge'
  //   Outport: '<Root>/MEA Logs OBSW'
  //   Saturate: '<S1>/Saturation'

  MEA_Y.MEALogsOBSW.Timestamp = 0ULL;
  MEA_Y.MEALogsOBSW.Mass = rtb_KRK_idx_1;
  MEA_Y.MEALogsOBSW.States[0] = rtb_AxBu_idx_0;
  MEA_Y.MEALogsOBSW.States[1] = rtb_AxBu_idx_1;
  MEA_Y.MEALogsOBSW.Pressure = rtb_CPCR;

  // BusCreator generated from: '<S1>/MEA Out_BusCreator' incorporates:
  //   Outport: '<Root>/MEA Out'
  //   Saturate: '<S1>/Saturation'

  MEA_Y.MEAOut_k.Timestamp = 0ULL;
  MEA_Y.MEAOut_k.Mass = rtb_KRK_idx_1;

  // Update for UnitDelay: '<S1>/Previous Covariance' incorporates:
  //   Merge: '<S2>/Merge1'

  MEA_DW.PreviousCovariance_DSTATE[0] = rtb_APAQ[0];
  MEA_DW.PreviousCovariance_DSTATE[1] = rtb_APAQ[1];
  MEA_DW.PreviousCovariance_DSTATE[2] = rtb_APAQ[2];
  MEA_DW.PreviousCovariance_DSTATE[3] = rtb_APAQ[3];

  // Update for UnitDelay: '<S2>/Unit Delay' incorporates:
  //   Inport: '<Root>/MEA In'

  MEA_DW.UnitDelay_DSTATE = MEA_U.MEAIn_o.CCPTTimestamp;

  // Update for UnitDelay: '<S1>/Previous State' incorporates:
  //   Merge: '<S2>/Merge'

  MEA_DW.PreviousState_DSTATE[0] = rtb_AxBu_idx_0;
  MEA_DW.PreviousState_DSTATE[1] = rtb_AxBu_idx_1;

  // Update for UnitDelay: '<S1>/Unit Delay2' incorporates:
  //   Saturate: '<S1>/Saturation'

  MEA_DW.UnitDelay2_DSTATE = rtb_KRK_idx_1;

  // End of Outputs for SubSystem: '<Root>/MEA II - Autocoding'
}

// Model initialize function
void MEA::initialize()
{
  // SystemInitialize for Atomic SubSystem: '<Root>/MEA II - Autocoding'
  // InitializeConditions for UnitDelay: '<S1>/Previous Covariance'
  MEA_DW.PreviousCovariance_DSTATE[0] = MEA_P.PreviousCovariance_InitialCondi[0];
  MEA_DW.PreviousCovariance_DSTATE[1] = MEA_P.PreviousCovariance_InitialCondi[1];
  MEA_DW.PreviousCovariance_DSTATE[2] = MEA_P.PreviousCovariance_InitialCondi[2];
  MEA_DW.PreviousCovariance_DSTATE[3] = MEA_P.PreviousCovariance_InitialCondi[3];

  // InitializeConditions for UnitDelay: '<S2>/Unit Delay'
  MEA_DW.UnitDelay_DSTATE = MEA_P.UnitDelay_InitialCondition;

  // InitializeConditions for UnitDelay: '<S1>/Previous State'
  MEA_DW.PreviousState_DSTATE[0] = MEA_P.PreviousState_InitialCondition[0];
  MEA_DW.PreviousState_DSTATE[1] = MEA_P.PreviousState_InitialCondition[1];

  // InitializeConditions for UnitDelay: '<S1>/Unit Delay2'
  MEA_DW.UnitDelay2_DSTATE = MEA_P.UnitDelay2_InitialCondition;

  // SystemInitialize for Enabled SubSystem: '<S2>/Active PT Correction'
  // Start for IdentityMatrix: '<S4>/IdentityMatrix'
  MEA_DW.I[0] = MEA_P.IdentityMatrix_IDMatrixData[0];
  MEA_DW.I[1] = MEA_P.IdentityMatrix_IDMatrixData[1];
  MEA_DW.I[2] = MEA_P.IdentityMatrix_IDMatrixData[2];
  MEA_DW.I[3] = MEA_P.IdentityMatrix_IDMatrixData[3];

  // End of SystemInitialize for SubSystem: '<S2>/Active PT Correction'
  // End of SystemInitialize for SubSystem: '<Root>/MEA II - Autocoding'
}

// Model terminate function
void MEA::terminate()
{
  // (no terminate code required)
}

// Constructor
MEA::MEA():
  MEA_U(),
  MEA_Y(),
  MEA_DW()
{
  // Currently there is no constructor body generated.
}

// Destructor
// Currently there is no destructor body generated.
MEA::~MEA() = default;
}
//
// File trailer for generated code.
//
// [EOF]
//
