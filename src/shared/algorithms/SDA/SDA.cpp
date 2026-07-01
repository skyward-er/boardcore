//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// File: SDA.cpp
//
// Code generated for Simulink model 'SDA'.
//
// Model version                  : 11.329
// Simulink Coder version         : 24.2 (R2024b) 21-Jun-2024
// C/C++ source code generated on : Mon Jun 29 14:46:21 2026
//
// Target selection: ert.tlc
// Embedded hardware selection: ARM Compatible->ARM Cortex-M
// Code generation objectives:
//    1. Execution efficiency
//    2. Debugging
//    3. RAM efficiency
// Validation result: Not run
//
#include "SDA.h"
#include <cstring>
#include <cmath>
#include <stdint.h>
#include <stdbool.h>
#include "SDA_private.h"

namespace SDA 
{


// Model step function
void SDA::step()
{
  float rtb_t[30];
  float rtb_ty[30];
  float rtb_y[30];
  float rtb_VectorConcatenate1[5];
  float rtb_Bias1;
  float rtb_Gain3;
  float rtb_Merge;
  float rtb_slope;
  float rtb_sumtsumy;
  int32_t idxS;
  int32_t idxV;
  int32_t j;
  int32_t k;
  int32_t rtb_Power_tmp;
  uint8_t rtb_residual;
  bool rtb_Compare_n;

  // Outputs for Atomic SubSystem: '<Root>/SDA'
  // Selector: '<S6>/Kriging X' incorporates:
  //   Constant: '<S6>/Kriging Data'
  //   Math: '<S6>/Power'

  std::memcpy(&SDA_DW.Power[0], &kriging[0], 1600U * sizeof(float));

  // SignalConversion generated from: '<S1>/Vector Concatenate1' incorporates:
  //   Inport: '<Root>/SDA IN'

  rtb_VectorConcatenate1[0] = SDA_U.SDAIN.ANASPosition[2];

  // SignalConversion generated from: '<S1>/Vector Concatenate1' incorporates:
  //   Inport: '<Root>/SDA IN'

  rtb_VectorConcatenate1[1] = SDA_U.SDAIN.ANASVelocity[2];

  // Sqrt: '<S8>/Sqrt' incorporates:
  //   DotProduct: '<S8>/Dot Product'
  //   Inport: '<Root>/SDA IN'
  //   Math: '<S8>/Transpose'

  rtb_Merge = std::sqrt(SDA_U.SDAIN.ANASVelocity[0] * SDA_U.SDAIN.ANASVelocity[0]
                        + SDA_U.SDAIN.ANASVelocity[1] *
                        SDA_U.SDAIN.ANASVelocity[1]);

  // SignalConversion generated from: '<S1>/Vector Concatenate1'
  rtb_VectorConcatenate1[2] = rtb_Merge;

  // Trigonometry: '<S1>/Atan2' incorporates:
  //   Inport: '<Root>/SDA IN'

  rtb_VectorConcatenate1[3] = std::atan2(SDA_U.SDAIN.ANASVelocity[2], rtb_Merge);

  // SignalConversion generated from: '<S1>/Vector Concatenate1' incorporates:
  //   Inport: '<Root>/SDA IN'

  rtb_VectorConcatenate1[4] = SDA_U.SDAIN.MEAMass;

  // Product: '<S5>/Divide' incorporates:
  //   Constant: '<S1>/Constant'
  //   Selector: '<S5>/Selector'
  //   Selector: '<S5>/Selector1'
  //   Sum: '<S5>/Subtract'
  //   Sum: '<S5>/Subtract1'

  idxS = 0;
  for (idxV = 0; idxV < 5; idxV++) {
    // Selector: '<S5>/Selector1' incorporates:
    //   Constant: '<S1>/Constant'

    rtb_Bias1 = SDA_P.Constant_Value_d[idxS];
    rtb_VectorConcatenate1[idxV] = (rtb_VectorConcatenate1[idxV] - rtb_Bias1) /
      (SDA_P.Constant_Value_d[idxS + 1] - rtb_Bias1);
    idxS += 2;
  }

  // End of Product: '<S5>/Divide'

  // S-Function (sdspdmult2): '<S6>/Array-Vector Subtract' incorporates:
  //   Math: '<S6>/Power'
  //   Product: '<S5>/Divide'

  idxS = 0;
  idxV = 0;
  for (k = 0; k < 5; k++) {
    for (j = 0; j < 320; j++) {
      rtb_Power_tmp = idxS + j;
      SDA_DW.Power[rtb_Power_tmp] -= rtb_VectorConcatenate1[idxV];
    }

    idxS += 320;
    idxV++;
  }

  // Product: '<S6>/Product' incorporates:
  //   Abs: '<S6>/Abs'
  //   Constant: '<S6>/Kriging Data'
  //   Math: '<S6>/Power'
  //   Selector: '<S6>/Kriging P'
  //   Selector: '<S6>/Kriging Theta'

  for (idxS = 0; idxS < 1600; idxS++) {
    SDA_DW.Power[idxS] = std::pow(std::abs(SDA_DW.Power[idxS]), kriging[idxS +
      3200]) * kriging[idxS + 1600];
  }

  // End of Product: '<S6>/Product'

  // DotProduct: '<S6>/Dot Product'
  rtb_Gain3 = 0.0F;
  for (k = 0; k < 320; k++) {
    // Sum: '<S6>/Sum of Elements' incorporates:
    //   Product: '<S6>/Product'
    //   S-Function (sdspdmult2): '<S6>/Array-Vector Subtract'
    //   Sum: '<S9>/Sum of Elements'
    //   Sum: '<S9>/Sum of Elements1'
    //   Sum: '<S9>/Sum of Elements2'
    //   Sum: '<S9>/Sum of Elements3'

    rtb_Bias1 = -0.0F;
    idxS = 0;
    for (idxV = 0; idxV < 5; idxV++) {
      rtb_Bias1 += SDA_DW.Power[idxS + k];
      idxS += 320;
    }

    // DotProduct: '<S6>/Dot Product' incorporates:
    //   Constant: '<S6>/Kriging Data'
    //   Gain: '<S6>/Gain'
    //   Math: '<S6>/Power1'
    //   S-Function (sdspdmult2): '<S6>/Array-Vector Subtract'
    //   Selector: '<S6>/Kriging Psi'
    //   Sum: '<S6>/Sum of Elements'
    //   Sum: '<S9>/Sum of Elements'
    //   Sum: '<S9>/Sum of Elements1'
    //   Sum: '<S9>/Sum of Elements2'
    //   Sum: '<S9>/Sum of Elements3'
    //
    //  About '<S6>/Power1':
    //   Operator: exp

    rtb_Gain3 += std::exp(SDA_P.Gain_Gain_m * rtb_Bias1) * kriging[k + 4800];
  }

  // Bias: '<S6>/Bias1' incorporates:
  //   Bias: '<S6>/Bias'
  //   DotProduct: '<S6>/Dot Product'
  //   Gain: '<S6>/Gain2'

  rtb_Bias1 = (rtb_Gain3 + SDA_P.Bias_Bias) * SDA_P.Gain2_Gain_i +
    SDA_P.Bias1_Bias;

  // If: '<S1>/If' incorporates:
  //   Constant: '<S1>/Constant2'

  if (SDA_P.Constant2_Value) {
    // Outputs for IfAction SubSystem: '<S1>/Propagation' incorporates:
    //   ActionPort: '<S7>/Action Port'

    // SignalConversion generated from: '<S9>/Vector Concatenate' incorporates:
    //   UnitDelay: '<S9>/prediction_samples'

    std::memcpy(&rtb_y[0], &SDA_DW.prediction_samples_DSTATE[1], 29U * sizeof
                (float));

    // SignalConversion generated from: '<S9>/Vector Concatenate1' incorporates:
    //   UnitDelay: '<S9>/time_samples'

    std::memcpy(&rtb_t[0], &SDA_DW.time_samples_DSTATE[1], 29U * sizeof(float));

    // SignalConversion generated from: '<S9>/Vector Concatenate'
    rtb_y[29] = rtb_Bias1;

    // Gain: '<S9>/Gain2' incorporates:
    //   UnitDelay: '<S9>/time_samples1'

    rtb_t[29] = SDA_P.Gain2_Gain * SDA_DW.time_samples1_DSTATE;

    // Sum: '<S9>/Sum of Elements3'
    rtb_Gain3 = -0.0F;
    for (k = 0; k < 30; k++) {
      // Product: '<S9>/Product'
      rtb_Merge = rtb_t[k];
      rtb_ty[k] = rtb_y[k] * rtb_Merge;

      // Sum: '<S9>/Sum of Elements3' incorporates:
      //   Math: '<S9>/Square'

      rtb_Gain3 += rtb_Merge * rtb_Merge;
    }

    // Gain: '<S9>/Gain1'
    rtb_Merge = SDA_P.Gain1_Gain * rtb_Gain3;

    // Sum: '<S9>/Sum of Elements'
    rtb_Gain3 = -0.0F;

    // Sum: '<S9>/Sum of Elements2'
    rtb_slope = -0.0F;
    for (j = 0; j < 30; j++) {
      // Sum: '<S9>/Sum of Elements' incorporates:
      //   S-Function (sdspdmult2): '<S6>/Array-Vector Subtract'
      //   Sum: '<S6>/Sum of Elements'
      //   Sum: '<S9>/Sum of Elements1'
      //   Sum: '<S9>/Sum of Elements2'
      //   Sum: '<S9>/Sum of Elements3'

      rtb_Gain3 += rtb_t[j];

      // Sum: '<S9>/Sum of Elements2' incorporates:
      //   S-Function (sdspdmult2): '<S6>/Array-Vector Subtract'
      //   Sum: '<S6>/Sum of Elements'
      //   Sum: '<S9>/Sum of Elements'
      //   Sum: '<S9>/Sum of Elements1'
      //   Sum: '<S9>/Sum of Elements3'

      rtb_slope += rtb_y[j];
    }

    // Product: '<S9>/Product1'
    rtb_sumtsumy = rtb_slope * rtb_Gain3;

    // Sum: '<S9>/Sum of Elements1'
    rtb_slope = -0.0F;

    // Sum: '<S7>/Subtract1' incorporates:
    //   Constant: '<S7>/Constant11'
    //   UnitDelay: '<S1>/Unit Delay'

    rtb_residual = static_cast<uint8_t>(SDA_P.Constant11_Value -
      SDA_DW.UnitDelay_DSTATE);
    for (j = 0; j < 30; j++) {
      // Sum: '<S9>/Sum of Elements1' incorporates:
      //   S-Function (sdspdmult2): '<S6>/Array-Vector Subtract'
      //   Sum: '<S6>/Sum of Elements'
      //   Sum: '<S9>/Sum of Elements'
      //   Sum: '<S9>/Sum of Elements2'
      //   Sum: '<S9>/Sum of Elements3'

      rtb_slope += rtb_ty[j];

      // Update for UnitDelay: '<S9>/prediction_samples' incorporates:
      //   S-Function (sdspdmult2): '<S6>/Array-Vector Subtract'
      //   Sum: '<S6>/Sum of Elements'
      //   Sum: '<S9>/Sum of Elements'
      //   Sum: '<S9>/Sum of Elements1'
      //   Sum: '<S9>/Sum of Elements2'
      //   Sum: '<S9>/Sum of Elements3'

      SDA_DW.prediction_samples_DSTATE[j] = rtb_y[j];

      // Update for UnitDelay: '<S9>/time_samples' incorporates:
      //   S-Function (sdspdmult2): '<S6>/Array-Vector Subtract'
      //   Sum: '<S6>/Sum of Elements'
      //   Sum: '<S9>/Sum of Elements'
      //   Sum: '<S9>/Sum of Elements1'
      //   Sum: '<S9>/Sum of Elements2'
      //   Sum: '<S9>/Sum of Elements3'

      SDA_DW.time_samples_DSTATE[j] = rtb_t[j];
    }

    // Product: '<S9>/Divide' incorporates:
    //   Gain: '<S9>/Gain'
    //   Math: '<S9>/Square1'
    //   Sum: '<S9>/Minus'
    //   Sum: '<S9>/Subtract'

    rtb_slope = (SDA_P.Gain_Gain * rtb_slope - rtb_sumtsumy) / (rtb_Merge -
      rtb_Gain3 * rtb_Gain3);

    // Switch: '<S7>/Switch4' incorporates:
    //   Constant: '<S7>/Constant12'

    if (rtb_residual <= SDA_P.Switch4_Threshold) {
      rtb_residual = SDA_P.Constant12_Value;
    }

    // Sum: '<S7>/Add1' incorporates:
    //   Constant: '<S7>/Constant15'
    //   Gain: '<S7>/Gain3'
    //   Product: '<S7>/Product'
    //   Sum: '<S7>/Add4'
    //   Switch: '<S7>/Switch4'

    rtb_Merge = (static_cast<float>(SDA_P.Gain3_Gain) * 0.000122070312F *
                 static_cast<float>(rtb_residual) + SDA_P.Constant15_Value) *
      rtb_slope + rtb_Bias1;

    // SignalConversion generated from: '<S7>/Derivative'
    SDA_Y.SDALogsOBSW.Apogee[2] = rtb_slope;

    // Update for UnitDelay: '<S9>/time_samples1' incorporates:
    //   Constant: '<S9>/Constant'
    //   Sum: '<S9>/Add'

    SDA_DW.time_samples1_DSTATE += SDA_P.Constant_Value;

    // End of Outputs for SubSystem: '<S1>/Propagation'
  } else {
    // Outputs for IfAction SubSystem: '<S1>/No Propagation' incorporates:
    //   ActionPort: '<S4>/Action Port'

    // SignalConversion generated from: '<S4>/Derivative' incorporates:
    //   Constant: '<S4>/Zero'

    SDA_Y.SDALogsOBSW.Apogee[2] = SDA_P.Zero_Value;

    // SignalConversion generated from: '<S4>/Apogee'
    rtb_Merge = rtb_Bias1;

    // End of Outputs for SubSystem: '<S1>/No Propagation'
  }

  // End of If: '<S1>/If'

  // Switch: '<S1>/Switch3' incorporates:
  //   Constant: '<S2>/Constant'
  //   RelationalOperator: '<S2>/Compare'

  if (rtb_Merge >= SDA_P.CompareToConstant2_const) {
    // Saturate: '<S1>/Saturation' incorporates:
    //   Constant: '<S1>/One'

    k = SDA_P.One_Value;
  } else {
    // Saturate: '<S1>/Saturation' incorporates:
    //   Gain: '<S1>/Gain1'
    //   UnitDelay: '<S1>/Unit Delay'

    k = SDA_P.Gain1_Gain_g * SDA_DW.UnitDelay_DSTATE;
  }

  // End of Switch: '<S1>/Switch3'

  // Saturate: '<S1>/Saturation'
  if (k > SDA_P.Saturation_UpperSat) {
    k = SDA_P.Saturation_UpperSat;
  } else if (k < SDA_P.Saturation_LowerSat) {
    k = SDA_P.Saturation_LowerSat;
  }

  // Sum: '<S1>/Add' incorporates:
  //   Saturate: '<S1>/Saturation'
  //   UnitDelay: '<S1>/Unit Delay'

  rtb_residual = static_cast<uint8_t>(static_cast<uint32_t>(k) +
    SDA_DW.UnitDelay_DSTATE);

  // RelationalOperator: '<S3>/Compare' incorporates:
  //   Constant: '<S3>/Constant'
  //   Sum: '<S1>/Add'

  rtb_Compare_n = (rtb_residual >= SDA_P.CompareToConstant3_const);

  // SignalConversion generated from: '<S1>/Vector Concatenate'
  SDA_Y.SDALogsOBSW.Apogee[0] = rtb_Bias1;

  // SignalConversion generated from: '<S1>/Vector Concatenate'
  SDA_Y.SDALogsOBSW.Apogee[1] = rtb_Merge;

  // BusCreator generated from: '<S1>/SDA Logs OBSW_BusCreator' incorporates:
  //   Outport: '<Root>/SDA Logs OBSW'
  //   Sum: '<S1>/Add'

  SDA_Y.SDALogsOBSW.ShutdownCommand = rtb_Compare_n;
  SDA_Y.SDALogsOBSW.Timestamp = 0ULL;
  SDA_Y.SDALogsOBSW.ShutdownCounter = rtb_residual;

  // Update for UnitDelay: '<S1>/Unit Delay' incorporates:
  //   Sum: '<S1>/Add'

  SDA_DW.UnitDelay_DSTATE = rtb_residual;

  // End of Outputs for SubSystem: '<Root>/SDA'

  // Outport: '<Root>/SDA Shutdown'
  SDA_Y.SDAShutdown = rtb_Compare_n;
}

// Model initialize function
void SDA::initialize()
{
  // SystemInitialize for Atomic SubSystem: '<Root>/SDA'
  // InitializeConditions for UnitDelay: '<S1>/Unit Delay'
  SDA_DW.UnitDelay_DSTATE = SDA_P.UnitDelay_InitialCondition;

  // SystemInitialize for IfAction SubSystem: '<S1>/Propagation'
  // InitializeConditions for UnitDelay: '<S9>/prediction_samples'
  std::memcpy(&SDA_DW.prediction_samples_DSTATE[0],
              &SDA_P.prediction_samples_InitialCondi[0], 30U * sizeof(float));

  // InitializeConditions for UnitDelay: '<S9>/time_samples' incorporates:
  //   UnitDelay: '<S9>/prediction_samples'

  std::memcpy(&SDA_DW.time_samples_DSTATE[0],
              &SDA_P.time_samples_InitialCondition[0], 30U * sizeof(float));

  // InitializeConditions for UnitDelay: '<S9>/time_samples1'
  SDA_DW.time_samples1_DSTATE = SDA_P.time_samples1_InitialCondition;

  // End of SystemInitialize for SubSystem: '<S1>/Propagation'
  // End of SystemInitialize for SubSystem: '<Root>/SDA'
}

// Model terminate function
void SDA::terminate()
{
  // (no terminate code required)
}

// Constructor
SDA::SDA():
  SDA_U(),
  SDA_Y(),
  SDA_DW()
{
  // Currently there is no constructor body generated.
}

// Destructor
// Currently there is no destructor body generated.
SDA::~SDA() = default;


} // fine namespace SDA

//
// File trailer for generated code.
//
// [EOF]
//
