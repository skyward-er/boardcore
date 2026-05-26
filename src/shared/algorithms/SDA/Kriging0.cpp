//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// File: Kriging0.cpp
//
// Code generated for Simulink model 'Kriging0'.
//
// Model version                  : 11.257
// Simulink Coder version         : 24.2 (R2024b) 21-Jun-2024
// C/C++ source code generated on : Sun May 24 14:54:33 2026
//
// Target selection: ert.tlc
// Embedded hardware selection: STMicroelectronics->ST10/Super10
// Code generation objectives:
//    1. RAM efficiency
//    2. Execution efficiency
//    3. Debugging
// Validation result: Not run
//
#include "Kriging0.h"
#include <cmath>
#include <stdint.h>
#include "Kriging0_types.h"
#include <stdbool.h>

// Model step function
void Kriging0::step()
{
  float rtb_VectorConcatenate1[5];
  float kriging_ranges;
  float rtb_Bias1;
  int32_t idxS;
  int32_t idxV;
  int32_t rtb_Gain1_tmp;
  int16_t i;
  int16_t j;
  int16_t k;
  uint8_t rtb_Add;
  bool rtb_Compare_c;

  // Outputs for Atomic SubSystem: '<Root>/Kriging'
  // SignalConversion generated from: '<S1>/Vector Concatenate1' incorporates:
  //   Inport: '<Root>/SDA IN'

  rtb_VectorConcatenate1[0] = Kriging0_U.SDAIN.ANASPosition[2];

  // SignalConversion generated from: '<S1>/Vector Concatenate1' incorporates:
  //   Inport: '<Root>/SDA IN'

  rtb_VectorConcatenate1[1] = Kriging0_U.SDAIN.ANASVelocity[2];

  // Sqrt: '<S8>/Sqrt' incorporates:
  //   DotProduct: '<S8>/Dot Product'
  //   Inport: '<Root>/SDA IN'
  //   Math: '<S8>/Transpose'

  rtb_Bias1 = std::sqrt(Kriging0_U.SDAIN.ANASVelocity[0] *
                        Kriging0_U.SDAIN.ANASVelocity[0] +
                        Kriging0_U.SDAIN.ANASVelocity[1] *
                        Kriging0_U.SDAIN.ANASVelocity[1]);

  // SignalConversion generated from: '<S1>/Vector Concatenate1'
  rtb_VectorConcatenate1[2] = rtb_Bias1;

  // Trigonometry: '<S1>/Atan2' incorporates:
  //   Inport: '<Root>/SDA IN'

  rtb_VectorConcatenate1[3] = std::atan2(Kriging0_U.SDAIN.ANASVelocity[2],
    rtb_Bias1);

  // SignalConversion generated from: '<S1>/Vector Concatenate1' incorporates:
  //   Inport: '<Root>/SDA IN'

  rtb_VectorConcatenate1[4] = Kriging0_U.SDAIN.MEAMass;

  // Product: '<S5>/Divide' incorporates:
  //   Constant: '<S1>/Constant'
  //   Selector: '<S5>/Selector'
  //   Selector: '<S5>/Selector1'
  //   Sum: '<S5>/Subtract'
  //   Sum: '<S5>/Subtract1'

  k = 0;
  for (j = 0; j < 5; j++) {
    // Selector: '<S5>/Selector1' incorporates:
    //   Constant: '<S1>/Constant'

    kriging_ranges = kriging.ranges[k];
    rtb_VectorConcatenate1[j] = (rtb_VectorConcatenate1[j] - kriging_ranges) /
      (kriging.ranges[k + 1] - kriging_ranges);
    k += 2;
  }

  // End of Product: '<S5>/Divide'

  // S-Function (sdspdmult2): '<S6>/Array-Vector Subtract' incorporates:
  //   Constant: '<S1>/Constant3'
  //   Math: '<S6>/Power'
  //   Product: '<S5>/Divide'

  idxS = 0L;
  idxV = 0L;
  for (k = 0; k < 5; k++) {
    for (j = 0; j < 320; j++) {
      rtb_Gain1_tmp = idxS + j;
      Kriging0_DW.Gain1[rtb_Gain1_tmp] = kriging.x[rtb_Gain1_tmp] -
        rtb_VectorConcatenate1[idxV];
    }

    idxS += 320L;
    idxV++;
  }

  // End of S-Function (sdspdmult2): '<S6>/Array-Vector Subtract'

  // Gain: '<S6>/Gain1' incorporates:
  //   Abs: '<S6>/Abs'
  //   Constant: '<S1>/Constant6'
  //   Math: '<S6>/Power'

  for (k = 0; k < 1600; k++) {
    Kriging0_DW.Gain1[k] = kriging.theta[k] * std::pow(std::abs
      (Kriging0_DW.Gain1[k]), kriging.p[k]);
  }

  // End of Gain: '<S6>/Gain1'

  // DotProduct: '<S6>/Dot Product'
  kriging_ranges = 0.0F;
  for (k = 0; k < 320; k++) {
    // Sum: '<S6>/Sum of Elements' incorporates:
    //   Gain: '<S6>/Gain1'

    rtb_Bias1 = -0.0F;
    j = 0;
    for (i = 0; i < 5; i++) {
      rtb_Bias1 += Kriging0_DW.Gain1[j + k];
      j += 320;
    }

    // DotProduct: '<S6>/Dot Product' incorporates:
    //   Constant: '<S1>/Constant4'
    //   Gain: '<S6>/Gain'
    //   Math: '<S6>/Power1'
    //   Sum: '<S6>/Sum of Elements'
    //
    //  About '<S6>/Power1':
    //   Operator: exp

    kriging_ranges += std::exp(Kriging0_P.Gain_Gain * rtb_Bias1) *
      kriging.psiV[k];
  }

  // Bias: '<S6>/Bias1' incorporates:
  //   Bias: '<S6>/Bias'
  //   DotProduct: '<S6>/Dot Product'
  //   Gain: '<S6>/Gain2'

  rtb_Bias1 = (kriging_ranges + kriging.mu) * kriging.ySigma + kriging.yMu;

  // Switch: '<S1>/Switch3' incorporates:
  //   Constant: '<S1>/One'
  //   Constant: '<S2>/Constant'
  //   Gain: '<S1>/Gain1'
  //   RelationalOperator: '<S2>/Compare'
  //   UnitDelay: '<S1>/Unit Delay'

  if (rtb_Bias1 >= Kriging0_P.CompareToConstant2_const) {
    k = Kriging0_P.One_Value;
  } else {
    k = Kriging0_P.Gain1_Gain * Kriging0_DW.UnitDelay_DSTATE;
  }

  // End of Switch: '<S1>/Switch3'

  // Saturate: '<S1>/Saturation'
  if (k > Kriging0_P.Saturation_UpperSat) {
    k = Kriging0_P.Saturation_UpperSat;
  } else if (k < Kriging0_P.Saturation_LowerSat) {
    k = Kriging0_P.Saturation_LowerSat;
  }

  // Sum: '<S1>/Add' incorporates:
  //   Saturate: '<S1>/Saturation'
  //   UnitDelay: '<S1>/Unit Delay'

  rtb_Add = static_cast<uint8_t>(static_cast<uint16_t>(k) +
    Kriging0_DW.UnitDelay_DSTATE);

  // RelationalOperator: '<S3>/Compare' incorporates:
  //   Constant: '<S3>/Constant'
  //   Sum: '<S1>/Add'

  rtb_Compare_c = (rtb_Add >= Kriging0_P.CompareToConstant3_const);

  // SignalConversion generated from: '<S1>/Vector Concatenate' incorporates:
  //   BusCreator generated from: '<S1>/SDA Logs OBSW_BusCreator'
  //   Outport: '<Root>/SDA Logs OBSW'

  Kriging0_Y.SDALogsOBSW.Apogee[0] = rtb_Bias1;

  // SignalConversion generated from: '<S1>/Vector Concatenate' incorporates:
  //   BusCreator generated from: '<S1>/SDA Logs OBSW_BusCreator'
  //   Outport: '<Root>/SDA Logs OBSW'

  Kriging0_Y.SDALogsOBSW.Apogee[1] = rtb_Bias1;

  // BusCreator generated from: '<S1>/SDA Logs OBSW_BusCreator' incorporates:
  //   Outport: '<Root>/SDA Logs OBSW'
  //   Sum: '<S1>/Add'

  Kriging0_Y.SDALogsOBSW.ShutdownCommand = rtb_Compare_c;
  Kriging0_Y.SDALogsOBSW.Timestamp = 0ULL;
  Kriging0_Y.SDALogsOBSW.ShutdownCounter = rtb_Add;

  // Update for UnitDelay: '<S1>/Unit Delay' incorporates:
  //   Sum: '<S1>/Add'

  Kriging0_DW.UnitDelay_DSTATE = rtb_Add;

  // End of Outputs for SubSystem: '<Root>/Kriging'

  // Outport: '<Root>/SDA Shutdown'
  Kriging0_Y.SDAShutdown = rtb_Compare_c;
}

// Model initialize function
void Kriging0::initialize()
{
  // SystemInitialize for Atomic SubSystem: '<Root>/Kriging'
  // InitializeConditions for UnitDelay: '<S1>/Unit Delay'
  Kriging0_DW.UnitDelay_DSTATE = Kriging0_P.UnitDelay_InitialCondition;

  // End of SystemInitialize for SubSystem: '<Root>/Kriging'
}

// Model terminate function
void Kriging0::terminate()
{
  // (no terminate code required)
}

// Constructor
Kriging0::Kriging0():
  Kriging0_U(),
  Kriging0_Y(),
  Kriging0_DW()
{
  // Currently there is no constructor body generated.
}

// Destructor
// Currently there is no destructor body generated.
Kriging0::~Kriging0() = default;

//
// File trailer for generated code.
//
// [EOF]
//
