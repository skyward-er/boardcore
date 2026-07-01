//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// File: ABK.cpp
//
// Code generated for Simulink model 'ABK'.
//
// Model version                  : 11.328
// Simulink Coder version         : 24.2 (R2024b) 21-Jun-2024
// C/C++ source code generated on : Mon Jun 29 09:42:34 2026
//
// Target selection: ert.tlc
// Embedded hardware selection: ARM Compatible->ARM Cortex-M
// Code generation objectives:
//    1. Execution efficiency
//    2. Debugging
//    3. RAM efficiency
// Validation result: Not run
//
#include "ABK.h"
#include <cmath>
#include <cstring>
#include <stdint.h>
#include <stdbool.h>
#include "ABK_private.h"
#include "zero_crossing_types.h"

namespace ABK 
{


uint32_t plook_u32ff_evenx(float u, float bp0, float bpSpace, uint32_t maxIndex,
  float *fraction)
{
  uint32_t bpIndex;

  // Prelookup - Index and Fraction
  // Index Search method: 'even'
  // Extrapolation method: 'Linear'
  // Use previous index: 'off'
  // Use last breakpoint for index at or above upper limit: 'off'
  // Remove protection against out-of-range input in generated code: 'off'

  if (u <= bp0) {
    bpIndex = 0U;
    *fraction = (u - bp0) / bpSpace;
  } else {
    float fbpIndex;
    float invSpc;
    invSpc = 1.0F / bpSpace;
    fbpIndex = (u - bp0) * invSpc;
    if (fbpIndex < maxIndex) {
      bpIndex = static_cast<uint32_t>(fbpIndex);
      *fraction = (u - (static_cast<float>(static_cast<uint32_t>(fbpIndex)) *
                        bpSpace + bp0)) * invSpc;
    } else {
      bpIndex = maxIndex - 1U;
      *fraction = (u - (static_cast<float>(maxIndex - 1U) * bpSpace + bp0)) *
        invSpc;
    }
  }

  return bpIndex;
}

uint8_t plook_u8f_evencka(float u, float bp0, float bpSpace, uint32_t maxIndex)
{
  uint8_t bpIndex;

  // Prelookup - Index only
  // Index Search method: 'even'
  // Extrapolation method: 'Clip'
  // Use previous index: 'off'
  // Use last breakpoint for index at or above upper limit: 'on'
  // Remove protection against out-of-range input in generated code: 'off'

  if (u <= bp0) {
    bpIndex = 0U;
  } else {
    float fbpIndex;
    fbpIndex = (u - bp0) * (1.0F / bpSpace);
    if (fbpIndex < maxIndex) {
      bpIndex = static_cast<uint8_t>(fbpIndex);
    } else {
      bpIndex = static_cast<uint8_t>(maxIndex);
    }
  }

  return bpIndex;
}

uint16_t plook_u16ff_binc(float u, const float bp[], uint32_t maxIndex, float
  *fraction)
{
  uint16_t bpIndex;

  // Prelookup - Index and Fraction
  // Index Search method: 'binary'
  // Extrapolation method: 'Clip'
  // Use previous index: 'off'
  // Use last breakpoint for index at or above upper limit: 'off'
  // Remove protection against out-of-range input in generated code: 'off'

  if (u <= bp[0U]) {
    bpIndex = 0U;
    *fraction = 0.0F;
  } else if (u < bp[maxIndex]) {
    bpIndex = binsearch_u16f(u, bp, maxIndex >> 1U, maxIndex);
    *fraction = (u - bp[static_cast<uint32_t>(bpIndex)]) / (bp[bpIndex + 1U] -
      bp[static_cast<uint32_t>(bpIndex)]);
  } else {
    bpIndex = static_cast<uint16_t>(maxIndex - 1U);
    *fraction = 1.0F;
  }

  return bpIndex;
}

uint16_t plook_u16fdf_evenx(float u, double bp0, double bpSpace, uint32_t
  maxIndex, float *fraction)
{
  uint16_t bpIndex;

  // Prelookup - Index and Fraction
  // Index Search method: 'even'
  // Extrapolation method: 'Linear'
  // Use previous index: 'off'
  // Use last breakpoint for index at or above upper limit: 'off'
  // Remove protection against out-of-range input in generated code: 'off'

  if (u < bp0) {
    bpIndex = 0U;
    *fraction = static_cast<float>((u - bp0) / bpSpace);
  } else {
    double fbpIndex;
    double invSpc;
    invSpc = 1.0 / bpSpace;
    fbpIndex = (u - bp0) * invSpc;
    if (fbpIndex < maxIndex) {
      bpIndex = static_cast<uint16_t>(fbpIndex);
      *fraction = static_cast<float>((u - (static_cast<double>(static_cast<
        uint16_t>(fbpIndex)) * bpSpace + bp0)) * invSpc);
    } else {
      bpIndex = static_cast<uint16_t>(maxIndex - 1U);
      *fraction = static_cast<float>((u - (static_cast<double>(maxIndex - 1U) *
        bpSpace + bp0)) * invSpc);
    }
  }

  return bpIndex;
}

uint16_t binsearch_u16f(float u, const float bp[], uint32_t startIndex, uint32_t
  maxIndex)
{
  uint32_t bpIdx;
  uint32_t iLeft;
  uint32_t iRght;

  // Binary Search
  bpIdx = startIndex;
  iLeft = 0U;
  iRght = maxIndex;
  while (iRght - iLeft > 1U) {
    if (u < bp[bpIdx]) {
      iRght = bpIdx;
    } else {
      iLeft = bpIdx;
    }

    bpIdx = (iRght + iLeft) >> 1U;
  }

  return static_cast<uint16_t>(iLeft);
}

// Model step function
void ABK::step()
{
  double u0;
  float rtb_Add1[5];
  float rtb_Bypass_vect_idx_1;
  float rtb_Prelookup_o2;
  float rtb_Saturation2;
  float rtb_Selector1_j;
  float rtb_integralError;
  float rtb_zNAS;
  float u0_tmp_0;
  int32_t rtb_Massselector_tmp;
  int32_t rtb_Prelookup_o1_0;
  int32_t u0_tmp;
  uint32_t rtb_Prelookup_o1;
  uint16_t rtb_Prelookup1_o1;
  int8_t rtAction;
  int8_t rtPrevAction;
  uint8_t u0_0;
  bool rtb_Lowercondition;
  bool rtb_OR;
  bool rtb_OR_h;

  // Outputs for Atomic SubSystem: '<Root>/ABK'
  // If: '<S1>/If'
  rtPrevAction = ABK_DW.If_ActiveSubsystem;

  // Outputs for Atomic SubSystem: '<S1>/Mach Check'
  // DataTypeConversion: '<S20>/Cast To Double' incorporates:
  //   Gain: '<S3>/Gain'
  //   Inport: '<Root>/ABK In'

  u0 = ABK_P.Gain_Gain_f * ABK_U.ABKIn_l.ANASPosition[2];

  // Saturate: '<S22>/Limit  altitude  to troposhere'
  if (u0 > ABK_P.Limitaltitudetotroposhere_Upper) {
    u0 = ABK_P.Limitaltitudetotroposhere_Upper;
  } else if (u0 < ABK_P.Limitaltitudetotroposhere_Lower) {
    u0 = ABK_P.Limitaltitudetotroposhere_Lower;
  }

  // If: '<S1>/If' incorporates:
  //   Constant: '<S22>/Sea Level  Temperature'
  //   Constant: '<S3>/Constant1'
  //   DataTypeConversion: '<S20>/Data Type Conversion1'
  //   DotProduct: '<S21>/Dot Product'
  //   Gain: '<S22>/Lapse Rate'
  //   Gain: '<S22>/gamma*R'
  //   Inport: '<Root>/ABK In'
  //   Math: '<S21>/Transpose'
  //   Product: '<S3>/Divide'
  //   RelationalOperator: '<S3>/Relational Operator'
  //   Saturate: '<S22>/Limit  altitude  to troposhere'
  //   Sqrt: '<S21>/Sqrt'
  //   Sqrt: '<S22>/a'
  //   Sum: '<S22>/Sum1'
  //   UnitDelay: '<S2>/Unit Delay1'

  rtAction = static_cast<int8_t>(std::sqrt((ABK_U.ABKIn_l.ANASVelocity[0] *
    ABK_U.ABKIn_l.ANASVelocity[0] + ABK_U.ABKIn_l.ANASVelocity[1] *
    ABK_U.ABKIn_l.ANASVelocity[1]) + ABK_U.ABKIn_l.ANASVelocity[2] *
    ABK_U.ABKIn_l.ANASVelocity[2]) / static_cast<float>(std::sqrt
    ((ABK_P.SeaLevelTemperature_Value - ABK_P.LapseRate_Gain * u0) *
     ABK_P.gammaR_Gain)) >= ABK_P.Constant1_Value);

  // End of Outputs for SubSystem: '<S1>/Mach Check'
  ABK_DW.If_ActiveSubsystem = rtAction;
  if ((rtPrevAction != rtAction) && (rtPrevAction == 0)) {
    // Disable for If: '<S2>/If'
    ABK_DW.If_ActiveSubsystem_k = -1;
  }

  if (rtAction == 0) {
    // Outputs for IfAction SubSystem: '<S1>/Algorothm Execution' incorporates:
    //   ActionPort: '<S2>/Action Port'

    // Outputs for Triggered SubSystem: '<S2>/Function-Call Subsystem' incorporates:
    //   TriggerPort: '<S6>/One-shot trigger'

    if ((ABK_DW.UnitDelay1_DSTATE > 0) &&
        (ABK_PrevZCX.FunctionCallSubsystem_Trig_ZCE != POS_ZCSIG)) {
      // PreLookup: '<S6>/Mass_Prelookup' incorporates:
      //   Constant: '<S6>/Trajectories Matrix1'

      ABK_DW.massIndex = plook_u8f_evencka(ABK_U.ABKIn_l.MEAMass,
        ABK_P.TrajectoriesMatrix1_Value[0], ABK_P.TrajectoriesMatrix1_Value[1] -
        ABK_P.TrajectoriesMatrix1_Value[0], 10U);

      // Selector: '<S6>/Mass selector' incorporates:
      //   Constant: '<S6>/Trajectories Matrix'

      for (u0_tmp = 0; u0_tmp < 6; u0_tmp++) {
        for (rtb_Prelookup_o1_0 = 0; rtb_Prelookup_o1_0 < 301;
             rtb_Prelookup_o1_0++) {
          rtb_Massselector_tmp = 301 * u0_tmp + rtb_Prelookup_o1_0;
          ABK_DW.Massselector[rtb_Massselector_tmp] = abkTraj[1806 *
            ABK_DW.massIndex + rtb_Massselector_tmp];
        }
      }

      // End of Selector: '<S6>/Mass selector'

      // Selector: '<S6>/Select trajectories' incorporates:
      //   Selector: '<S6>/Mass selector'

      std::memcpy(&ABK_DW.Selecttrajectories[0], &ABK_DW.Massselector[301],
                  1505U * sizeof(float));

      // Selector: '<S6>/Select column z' incorporates:
      //   Selector: '<S6>/Mass selector'

      std::memcpy(&ABK_DW.Selectcolumnz[0], &ABK_DW.Massselector[0], 301U *
                  sizeof(float));
    }

    ABK_PrevZCX.FunctionCallSubsystem_Trig_ZCE = (ABK_DW.UnitDelay1_DSTATE > 0);

    // End of Outputs for SubSystem: '<S2>/Function-Call Subsystem'

    // Gain: '<S2>/Gain' incorporates:
    //   UnitDelay: '<S2>/Unit Delay1'

    rtb_zNAS = ABK_P.Gain_Gain * ABK_U.ABKIn_l.ANASPosition[2];

    // PreLookup: '<S9>/Prelookup' incorporates:
    //   Selector: '<S6>/Select column z'

    rtb_Prelookup_o1 = plook_u32ff_evenx(rtb_zNAS, ABK_DW.Selectcolumnz[0],
      ABK_DW.Selectcolumnz[1] - ABK_DW.Selectcolumnz[0], 300U, &rtb_Prelookup_o2);

    // Selector: '<S9>/Selector' incorporates:
    //   Bias: '<S9>/Bias'

    rtb_Prelookup_o1_0 = static_cast<int32_t>(rtb_Prelookup_o1 + ABK_P.Bias_Bias);

    // Sum: '<S9>/Add1' incorporates:
    //   Product: '<S9>/Product'
    //   Selector: '<S6>/Select trajectories'
    //   Selector: '<S9>/Selector'
    //   Selector: '<S9>/Selector1'
    //   Sum: '<S9>/Add'

    for (u0_tmp = 0; u0_tmp < 5; u0_tmp++) {
      // Selector: '<S9>/Selector1' incorporates:
      //   Selector: '<S6>/Select trajectories'
      //   Selector: '<S9>/Selector'

      rtb_Selector1_j = ABK_DW.Selecttrajectories[301 * u0_tmp +
        static_cast<int32_t>(rtb_Prelookup_o1)];
      rtb_Add1[u0_tmp] = (ABK_DW.Selecttrajectories[301 * u0_tmp +
                          rtb_Prelookup_o1_0] - rtb_Selector1_j) *
        rtb_Prelookup_o2 + rtb_Selector1_j;
    }

    // End of Sum: '<S9>/Add1'

    // Gain: '<S2>/Gain1'
    rtb_Prelookup_o2 = ABK_P.Gain1_Gain * ABK_U.ABKIn_l.ANASVelocity[2];

    // RelationalOperator: '<S2>/Relational Operator' incorporates:
    //   Bias: '<S2>/Bypass_vect'
    //   Selector: '<S2>/Selector1'
    //   Sum: '<S9>/Add1'

    rtb_Lowercondition = (rtb_Prelookup_o2 <= rtb_Add1[0] +
                          ABK_P.Bypass_vect_Bias[0]);

    // RelationalOperator: '<S2>/Relational Operator1' incorporates:
    //   Bias: '<S2>/Bypass_vect'
    //   Selector: '<S2>/Selector1'
    //   Sum: '<S9>/Add1'

    rtb_OR_h = (rtb_Prelookup_o2 >= ABK_P.Bypass_vect_Bias[1] + rtb_Add1[4]);

    // If: '<S2>/If' incorporates:
    //   Constant: '<S7>/Full close'
    //   Constant: '<S8>/Full open'

    rtPrevAction = ABK_DW.If_ActiveSubsystem_k;
    if (!rtb_Lowercondition) {
      if (rtb_OR_h) {
        rtAction = 1;
      } else {
        rtAction = 2;
      }
    }

    ABK_DW.If_ActiveSubsystem_k = rtAction;
    switch (rtAction) {
     case 0:
      // Outputs for IfAction SubSystem: '<S2>/Lower Bypass' incorporates:
      //   ActionPort: '<S7>/Action Port'

      rtb_zNAS = ABK_P.Fullclose_Value;

      // End of Outputs for SubSystem: '<S2>/Lower Bypass'
      break;

     case 1:
      // Outputs for IfAction SubSystem: '<S2>/Upper Bypass' incorporates:
      //   ActionPort: '<S8>/Action Port'

      rtb_zNAS = ABK_P.Fullopen_Value;

      // End of Outputs for SubSystem: '<S2>/Upper Bypass'
      break;

     default:
      if (rtAction != rtPrevAction) {
        // SystemReset for IfAction SubSystem: '<S2>/Active control system' incorporates:
        //   ActionPort: '<S5>/Action Port'

        // SystemReset for Atomic SubSystem: '<S5>/PID Controller'
        // SystemReset for If: '<S2>/If' incorporates:
        //   Memory: '<S13>/Memory'
        //   Memory: '<S13>/Memory1'
        //   Memory: '<S17>/Memory'
        //   Memory: '<S17>/Memory3'
        //   UnitDelay: '<S16>/Unit Delay'

        ABK_DW.Memory1_PreviousInput = ABK_P.Memory1_InitialCondition;
        ABK_DW.Memory_PreviousInput_h = ABK_P.Memory_InitialCondition;

        // End of SystemReset for SubSystem: '<S5>/PID Controller'

        // SystemReset for Atomic SubSystem: '<S5>/Variant Filter'
        ABK_DW.Memory_PreviousInput[0] = ABK_P.Memory_InitialCondition_f;
        ABK_DW.Memory3_PreviousInput[0] = ABK_P.Memory3_InitialCondition;
        ABK_DW.Memory_PreviousInput[1] = ABK_P.Memory_InitialCondition_f;
        ABK_DW.Memory3_PreviousInput[1] = ABK_P.Memory3_InitialCondition;
        ABK_DW.UnitDelay_DSTATE_l = ABK_P.UnitDelay_InitialCondition_e;

        // End of SystemReset for SubSystem: '<S5>/Variant Filter'
        // End of SystemReset for SubSystem: '<S2>/Active control system'
      }

      // Outputs for IfAction SubSystem: '<S2>/Active control system' incorporates:
      //   ActionPort: '<S5>/Action Port'

      // PreLookup: '<S10>/Prelookup1'
      rtb_Prelookup1_o1 = plook_u16ff_binc(rtb_Prelookup_o2, rtb_Add1, 4U,
        &rtb_Selector1_j);

      // Selector: '<S10>/Selector4' incorporates:
      //   Constant: '<S10>/Extensions'

      rtb_Prelookup_o2 = ABK_P.Extensions_Value[rtb_Prelookup1_o1];

      // Sum: '<S10>/Sum5' incorporates:
      //   Bias: '<S10>/Bias'
      //   Constant: '<S10>/Extensions'
      //   Product: '<S10>/Product1'
      //   Selector: '<S10>/Selector4'
      //   Selector: '<S10>/Selector5'
      //   Sum: '<S10>/Sum4'

      rtb_Prelookup_o2 += (ABK_P.Extensions_Value[static_cast<uint16_t>
                           (rtb_Prelookup1_o1 + ABK_P.Bias_Bias_l)] -
                           rtb_Prelookup_o2) * rtb_Selector1_j;

      // Outputs for Atomic SubSystem: '<S5>/PID Controller'
      // Bias: '<S13>/refBias1'
      rtb_Selector1_j = rtb_Prelookup_o2 + ABK_P.refBias1_Bias;

      // Switch: '<S13>/Switch' incorporates:
      //   Constant: '<S13>/Constant'
      //   Constant: '<S13>/Sample Time'
      //   Memory: '<S13>/Memory1'
      //   Product: '<S13>/Product1'

      if (ABK_DW.Memory1_PreviousInput) {
        u0_tmp_0 = ABK_P.Constant_Value;
      } else {
        u0_tmp_0 = ABK_P.SampleTime_Value * rtb_Selector1_j;
      }

      // Sum: '<S13>/Add' incorporates:
      //   Memory: '<S13>/Memory'
      //   Switch: '<S13>/Switch'

      rtb_integralError = u0_tmp_0 + ABK_DW.Memory_PreviousInput_h;

      // Bias: '<S13>/refBias2' incorporates:
      //   Bias: '<S13>/refBias'
      //   Constant: '<S13>/Kd'
      //   Constant: '<S13>/Ki'
      //   Constant: '<S13>/Kp'
      //   Constant: '<S13>/Sample Time'
      //   Product: '<S13>/Product2'
      //   Product: '<S13>/Product3'
      //   Product: '<S13>/Product4'
      //   Product: '<S13>/Product5'
      //   Sum: '<S13>/Add1'
      //   UnitDelay: '<S2>/Unit Delay'

      rtb_Selector1_j = (((ABK_DW.UnitDelay_DSTATE + ABK_P.refBias_Bias) /
                          ABK_P.SampleTime_Value * ABK_P.Kd_Value +
                          ABK_P.Ki_Value * rtb_integralError) + ABK_P.Kp_Value *
                         rtb_Selector1_j) + ABK_P.refBias2_Bias;

      // Saturate: '<S13>/Saturation'
      if (rtb_Selector1_j > ABK_P.Saturation_UpperSat) {
        rtb_Selector1_j = ABK_P.Saturation_UpperSat;
      } else if (rtb_Selector1_j < ABK_P.Saturation_LowerSat) {
        rtb_Selector1_j = ABK_P.Saturation_LowerSat;
      }

      // End of Saturate: '<S13>/Saturation'

      // Update for Memory: '<S13>/Memory1' incorporates:
      //   Constant: '<S14>/Constant'
      //   Constant: '<S15>/Constant'
      //   Logic: '<S13>/OR'
      //   RelationalOperator: '<S14>/Compare'
      //   RelationalOperator: '<S15>/Compare'

      ABK_DW.Memory1_PreviousInput = ((rtb_Selector1_j <=
        ABK_P.FullClose_compare_const) || (rtb_Selector1_j >=
        ABK_P.FullOpen_compare_const));

      // Update for Memory: '<S13>/Memory'
      ABK_DW.Memory_PreviousInput_h = rtb_integralError;

      // End of Outputs for SubSystem: '<S5>/PID Controller'

      // Outputs for Atomic SubSystem: '<S5>/Variant Filter'
      // SignalConversion generated from: '<S17>/Vector Concatenate1' incorporates:
      //   Memory: '<S17>/Memory'

      rtb_integralError = ABK_DW.Memory_PreviousInput[1];

      // SignalConversion generated from: '<S17>/Vector Concatenate' incorporates:
      //   Memory: '<S17>/Memory'
      //
      ABK_DW.Memory_PreviousInput[1] = ABK_DW.Memory_PreviousInput[0];

      // SignalConversion generated from: '<S17>/Vector Concatenate2' incorporates:
      //   Memory: '<S17>/Memory3'

      rtb_Bypass_vect_idx_1 = ABK_DW.Memory3_PreviousInput[0];

      // Product: '<S17>/Matrix Multiply1' incorporates:
      //   Constant: '<S17>/coeff a'
      //   Memory: '<S17>/Memory3'

      rtb_Saturation2 = ABK_P.coeffa_Value[0] * ABK_DW.Memory3_PreviousInput[0]
        + ABK_P.coeffa_Value[1] * ABK_DW.Memory3_PreviousInput[1];

      // Sum: '<S17>/Add' incorporates:
      //   Constant: '<S17>/coeff b'
      //   Memory: '<S17>/Memory'
      //   Product: '<S17>/Matrix Multiply'
      //   Product: '<S17>/Matrix Multiply1'
      //   SignalConversion generated from: '<S17>/Vector Concatenate1'

      rtb_integralError = ((ABK_P.coeffb_Value[0] * rtb_Selector1_j +
                            ABK_DW.Memory_PreviousInput[0] * ABK_P.coeffb_Value
                            [1]) + ABK_P.coeffb_Value[2] * rtb_integralError) -
        rtb_Saturation2;

      // PreLookup: '<S19>/Prelookup1' incorporates:
      //   Constant: '<S19>/zColumn'

      rtb_Prelookup1_o1 = plook_u16fdf_evenx(rtb_zNAS, ABK_P.zColumn_Value[0],
        ABK_P.zColumn_Value[1] - ABK_P.zColumn_Value[0], 300U, &rtb_Saturation2);

      // Selector: '<S19>/Select on mass' incorporates:
      //   Constant: '<S19>/Trajectory difference'
      //   Selector: '<S19>/Selector'
      //   Selector: '<S19>/Selector1'

      u0_tmp = 301 * ABK_DW.massIndex;
      u0_tmp_0 = ABK_P.Trajectorydifference_Value[u0_tmp + rtb_Prelookup1_o1];

      // Gain: '<S19>/T_refGain' incorporates:
      //   Bias: '<S19>/Bias'
      //   Constant: '<S19>/Trajectory difference'
      //   Product: '<S19>/Product'
      //   Selector: '<S19>/Select on mass'
      //   Selector: '<S19>/Selector'
      //   Selector: '<S19>/Selector1'
      //   Sum: '<S19>/Add'
      //   Sum: '<S19>/Add1'

      rtb_Saturation2 = ((ABK_P.Trajectorydifference_Value[static_cast<uint16_t>
                          (rtb_Prelookup1_o1 + ABK_P.Bias_Bias_g) + u0_tmp] -
                          u0_tmp_0) * rtb_Saturation2 + u0_tmp_0) *
        ABK_P.T_refGain_Gain;

      // Saturate: '<S19>/Saturation2'
      if (rtb_Saturation2 > ABK_P.Saturation2_UpperSat) {
        rtb_Saturation2 = ABK_P.Saturation2_UpperSat;
      } else if (rtb_Saturation2 < ABK_P.Saturation2_LowerSat) {
        rtb_Saturation2 = ABK_P.Saturation2_LowerSat;
      }

      // Bias: '<S19>/Bias3' incorporates:
      //   Saturate: '<S19>/Saturation2'

      rtb_Saturation2 += ABK_P.Bias3_Bias;

      // Logic: '<S16>/OR' incorporates:
      //   Constant: '<S18>/Constant'
      //   RelationalOperator: '<S18>/Compare'
      //   UnitDelay: '<S16>/Unit Delay'

      rtb_OR = (ABK_DW.UnitDelay_DSTATE_l || (rtb_zNAS >=
                 ABK_P.FullOpen_compare_const_o));

      // Switch: '<S16>/Switch' incorporates:
      //   Bias: '<S19>/Bias3'
      //   Constant: '<S16>/fullOpen'
      //   Product: '<S16>/Product'
      //   Saturate: '<S17>/Saturation'
      //   Sum: '<S16>/Add'
      //   Sum: '<S16>/Add1'

      if (rtb_OR) {
        rtb_zNAS = ABK_P.fullOpen_Value;
      } else {
        if (rtb_integralError > ABK_P.Saturation_UpperSat_m) {
          // Saturate: '<S17>/Saturation'
          u0_tmp_0 = ABK_P.Saturation_UpperSat_m;
        } else if (rtb_integralError < ABK_P.Saturation_LowerSat_p) {
          // Saturate: '<S17>/Saturation'
          u0_tmp_0 = ABK_P.Saturation_LowerSat_p;
        } else {
          // Saturate: '<S17>/Saturation'
          u0_tmp_0 = rtb_integralError;
        }

        rtb_zNAS = (u0_tmp_0 - rtb_Selector1_j) * rtb_Saturation2 +
          rtb_Selector1_j;
      }

      // End of Switch: '<S16>/Switch'

      // SignalConversion generated from: '<S17>/Vector Concatenate' incorporates:
      //   Memory: '<S17>/Memory'

      ABK_DW.Memory_PreviousInput[0] = rtb_Selector1_j;

      // Update for Memory: '<S17>/Memory3' incorporates:
      //   SignalConversion generated from: '<S17>/Vector Concatenate2'

      ABK_DW.Memory3_PreviousInput[0] = rtb_integralError;
      ABK_DW.Memory3_PreviousInput[1] = rtb_Bypass_vect_idx_1;

      // Update for UnitDelay: '<S16>/Unit Delay'
      ABK_DW.UnitDelay_DSTATE_l = rtb_OR;

      // End of Outputs for SubSystem: '<S5>/Variant Filter'

      // BusCreator generated from: '<S5>/ABK Logs OBSW_BusCreator' incorporates:
      //   Bias: '<S19>/Bias3'

      ABK_DW.ABKLogsOBSW_BusCreator_BusCreat.ABKCommand = 0.0F;
      ABK_DW.ABKLogsOBSW_BusCreator_BusCreat.Timestamp = 0ULL;
      ABK_DW.ABKLogsOBSW_BusCreator_BusCreat.FilterCoefficient = rtb_Saturation2;
      ABK_DW.ABKLogsOBSW_BusCreator_BusCreat.PrePIDCommand = rtb_Prelookup_o2;
      ABK_DW.ABKLogsOBSW_BusCreator_BusCreat.PostPIDCommand = rtb_Selector1_j;
      ABK_DW.ABKLogsOBSW_BusCreator_BusCreat.BypassActivation = false;

      // End of Outputs for SubSystem: '<S2>/Active control system'
      break;
    }

    // End of If: '<S2>/If'

    // BusAssignment: '<S2>/Bus Assignment1' incorporates:
    //   BusCreator generated from: '<S5>/ABK Logs OBSW_BusCreator'

    ABK_DW.BusAssignment1 = ABK_DW.ABKLogsOBSW_BusCreator_BusCreat;

    // BusAssignment: '<S2>/Bus Assignment1' incorporates:
    //   Logic: '<S2>/OR'

    ABK_DW.BusAssignment1.BypassActivation = (rtb_OR_h || rtb_Lowercondition);

    // Bias: '<S2>/Bias' incorporates:
    //   UnitDelay: '<S2>/Unit Delay1'

    u0_0 = static_cast<uint8_t>(ABK_DW.UnitDelay1_DSTATE + ABK_P.Bias_Bias_c);

    // Saturate: '<S2>/Saturation'
    if (u0_0 > ABK_P.Saturation_UpperSat_h) {
      // Update for UnitDelay: '<S2>/Unit Delay1'
      ABK_DW.UnitDelay1_DSTATE = ABK_P.Saturation_UpperSat_h;
    } else if (u0_0 < ABK_P.Saturation_LowerSat_k) {
      // Update for UnitDelay: '<S2>/Unit Delay1'
      ABK_DW.UnitDelay1_DSTATE = ABK_P.Saturation_LowerSat_k;
    } else {
      // Update for UnitDelay: '<S2>/Unit Delay1'
      ABK_DW.UnitDelay1_DSTATE = u0_0;
    }

    // End of Saturate: '<S2>/Saturation'

    // Update for UnitDelay: '<S2>/Unit Delay'
    ABK_DW.UnitDelay_DSTATE = rtb_zNAS;

    // End of Outputs for SubSystem: '<S1>/Algorothm Execution'
  } else {
    // Outputs for IfAction SubSystem: '<S1>/OverMach Protection' incorporates:
    //   ActionPort: '<S4>/Action Port'

    // SignalConversion generated from: '<S4>/ABK %' incorporates:
    //   Constant: '<S4>/Constant'

    rtb_zNAS = ABK_P.Constant_Value_o;

    // End of Outputs for SubSystem: '<S1>/OverMach Protection'
  }

  // Outport: '<Root>/ABK Logs OBSW' incorporates:
  //   BusAssignment: '<S1>/Bus Assignment1'
  //   BusAssignment: '<S2>/Bus Assignment1'

  ABK_Y.ABKLogsOBSW = ABK_DW.BusAssignment1;

  // BusAssignment: '<S1>/Bus Assignment1' incorporates:
  //   Outport: '<Root>/ABK Logs OBSW'

  ABK_Y.ABKLogsOBSW.ABKCommand = rtb_zNAS;

  // End of Outputs for SubSystem: '<Root>/ABK'

  // Outport: '<Root>/ABK Control'
  ABK_Y.ABKControl = rtb_zNAS;
}

// Model initialize function
void ABK::initialize()
{
  {
    int32_t i;

    // SystemInitialize for Atomic SubSystem: '<Root>/ABK'
    // Start for If: '<S1>/If'
    ABK_DW.If_ActiveSubsystem = -1;

    // SystemInitialize for IfAction SubSystem: '<S1>/Algorothm Execution'
    // Start for If: '<S2>/If'
    ABK_DW.If_ActiveSubsystem_k = -1;

    // InitializeConditions for UnitDelay: '<S2>/Unit Delay1'
    ABK_DW.UnitDelay1_DSTATE = ABK_P.UnitDelay1_InitialCondition;

    // InitializeConditions for UnitDelay: '<S2>/Unit Delay'
    ABK_DW.UnitDelay_DSTATE = ABK_P.UnitDelay_InitialCondition;

    // SystemInitialize for Triggered SubSystem: '<S2>/Function-Call Subsystem'
    // SystemInitialize for PreLookup: '<S6>/Mass_Prelookup' incorporates:
    //   Outport: '<S6>/massIndex'

    ABK_DW.massIndex = ABK_P.massIndex_Y0;
    for (i = 0; i < 1505; i++) {
      // SystemInitialize for Selector: '<S6>/Select trajectories' incorporates:
      //   Outport: '<S6>/traj'

      ABK_DW.Selecttrajectories[i] = ABK_P.traj_Y0;
    }

    for (i = 0; i < 301; i++) {
      // SystemInitialize for Selector: '<S6>/Select column z' incorporates:
      //   Outport: '<S6>/zVect'

      ABK_DW.Selectcolumnz[i] = ABK_P.zVect_Y0;
    }

    ABK_PrevZCX.FunctionCallSubsystem_Trig_ZCE = ZERO_ZCSIG;

    // End of SystemInitialize for SubSystem: '<S2>/Function-Call Subsystem'

    // SystemInitialize for IfAction SubSystem: '<S2>/Active control system'
    // SystemInitialize for Atomic SubSystem: '<S5>/PID Controller'
    // InitializeConditions for Memory: '<S13>/Memory1'
    ABK_DW.Memory1_PreviousInput = ABK_P.Memory1_InitialCondition;

    // InitializeConditions for Memory: '<S13>/Memory'
    ABK_DW.Memory_PreviousInput_h = ABK_P.Memory_InitialCondition;

    // End of SystemInitialize for SubSystem: '<S5>/PID Controller'

    // SystemInitialize for Atomic SubSystem: '<S5>/Variant Filter'
    // InitializeConditions for Memory: '<S17>/Memory'
    ABK_DW.Memory_PreviousInput[0] = ABK_P.Memory_InitialCondition_f;

    // InitializeConditions for Memory: '<S17>/Memory3'
    ABK_DW.Memory3_PreviousInput[0] = ABK_P.Memory3_InitialCondition;

    // InitializeConditions for Memory: '<S17>/Memory'
    ABK_DW.Memory_PreviousInput[1] = ABK_P.Memory_InitialCondition_f;

    // InitializeConditions for Memory: '<S17>/Memory3'
    ABK_DW.Memory3_PreviousInput[1] = ABK_P.Memory3_InitialCondition;

    // InitializeConditions for UnitDelay: '<S16>/Unit Delay'
    ABK_DW.UnitDelay_DSTATE_l = ABK_P.UnitDelay_InitialCondition_e;

    // End of SystemInitialize for SubSystem: '<S5>/Variant Filter'

    // SystemInitialize for BusCreator generated from: '<S5>/ABK Logs OBSW_BusCreator' incorporates:
    //   Outport: '<S5>/ABK Logs OBSW_Outport_2'

    ABK_DW.ABKLogsOBSW_BusCreator_BusCreat = ABK_P.ABKLogsOBSW_Outport_2_Y0;

    // End of SystemInitialize for SubSystem: '<S2>/Active control system'

    // SystemInitialize for BusAssignment: '<S2>/Bus Assignment1' incorporates:
    //   Outport: '<S2>/ABK Logs OBSW'

    ABK_DW.BusAssignment1 = ABK_P.ABKLogsOBSW_Y0;

    // End of SystemInitialize for SubSystem: '<S1>/Algorothm Execution'
    // End of SystemInitialize for SubSystem: '<Root>/ABK'
  }
}

// Model terminate function
void ABK::terminate()
{
  // (no terminate code required)
}

// Constructor
ABK::ABK():
  ABK_U(),
  ABK_Y(),
  ABK_DW(),
  ABK_PrevZCX()
{
  // Currently there is no constructor body generated.
}

// Destructor
// Currently there is no destructor body generated.
ABK::~ABK() = default;


} // fine namespace ABK

//
// File trailer for generated code.
//
// [EOF]
//
