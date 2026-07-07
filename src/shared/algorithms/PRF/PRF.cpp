//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// File: PRF.cpp
//
// Code generated for Simulink model 'PRF'.
//
// Model version                  : 11.335
// Simulink Coder version         : 24.2 (R2024b) 21-Jun-2024
// C/C++ source code generated on : Tue Jul  7 11:23:43 2026
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
#include <stdint.h>
#include <cmath>
#include "PRF_private.h"
#include <stdbool.h>
#include "zero_crossing_types.h"
#include <cfloat>

namespace PRF 
{


// Named constants for Chart: '<S1>/Point Selection'
const uint8_t PRF_IN_Q1{ 1U };

const uint8_t PRF_IN_Q2{ 2U };

const uint8_t PRF_IN_Target{ 3U };

float rt_modf(float u0, float u1)
{
  float y;
  y = u0;
  if (u1 == 0.0F) {
    if (u0 == 0.0F) {
      y = u1;
    }
  } else if (u0 == 0.0F) {
    y = 0.0F / u1;
  } else {
    bool yEq;
    y = std::fmod(u0, u1);
    yEq = (y == 0.0F);
    if ((!yEq) && (u1 > std::floor(u1))) {
      float q;
      q = std::abs(u0 / u1);
      yEq = (std::abs(q - std::floor(q + 0.5F)) <= FLT_EPSILON * q);
    }

    if (yEq) {
      y = 0.0F;
    } else if ((u0 < 0.0F) != (u1 < 0.0F)) {
      y += u1;
    }
  }

  return y;
}

// Model step function
void PRF::step()
{
  float rtb_VectorConcatenate3[6];
  float rtb_DotProduct1;
  float rtb_Gain1;
  float rtb_Gain4;
  float rtb_Sum1_idx_1;
  float rtb_Sum2_idx_0;
  float rtb_Sum2_idx_1;
  float rtb_direct_distance;
  float rtb_inversion;
  float rtb_k_par;
  float rtb_k_perp;
  int32_t i;
  int32_t i_0;
  uint8_t rtb_target;
  uint8_t u0;
  bool rtb_Compare;

  // Outputs for Atomic SubSystem: '<Root>/PRF'
  // Outputs for Triggered SubSystem: '<S6>/Target Points Generation' incorporates:
  //   TriggerPort: '<S13>/function'

  // UnitDelay: '<S6>/Unit Delay1' incorporates:
  //   Logic: '<S13>/NOT'

  if ((PRF_DW.UnitDelay1_DSTATE > 0) &&
      (PRF_PrevZCX.TargetPointsGeneration_Trig_ZCE != POS_ZCSIG)) {
    // Switch: '<S20>/FixPt Switch' incorporates:
    //   Constant: '<S20>/Constant'
    //   Inport: '<Root>/PRF Reference'

    if (PRF_U.PRFReference_i.WindDirection > PRF_P.WrapToZero_Threshold) {
      rtb_Sum2_idx_0 = PRF_P.Constant_Value_k;
    } else {
      rtb_Sum2_idx_0 = PRF_U.PRFReference_i.WindDirection;
    }

    // Gain: '<S19>/Gain1' incorporates:
    //   Switch: '<S20>/FixPt Switch'

    rtb_DotProduct1 = PRF_P.Gain1_Gain_b * rtb_Sum2_idx_0;

    // Sum: '<S18>/Sum2' incorporates:
    //   Constant: '<S13>/Constant3'
    //   Constant: '<S13>/Constant5'
    //   Math: '<S13>/Transpose'
    //   Product: '<S18>/Product'
    //   Product: '<S18>/Product1'
    //   Trigonometry: '<S18>/Cos'
    //   Trigonometry: '<S18>/Sin'

    PRF_DW.Q2[0] = std::cos(rtb_DotProduct1) * PRF_P.Constant5_Value_b +
      PRF_P.Constant3_Value[0];
    PRF_DW.Q2[1] = std::sin(rtb_DotProduct1) * PRF_P.Constant5_Value_b +
      PRF_P.Constant3_Value[1];

    // SignalConversion generated from: '<S13>/Vector Concatenate3' incorporates:
    //   Constant: '<S13>/Constant3'
    //   Math: '<S13>/Transpose'

    rtb_VectorConcatenate3[0] = PRF_P.Constant3_Value[0];

    // SignalConversion generated from: '<S13>/Vector Concatenate3' incorporates:
    //   DataTypeConversion: '<S13>/Cast To Single1'
    //   Sum: '<S18>/Sum2'

    rtb_VectorConcatenate3[2] = PRF_DW.Q2[0];

    // Sum: '<S17>/Sum' incorporates:
    //   DataTypeConversion: '<S13>/Cast To Single1'
    //   Inport: '<Root>/PRF In'
    //   Sum: '<S18>/Sum2'
    //   Sum: '<S4>/Sum2'

    rtb_Sum2_idx_1 = PRF_DW.Q2[0] - PRF_U.PRFIn_f.NASDAQPosition[0];
    rtb_Sum2_idx_0 = rtb_Sum2_idx_1;

    // Abs: '<S17>/Abs' incorporates:
    //   Sum: '<S4>/Sum1'
    //   Sum: '<S4>/Sum2'

    rtb_inversion = std::abs(rtb_Sum2_idx_1);

    // DotProduct: '<S17>/Dot Product' incorporates:
    //   Sum: '<S4>/Sum1'

    rtb_DotProduct1 = rtb_inversion * rtb_inversion;

    // SignalConversion generated from: '<S13>/Vector Concatenate3' incorporates:
    //   Constant: '<S13>/Constant3'
    //   Math: '<S13>/Transpose'

    rtb_VectorConcatenate3[1] = PRF_P.Constant3_Value[1];

    // SignalConversion generated from: '<S13>/Vector Concatenate3' incorporates:
    //   DataTypeConversion: '<S13>/Cast To Single1'
    //   Sum: '<S18>/Sum2'

    rtb_VectorConcatenate3[3] = PRF_DW.Q2[1];

    // Sum: '<S17>/Sum' incorporates:
    //   DataTypeConversion: '<S13>/Cast To Single1'
    //   Inport: '<Root>/PRF In'
    //   Sum: '<S18>/Sum2'
    //   Sum: '<S4>/Sum2'

    rtb_Sum2_idx_1 = PRF_DW.Q2[1] - PRF_U.PRFIn_f.NASDAQPosition[1];

    // Abs: '<S17>/Abs' incorporates:
    //   Sum: '<S4>/Sum1'
    //   Sum: '<S4>/Sum2'

    rtb_inversion = std::abs(rtb_Sum2_idx_1);

    // Sqrt: '<S17>/Sqrt2' incorporates:
    //   DotProduct: '<S17>/Dot Product'
    //   Sum: '<S4>/Sum1'

    rtb_direct_distance = std::sqrt(rtb_inversion * rtb_inversion +
      rtb_DotProduct1);

    // Abs: '<S17>/Abs1' incorporates:
    //   Constant: '<S13>/glide ratio'
    //   Inport: '<Root>/PRF In'
    //   Product: '<S17>/Product1'

    rtb_DotProduct1 = std::abs(PRF_P.glideratio_Value *
      PRF_U.PRFIn_f.NASDAQPosition[2]);

    // RelationalOperator: '<S14>/Compare' incorporates:
    //   Constant: '<S14>/Constant'
    //   Sum: '<S13>/Sum'

    rtb_Compare = (rtb_DotProduct1 - rtb_direct_distance >
                   PRF_P.Comparetoconstant_const);

    // Outputs for Enabled SubSystem: '<S13>/Enabled Subsystem1' incorporates:
    //   EnablePort: '<S16>/Enable'

    if (!rtb_Compare) {
      // Merge: '<S13>/Merge' incorporates:
      //   DataTypeConversion: '<S13>/Cast To Single1'
      //   SignalConversion generated from: '<S16>/Q2'
      //   Sum: '<S18>/Sum2'

      PRF_DW.Q1[0] = PRF_DW.Q2[0];
      PRF_DW.Q1[1] = PRF_DW.Q2[1];
    }

    // End of Outputs for SubSystem: '<S13>/Enabled Subsystem1'

    // Product: '<S17>/Product2' incorporates:
    //   Logic: '<S13>/NOT'
    //   Sum: '<S17>/Sum'
    //   Sum: '<S4>/Sum2'

    rtb_Sum2_idx_0 /= rtb_direct_distance;
    rtb_Sum2_idx_1 /= rtb_direct_distance;

    // Gain: '<S17>/Gain'
    rtb_Sum1_idx_1 = PRF_P.Gain_Gain * rtb_Sum2_idx_0;

    // DotProduct: '<S13>/Dot Product' incorporates:
    //   Concatenate: '<S17>/Vector Concatenate'
    //   Constant: '<S13>/Constant3'
    //   Math: '<S13>/Transpose'
    //   SignalConversion generated from: '<S17>/Vector Concatenate'
    //   Sum: '<S18>/Sum'
    //   Sum: '<S18>/Sum2'

    rtb_inversion = (PRF_DW.Q2[0] - PRF_P.Constant3_Value[0]) * rtb_Sum2_idx_1 +
      (PRF_DW.Q2[1] - PRF_P.Constant3_Value[1]) * rtb_Sum1_idx_1;

    // Signum: '<S13>/1 or -1' incorporates:
    //   DotProduct: '<S13>/Dot Product'

    if (rtb_inversion < 0.0F) {
      rtb_inversion = -1.0F;
    } else {
      rtb_inversion = (rtb_inversion > 0.0F);
    }

    // End of Signum: '<S13>/1 or -1'

    // Gain: '<S17>/Gain1'
    rtb_Gain1 = PRF_P.Gain1_Gain_c * rtb_direct_distance;

    // Outputs for Enabled SubSystem: '<S13>/Enabled Subsystem' incorporates:
    //   EnablePort: '<S15>/Enable'

    if (rtb_Compare) {
      // Gain: '<S15>/Gain2'
      rtb_DotProduct1 *= PRF_P.Gain2_Gain;

      // Gain: '<S15>/Gain4'
      rtb_Gain4 = PRF_P.Gain4_Gain * rtb_DotProduct1;

      // Trigonometry: '<S15>/Sin' incorporates:
      //   Constant: '<S13>/theta'

      rtb_k_par = std::sin(PRF_P.theta_Value);

      // Trigonometry: '<S15>/Cos' incorporates:
      //   Constant: '<S13>/theta'

      rtb_k_perp = std::cos(PRF_P.theta_Value);

      // Gain: '<S15>/Gain3'
      rtb_direct_distance *= PRF_P.Gain3_Gain;

      // Gain: '<S15>/Gain1' incorporates:
      //   Math: '<S15>/Square'
      //   Math: '<S15>/Square1'
      //   Sqrt: '<S15>/Sqrt1'
      //   Sum: '<S15>/Sum2'

      rtb_direct_distance = std::sqrt(rtb_DotProduct1 * rtb_DotProduct1 -
        rtb_direct_distance * rtb_direct_distance) * PRF_P.Gain1_Gain;

      // Merge: '<S13>/Merge' incorporates:
      //   Concatenate: '<S17>/Vector Concatenate'
      //   Inport: '<Root>/PRF In'
      //   Product: '<S15>/Product'
      //   Product: '<S15>/Product1'
      //   Product: '<S17>/Product'
      //   Product: '<S17>/Product2'
      //   SignalConversion generated from: '<S17>/Vector Concatenate'
      //   Sum: '<S15>/Sum'
      //   Sum: '<S17>/Sum1'

      PRF_DW.Q1[0] = (rtb_Sum2_idx_1 * rtb_direct_distance * rtb_k_perp *
                      rtb_inversion + rtb_Sum2_idx_0 * rtb_Gain4 * rtb_k_par) +
        (rtb_Sum2_idx_0 * rtb_Gain1 + PRF_U.PRFIn_f.NASDAQPosition[0]);
      PRF_DW.Q1[1] = (rtb_Sum1_idx_1 * rtb_direct_distance * rtb_k_perp *
                      rtb_inversion + rtb_Sum2_idx_1 * rtb_Gain4 * rtb_k_par) +
        (rtb_Sum2_idx_1 * rtb_Gain1 + PRF_U.PRFIn_f.NASDAQPosition[1]);
    }

    // End of Outputs for SubSystem: '<S13>/Enabled Subsystem'

    // SignalConversion generated from: '<S13>/Vector Concatenate3' incorporates:
    //   Merge: '<S13>/Merge'

    rtb_VectorConcatenate3[4] = PRF_DW.Q1[0];
    rtb_VectorConcatenate3[5] = PRF_DW.Q1[1];

    // Math: '<S13>/Transpose1' incorporates:
    //   Concatenate: '<S13>/Vector Concatenate3'

    i = 0;
    for (i_0 = 0; i_0 < 2; i_0++) {
      PRF_DW.TargetPoints[i] = rtb_VectorConcatenate3[i_0];
      PRF_DW.TargetPoints[i + 1] = rtb_VectorConcatenate3[i_0 + 2];
      PRF_DW.TargetPoints[i + 2] = rtb_VectorConcatenate3[i_0 + 4];
      i += 3;
    }
  }

  PRF_PrevZCX.TargetPointsGeneration_Trig_ZCE = (PRF_DW.UnitDelay1_DSTATE > 0);

  // End of Outputs for SubSystem: '<S6>/Target Points Generation'

  // Abs: '<S4>/Abs' incorporates:
  //   Inport: '<Root>/PRF In'
  //   Inport: '<Root>/PRF Reference'
  //   Sum: '<S4>/Sum'

  rtb_Sum2_idx_0 = std::abs(PRF_U.PRFReference_i.TargetPositionNED[0] -
    PRF_U.PRFIn_f.NASDAQPosition[0]);

  // DotProduct: '<S4>/Dot Product'
  rtb_DotProduct1 = rtb_Sum2_idx_0 * rtb_Sum2_idx_0;

  // Abs: '<S4>/Abs' incorporates:
  //   Inport: '<Root>/PRF In'
  //   Inport: '<Root>/PRF Reference'
  //   Sum: '<S4>/Sum'

  rtb_Sum2_idx_0 = std::abs(PRF_U.PRFReference_i.TargetPositionNED[1] -
    PRF_U.PRFIn_f.NASDAQPosition[1]);

  // Gain: '<S4>/PRFControl.zThresholdGain' incorporates:
  //   Constant: '<S4>/glide ratio'
  //   DotProduct: '<S4>/Dot Product'
  //   Product: '<S4>/Product'
  //   Sqrt: '<S4>/Sqrt'

  rtb_direct_distance = std::sqrt(rtb_Sum2_idx_0 * rtb_Sum2_idx_0 +
    rtb_DotProduct1) / PRF_P.glideratio_Value_o *
    PRF_P.PRFControlzThresholdGain_Gain;

  // Chart: '<S1>/Point Selection' incorporates:
  //   Constant: '<S4>/-'
  //   Inport: '<Root>/PRF In'
  //   Memory: '<S4>/Memory'
  //   Memory: '<S4>/Memory1'

  if (PRF_DW.is_active_c9_PRF == 0) {
    PRF_DW.is_active_c9_PRF = 1U;
    PRF_DW.is_c9_PRF = PRF_IN_Q1;

    // :  target=2;
    rtb_target = 2U;
  } else {
    switch (PRF_DW.is_c9_PRF) {
     case PRF_IN_Q1:
      // :  sf_internal_predicateOutput = (z>-z_threshold);
      if (PRF_U.PRFIn_f.NASDAQPosition[2] > -rtb_direct_distance) {
        PRF_DW.is_c9_PRF = PRF_IN_Target;

        // :  target=0;
        rtb_target = 0U;

        // :  sf_internal_predicateOutput = (deltaQ1<Q_threshold);
      } else if (PRF_DW.Memory1_PreviousInput < PRF_P._Value) {
        PRF_DW.is_c9_PRF = PRF_IN_Q2;

        // :  target=1;
        rtb_target = 1U;
      } else {
        // :  target=2;
        rtb_target = 2U;
      }
      break;

     case PRF_IN_Q2:
      // :  sf_internal_predicateOutput = ((deltaQ2<Q_threshold)||(z>-z_threshold)); 
      if ((PRF_DW.Memory_PreviousInput < PRF_P._Value) ||
          (PRF_U.PRFIn_f.NASDAQPosition[2] > -rtb_direct_distance)) {
        PRF_DW.is_c9_PRF = PRF_IN_Target;

        // :  target=0;
        rtb_target = 0U;
      } else {
        // :  target=1;
        rtb_target = 1U;
      }
      break;

     default:
      // case IN_Target:
      // :  target=0;
      rtb_target = 0U;
      break;
    }
  }

  // End of Chart: '<S1>/Point Selection'

  // Sum: '<S4>/Sum1' incorporates:
  //   Inport: '<Root>/PRF In'
  //   Merge: '<S13>/Merge'
  //   Selector: '<S6>/Selector'
  //   Sum: '<S5>/Subtract1'

  rtb_k_par = PRF_DW.Q1[0] - PRF_U.PRFIn_f.NASDAQPosition[0];

  // Sum: '<S4>/Sum2' incorporates:
  //   Inport: '<Root>/PRF In'
  //   Selector: '<S6>/Selector'
  //   Sum: '<S18>/Sum2'
  //   Sum: '<S5>/Subtract1'

  rtb_Sum2_idx_0 = PRF_DW.Q2[0] - PRF_U.PRFIn_f.NASDAQPosition[0];

  // Sum: '<S4>/Sum1' incorporates:
  //   Inport: '<Root>/PRF In'
  //   Merge: '<S13>/Merge'
  //   Selector: '<S6>/Selector'
  //   Sum: '<S5>/Subtract1'

  rtb_Sum1_idx_1 = PRF_DW.Q1[1] - PRF_U.PRFIn_f.NASDAQPosition[1];

  // Sum: '<S4>/Sum2' incorporates:
  //   Inport: '<Root>/PRF In'
  //   Selector: '<S6>/Selector'
  //   Sum: '<S18>/Sum2'
  //   Sum: '<S5>/Subtract1'

  rtb_Sum2_idx_1 = PRF_DW.Q2[1] - PRF_U.PRFIn_f.NASDAQPosition[1];

  // Trigonometry: '<S5>/Atan2' incorporates:
  //   Inport: '<Root>/PRF In'

  rtb_inversion = std::atan2(PRF_U.PRFIn_f.NASDAQVelocity[1],
    PRF_U.PRFIn_f.NASDAQVelocity[0]);

  // Trigonometry: '<S5>/Atan1' incorporates:
  //   Inport: '<Root>/PRF In'
  //   Math: '<S13>/Transpose1'
  //   Selector: '<S6>/Selector'
  //   Sum: '<S5>/Subtract1'

  rtb_direct_distance = std::atan2(PRF_DW.TargetPoints[rtb_target + 3] -
    PRF_U.PRFIn_f.NASDAQPosition[1], PRF_DW.TargetPoints[rtb_target] -
    PRF_U.PRFIn_f.NASDAQPosition[0]);

  // Sum: '<S5>/Subtract'
  rtb_Gain1 = rtb_direct_distance - rtb_inversion;

  // Switch: '<S5>/Switch' incorporates:
  //   Abs: '<S5>/Abs'
  //   Bias: '<S5>/Bias'
  //   Switch: '<S10>/Switch'

  if (std::abs(rtb_Gain1) > PRF_P.Switch_Threshold) {
    // Bias: '<S5>/Bias1'
    rtb_DotProduct1 = rtb_Gain1 + PRF_P.Bias1_Bias;

    // Math: '<S10>/Mod' incorporates:
    //   Constant: '<S10>/Constant'

    rtb_Gain1 = rt_modf(rtb_DotProduct1, PRF_P.Constant_Value);

    // Switch: '<S10>/Switch' incorporates:
    //   Constant: '<S10>/Constant1'
    //   Constant: '<S11>/Constant'
    //   Constant: '<S12>/Constant'
    //   Logic: '<S10>/AND'
    //   RelationalOperator: '<S11>/Compare'
    //   RelationalOperator: '<S12>/Compare'

    if ((rtb_DotProduct1 > PRF_P.Constant_Value_a) && (rtb_Gain1 ==
         PRF_P.Constant_Value_j)) {
      rtb_Gain1 = PRF_P.Constant1_Value;
    }

    rtb_Gain1 += PRF_P.Bias_Bias;
  }

  // End of Switch: '<S5>/Switch'

  // Switch: '<S5>/Switch1' incorporates:
  //   Constant: '<S5>/Constant4'
  //   Constant: '<S5>/Constant5'
  //   Memory: '<S5>/Memory'
  //   Memory: '<S5>/Memory2'
  //   Product: '<S5>/Product3'

  if (PRF_DW.Memory2_PreviousInput > PRF_P.Switch1_Threshold) {
    rtb_Gain4 = PRF_DW.Memory_PreviousInput_k;
  } else {
    rtb_Gain4 = PRF_P.Constant4_Value * PRF_P.Constant5_Value * rtb_Gain1;
  }

  // End of Switch: '<S5>/Switch1'

  // Sum: '<S5>/Add1' incorporates:
  //   Constant: '<S5>/Constant1'
  //   Constant: '<S5>/Constant2'
  //   Constant: '<S5>/Constant3'
  //   Product: '<S5>/Product1'
  //   Product: '<S5>/Product2'
  //   UnitDelay: '<S5>/Unit Delay'

  rtb_DotProduct1 = (rtb_Gain1 * PRF_P.Constant3_Value_c + rtb_Gain4) +
    PRF_DW.UnitDelay_DSTATE * PRF_P.Constant2_Value / PRF_P.Constant1_Value_k;

  // Saturate: '<S5>/Saturation1'
  if (rtb_DotProduct1 > PRF_P.Saturation1_UpperSat) {
    rtb_DotProduct1 = PRF_P.Saturation1_UpperSat;
  } else if (rtb_DotProduct1 < PRF_P.Saturation1_LowerSat) {
    rtb_DotProduct1 = PRF_P.Saturation1_LowerSat;
  }

  // End of Saturate: '<S5>/Saturation1'

  // Signum: '<S5>/Sign'
  if (rtb_DotProduct1 < 0.0F) {
    i = -1;
  } else {
    i = (rtb_DotProduct1 > 0.0F);
  }

  // SwitchCase: '<S5>/Switch Case' incorporates:
  //   Constant: '<S8>/Zero'
  //   Constant: '<S9>/Zero'
  //   Merge: '<S5>/Merge'
  //   Signum: '<S5>/Sign'

  switch (i) {
   case 1:
    // Outputs for IfAction SubSystem: '<S5>/Servo Left' incorporates:
    //   ActionPort: '<S8>/Action Port'

    // SignalConversion generated from: '<S8>/Command' incorporates:
    //   Abs: '<S5>/Abs1'
    //   Merge: '<S5>/Merge'

    PRF_DW.Merge[0] = std::abs(rtb_DotProduct1);
    PRF_DW.Merge[1] = PRF_P.Zero_Value;

    // End of Outputs for SubSystem: '<S5>/Servo Left'
    break;

   case 0:
    // Outputs for IfAction SubSystem: '<S5>/No Activation' incorporates:
    //   ActionPort: '<S7>/Action Port'

    // Merge: '<S5>/Merge' incorporates:
    //   Constant: '<S7>/Zero'
    //   SignalConversion generated from: '<S7>/Servo'

    PRF_DW.Merge[0] = PRF_P.Zero_Value_c[0];
    PRF_DW.Merge[1] = PRF_P.Zero_Value_c[1];

    // End of Outputs for SubSystem: '<S5>/No Activation'
    break;

   default:
    // Outputs for IfAction SubSystem: '<S5>/Servo Right' incorporates:
    //   ActionPort: '<S9>/Action Port'

    // SignalConversion generated from: '<S9>/Command' incorporates:
    //   Abs: '<S5>/Abs1'
    //   Merge: '<S5>/Merge'

    PRF_DW.Merge[1] = std::abs(rtb_DotProduct1);
    PRF_DW.Merge[0] = PRF_P.Zero_Value_p;

    // End of Outputs for SubSystem: '<S5>/Servo Right'
    break;
  }

  // End of SwitchCase: '<S5>/Switch Case'

  // Bias: '<S6>/Bias1' incorporates:
  //   UnitDelay: '<S6>/Unit Delay1'

  u0 = static_cast<uint8_t>(PRF_DW.UnitDelay1_DSTATE + PRF_P.Bias1_Bias_b);

  // Saturate: '<S6>/Saturation'
  if (u0 > PRF_P.Saturation_UpperSat) {
    // Update for UnitDelay: '<S6>/Unit Delay1'
    PRF_DW.UnitDelay1_DSTATE = PRF_P.Saturation_UpperSat;
  } else if (u0 < PRF_P.Saturation_LowerSat) {
    // Update for UnitDelay: '<S6>/Unit Delay1'
    PRF_DW.UnitDelay1_DSTATE = PRF_P.Saturation_LowerSat;
  } else {
    // Update for UnitDelay: '<S6>/Unit Delay1'
    PRF_DW.UnitDelay1_DSTATE = u0;
  }

  // End of Saturate: '<S6>/Saturation'

  // Update for Memory: '<S5>/Memory'
  PRF_DW.Memory_PreviousInput_k = rtb_Gain4;

  // Update for Memory: '<S5>/Memory2'
  PRF_DW.Memory2_PreviousInput = 0.0F;

  // Update for UnitDelay: '<S5>/Unit Delay'
  PRF_DW.UnitDelay_DSTATE = rtb_Gain1;

  // End of Outputs for SubSystem: '<Root>/PRF'

  // Outport: '<Root>/Servo Commands'
  PRF_Y.ServoCommands[0] = PRF_DW.Merge[0];
  PRF_Y.ServoCommands[1] = PRF_DW.Merge[1];

  // Outputs for Atomic SubSystem: '<Root>/PRF'
  // Update for Memory: '<S4>/Memory1' incorporates:
  //   DotProduct: '<S4>/Dot Product1'
  //   Sqrt: '<S4>/Sqrt1'
  //   Sum: '<S4>/Sum1'

  PRF_DW.Memory1_PreviousInput = std::sqrt(rtb_k_par * rtb_k_par +
    rtb_Sum1_idx_1 * rtb_Sum1_idx_1);

  // Update for Memory: '<S4>/Memory' incorporates:
  //   DotProduct: '<S4>/Dot Product2'
  //   Sqrt: '<S4>/Sqrt2'
  //   Sum: '<S4>/Sum2'

  PRF_DW.Memory_PreviousInput = std::sqrt(rtb_Sum2_idx_0 * rtb_Sum2_idx_0 +
    rtb_Sum2_idx_1 * rtb_Sum2_idx_1);

  // End of Outputs for SubSystem: '<Root>/PRF'

  // Outport: '<Root>/PRF Logs OBSW' incorporates:
  //   Bias: '<S6>/Bias'
  //   BusAssignment: '<S5>/Bus Assignment'
  //   BusCreator generated from: '<S6>/PRF Logs OBSW_BusCreator'
  //   Math: '<S13>/Transpose1'

  for (i = 0; i < 6; i++) {
    // Outputs for Atomic SubSystem: '<Root>/PRF'
    PRF_Y.PRFLogsOBSW.Targets[i] = PRF_DW.TargetPoints[i];

    // End of Outputs for SubSystem: '<Root>/PRF'
  }

  // Outputs for Atomic SubSystem: '<Root>/PRF'
  PRF_Y.PRFLogsOBSW.TargetIndex = static_cast<uint8_t>(rtb_target +
    PRF_P.Bias_Bias_c);
  PRF_Y.PRFLogsOBSW.Heading = rtb_inversion;
  PRF_Y.PRFLogsOBSW.Reference = rtb_direct_distance;
  PRF_Y.PRFLogsOBSW.ServoCommands[0] = PRF_DW.Merge[0];
  PRF_Y.PRFLogsOBSW.Q1[0] = PRF_DW.Q1[0];
  PRF_Y.PRFLogsOBSW.Q2[0] = PRF_DW.Q2[0];
  PRF_Y.PRFLogsOBSW.ServoCommands[1] = PRF_DW.Merge[1];
  PRF_Y.PRFLogsOBSW.Q1[1] = PRF_DW.Q1[1];
  PRF_Y.PRFLogsOBSW.Q2[1] = PRF_DW.Q2[1];

  // End of Outport: '<Root>/PRF Logs OBSW'
  // End of Outputs for SubSystem: '<Root>/PRF'
}

// Model initialize function
void PRF::initialize()
{
  {
    int32_t i;
    PRF_PrevZCX.TargetPointsGeneration_Trig_ZCE = POS_ZCSIG;

    // SystemInitialize for Atomic SubSystem: '<Root>/PRF'
    // InitializeConditions for UnitDelay: '<S6>/Unit Delay1'
    PRF_DW.UnitDelay1_DSTATE = PRF_P.UnitDelay1_InitialCondition;

    // InitializeConditions for Memory: '<S4>/Memory1'
    PRF_DW.Memory1_PreviousInput = PRF_P.Memory1_InitialCondition;

    // InitializeConditions for Memory: '<S4>/Memory'
    PRF_DW.Memory_PreviousInput = PRF_P.Memory_InitialCondition;

    // InitializeConditions for Memory: '<S5>/Memory'
    PRF_DW.Memory_PreviousInput_k = PRF_P.Memory_InitialCondition_n;

    // InitializeConditions for Memory: '<S5>/Memory2'
    PRF_DW.Memory2_PreviousInput = PRF_P.Memory2_InitialCondition;

    // InitializeConditions for UnitDelay: '<S5>/Unit Delay'
    PRF_DW.UnitDelay_DSTATE = PRF_P.UnitDelay_InitialCondition;

    // SystemInitialize for Triggered SubSystem: '<S6>/Target Points Generation' 
    // SystemInitialize for Merge: '<S13>/Merge'
    PRF_DW.Q1[0] = PRF_P.Merge_InitialOutput;
    PRF_DW.Q1[1] = PRF_P.Merge_InitialOutput;
    for (i = 0; i < 6; i++) {
      // SystemInitialize for Math: '<S13>/Transpose1' incorporates:
      //   Outport: '<S13>/Target Points'

      PRF_DW.TargetPoints[i] = PRF_P.TargetPoints_Y0[i];
    }

    // SystemInitialize for Sum: '<S18>/Sum2' incorporates:
    //   Outport: '<S13>/Q2'

    PRF_DW.Q2[0] = PRF_P.Q2_Y0;

    // End of SystemInitialize for SubSystem: '<S6>/Target Points Generation'
    // End of SystemInitialize for SubSystem: '<Root>/PRF'

    // SystemInitialize for Merge: '<S5>/Merge'
    PRF_DW.Merge[0] = PRF_P.Merge_InitialOutput_l;

    // SystemInitialize for Outport: '<Root>/Servo Commands' incorporates:
    //   Merge: '<S5>/Merge'

    PRF_Y.ServoCommands[0] = PRF_P.Merge_InitialOutput_l;

    // SystemInitialize for Atomic SubSystem: '<Root>/PRF'
    // SystemInitialize for Triggered SubSystem: '<S6>/Target Points Generation' 
    // SystemInitialize for Sum: '<S18>/Sum2' incorporates:
    //   Outport: '<S13>/Q2'

    PRF_DW.Q2[1] = PRF_P.Q2_Y0;

    // End of SystemInitialize for SubSystem: '<S6>/Target Points Generation'
    // End of SystemInitialize for SubSystem: '<Root>/PRF'

    // SystemInitialize for Merge: '<S5>/Merge'
    PRF_DW.Merge[1] = PRF_P.Merge_InitialOutput_l;

    // SystemInitialize for Outport: '<Root>/Servo Commands' incorporates:
    //   Merge: '<S5>/Merge'

    PRF_Y.ServoCommands[1] = PRF_P.Merge_InitialOutput_l;
  }
}

// Model terminate function
void PRF::terminate()
{
  // (no terminate code required)
}

// Constructor
PRF::PRF():
  PRF_U(),
  PRF_Y(),
  PRF_DW(),
  PRF_PrevZCX()
{
  // Currently there is no constructor body generated.
}

// Destructor
// Currently there is no destructor body generated.
PRF::~PRF() = default;


} // fine namespace PRF

//
// File trailer for generated code.
//
// [EOF]
//
