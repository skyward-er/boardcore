//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// File: PRF.cpp
//
// Code generated for Simulink model 'PRF'.
//
// Model version                  : 11.338
// Simulink Coder version         : 24.2 (R2024b) 21-Jun-2024
// C/C++ source code generated on : Tue Jul 14 15:22:34 2026
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
  float rtb_Gain1;
  float rtb_Gain4;
  float rtb_L_glide;
  float rtb_Saturation1;
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
  bool rtb_Compare_b;

  // Outputs for Atomic SubSystem: '<Root>/PRF'
  // Outputs for Triggered SubSystem: '<S6>/Target Points Generation' incorporates:
  //   TriggerPort: '<S15>/function'

  // UnitDelay: '<S6>/Unit Delay1' incorporates:
  //   Logic: '<S15>/NOT'

  if ((PRF_DW.UnitDelay1_DSTATE > 0) &&
      (PRF_PrevZCX.TargetPointsGeneration_Trig_ZCE != POS_ZCSIG)) {
    // Switch: '<S22>/FixPt Switch' incorporates:
    //   Constant: '<S22>/Constant'
    //   Inport: '<Root>/PRF Reference'

    if (PRF_U.PRFReference_i.WindDirection > PRF_P.WrapToZero_Threshold) {
      rtb_Saturation1 = PRF_P.Constant_Value_p;
    } else {
      rtb_Saturation1 = PRF_U.PRFReference_i.WindDirection;
    }

    // Gain: '<S21>/Gain1' incorporates:
    //   Switch: '<S22>/FixPt Switch'

    rtb_Saturation1 *= PRF_P.Gain1_Gain_g;

    // Sum: '<S20>/Sum2' incorporates:
    //   Constant: '<S15>/Constant5'
    //   Inport: '<Root>/PRF Reference'
    //   Product: '<S20>/Product'
    //   Product: '<S20>/Product1'
    //   Trigonometry: '<S20>/Cos'
    //   Trigonometry: '<S20>/Sin'

    PRF_DW.Q2[0] = std::cos(rtb_Saturation1) * PRF_P.Constant5_Value_d +
      PRF_U.PRFReference_i.TargetPositionNED[0];
    PRF_DW.Q2[1] = std::sin(rtb_Saturation1) * PRF_P.Constant5_Value_d +
      PRF_U.PRFReference_i.TargetPositionNED[1];

    // SignalConversion generated from: '<S15>/Vector Concatenate3' incorporates:
    //   Inport: '<Root>/PRF Reference'

    rtb_VectorConcatenate3[0] = PRF_U.PRFReference_i.TargetPositionNED[0];

    // SignalConversion generated from: '<S15>/Vector Concatenate3' incorporates:
    //   DataTypeConversion: '<S15>/Cast To Single1'

    rtb_VectorConcatenate3[2] = PRF_DW.Q2[0];

    // Sum: '<S19>/Sum' incorporates:
    //   DataTypeConversion: '<S15>/Cast To Single1'
    //   Inport: '<Root>/PRF In'

    rtb_Sum2_idx_1 = PRF_DW.Q2[0] - PRF_U.PRFIn_f.NASDAQPosition[0];
    rtb_Sum2_idx_0 = rtb_Sum2_idx_1;

    // Abs: '<S19>/Abs'
    rtb_inversion = std::abs(rtb_Sum2_idx_1);

    // DotProduct: '<S19>/Dot Product'
    rtb_direct_distance = rtb_inversion * rtb_inversion;

    // SignalConversion generated from: '<S15>/Vector Concatenate3' incorporates:
    //   Inport: '<Root>/PRF Reference'

    rtb_VectorConcatenate3[1] = PRF_U.PRFReference_i.TargetPositionNED[1];

    // SignalConversion generated from: '<S15>/Vector Concatenate3' incorporates:
    //   DataTypeConversion: '<S15>/Cast To Single1'

    rtb_VectorConcatenate3[3] = PRF_DW.Q2[1];

    // Sum: '<S19>/Sum' incorporates:
    //   DataTypeConversion: '<S15>/Cast To Single1'
    //   Inport: '<Root>/PRF In'

    rtb_Sum2_idx_1 = PRF_DW.Q2[1] - PRF_U.PRFIn_f.NASDAQPosition[1];

    // Abs: '<S19>/Abs'
    rtb_inversion = std::abs(rtb_Sum2_idx_1);

    // Sqrt: '<S19>/Sqrt2' incorporates:
    //   DotProduct: '<S19>/Dot Product'

    rtb_direct_distance = std::sqrt(rtb_inversion * rtb_inversion +
      rtb_direct_distance);

    // Abs: '<S19>/Abs1' incorporates:
    //   Constant: '<S15>/glide ratio'
    //   Inport: '<Root>/PRF In'
    //   Product: '<S19>/Product1'

    rtb_L_glide = std::abs(PRF_P.glideratio_Value *
      PRF_U.PRFIn_f.NASDAQPosition[2]);

    // RelationalOperator: '<S16>/Compare' incorporates:
    //   Constant: '<S16>/Constant'
    //   Sum: '<S15>/Sum'

    rtb_Compare_b = (rtb_L_glide - rtb_direct_distance >
                     PRF_P.Comparetoconstant_const);

    // Outputs for Enabled SubSystem: '<S15>/Enabled Subsystem1' incorporates:
    //   EnablePort: '<S18>/Enable'

    if (!rtb_Compare_b) {
      // Merge: '<S15>/Merge' incorporates:
      //   DataTypeConversion: '<S15>/Cast To Single1'
      //   SignalConversion generated from: '<S18>/Q2'

      PRF_DW.Q1[0] = PRF_DW.Q2[0];
      PRF_DW.Q1[1] = PRF_DW.Q2[1];
    }

    // End of Outputs for SubSystem: '<S15>/Enabled Subsystem1'

    // Product: '<S19>/Product2' incorporates:
    //   Logic: '<S15>/NOT'
    //   Sum: '<S19>/Sum'

    rtb_Sum2_idx_0 /= rtb_direct_distance;
    rtb_Sum2_idx_1 /= rtb_direct_distance;

    // Gain: '<S19>/Gain'
    rtb_Sum1_idx_1 = PRF_P.Gain_Gain * rtb_Sum2_idx_0;

    // DotProduct: '<S15>/Dot Product' incorporates:
    //   Concatenate: '<S19>/Vector Concatenate'
    //   Inport: '<Root>/PRF Reference'
    //   SignalConversion generated from: '<S19>/Vector Concatenate'
    //   Sum: '<S20>/Sum'

    rtb_inversion = (PRF_DW.Q2[0] - PRF_U.PRFReference_i.TargetPositionNED[0]) *
      rtb_Sum2_idx_1 + (PRF_DW.Q2[1] - PRF_U.PRFReference_i.TargetPositionNED[1])
      * rtb_Sum1_idx_1;

    // Signum: '<S15>/1 or -1' incorporates:
    //   DotProduct: '<S15>/Dot Product'

    if (rtb_inversion < 0.0F) {
      rtb_inversion = -1.0F;
    } else {
      rtb_inversion = (rtb_inversion > 0.0F);
    }

    // End of Signum: '<S15>/1 or -1'

    // Gain: '<S19>/Gain1'
    rtb_Gain1 = PRF_P.Gain1_Gain_h * rtb_direct_distance;

    // Outputs for Enabled SubSystem: '<S15>/Enabled Subsystem' incorporates:
    //   EnablePort: '<S17>/Enable'

    if (rtb_Compare_b) {
      // Gain: '<S17>/Gain2'
      rtb_L_glide *= PRF_P.Gain2_Gain;

      // Gain: '<S17>/Gain4'
      rtb_Gain4 = PRF_P.Gain4_Gain * rtb_L_glide;

      // Trigonometry: '<S17>/Sin' incorporates:
      //   Constant: '<S15>/theta'

      rtb_k_par = std::sin(PRF_P.theta_Value);

      // Trigonometry: '<S17>/Cos' incorporates:
      //   Constant: '<S15>/theta'

      rtb_k_perp = std::cos(PRF_P.theta_Value);

      // Gain: '<S17>/Gain3'
      rtb_direct_distance *= PRF_P.Gain3_Gain;

      // Gain: '<S17>/Gain1' incorporates:
      //   Math: '<S17>/Square'
      //   Math: '<S17>/Square1'
      //   Sqrt: '<S17>/Sqrt1'
      //   Sum: '<S17>/Sum2'

      rtb_L_glide = std::sqrt(rtb_L_glide * rtb_L_glide - rtb_direct_distance *
        rtb_direct_distance) * PRF_P.Gain1_Gain;

      // Merge: '<S15>/Merge' incorporates:
      //   Concatenate: '<S19>/Vector Concatenate'
      //   Inport: '<Root>/PRF In'
      //   Product: '<S17>/Product'
      //   Product: '<S17>/Product1'
      //   Product: '<S19>/Product'
      //   SignalConversion generated from: '<S19>/Vector Concatenate'
      //   Sum: '<S17>/Sum'
      //   Sum: '<S19>/Sum1'

      PRF_DW.Q1[0] = (rtb_Sum2_idx_1 * rtb_L_glide * rtb_k_perp * rtb_inversion
                      + rtb_Sum2_idx_0 * rtb_Gain4 * rtb_k_par) +
        (rtb_Sum2_idx_0 * rtb_Gain1 + PRF_U.PRFIn_f.NASDAQPosition[0]);
      PRF_DW.Q1[1] = (rtb_Sum1_idx_1 * rtb_L_glide * rtb_k_perp * rtb_inversion
                      + rtb_Sum2_idx_1 * rtb_Gain4 * rtb_k_par) +
        (rtb_Sum2_idx_1 * rtb_Gain1 + PRF_U.PRFIn_f.NASDAQPosition[1]);
    }

    // End of Outputs for SubSystem: '<S15>/Enabled Subsystem'

    // SignalConversion generated from: '<S15>/Vector Concatenate3' incorporates:
    //   Merge: '<S15>/Merge'

    rtb_VectorConcatenate3[4] = PRF_DW.Q1[0];
    rtb_VectorConcatenate3[5] = PRF_DW.Q1[1];

    // Math: '<S15>/Transpose1' incorporates:
    //   Concatenate: '<S15>/Vector Concatenate3'

    i = 0;
    for (i_0 = 0; i_0 < 2; i_0++) {
      PRF_DW.TargetPoints[i] = rtb_VectorConcatenate3[i_0];
      PRF_DW.TargetPoints[i + 1] = rtb_VectorConcatenate3[i_0 + 2];
      PRF_DW.TargetPoints[i + 2] = rtb_VectorConcatenate3[i_0 + 4];
      i += 3;
    }

    // BusCreator generated from: '<S20>/PRF Logs OBSW_BusCreator' incorporates:
    //   Constant: '<S15>/Constant5'

    for (i = 0; i < 6; i++) {
      PRF_DW.PRFLogsOBSW_BusCreator_BusCreat.Targets[i] = 0.0F;
    }

    PRF_DW.PRFLogsOBSW_BusCreator_BusCreat.TargetIndex = 0U;
    PRF_DW.PRFLogsOBSW_BusCreator_BusCreat.Heading = 0.0F;
    PRF_DW.PRFLogsOBSW_BusCreator_BusCreat.Reference = 0.0F;
    PRF_DW.PRFLogsOBSW_BusCreator_BusCreat.TerminalTarget[0] = 0.0F;
    PRF_DW.PRFLogsOBSW_BusCreator_BusCreat.ServoCommands[0] = 0.0F;
    PRF_DW.PRFLogsOBSW_BusCreator_BusCreat.TerminalTarget[1] = 0.0F;
    PRF_DW.PRFLogsOBSW_BusCreator_BusCreat.ServoCommands[1] = 0.0F;
    PRF_DW.PRFLogsOBSW_BusCreator_BusCreat.WindHeading = rtb_Saturation1;
    PRF_DW.PRFLogsOBSW_BusCreator_BusCreat.WindAlignmentRadius =
      PRF_P.Constant5_Value_d;
  }

  PRF_PrevZCX.TargetPointsGeneration_Trig_ZCE = (PRF_DW.UnitDelay1_DSTATE > 0);

  // End of Outputs for SubSystem: '<S6>/Target Points Generation'

  // Abs: '<S4>/Abs' incorporates:
  //   Inport: '<Root>/PRF In'
  //   Inport: '<Root>/PRF Reference'
  //   Sum: '<S4>/Sum'

  rtb_Saturation1 = std::abs(PRF_U.PRFReference_i.TargetPositionNED[0] -
    PRF_U.PRFIn_f.NASDAQPosition[0]);

  // DotProduct: '<S4>/Dot Product'
  rtb_Sum2_idx_0 = rtb_Saturation1 * rtb_Saturation1;

  // Abs: '<S4>/Abs' incorporates:
  //   Inport: '<Root>/PRF In'
  //   Inport: '<Root>/PRF Reference'
  //   Sum: '<S4>/Sum'

  rtb_Saturation1 = std::abs(PRF_U.PRFReference_i.TargetPositionNED[1] -
    PRF_U.PRFIn_f.NASDAQPosition[1]);

  // Gain: '<S4>/PRFControl.zThresholdGain' incorporates:
  //   Constant: '<S4>/glide ratio'
  //   DotProduct: '<S4>/Dot Product'
  //   Product: '<S4>/Product'
  //   Sqrt: '<S4>/Sqrt'

  rtb_Saturation1 = std::sqrt(rtb_Saturation1 * rtb_Saturation1 + rtb_Sum2_idx_0)
    / PRF_P.glideratio_Value_o * PRF_P.PRFControlzThresholdGain_Gain;

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
      if (PRF_U.PRFIn_f.NASDAQPosition[2] > -rtb_Saturation1) {
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
          (PRF_U.PRFIn_f.NASDAQPosition[2] > -rtb_Saturation1)) {
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
  //   Math: '<S5>/Transpose'
  //   Merge: '<S15>/Merge'
  //   Selector: '<S6>/Selector'

  rtb_Gain4 = PRF_DW.Q1[0] - PRF_U.PRFIn_f.NASDAQPosition[0];

  // Sum: '<S4>/Sum2' incorporates:
  //   Inport: '<Root>/PRF In'
  //   Math: '<S5>/Transpose'
  //   Selector: '<S6>/Selector'

  rtb_Sum2_idx_0 = PRF_DW.Q2[0] - PRF_U.PRFIn_f.NASDAQPosition[0];

  // Sum: '<S4>/Sum1' incorporates:
  //   Inport: '<Root>/PRF In'
  //   Math: '<S5>/Transpose'
  //   Merge: '<S15>/Merge'
  //   Selector: '<S6>/Selector'

  rtb_Sum1_idx_1 = PRF_DW.Q1[1] - PRF_U.PRFIn_f.NASDAQPosition[1];

  // Sum: '<S4>/Sum2' incorporates:
  //   Inport: '<Root>/PRF In'
  //   Math: '<S5>/Transpose'
  //   Selector: '<S6>/Selector'

  rtb_Sum2_idx_1 = PRF_DW.Q2[1] - PRF_U.PRFIn_f.NASDAQPosition[1];

  // Trigonometry: '<S5>/Atan1' incorporates:
  //   Inport: '<Root>/PRF In'
  //   Math: '<S15>/Transpose1'
  //   Math: '<S5>/Transpose'
  //   Selector: '<S6>/Selector'
  //   Sum: '<S5>/Subtract1'

  rtb_direct_distance = std::atan2(PRF_DW.TargetPoints[rtb_target + 3] -
    PRF_U.PRFIn_f.NASDAQPosition[1], PRF_DW.TargetPoints[rtb_target] -
    PRF_U.PRFIn_f.NASDAQPosition[0]);

  // Trigonometry: '<S5>/Atan2' incorporates:
  //   Inport: '<Root>/PRF In'

  rtb_L_glide = std::atan2(PRF_U.PRFIn_f.NASDAQVelocity[1],
    PRF_U.PRFIn_f.NASDAQVelocity[0]);

  // Sum: '<S5>/Subtract'
  rtb_inversion = rtb_direct_distance - rtb_L_glide;

  // Switch: '<S5>/Switch' incorporates:
  //   Abs: '<S5>/Abs'
  //   Bias: '<S5>/Bias'
  //   Switch: '<S12>/Switch'

  if (std::abs(rtb_inversion) > PRF_P.Switch_Threshold) {
    // Bias: '<S5>/Bias1'
    rtb_Saturation1 = rtb_inversion + PRF_P.Bias1_Bias;

    // Math: '<S12>/Mod' incorporates:
    //   Constant: '<S12>/Constant'

    rtb_inversion = rt_modf(rtb_Saturation1, PRF_P.Constant_Value);

    // Switch: '<S12>/Switch' incorporates:
    //   Constant: '<S12>/Constant1'
    //   Constant: '<S13>/Constant'
    //   Constant: '<S14>/Constant'
    //   Logic: '<S12>/AND'
    //   RelationalOperator: '<S13>/Compare'
    //   RelationalOperator: '<S14>/Compare'

    if ((rtb_Saturation1 > PRF_P.Constant_Value_i) && (rtb_inversion ==
         PRF_P.Constant_Value_a)) {
      rtb_inversion = PRF_P.Constant1_Value;
    }

    rtb_inversion += PRF_P.Bias_Bias;
  }

  // End of Switch: '<S5>/Switch'

  // Switch: '<S5>/Switch1' incorporates:
  //   Constant: '<S5>/Constant4'
  //   Constant: '<S5>/Constant5'
  //   Memory: '<S5>/Memory'
  //   Memory: '<S5>/Memory2'
  //   Product: '<S5>/Product3'

  if (PRF_DW.Memory2_PreviousInput) {
    rtb_Gain1 = PRF_DW.Memory_PreviousInput_b;
  } else {
    rtb_Gain1 = PRF_P.Constant4_Value * PRF_P.Constant5_Value * rtb_inversion;
  }

  // End of Switch: '<S5>/Switch1'

  // Sum: '<S5>/Add1' incorporates:
  //   Constant: '<S5>/Constant1'
  //   Constant: '<S5>/Constant2'
  //   Constant: '<S5>/Constant3'
  //   Product: '<S5>/Product1'
  //   Product: '<S5>/Product2'
  //   UnitDelay: '<S5>/Unit Delay'

  rtb_Saturation1 = (rtb_inversion * PRF_P.Constant3_Value + rtb_Gain1) +
    PRF_DW.UnitDelay_DSTATE * PRF_P.Constant2_Value / PRF_P.Constant1_Value_p;

  // Saturate: '<S5>/Saturation1'
  if (rtb_Saturation1 > PRF_P.Saturation1_UpperSat) {
    rtb_Saturation1 = PRF_P.Saturation1_UpperSat;
  } else if (rtb_Saturation1 < PRF_P.Saturation1_LowerSat) {
    rtb_Saturation1 = PRF_P.Saturation1_LowerSat;
  }

  // End of Saturate: '<S5>/Saturation1'

  // Signum: '<S5>/Sign'
  if (rtb_Saturation1 < 0.0F) {
    i = -1;
  } else {
    i = (rtb_Saturation1 > 0.0F);
  }

  // SwitchCase: '<S5>/Switch Case' incorporates:
  //   Constant: '<S10>/Zero'
  //   Constant: '<S11>/Zero'
  //   Merge: '<S5>/Merge'
  //   Signum: '<S5>/Sign'

  switch (i) {
   case 1:
    // Outputs for IfAction SubSystem: '<S5>/Servo Right' incorporates:
    //   ActionPort: '<S11>/Action Port'

    // SignalConversion generated from: '<S11>/Command' incorporates:
    //   Abs: '<S5>/Abs1'
    //   Merge: '<S5>/Merge'

    PRF_DW.Merge[1] = std::abs(rtb_Saturation1);
    PRF_DW.Merge[0] = PRF_P.Zero_Value;

    // End of Outputs for SubSystem: '<S5>/Servo Right'
    break;

   case 0:
    // Outputs for IfAction SubSystem: '<S5>/No Activation' incorporates:
    //   ActionPort: '<S7>/Action Port'

    // Merge: '<S5>/Merge' incorporates:
    //   Constant: '<S7>/Zero'
    //   SignalConversion generated from: '<S7>/Servo'

    PRF_DW.Merge[0] = PRF_P.Zero_Value_b[0];
    PRF_DW.Merge[1] = PRF_P.Zero_Value_b[1];

    // End of Outputs for SubSystem: '<S5>/No Activation'
    break;

   default:
    // Outputs for IfAction SubSystem: '<S5>/Servo Left' incorporates:
    //   ActionPort: '<S10>/Action Port'

    // SignalConversion generated from: '<S10>/Command' incorporates:
    //   Abs: '<S5>/Abs1'
    //   Merge: '<S5>/Merge'

    PRF_DW.Merge[0] = std::abs(rtb_Saturation1);
    PRF_DW.Merge[1] = PRF_P.Zero_Value_f;

    // End of Outputs for SubSystem: '<S5>/Servo Left'
    break;
  }

  // End of SwitchCase: '<S5>/Switch Case'

  // Bias: '<S6>/Bias1' incorporates:
  //   UnitDelay: '<S6>/Unit Delay1'

  u0 = static_cast<uint8_t>(PRF_DW.UnitDelay1_DSTATE + PRF_P.Bias1_Bias_n);

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
  PRF_DW.Memory_PreviousInput_b = rtb_Gain1;

  // Update for Memory: '<S5>/Memory2' incorporates:
  //   Constant: '<S8>/Constant'
  //   Constant: '<S9>/Constant'
  //   Logic: '<S5>/OR2'
  //   RelationalOperator: '<S8>/Compare'
  //   RelationalOperator: '<S9>/Compare'

  PRF_DW.Memory2_PreviousInput = ((rtb_Saturation1 ==
    PRF_P.SaturationCheckUp_const) || (rtb_Saturation1 ==
    PRF_P.SaturationCheckLw_const));

  // Update for UnitDelay: '<S5>/Unit Delay'
  PRF_DW.UnitDelay_DSTATE = rtb_inversion;

  // End of Outputs for SubSystem: '<Root>/PRF'

  // Outport: '<Root>/Servo Commands'
  PRF_Y.ServoCommands[0] = PRF_DW.Merge[0];
  PRF_Y.ServoCommands[1] = PRF_DW.Merge[1];

  // Outputs for Atomic SubSystem: '<Root>/PRF'
  // Update for Memory: '<S4>/Memory1' incorporates:
  //   DotProduct: '<S4>/Dot Product1'
  //   Sqrt: '<S4>/Sqrt1'
  //   Sum: '<S4>/Sum1'

  PRF_DW.Memory1_PreviousInput = std::sqrt(rtb_Gain4 * rtb_Gain4 +
    rtb_Sum1_idx_1 * rtb_Sum1_idx_1);

  // Update for Memory: '<S4>/Memory' incorporates:
  //   DotProduct: '<S4>/Dot Product2'
  //   Sqrt: '<S4>/Sqrt2'

  PRF_DW.Memory_PreviousInput = std::sqrt(rtb_Sum2_idx_0 * rtb_Sum2_idx_0 +
    rtb_Sum2_idx_1 * rtb_Sum2_idx_1);

  // End of Outputs for SubSystem: '<Root>/PRF'

  // Outport: '<Root>/PRF Logs OBSW' incorporates:
  //   Bias: '<S6>/Bias'
  //   BusAssignment: '<S5>/Bus Assignment'
  //   BusAssignment: '<S6>/Bus Assignment1'
  //   BusCreator generated from: '<S20>/PRF Logs OBSW_BusCreator'
  //   Math: '<S15>/Transpose1'
  //   SignalConversion generated from: '<S6>/Bus Assignment1'

  for (i = 0; i < 6; i++) {
    // Outputs for Atomic SubSystem: '<Root>/PRF'
    PRF_Y.PRFLogsOBSW.Targets[i] = PRF_DW.TargetPoints[i];

    // End of Outputs for SubSystem: '<Root>/PRF'
  }

  // Outputs for Atomic SubSystem: '<Root>/PRF'
  PRF_Y.PRFLogsOBSW.TargetIndex = static_cast<uint8_t>(rtb_target +
    PRF_P.Bias_Bias_i);
  PRF_Y.PRFLogsOBSW.Heading = rtb_L_glide;
  PRF_Y.PRFLogsOBSW.Reference = rtb_direct_distance;
  PRF_Y.PRFLogsOBSW.TerminalTarget[0] =
    PRF_DW.PRFLogsOBSW_BusCreator_BusCreat.TerminalTarget[0];
  PRF_Y.PRFLogsOBSW.ServoCommands[0] = PRF_DW.Merge[0];
  PRF_Y.PRFLogsOBSW.TerminalTarget[1] =
    PRF_DW.PRFLogsOBSW_BusCreator_BusCreat.TerminalTarget[1];
  PRF_Y.PRFLogsOBSW.ServoCommands[1] = PRF_DW.Merge[1];
  PRF_Y.PRFLogsOBSW.WindHeading =
    PRF_DW.PRFLogsOBSW_BusCreator_BusCreat.WindHeading;
  PRF_Y.PRFLogsOBSW.WindAlignmentRadius =
    PRF_DW.PRFLogsOBSW_BusCreator_BusCreat.WindAlignmentRadius;

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
    PRF_DW.Memory_PreviousInput_b = PRF_P.Memory_InitialCondition_b;

    // InitializeConditions for Memory: '<S5>/Memory2'
    PRF_DW.Memory2_PreviousInput = PRF_P.Memory2_InitialCondition;

    // InitializeConditions for UnitDelay: '<S5>/Unit Delay'
    PRF_DW.UnitDelay_DSTATE = PRF_P.UnitDelay_InitialCondition;

    // SystemInitialize for Triggered SubSystem: '<S6>/Target Points Generation' 
    // SystemInitialize for Merge: '<S15>/Merge'
    PRF_DW.Q1[0] = PRF_P.Merge_InitialOutput;
    PRF_DW.Q1[1] = PRF_P.Merge_InitialOutput;
    for (i = 0; i < 6; i++) {
      // SystemInitialize for Math: '<S15>/Transpose1' incorporates:
      //   Outport: '<S15>/Target Points'

      PRF_DW.TargetPoints[i] = PRF_P.TargetPoints_Y0;
    }

    // SystemInitialize for BusCreator generated from: '<S20>/PRF Logs OBSW_BusCreator' incorporates:
    //   Outport: '<S15>/PRF Logs OBSW'

    PRF_DW.PRFLogsOBSW_BusCreator_BusCreat = PRF_P.PRFLogsOBSW_Y0;

    // SystemInitialize for Sum: '<S20>/Sum2' incorporates:
    //   Outport: '<S15>/Q2'

    PRF_DW.Q2[0] = PRF_P.Q2_Y0;

    // End of SystemInitialize for SubSystem: '<S6>/Target Points Generation'
    // End of SystemInitialize for SubSystem: '<Root>/PRF'

    // SystemInitialize for Merge: '<S5>/Merge'
    PRF_DW.Merge[0] = PRF_P.Merge_InitialOutput_g;

    // SystemInitialize for Outport: '<Root>/Servo Commands' incorporates:
    //   Merge: '<S5>/Merge'

    PRF_Y.ServoCommands[0] = PRF_P.Merge_InitialOutput_g;

    // SystemInitialize for Atomic SubSystem: '<Root>/PRF'
    // SystemInitialize for Triggered SubSystem: '<S6>/Target Points Generation' 
    // SystemInitialize for Sum: '<S20>/Sum2' incorporates:
    //   Outport: '<S15>/Q2'

    PRF_DW.Q2[1] = PRF_P.Q2_Y0;

    // End of SystemInitialize for SubSystem: '<S6>/Target Points Generation'
    // End of SystemInitialize for SubSystem: '<Root>/PRF'

    // SystemInitialize for Merge: '<S5>/Merge'
    PRF_DW.Merge[1] = PRF_P.Merge_InitialOutput_g;

    // SystemInitialize for Outport: '<Root>/Servo Commands' incorporates:
    //   Merge: '<S5>/Merge'

    PRF_Y.ServoCommands[1] = PRF_P.Merge_InitialOutput_g;
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
