//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// File: PRF.cpp
//
// Code generated for Simulink model 'PRF'.
//
// Model version                  : 11.339
// Simulink Coder version         : 24.2 (R2024b) 21-Jun-2024
// C/C++ source code generated on : Wed Jul 15 16:02:04 2026
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
  float RateTransition_idx_0;
  float RateTransition_idx_1;
  float rtb_Gain1_h;
  float rtb_Gain4;
  float rtb_Heading;
  float rtb_Reference;
  float rtb_Saturation1;
  float rtb_Sum1_a_idx_1;
  float rtb_Sum2_idx_0;
  float rtb_Sum2_idx_1;
  float rtb_k_par;
  float rtb_k_perp;
  float rtb_p_perp;
  int32_t i;
  int32_t i_0;
  uint8_t rtb_target;
  uint8_t u0;
  bool rtb_Compare_b;

  // Outputs for Atomic SubSystem: '<Root>/PRF'
  // Outputs for Atomic SubSystem: '<S1>/LLA to NED'
  // Sum: '<S26>/Sum' incorporates:
  //   Constant: '<S26>/Constant2'
  //   Constant: '<S26>/f'

  // Unit Conversion - from: deg to: rad
  // Expression: output = (0.0174533*input) + (0)
  rtb_Saturation1 = PRF_P.f_Value - PRF_P.Constant2_Value;

  // Sqrt: '<S26>/sqrt1' incorporates:
  //   Constant: '<S26>/Constant3'
  //   Product: '<S26>/Product5'
  //   Sum: '<S26>/Sum2'

  rtb_Saturation1 = std::sqrt(PRF_P.Constant3_Value - rtb_Saturation1 *
    rtb_Saturation1);

  // UnitConversion: '<S27>/Unit Conversion' incorporates:
  //   Constant: '<S1>/Constant1'

  // Unit Conversion - from: deg to: rad
  // Expression: output = (0.0174533*input) + (0)
  rtb_Heading = 0.0174532924F * PRF_P.Constant1_Value_h[0];

  // Trigonometry: '<S28>/Trigonometric Function1'
  rtb_Reference = std::sin(rtb_Heading);

  // Product: '<S28>/Product1' incorporates:
  //   Product: '<S26>/Product2'

  RateTransition_idx_0 = rtb_Saturation1 * rtb_Saturation1;

  // Sum: '<S28>/Sum1' incorporates:
  //   Constant: '<S28>/Constant'
  //   Product: '<S28>/Product1'

  rtb_Reference = PRF_P.Constant_Value_o - RateTransition_idx_0 * rtb_Reference *
    rtb_Reference;

  // Product: '<S26>/Product1' incorporates:
  //   Constant: '<S26>/Constant1'
  //   Sqrt: '<S26>/sqrt'

  rtb_p_perp = PRF_P.Constant1_Value_l / std::sqrt(rtb_Reference);

  // Product: '<S24>/dNorth' incorporates:
  //   Constant: '<S1>/Constant1'
  //   Constant: '<S26>/Constant'
  //   Inport: '<Root>/PRF Reference'
  //   Product: '<S26>/Product3'
  //   Sum: '<S26>/Sum1'
  //   Sum: '<S3>/Sum1'
  //   UnitConversion: '<S25>/Unit Conversion'

  rtb_Reference = (PRF_P.Constant_Value_i - RateTransition_idx_0) * rtb_p_perp /
    rtb_Reference * ((PRF_U.PRFReference_i.TargetPositionLLA[0] -
                      PRF_P.Constant1_Value_h[0]) * 0.0174532924F);

  // Trigonometry: '<S24>/SinCos' incorporates:
  //   Constant: '<S24>/Zero'

  rtb_Saturation1 = std::sin(PRF_P.Zero_Value_m);
  rtb_Gain1_h = std::cos(PRF_P.Zero_Value_m);

  // Product: '<S24>/dEast' incorporates:
  //   Constant: '<S1>/Constant1'
  //   Inport: '<Root>/PRF Reference'
  //   Product: '<S26>/Product4'
  //   Sum: '<S3>/Sum1'
  //   Trigonometry: '<S26>/Trigonometric Function'
  //   UnitConversion: '<S25>/Unit Conversion'

  rtb_p_perp = (PRF_U.PRFReference_i.TargetPositionLLA[1] -
                PRF_P.Constant1_Value_h[1]) * 0.0174532924F * (rtb_p_perp * std::
    cos(rtb_Heading));

  // RateTransition: '<S1>/Rate Transition' incorporates:
  //   Product: '<S24>/x*cos'
  //   Product: '<S24>/x*sin'
  //   Product: '<S24>/y*cos'
  //   Product: '<S24>/y*sin'
  //   Sum: '<S24>/Sum2'
  //   Sum: '<S24>/Sum3'

  RateTransition_idx_0 = rtb_Reference * rtb_Gain1_h + rtb_p_perp *
    rtb_Saturation1;
  RateTransition_idx_1 = rtb_p_perp * rtb_Gain1_h - rtb_Reference *
    rtb_Saturation1;

  // End of Outputs for SubSystem: '<S1>/LLA to NED'

  // Outputs for Triggered SubSystem: '<S7>/Target Points Generation' incorporates:
  //   TriggerPort: '<S16>/function'

  // UnitDelay: '<S7>/Unit Delay1' incorporates:
  //   Logic: '<S16>/NOT'

  if ((PRF_DW.UnitDelay1_DSTATE > 0) &&
      (PRF_PrevZCX.TargetPointsGeneration_Trig_ZCE != POS_ZCSIG)) {
    // Switch: '<S23>/FixPt Switch' incorporates:
    //   Constant: '<S23>/Constant'

    if (PRF_P.WrapToZero_Threshold < 0.0F) {
      rtb_p_perp = PRF_P.Constant_Value_p;
    } else {
      rtb_p_perp = 0.0F;
    }

    // Gain: '<S22>/Gain1' incorporates:
    //   Switch: '<S23>/FixPt Switch'

    rtb_Gain1_h = PRF_P.Gain1_Gain_g * rtb_p_perp;

    // Sum: '<S21>/Sum2' incorporates:
    //   Constant: '<S16>/Constant5'
    //   Product: '<S21>/Product'
    //   Product: '<S21>/Product1'
    //   Trigonometry: '<S21>/Cos'
    //   Trigonometry: '<S21>/Sin'

    PRF_DW.Q2[0] = std::cos(rtb_Gain1_h) * PRF_P.Constant5_Value_d +
      RateTransition_idx_0;
    PRF_DW.Q2[1] = std::sin(rtb_Gain1_h) * PRF_P.Constant5_Value_d +
      RateTransition_idx_1;

    // SignalConversion generated from: '<S16>/Vector Concatenate3'
    rtb_VectorConcatenate3[0] = RateTransition_idx_0;

    // SignalConversion generated from: '<S16>/Vector Concatenate3' incorporates:
    //   DataTypeConversion: '<S16>/Cast To Single1'

    rtb_VectorConcatenate3[2] = PRF_DW.Q2[0];

    // Sum: '<S20>/Sum' incorporates:
    //   DataTypeConversion: '<S16>/Cast To Single1'
    //   Inport: '<Root>/PRF In'

    rtb_Reference = PRF_DW.Q2[0] - PRF_U.PRFIn_f.NASDAQPosition[0];
    rtb_Sum2_idx_0 = rtb_Reference;

    // Abs: '<S20>/Abs'
    rtb_Heading = std::abs(rtb_Reference);

    // DotProduct: '<S20>/Dot Product'
    rtb_p_perp = rtb_Heading * rtb_Heading;

    // SignalConversion generated from: '<S16>/Vector Concatenate3'
    rtb_VectorConcatenate3[1] = RateTransition_idx_1;

    // SignalConversion generated from: '<S16>/Vector Concatenate3' incorporates:
    //   DataTypeConversion: '<S16>/Cast To Single1'

    rtb_VectorConcatenate3[3] = PRF_DW.Q2[1];

    // Sum: '<S20>/Sum' incorporates:
    //   DataTypeConversion: '<S16>/Cast To Single1'
    //   Inport: '<Root>/PRF In'

    rtb_Reference = PRF_DW.Q2[1] - PRF_U.PRFIn_f.NASDAQPosition[1];

    // Abs: '<S20>/Abs'
    rtb_Heading = std::abs(rtb_Reference);

    // Sqrt: '<S20>/Sqrt2' incorporates:
    //   DotProduct: '<S20>/Dot Product'

    rtb_Saturation1 = std::sqrt(rtb_Heading * rtb_Heading + rtb_p_perp);

    // Abs: '<S20>/Abs1' incorporates:
    //   Constant: '<S16>/glide ratio'
    //   Inport: '<Root>/PRF In'
    //   Product: '<S20>/Product1'

    rtb_p_perp = std::abs(PRF_P.glideratio_Value * PRF_U.PRFIn_f.NASDAQPosition
                          [2]);

    // RelationalOperator: '<S17>/Compare' incorporates:
    //   Constant: '<S17>/Constant'
    //   Sum: '<S16>/Sum'

    rtb_Compare_b = (rtb_p_perp - rtb_Saturation1 >
                     PRF_P.Comparetoconstant_const);

    // Outputs for Enabled SubSystem: '<S16>/Enabled Subsystem1' incorporates:
    //   EnablePort: '<S19>/Enable'

    if (!rtb_Compare_b) {
      // Merge: '<S16>/Merge' incorporates:
      //   DataTypeConversion: '<S16>/Cast To Single1'
      //   SignalConversion generated from: '<S19>/Q2'

      PRF_DW.Q1[0] = PRF_DW.Q2[0];
      PRF_DW.Q1[1] = PRF_DW.Q2[1];
    }

    // End of Outputs for SubSystem: '<S16>/Enabled Subsystem1'

    // Product: '<S20>/Product2' incorporates:
    //   Logic: '<S16>/NOT'
    //   Sum: '<S20>/Sum'

    rtb_Sum2_idx_0 /= rtb_Saturation1;
    rtb_Sum2_idx_1 = rtb_Reference / rtb_Saturation1;

    // Gain: '<S20>/Gain'
    rtb_Sum1_a_idx_1 = PRF_P.Gain_Gain * rtb_Sum2_idx_0;

    // DotProduct: '<S16>/Dot Product' incorporates:
    //   Concatenate: '<S20>/Vector Concatenate'
    //   SignalConversion generated from: '<S20>/Vector Concatenate'
    //   Sum: '<S21>/Sum'

    rtb_Heading = (PRF_DW.Q2[0] - RateTransition_idx_0) * rtb_Sum2_idx_1 +
      (PRF_DW.Q2[1] - RateTransition_idx_1) * rtb_Sum1_a_idx_1;

    // Signum: '<S16>/1 or -1' incorporates:
    //   DotProduct: '<S16>/Dot Product'

    if (rtb_Heading < 0.0F) {
      rtb_Heading = -1.0F;
    } else {
      rtb_Heading = (rtb_Heading > 0.0F);
    }

    // End of Signum: '<S16>/1 or -1'

    // Gain: '<S20>/Gain1'
    rtb_Reference = PRF_P.Gain1_Gain_h * rtb_Saturation1;

    // Outputs for Enabled SubSystem: '<S16>/Enabled Subsystem' incorporates:
    //   EnablePort: '<S18>/Enable'

    if (rtb_Compare_b) {
      // Gain: '<S18>/Gain2'
      rtb_p_perp *= PRF_P.Gain2_Gain;

      // Gain: '<S18>/Gain4'
      rtb_Gain4 = PRF_P.Gain4_Gain * rtb_p_perp;

      // Trigonometry: '<S18>/Sin' incorporates:
      //   Constant: '<S16>/theta'

      rtb_k_par = std::sin(PRF_P.theta_Value);

      // Trigonometry: '<S18>/Cos' incorporates:
      //   Constant: '<S16>/theta'

      rtb_k_perp = std::cos(PRF_P.theta_Value);

      // Gain: '<S18>/Gain3'
      rtb_Saturation1 *= PRF_P.Gain3_Gain;

      // Gain: '<S18>/Gain1' incorporates:
      //   Math: '<S18>/Square'
      //   Math: '<S18>/Square1'
      //   Sqrt: '<S18>/Sqrt1'
      //   Sum: '<S18>/Sum2'

      rtb_Saturation1 = std::sqrt(rtb_p_perp * rtb_p_perp - rtb_Saturation1 *
        rtb_Saturation1) * PRF_P.Gain1_Gain;

      // Merge: '<S16>/Merge' incorporates:
      //   Concatenate: '<S20>/Vector Concatenate'
      //   Inport: '<Root>/PRF In'
      //   Product: '<S18>/Product'
      //   Product: '<S18>/Product1'
      //   Product: '<S20>/Product'
      //   SignalConversion generated from: '<S20>/Vector Concatenate'
      //   Sum: '<S18>/Sum'
      //   Sum: '<S20>/Sum1'

      PRF_DW.Q1[0] = (rtb_Sum2_idx_1 * rtb_Saturation1 * rtb_k_perp *
                      rtb_Heading + rtb_Sum2_idx_0 * rtb_Gain4 * rtb_k_par) +
        (rtb_Sum2_idx_0 * rtb_Reference + PRF_U.PRFIn_f.NASDAQPosition[0]);
      PRF_DW.Q1[1] = (rtb_Sum1_a_idx_1 * rtb_Saturation1 * rtb_k_perp *
                      rtb_Heading + rtb_Sum2_idx_1 * rtb_Gain4 * rtb_k_par) +
        (rtb_Sum2_idx_1 * rtb_Reference + PRF_U.PRFIn_f.NASDAQPosition[1]);
    }

    // End of Outputs for SubSystem: '<S16>/Enabled Subsystem'

    // SignalConversion generated from: '<S16>/Vector Concatenate3'
    rtb_VectorConcatenate3[4] = PRF_DW.Q1[0];
    rtb_VectorConcatenate3[5] = PRF_DW.Q1[1];

    // Math: '<S16>/Transpose1' incorporates:
    //   BusCreator generated from: '<S21>/PRF Logs OBSW_BusCreator'
    //   Concatenate: '<S16>/Vector Concatenate3'

    i_0 = 0;
    for (i = 0; i < 2; i++) {
      PRF_DW.TargetPoints[i_0] = rtb_VectorConcatenate3[i];
      PRF_DW.TargetPoints[i_0 + 1] = rtb_VectorConcatenate3[i + 2];
      PRF_DW.TargetPoints[i_0 + 2] = rtb_VectorConcatenate3[i + 4];
      PRF_DW.PRFLogsOBSW_BusCreator_BusCreat.Q1[i] = 0.0F;
      PRF_DW.PRFLogsOBSW_BusCreator_BusCreat.Q2[i] = 0.0F;
      i_0 += 3;
    }

    // BusCreator generated from: '<S21>/PRF Logs OBSW_BusCreator'
    PRF_DW.PRFLogsOBSW_BusCreator_BusCreat.TerminalTarget[0] = 0.0F;
    PRF_DW.PRFLogsOBSW_BusCreator_BusCreat.TerminalTarget[1] = 0.0F;
    PRF_DW.PRFLogsOBSW_BusCreator_BusCreat.TerminalTarget[2] = 0.0F;
    PRF_DW.PRFLogsOBSW_BusCreator_BusCreat.TerminalTarget[3] = 0.0F;
    PRF_DW.PRFLogsOBSW_BusCreator_BusCreat.TargetIndex = 0U;
    PRF_DW.PRFLogsOBSW_BusCreator_BusCreat.Heading = 0.0F;
    PRF_DW.PRFLogsOBSW_BusCreator_BusCreat.Reference = 0.0F;
    PRF_DW.PRFLogsOBSW_BusCreator_BusCreat.ServoCommands[0] = 0.0F;
    PRF_DW.PRFLogsOBSW_BusCreator_BusCreat.ServoCommands[1] = 0.0F;
    PRF_DW.PRFLogsOBSW_BusCreator_BusCreat.WindHeading = rtb_Gain1_h;
  }

  PRF_PrevZCX.TargetPointsGeneration_Trig_ZCE = (PRF_DW.UnitDelay1_DSTATE > 0);

  // End of Outputs for SubSystem: '<S7>/Target Points Generation'

  // Abs: '<S5>/Abs' incorporates:
  //   Inport: '<Root>/PRF In'
  //   Sum: '<S5>/Sum'

  rtb_p_perp = std::abs(RateTransition_idx_0 - PRF_U.PRFIn_f.NASDAQPosition[0]);

  // DotProduct: '<S5>/Dot Product'
  rtb_Gain1_h = rtb_p_perp * rtb_p_perp;

  // Abs: '<S5>/Abs' incorporates:
  //   Inport: '<Root>/PRF In'
  //   Sum: '<S5>/Sum'

  rtb_p_perp = std::abs(RateTransition_idx_1 - PRF_U.PRFIn_f.NASDAQPosition[1]);

  // Gain: '<S5>/PRFControl.zThresholdGain' incorporates:
  //   Constant: '<S5>/glide ratio'
  //   DotProduct: '<S5>/Dot Product'
  //   Product: '<S5>/Product'
  //   Sqrt: '<S5>/Sqrt'

  rtb_Gain1_h = std::sqrt(rtb_p_perp * rtb_p_perp + rtb_Gain1_h) /
    PRF_P.glideratio_Value_o * PRF_P.PRFControlzThresholdGain_Gain;

  // Chart: '<S1>/Point Selection' incorporates:
  //   Constant: '<S5>/-'
  //   Inport: '<Root>/PRF In'
  //   Memory: '<S5>/Memory'
  //   Memory: '<S5>/Memory1'

  if (PRF_DW.is_active_c9_PRF == 0) {
    PRF_DW.is_active_c9_PRF = 1U;
    PRF_DW.is_c9_PRF = PRF_IN_Q1;

    // :  target=2;
    rtb_target = 2U;
  } else {
    switch (PRF_DW.is_c9_PRF) {
     case PRF_IN_Q1:
      // :  sf_internal_predicateOutput = (z>-z_threshold);
      if (PRF_U.PRFIn_f.NASDAQPosition[2] > -rtb_Gain1_h) {
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
          (PRF_U.PRFIn_f.NASDAQPosition[2] > -rtb_Gain1_h)) {
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

  // Sum: '<S5>/Sum1' incorporates:
  //   Inport: '<Root>/PRF In'
  //   Math: '<S6>/Transpose'
  //   Selector: '<S7>/Selector'

  rtb_Gain4 = PRF_DW.Q1[0] - PRF_U.PRFIn_f.NASDAQPosition[0];

  // Sum: '<S5>/Sum2' incorporates:
  //   Inport: '<Root>/PRF In'
  //   Math: '<S6>/Transpose'
  //   Selector: '<S7>/Selector'

  rtb_Sum2_idx_0 = PRF_DW.Q2[0] - PRF_U.PRFIn_f.NASDAQPosition[0];

  // SignalConversion generated from: '<S7>/Vector Concatenate' incorporates:
  //   Outport: '<Root>/PRF Logs OBSW'

  PRF_Y.PRFLogsOBSW.TerminalTarget[0] = RateTransition_idx_0;

  // SignalConversion generated from: '<S7>/Vector Concatenate' incorporates:
  //   Outport: '<Root>/PRF Logs OBSW'

  PRF_Y.PRFLogsOBSW.TerminalTarget[2] = 0.0F;

  // Sum: '<S5>/Sum1' incorporates:
  //   Inport: '<Root>/PRF In'
  //   Math: '<S6>/Transpose'
  //   Selector: '<S7>/Selector'

  rtb_Sum1_a_idx_1 = PRF_DW.Q1[1] - PRF_U.PRFIn_f.NASDAQPosition[1];

  // Sum: '<S5>/Sum2' incorporates:
  //   Inport: '<Root>/PRF In'
  //   Math: '<S6>/Transpose'
  //   Selector: '<S7>/Selector'

  rtb_Sum2_idx_1 = PRF_DW.Q2[1] - PRF_U.PRFIn_f.NASDAQPosition[1];

  // SignalConversion generated from: '<S7>/Vector Concatenate' incorporates:
  //   Outport: '<Root>/PRF Logs OBSW'

  PRF_Y.PRFLogsOBSW.TerminalTarget[1] = RateTransition_idx_1;

  // SignalConversion generated from: '<S7>/Vector Concatenate' incorporates:
  //   Outport: '<Root>/PRF Logs OBSW'

  PRF_Y.PRFLogsOBSW.TerminalTarget[3] = 0.0F;

  // Trigonometry: '<S6>/Atan1' incorporates:
  //   Inport: '<Root>/PRF In'
  //   Math: '<S16>/Transpose1'
  //   Math: '<S6>/Transpose'
  //   Selector: '<S7>/Selector'
  //   Sum: '<S6>/Subtract1'

  rtb_Reference = std::atan2(PRF_DW.TargetPoints[rtb_target + 3] -
    PRF_U.PRFIn_f.NASDAQPosition[1], PRF_DW.TargetPoints[rtb_target] -
    PRF_U.PRFIn_f.NASDAQPosition[0]);

  // Trigonometry: '<S6>/Atan2' incorporates:
  //   Inport: '<Root>/PRF In'

  rtb_Heading = std::atan2(PRF_U.PRFIn_f.NASDAQVelocity[1],
    PRF_U.PRFIn_f.NASDAQVelocity[0]);

  // Sum: '<S6>/Subtract'
  rtb_Gain1_h = rtb_Reference - rtb_Heading;

  // Switch: '<S6>/Switch' incorporates:
  //   Abs: '<S6>/Abs'
  //   Bias: '<S6>/Bias'
  //   Switch: '<S13>/Switch'

  if (std::abs(rtb_Gain1_h) > PRF_P.Switch_Threshold) {
    // Bias: '<S6>/Bias1'
    rtb_Gain1_h += PRF_P.Bias1_Bias;

    // Math: '<S13>/Mod' incorporates:
    //   Constant: '<S13>/Constant'

    rtb_Saturation1 = rt_modf(rtb_Gain1_h, PRF_P.Constant_Value);

    // Switch: '<S13>/Switch' incorporates:
    //   Constant: '<S13>/Constant1'
    //   Constant: '<S14>/Constant'
    //   Constant: '<S15>/Constant'
    //   Logic: '<S13>/AND'
    //   RelationalOperator: '<S14>/Compare'
    //   RelationalOperator: '<S15>/Compare'

    if ((rtb_Gain1_h > PRF_P.Constant_Value_ir) && (rtb_Saturation1 ==
         PRF_P.Constant_Value_a)) {
      rtb_Saturation1 = PRF_P.Constant1_Value;
    }

    rtb_Gain1_h = rtb_Saturation1 + PRF_P.Bias_Bias;
  }

  // End of Switch: '<S6>/Switch'

  // Switch: '<S6>/Switch1' incorporates:
  //   Constant: '<S6>/Constant4'
  //   Constant: '<S6>/Constant5'
  //   Memory: '<S6>/Memory'
  //   Memory: '<S6>/Memory2'
  //   Product: '<S6>/Product3'

  if (PRF_DW.Memory2_PreviousInput) {
    rtb_p_perp = PRF_DW.Memory_PreviousInput_b;
  } else {
    rtb_p_perp = PRF_P.Constant4_Value * PRF_P.Constant5_Value * rtb_Gain1_h;
  }

  // End of Switch: '<S6>/Switch1'

  // Sum: '<S6>/Add1' incorporates:
  //   Constant: '<S6>/Constant1'
  //   Constant: '<S6>/Constant2'
  //   Constant: '<S6>/Constant3'
  //   Product: '<S6>/Product1'
  //   Product: '<S6>/Product2'
  //   UnitDelay: '<S6>/Unit Delay'

  rtb_Saturation1 = (rtb_Gain1_h * PRF_P.Constant3_Value_l + rtb_p_perp) +
    PRF_DW.UnitDelay_DSTATE * PRF_P.Constant2_Value_c / PRF_P.Constant1_Value_p;

  // Saturate: '<S6>/Saturation1'
  if (rtb_Saturation1 > PRF_P.Saturation1_UpperSat) {
    rtb_Saturation1 = PRF_P.Saturation1_UpperSat;
  } else if (rtb_Saturation1 < PRF_P.Saturation1_LowerSat) {
    rtb_Saturation1 = PRF_P.Saturation1_LowerSat;
  }

  // End of Saturate: '<S6>/Saturation1'

  // Signum: '<S6>/Sign'
  if (rtb_Saturation1 < 0.0F) {
    i_0 = -1;
  } else {
    i_0 = (rtb_Saturation1 > 0.0F);
  }

  // SwitchCase: '<S6>/Switch Case' incorporates:
  //   Constant: '<S11>/Zero'
  //   Constant: '<S12>/Zero'
  //   Merge: '<S6>/Merge'
  //   Signum: '<S6>/Sign'

  switch (i_0) {
   case 1:
    // Outputs for IfAction SubSystem: '<S6>/Servo Right' incorporates:
    //   ActionPort: '<S12>/Action Port'

    // SignalConversion generated from: '<S12>/Command' incorporates:
    //   Abs: '<S6>/Abs1'
    //   Merge: '<S6>/Merge'

    PRF_DW.Merge[1] = std::abs(rtb_Saturation1);
    PRF_DW.Merge[0] = PRF_P.Zero_Value;

    // End of Outputs for SubSystem: '<S6>/Servo Right'
    break;

   case 0:
    // Outputs for IfAction SubSystem: '<S6>/No Activation' incorporates:
    //   ActionPort: '<S8>/Action Port'

    // Merge: '<S6>/Merge' incorporates:
    //   Constant: '<S8>/Zero'
    //   SignalConversion generated from: '<S8>/Servo'

    PRF_DW.Merge[0] = PRF_P.Zero_Value_b[0];
    PRF_DW.Merge[1] = PRF_P.Zero_Value_b[1];

    // End of Outputs for SubSystem: '<S6>/No Activation'
    break;

   default:
    // Outputs for IfAction SubSystem: '<S6>/Servo Left' incorporates:
    //   ActionPort: '<S11>/Action Port'

    // SignalConversion generated from: '<S11>/Command' incorporates:
    //   Abs: '<S6>/Abs1'
    //   Merge: '<S6>/Merge'

    PRF_DW.Merge[0] = std::abs(rtb_Saturation1);
    PRF_DW.Merge[1] = PRF_P.Zero_Value_f;

    // End of Outputs for SubSystem: '<S6>/Servo Left'
    break;
  }

  // End of SwitchCase: '<S6>/Switch Case'

  // Bias: '<S7>/Bias1' incorporates:
  //   UnitDelay: '<S7>/Unit Delay1'

  u0 = static_cast<uint8_t>(PRF_DW.UnitDelay1_DSTATE + PRF_P.Bias1_Bias_n);

  // Saturate: '<S7>/Saturation'
  if (u0 > PRF_P.Saturation_UpperSat) {
    // Update for UnitDelay: '<S7>/Unit Delay1'
    PRF_DW.UnitDelay1_DSTATE = PRF_P.Saturation_UpperSat;
  } else if (u0 < PRF_P.Saturation_LowerSat) {
    // Update for UnitDelay: '<S7>/Unit Delay1'
    PRF_DW.UnitDelay1_DSTATE = PRF_P.Saturation_LowerSat;
  } else {
    // Update for UnitDelay: '<S7>/Unit Delay1'
    PRF_DW.UnitDelay1_DSTATE = u0;
  }

  // End of Saturate: '<S7>/Saturation'

  // Update for Memory: '<S6>/Memory'
  PRF_DW.Memory_PreviousInput_b = rtb_p_perp;

  // Update for Memory: '<S6>/Memory2' incorporates:
  //   Constant: '<S10>/Constant'
  //   Constant: '<S9>/Constant'
  //   Logic: '<S6>/OR2'
  //   RelationalOperator: '<S10>/Compare'
  //   RelationalOperator: '<S9>/Compare'

  PRF_DW.Memory2_PreviousInput = ((rtb_Saturation1 ==
    PRF_P.SaturationCheckUp_const) || (rtb_Saturation1 ==
    PRF_P.SaturationCheckLw_const));

  // Update for UnitDelay: '<S6>/Unit Delay'
  PRF_DW.UnitDelay_DSTATE = rtb_Gain1_h;

  // Outport: '<Root>/PRF Logs OBSW' incorporates:
  //   Bias: '<S7>/Bias'
  //   BusAssignment: '<S6>/Bus Assignment'

  PRF_Y.PRFLogsOBSW.TargetIndex = static_cast<uint8_t>(rtb_target +
    PRF_P.Bias_Bias_i);
  PRF_Y.PRFLogsOBSW.Heading = rtb_Heading;
  PRF_Y.PRFLogsOBSW.Reference = rtb_Reference;

  // End of Outputs for SubSystem: '<Root>/PRF'

  // Outport: '<Root>/Servo Commands'
  PRF_Y.ServoCommands[0] = PRF_DW.Merge[0];

  // Outputs for Atomic SubSystem: '<Root>/PRF'
  // Outport: '<Root>/PRF Logs OBSW' incorporates:
  //   BusAssignment: '<S7>/Bus Assignment1'
  //   Outport: '<Root>/Servo Commands'

  PRF_Y.PRFLogsOBSW.Q1[0] = PRF_DW.Q1[0];
  PRF_Y.PRFLogsOBSW.Q2[0] = PRF_DW.Q2[0];

  // End of Outputs for SubSystem: '<Root>/PRF'
  PRF_Y.PRFLogsOBSW.ServoCommands[0] = PRF_DW.Merge[0];

  // Outport: '<Root>/Servo Commands'
  PRF_Y.ServoCommands[1] = PRF_DW.Merge[1];

  // Outputs for Atomic SubSystem: '<Root>/PRF'
  // Outport: '<Root>/PRF Logs OBSW' incorporates:
  //   BusAssignment: '<S7>/Bus Assignment1'
  //   Outport: '<Root>/Servo Commands'

  PRF_Y.PRFLogsOBSW.Q1[1] = PRF_DW.Q1[1];
  PRF_Y.PRFLogsOBSW.Q2[1] = PRF_DW.Q2[1];

  // End of Outputs for SubSystem: '<Root>/PRF'
  PRF_Y.PRFLogsOBSW.ServoCommands[1] = PRF_DW.Merge[1];

  // Outputs for Atomic SubSystem: '<Root>/PRF'
  // Update for Memory: '<S5>/Memory1' incorporates:
  //   DotProduct: '<S5>/Dot Product1'
  //   Sqrt: '<S5>/Sqrt1'

  PRF_DW.Memory1_PreviousInput = std::sqrt(rtb_Gain4 * rtb_Gain4 +
    rtb_Sum1_a_idx_1 * rtb_Sum1_a_idx_1);

  // Update for Memory: '<S5>/Memory' incorporates:
  //   DotProduct: '<S5>/Dot Product2'
  //   Sqrt: '<S5>/Sqrt2'

  PRF_DW.Memory_PreviousInput = std::sqrt(rtb_Sum2_idx_0 * rtb_Sum2_idx_0 +
    rtb_Sum2_idx_1 * rtb_Sum2_idx_1);

  // Outport: '<Root>/PRF Logs OBSW' incorporates:
  //   SignalConversion generated from: '<S7>/Bus Assignment1'

  PRF_Y.PRFLogsOBSW.WindHeading =
    PRF_DW.PRFLogsOBSW_BusCreator_BusCreat.WindHeading;

  // End of Outputs for SubSystem: '<Root>/PRF'
}

// Model initialize function
void PRF::initialize()
{
  {
    int32_t i;
    PRF_PrevZCX.TargetPointsGeneration_Trig_ZCE = POS_ZCSIG;

    // SystemInitialize for Atomic SubSystem: '<Root>/PRF'
    // InitializeConditions for UnitDelay: '<S7>/Unit Delay1'
    PRF_DW.UnitDelay1_DSTATE = PRF_P.UnitDelay1_InitialCondition;

    // InitializeConditions for Memory: '<S5>/Memory1'
    PRF_DW.Memory1_PreviousInput = PRF_P.Memory1_InitialCondition;

    // InitializeConditions for Memory: '<S5>/Memory'
    PRF_DW.Memory_PreviousInput = PRF_P.Memory_InitialCondition;

    // InitializeConditions for Memory: '<S6>/Memory'
    PRF_DW.Memory_PreviousInput_b = PRF_P.Memory_InitialCondition_b;

    // InitializeConditions for Memory: '<S6>/Memory2'
    PRF_DW.Memory2_PreviousInput = PRF_P.Memory2_InitialCondition;

    // InitializeConditions for UnitDelay: '<S6>/Unit Delay'
    PRF_DW.UnitDelay_DSTATE = PRF_P.UnitDelay_InitialCondition;

    // SystemInitialize for Triggered SubSystem: '<S7>/Target Points Generation' 
    // SystemInitialize for Merge: '<S16>/Merge'
    PRF_DW.Q1[0] = PRF_P.Merge_InitialOutput;
    PRF_DW.Q1[1] = PRF_P.Merge_InitialOutput;
    for (i = 0; i < 6; i++) {
      // SystemInitialize for Math: '<S16>/Transpose1' incorporates:
      //   Outport: '<S16>/Target Points'

      PRF_DW.TargetPoints[i] = PRF_P.TargetPoints_Y0;
    }

    // SystemInitialize for BusCreator generated from: '<S21>/PRF Logs OBSW_BusCreator' incorporates:
    //   Outport: '<S16>/PRF Logs OBSW'

    PRF_DW.PRFLogsOBSW_BusCreator_BusCreat = PRF_P.PRFLogsOBSW_Y0;

    // SystemInitialize for Sum: '<S21>/Sum2' incorporates:
    //   Outport: '<S16>/Q2'

    PRF_DW.Q2[0] = PRF_P.Q2_Y0;

    // End of SystemInitialize for SubSystem: '<S7>/Target Points Generation'
    // End of SystemInitialize for SubSystem: '<Root>/PRF'

    // SystemInitialize for Merge: '<S6>/Merge'
    PRF_DW.Merge[0] = PRF_P.Merge_InitialOutput_g;

    // SystemInitialize for Outport: '<Root>/Servo Commands' incorporates:
    //   Merge: '<S6>/Merge'

    PRF_Y.ServoCommands[0] = PRF_P.Merge_InitialOutput_g;

    // SystemInitialize for Atomic SubSystem: '<Root>/PRF'
    // SystemInitialize for Triggered SubSystem: '<S7>/Target Points Generation' 
    // SystemInitialize for Sum: '<S21>/Sum2' incorporates:
    //   Outport: '<S16>/Q2'

    PRF_DW.Q2[1] = PRF_P.Q2_Y0;

    // End of SystemInitialize for SubSystem: '<S7>/Target Points Generation'
    // End of SystemInitialize for SubSystem: '<Root>/PRF'

    // SystemInitialize for Merge: '<S6>/Merge'
    PRF_DW.Merge[1] = PRF_P.Merge_InitialOutput_g;

    // SystemInitialize for Outport: '<Root>/Servo Commands' incorporates:
    //   Merge: '<S6>/Merge'

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
