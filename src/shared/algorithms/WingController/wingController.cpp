//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// File: wingController.cpp
//
// Code generated for Simulink model 'wingController'.
//
// Model version                  : 11.284
// Simulink Coder version         : 24.2 (R2024b) 21-Jun-2024
// C/C++ source code generated on : Wed Jun  3 10:43:26 2026
//
// Target selection: ert.tlc
// Embedded hardware selection: ARM Compatible->ARM Cortex-M
// Code generation objectives:
//    1. Execution efficiency
//    2. Debugging
//    3. RAM efficiency
// Validation result: Not run
//
#include "wingController.h"
#include <stdint.h>
#include <cmath>
#include "wingController_private.h"
#include <stdbool.h>
#include "zero_crossing_types.h"
#include <cfloat>

// Named constants for Chart: '<S1>/Point Selection'
const uint8_t wingController_IN_Q1{ 1U };

const uint8_t wingController_IN_Q2{ 2U };

const uint8_t wingController_IN_Target{ 3U };

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
void wingController::step()
{
  float rtb_VectorConcatenate3[6];
  float rtb_DotProduct1;
  float rtb_Gain1;
  float rtb_Gain4;
  float rtb_Sum1_idx_0;
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
  bool rtb_Compare_f;

  // Outputs for Atomic SubSystem: '<Root>/wingController DPG'
  // Outputs for Triggered SubSystem: '<S6>/Target Points Generation' incorporates:
  //   TriggerPort: '<S17>/function'

  // UnitDelay: '<S6>/Unit Delay1' incorporates:
  //   Logic: '<S17>/NOT'

  if ((wingController_DW.UnitDelay1_DSTATE > 0) &&
      (wingController_PrevZCX.TargetPointsGeneration_Trig_ZCE != POS_ZCSIG)) {
    // Switch: '<S24>/FixPt Switch' incorporates:
    //   Constant: '<S17>/Constant1'
    //   Constant: '<S24>/Constant'

    if (wingController_P.Constant1_Value_e >
        wingController_P.WrapToZero_Threshold) {
      rtb_Sum2_idx_0 = wingController_P.Constant_Value_a;
    } else {
      rtb_Sum2_idx_0 = wingController_P.Constant1_Value_e;
    }

    // Gain: '<S23>/Gain1' incorporates:
    //   Switch: '<S24>/FixPt Switch'

    rtb_DotProduct1 = wingController_P.Gain1_Gain_d * rtb_Sum2_idx_0;

    // Sum: '<S22>/Sum2' incorporates:
    //   Constant: '<S17>/Constant3'
    //   Constant: '<S17>/Constant5'
    //   Product: '<S22>/Product'
    //   Product: '<S22>/Product1'
    //   Trigonometry: '<S22>/Cos'
    //   Trigonometry: '<S22>/Sin'

    wingController_DW.Q2[0] = std::cos(rtb_DotProduct1) *
      wingController_P.Constant5_Value_h + wingController_P.Constant3_Value[0];
    wingController_DW.Q2[1] = std::sin(rtb_DotProduct1) *
      wingController_P.Constant5_Value_h + wingController_P.Constant3_Value[1];

    // SignalConversion generated from: '<S17>/Vector Concatenate3' incorporates:
    //   Constant: '<S17>/Constant3'

    rtb_VectorConcatenate3[0] = wingController_P.Constant3_Value[0];

    // SignalConversion generated from: '<S17>/Vector Concatenate3' incorporates:
    //   DataTypeConversion: '<S17>/Cast To Single1'
    //   Sum: '<S22>/Sum2'

    rtb_VectorConcatenate3[2] = wingController_DW.Q2[0];

    // Sum: '<S21>/Sum' incorporates:
    //   DataTypeConversion: '<S17>/Cast To Single1'
    //   Inport: '<Root>/PRF In'
    //   Sum: '<S22>/Sum2'
    //   Sum: '<S4>/Sum2'

    rtb_Sum2_idx_1 = wingController_DW.Q2[0] -
      wingController_U.PRFIn_d.NASDAQPosition[0];
    rtb_Sum2_idx_0 = rtb_Sum2_idx_1;

    // Abs: '<S21>/Abs' incorporates:
    //   Sum: '<S4>/Sum1'
    //   Sum: '<S4>/Sum2'

    rtb_inversion = std::abs(rtb_Sum2_idx_1);

    // DotProduct: '<S21>/Dot Product' incorporates:
    //   Sum: '<S4>/Sum1'

    rtb_DotProduct1 = rtb_inversion * rtb_inversion;

    // SignalConversion generated from: '<S17>/Vector Concatenate3' incorporates:
    //   Constant: '<S17>/Constant3'

    rtb_VectorConcatenate3[1] = wingController_P.Constant3_Value[1];

    // SignalConversion generated from: '<S17>/Vector Concatenate3' incorporates:
    //   DataTypeConversion: '<S17>/Cast To Single1'
    //   Sum: '<S22>/Sum2'

    rtb_VectorConcatenate3[3] = wingController_DW.Q2[1];

    // Sum: '<S21>/Sum' incorporates:
    //   DataTypeConversion: '<S17>/Cast To Single1'
    //   Inport: '<Root>/PRF In'
    //   Sum: '<S22>/Sum2'
    //   Sum: '<S4>/Sum2'

    rtb_Sum2_idx_1 = wingController_DW.Q2[1] -
      wingController_U.PRFIn_d.NASDAQPosition[1];

    // Abs: '<S21>/Abs' incorporates:
    //   Sum: '<S4>/Sum1'
    //   Sum: '<S4>/Sum2'

    rtb_inversion = std::abs(rtb_Sum2_idx_1);

    // Sqrt: '<S21>/Sqrt2' incorporates:
    //   DotProduct: '<S21>/Dot Product'
    //   Sum: '<S4>/Sum1'

    rtb_direct_distance = std::sqrt(rtb_inversion * rtb_inversion +
      rtb_DotProduct1);

    // Abs: '<S21>/Abs1' incorporates:
    //   Constant: '<S17>/glide ratio'
    //   Inport: '<Root>/PRF In'
    //   Product: '<S21>/Product1'

    rtb_DotProduct1 = std::abs(wingController_P.glideratio_Value *
      wingController_U.PRFIn_d.NASDAQPosition[2]);

    // RelationalOperator: '<S18>/Compare' incorporates:
    //   Constant: '<S18>/Constant'
    //   Sum: '<S17>/Sum'

    rtb_Compare_f = (rtb_DotProduct1 - rtb_direct_distance >
                     wingController_P.Comparetoconstant_const);

    // Outputs for Enabled SubSystem: '<S17>/Enabled Subsystem1' incorporates:
    //   EnablePort: '<S20>/Enable'

    if (!rtb_Compare_f) {
      // Merge: '<S17>/Merge' incorporates:
      //   DataTypeConversion: '<S17>/Cast To Single1'
      //   SignalConversion generated from: '<S20>/Q2'
      //   Sum: '<S22>/Sum2'

      wingController_DW.Q1[0] = wingController_DW.Q2[0];
      wingController_DW.Q1[1] = wingController_DW.Q2[1];
    }

    // End of Outputs for SubSystem: '<S17>/Enabled Subsystem1'

    // Product: '<S21>/Product2' incorporates:
    //   Logic: '<S17>/NOT'
    //   Sum: '<S21>/Sum'
    //   Sum: '<S4>/Sum2'

    rtb_Sum2_idx_0 /= rtb_direct_distance;
    rtb_Sum2_idx_1 /= rtb_direct_distance;

    // Gain: '<S21>/Gain'
    rtb_Sum1_idx_1 = wingController_P.Gain_Gain_e * rtb_Sum2_idx_0;

    // DotProduct: '<S17>/Dot Product' incorporates:
    //   Concatenate: '<S21>/Vector Concatenate'
    //   Constant: '<S17>/Constant3'
    //   SignalConversion generated from: '<S21>/Vector Concatenate'
    //   Sum: '<S22>/Sum'
    //   Sum: '<S22>/Sum2'

    rtb_inversion = (wingController_DW.Q2[0] - wingController_P.Constant3_Value
                     [0]) * rtb_Sum2_idx_1 + (wingController_DW.Q2[1] -
      wingController_P.Constant3_Value[1]) * rtb_Sum1_idx_1;

    // Signum: '<S17>/1 or -1' incorporates:
    //   DotProduct: '<S17>/Dot Product'

    if (rtb_inversion < 0.0F) {
      rtb_inversion = -1.0F;
    } else {
      rtb_inversion = (rtb_inversion > 0.0F);
    }

    // End of Signum: '<S17>/1 or -1'

    // Gain: '<S21>/Gain1'
    rtb_Gain1 = wingController_P.Gain1_Gain_dp * rtb_direct_distance;

    // Outputs for Enabled SubSystem: '<S17>/Enabled Subsystem' incorporates:
    //   EnablePort: '<S19>/Enable'

    if (rtb_Compare_f) {
      // Gain: '<S19>/Gain2'
      rtb_DotProduct1 *= wingController_P.Gain2_Gain;

      // Gain: '<S19>/Gain4'
      rtb_Gain4 = wingController_P.Gain4_Gain * rtb_DotProduct1;

      // Trigonometry: '<S19>/Sin' incorporates:
      //   Constant: '<S17>/theta'

      rtb_k_par = std::sin(wingController_P.theta_Value);

      // Trigonometry: '<S19>/Cos' incorporates:
      //   Constant: '<S17>/theta'

      rtb_k_perp = std::cos(wingController_P.theta_Value);

      // Gain: '<S19>/Gain3'
      rtb_direct_distance *= wingController_P.Gain3_Gain_m;

      // Gain: '<S19>/Gain1' incorporates:
      //   Math: '<S19>/Square'
      //   Math: '<S19>/Square1'
      //   Sqrt: '<S19>/Sqrt1'
      //   Sum: '<S19>/Sum2'

      rtb_direct_distance = std::sqrt(rtb_DotProduct1 * rtb_DotProduct1 -
        rtb_direct_distance * rtb_direct_distance) *
        wingController_P.Gain1_Gain_e;

      // Merge: '<S17>/Merge' incorporates:
      //   Concatenate: '<S21>/Vector Concatenate'
      //   Inport: '<Root>/PRF In'
      //   Product: '<S19>/Product'
      //   Product: '<S19>/Product1'
      //   Product: '<S21>/Product'
      //   Product: '<S21>/Product2'
      //   SignalConversion generated from: '<S21>/Vector Concatenate'
      //   Sum: '<S19>/Sum'
      //   Sum: '<S21>/Sum1'

      wingController_DW.Q1[0] = (rtb_Sum2_idx_1 * rtb_direct_distance *
        rtb_k_perp * rtb_inversion + rtb_Sum2_idx_0 * rtb_Gain4 * rtb_k_par) +
        (rtb_Sum2_idx_0 * rtb_Gain1 + wingController_U.PRFIn_d.NASDAQPosition[0]);
      wingController_DW.Q1[1] = (rtb_Sum1_idx_1 * rtb_direct_distance *
        rtb_k_perp * rtb_inversion + rtb_Sum2_idx_1 * rtb_Gain4 * rtb_k_par) +
        (rtb_Sum2_idx_1 * rtb_Gain1 + wingController_U.PRFIn_d.NASDAQPosition[1]);
    }

    // End of Outputs for SubSystem: '<S17>/Enabled Subsystem'

    // SignalConversion generated from: '<S17>/Vector Concatenate3' incorporates:
    //   Merge: '<S17>/Merge'

    rtb_VectorConcatenate3[4] = wingController_DW.Q1[0];
    rtb_VectorConcatenate3[5] = wingController_DW.Q1[1];

    // Math: '<S17>/Transpose1' incorporates:
    //   Concatenate: '<S17>/Vector Concatenate3'

    i = 0;
    for (i_0 = 0; i_0 < 2; i_0++) {
      wingController_DW.TargetPoints[i] = rtb_VectorConcatenate3[i_0];
      wingController_DW.TargetPoints[i + 1] = rtb_VectorConcatenate3[i_0 + 2];
      wingController_DW.TargetPoints[i + 2] = rtb_VectorConcatenate3[i_0 + 4];
      i += 3;
    }
  }

  wingController_PrevZCX.TargetPointsGeneration_Trig_ZCE =
    (wingController_DW.UnitDelay1_DSTATE > 0);

  // End of Outputs for SubSystem: '<S6>/Target Points Generation'

  // Abs: '<S4>/Abs' incorporates:
  //   Constant: '<S4>/Constant1'
  //   Inport: '<Root>/PRF In'
  //   Sum: '<S4>/Sum'

  rtb_Sum2_idx_0 = std::abs(wingController_P.Constant1_Value_d[0] -
    wingController_U.PRFIn_d.NASDAQPosition[0]);

  // DotProduct: '<S4>/Dot Product'
  rtb_DotProduct1 = rtb_Sum2_idx_0 * rtb_Sum2_idx_0;

  // Abs: '<S4>/Abs' incorporates:
  //   Constant: '<S4>/Constant1'
  //   Inport: '<Root>/PRF In'
  //   Sum: '<S4>/Sum'

  rtb_Sum2_idx_0 = std::abs(wingController_P.Constant1_Value_d[1] -
    wingController_U.PRFIn_d.NASDAQPosition[1]);

  // Gain: '<S4>/PRFControl.zThresholdGain' incorporates:
  //   Constant: '<S4>/glide ratio'
  //   DotProduct: '<S4>/Dot Product'
  //   Product: '<S4>/Product'
  //   Sqrt: '<S4>/Sqrt'

  rtb_direct_distance = std::sqrt(rtb_Sum2_idx_0 * rtb_Sum2_idx_0 +
    rtb_DotProduct1) / wingController_P.glideratio_Value_h *
    wingController_P.PRFControlzThresholdGain_Gain;

  // Chart: '<S1>/Point Selection' incorporates:
  //   Constant: '<S4>/-'
  //   Inport: '<Root>/PRF In'
  //   Memory: '<S4>/Memory'
  //   Memory: '<S4>/Memory1'

  if (wingController_DW.is_active_c9_wingController == 0) {
    wingController_DW.is_active_c9_wingController = 1U;
    wingController_DW.is_c9_wingController = wingController_IN_Q1;

    // :  target=2;
    rtb_target = 2U;
  } else {
    switch (wingController_DW.is_c9_wingController) {
     case wingController_IN_Q1:
      // :  sf_internal_predicateOutput = (z>-z_threshold);
      if (wingController_U.PRFIn_d.NASDAQPosition[2] > -rtb_direct_distance) {
        wingController_DW.is_c9_wingController = wingController_IN_Target;

        // :  target=0;
        rtb_target = 0U;

        // :  sf_internal_predicateOutput = (deltaQ1<Q_threshold);
      } else if (wingController_DW.Memory1_PreviousInput <
                 wingController_P._Value) {
        wingController_DW.is_c9_wingController = wingController_IN_Q2;

        // :  target=1;
        rtb_target = 1U;
      } else {
        // :  target=2;
        rtb_target = 2U;
      }
      break;

     case wingController_IN_Q2:
      // :  sf_internal_predicateOutput = ((deltaQ2<Q_threshold)||(z>-z_threshold)); 
      if ((wingController_DW.Memory_PreviousInput < wingController_P._Value) ||
          (wingController_U.PRFIn_d.NASDAQPosition[2] > -rtb_direct_distance)) {
        wingController_DW.is_c9_wingController = wingController_IN_Target;

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
  //   Merge: '<S17>/Merge'
  //   Selector: '<S6>/Selector'
  //   Sum: '<S5>/Subtract1'

  rtb_Sum1_idx_0 = wingController_DW.Q1[0] -
    wingController_U.PRFIn_d.NASDAQPosition[0];

  // Sum: '<S4>/Sum2' incorporates:
  //   Inport: '<Root>/PRF In'
  //   Selector: '<S6>/Selector'
  //   Sum: '<S22>/Sum2'
  //   Sum: '<S5>/Subtract1'

  rtb_Sum2_idx_0 = wingController_DW.Q2[0] -
    wingController_U.PRFIn_d.NASDAQPosition[0];

  // Sum: '<S4>/Sum1' incorporates:
  //   Inport: '<Root>/PRF In'
  //   Merge: '<S17>/Merge'
  //   Selector: '<S6>/Selector'
  //   Sum: '<S5>/Subtract1'

  rtb_Sum1_idx_1 = wingController_DW.Q1[1] -
    wingController_U.PRFIn_d.NASDAQPosition[1];

  // Sum: '<S4>/Sum2' incorporates:
  //   Inport: '<Root>/PRF In'
  //   Selector: '<S6>/Selector'
  //   Sum: '<S22>/Sum2'
  //   Sum: '<S5>/Subtract1'

  rtb_Sum2_idx_1 = wingController_DW.Q2[1] -
    wingController_U.PRFIn_d.NASDAQPosition[1];

  // Trigonometry: '<S5>/Atan2' incorporates:
  //   Inport: '<Root>/PRF In'

  rtb_inversion = std::atan2(wingController_U.PRFIn_d.NASDAQVelocity[1],
    wingController_U.PRFIn_d.NASDAQVelocity[0]);

  // Trigonometry: '<S5>/Atan1' incorporates:
  //   Inport: '<Root>/PRF In'
  //   Math: '<S17>/Transpose1'
  //   Selector: '<S6>/Selector'
  //   Sum: '<S5>/Subtract1'

  rtb_direct_distance = std::atan2(wingController_DW.TargetPoints[rtb_target + 3]
    - wingController_U.PRFIn_d.NASDAQPosition[1],
    wingController_DW.TargetPoints[rtb_target] -
    wingController_U.PRFIn_d.NASDAQPosition[0]);

  // Sum: '<S5>/Subtract'
  rtb_DotProduct1 = rtb_direct_distance - rtb_inversion;

  // Switch: '<S5>/Switch' incorporates:
  //   Abs: '<S5>/Abs'
  //   Bias: '<S5>/Bias'
  //   Switch: '<S14>/Switch'

  if (std::abs(rtb_DotProduct1) > wingController_P.Switch_Threshold) {
    // Bias: '<S5>/Bias1'
    rtb_DotProduct1 += wingController_P.Bias1_Bias;

    // Math: '<S14>/Mod' incorporates:
    //   Constant: '<S14>/Constant'

    rtb_Gain1 = rt_modf(rtb_DotProduct1, wingController_P.Constant_Value);

    // Switch: '<S14>/Switch' incorporates:
    //   Constant: '<S14>/Constant1'
    //   Constant: '<S15>/Constant'
    //   Constant: '<S16>/Constant'
    //   Logic: '<S14>/AND'
    //   RelationalOperator: '<S15>/Compare'
    //   RelationalOperator: '<S16>/Compare'

    if ((rtb_DotProduct1 > wingController_P.Constant_Value_o) && (rtb_Gain1 ==
         wingController_P.Constant_Value_oa)) {
      rtb_Gain1 = wingController_P.Constant1_Value;
    }

    rtb_DotProduct1 = rtb_Gain1 + wingController_P.Bias_Bias;
  }

  // End of Switch: '<S5>/Switch'

  // Switch: '<S5>/Switch1' incorporates:
  //   Constant: '<S5>/Constant4'
  //   Constant: '<S5>/Constant5'
  //   Memory: '<S5>/Memory'
  //   Memory: '<S5>/Memory2'
  //   Product: '<S5>/Product3'

  if (wingController_DW.Memory2_PreviousInput) {
    rtb_Gain1 = wingController_DW.Memory_PreviousInput_e;
  } else {
    rtb_Gain1 = wingController_P.Constant4_Value *
      wingController_P.Constant5_Value * rtb_DotProduct1;
  }

  // End of Switch: '<S5>/Switch1'

  // Sum: '<S5>/Add1' incorporates:
  //   Constant: '<S5>/Constant3'
  //   Product: '<S5>/Product2'

  rtb_Gain4 = rtb_DotProduct1 * wingController_P.Constant3_Value_d + rtb_Gain1;

  // Saturate: '<S5>/Saturation1'
  if (rtb_Gain4 > wingController_P.Saturation1_UpperSat) {
    rtb_Gain4 = wingController_P.Saturation1_UpperSat;
  } else if (rtb_Gain4 < wingController_P.Saturation1_LowerSat) {
    rtb_Gain4 = wingController_P.Saturation1_LowerSat;
  }

  // End of Saturate: '<S5>/Saturation1'

  // Switch: '<S5>/Switch2' incorporates:
  //   Constant: '<S5>/Constant2'
  //   Constant: '<S5>/Constant8'
  //   Memory: '<S5>/Memory1'
  //   Memory: '<S5>/Memory3'
  //   Product: '<S5>/Product5'

  if (wingController_DW.Memory3_PreviousInput) {
    rtb_k_par = wingController_DW.Memory1_PreviousInput_o;
  } else {
    rtb_k_par = wingController_P.Constant2_Value *
      wingController_P.Constant8_Value * rtb_DotProduct1;
  }

  // End of Switch: '<S5>/Switch2'

  // Sum: '<S5>/Add2' incorporates:
  //   Constant: '<S5>/Constant1'
  //   Product: '<S5>/Product1'

  rtb_DotProduct1 = rtb_DotProduct1 * wingController_P.Constant1_Value_m +
    rtb_k_par;

  // Saturate: '<S5>/Saturation'
  if (rtb_DotProduct1 > wingController_P.Saturation_UpperSat) {
    rtb_DotProduct1 = wingController_P.Saturation_UpperSat;
  } else if (rtb_DotProduct1 < wingController_P.Saturation_LowerSat) {
    rtb_DotProduct1 = wingController_P.Saturation_LowerSat;
  }

  // End of Saturate: '<S5>/Saturation'

  // Switch: '<S5>/Switch3' incorporates:
  //   Constant: '<S5>/Constant9'
  //   Gain: '<S5>/Gain1'
  //   Gain: '<S5>/Gain3'
  //   Trigonometry: '<S5>/Sin'

  if (wingController_P.Constant9_Value) {
    // Gain: '<S5>/Gain' incorporates:
    //   Constant: '<S5>/Constant6'
    //   Constant: '<S5>/Constant7'
    //   Product: '<S5>/Product4'

    rtb_k_perp = 1.0F / wingController_P.Constant6_Value *
      wingController_P.Constant7_Value * rtb_Gain4 * wingController_P.Gain_Gain;

    // Trigonometry: '<S5>/Sin'
    if (rtb_k_perp > 1.0F) {
      rtb_k_perp = 1.0F;
    } else if (rtb_k_perp < -1.0F) {
      rtb_k_perp = -1.0F;
    }

    rtb_k_perp = wingController_P.Gain1_Gain * std::asin(rtb_k_perp) *
      wingController_P.Gain3_Gain;
  } else {
    rtb_k_perp = rtb_DotProduct1;
  }

  // End of Switch: '<S5>/Switch3'

  // Gain: '<S5>/Gain2'
  rtb_k_perp *= wingController_P.Gain2_Gain_g;

  // Signum: '<S5>/Sign'
  if (rtb_k_perp < 0.0F) {
    i = -1;
  } else {
    i = (rtb_k_perp > 0.0F);
  }

  // SwitchCase: '<S5>/Switch Case' incorporates:
  //   Constant: '<S12>/Zero'
  //   Constant: '<S13>/Zero'
  //   Merge: '<S5>/Merge'
  //   Signum: '<S5>/Sign'

  switch (i) {
   case 1:
    // Outputs for IfAction SubSystem: '<S5>/Servo Left' incorporates:
    //   ActionPort: '<S12>/Action Port'

    // SignalConversion generated from: '<S12>/Command' incorporates:
    //   Abs: '<S5>/Abs1'
    //   Merge: '<S5>/Merge'

    wingController_DW.Merge[0] = std::abs(rtb_k_perp);
    wingController_DW.Merge[1] = wingController_P.Zero_Value;

    // End of Outputs for SubSystem: '<S5>/Servo Left'
    break;

   case 0:
    // Outputs for IfAction SubSystem: '<S5>/No Activation' incorporates:
    //   ActionPort: '<S7>/Action Port'

    // Merge: '<S5>/Merge' incorporates:
    //   Constant: '<S7>/Zero'
    //   SignalConversion generated from: '<S7>/Servo'

    wingController_DW.Merge[0] = wingController_P.Zero_Value_d[0];
    wingController_DW.Merge[1] = wingController_P.Zero_Value_d[1];

    // End of Outputs for SubSystem: '<S5>/No Activation'
    break;

   default:
    // Outputs for IfAction SubSystem: '<S5>/Servo Right' incorporates:
    //   ActionPort: '<S13>/Action Port'

    // SignalConversion generated from: '<S13>/Command' incorporates:
    //   Abs: '<S5>/Abs1'
    //   Merge: '<S5>/Merge'

    wingController_DW.Merge[1] = std::abs(rtb_k_perp);
    wingController_DW.Merge[0] = wingController_P.Zero_Value_p;

    // End of Outputs for SubSystem: '<S5>/Servo Right'
    break;
  }

  // End of SwitchCase: '<S5>/Switch Case'

  // Bias: '<S6>/Bias1' incorporates:
  //   UnitDelay: '<S6>/Unit Delay1'

  u0 = static_cast<uint8_t>(wingController_DW.UnitDelay1_DSTATE +
    wingController_P.Bias1_Bias_b);

  // Saturate: '<S6>/Saturation'
  if (u0 > wingController_P.Saturation_UpperSat_f) {
    // Update for UnitDelay: '<S6>/Unit Delay1'
    wingController_DW.UnitDelay1_DSTATE = wingController_P.Saturation_UpperSat_f;
  } else if (u0 < wingController_P.Saturation_LowerSat_b) {
    // Update for UnitDelay: '<S6>/Unit Delay1'
    wingController_DW.UnitDelay1_DSTATE = wingController_P.Saturation_LowerSat_b;
  } else {
    // Update for UnitDelay: '<S6>/Unit Delay1'
    wingController_DW.UnitDelay1_DSTATE = u0;
  }

  // End of Saturate: '<S6>/Saturation'

  // Update for Memory: '<S5>/Memory'
  wingController_DW.Memory_PreviousInput_e = rtb_Gain1;

  // Update for Memory: '<S5>/Memory2' incorporates:
  //   Constant: '<S10>/Constant'
  //   Constant: '<S8>/Constant'
  //   Logic: '<S5>/OR1'
  //   RelationalOperator: '<S10>/Compare'
  //   RelationalOperator: '<S8>/Compare'

  wingController_DW.Memory2_PreviousInput = ((rtb_Gain4 ==
    wingController_P.SaturationCheckUp_const) || (rtb_Gain4 ==
    wingController_P.SaturationCheckLw_const));

  // Update for Memory: '<S5>/Memory1'
  wingController_DW.Memory1_PreviousInput_o = rtb_k_par;

  // Update for Memory: '<S5>/Memory3' incorporates:
  //   Constant: '<S11>/Constant'
  //   Constant: '<S9>/Constant'
  //   Logic: '<S5>/OR2'
  //   RelationalOperator: '<S11>/Compare'
  //   RelationalOperator: '<S9>/Compare'

  wingController_DW.Memory3_PreviousInput = ((rtb_DotProduct1 ==
    wingController_P.SaturationCheckUp1_const) || (rtb_DotProduct1 ==
    wingController_P.SaturationCheckLw1_const));

  // End of Outputs for SubSystem: '<Root>/wingController DPG'

  // Outport: '<Root>/Servo Commands'
  wingController_Y.ServoCommands[0] = wingController_DW.Merge[0];
  wingController_Y.ServoCommands[1] = wingController_DW.Merge[1];

  // Outputs for Atomic SubSystem: '<Root>/wingController DPG'
  // Update for Memory: '<S4>/Memory1' incorporates:
  //   DotProduct: '<S4>/Dot Product1'
  //   Sqrt: '<S4>/Sqrt1'
  //   Sum: '<S4>/Sum1'

  wingController_DW.Memory1_PreviousInput = std::sqrt(rtb_Sum1_idx_0 *
    rtb_Sum1_idx_0 + rtb_Sum1_idx_1 * rtb_Sum1_idx_1);

  // Update for Memory: '<S4>/Memory' incorporates:
  //   DotProduct: '<S4>/Dot Product2'
  //   Sqrt: '<S4>/Sqrt2'
  //   Sum: '<S4>/Sum2'

  wingController_DW.Memory_PreviousInput = std::sqrt(rtb_Sum2_idx_0 *
    rtb_Sum2_idx_0 + rtb_Sum2_idx_1 * rtb_Sum2_idx_1);

  // End of Outputs for SubSystem: '<Root>/wingController DPG'

  // Outport: '<Root>/PRF Logs OBSW' incorporates:
  //   Bias: '<S6>/Bias'
  //   BusAssignment: '<S5>/Bus Assignment'
  //   BusCreator generated from: '<S6>/PRF Logs OBSW_BusCreator'
  //   Math: '<S17>/Transpose1'

  for (i = 0; i < 6; i++) {
    // Outputs for Atomic SubSystem: '<Root>/wingController DPG'
    wingController_Y.PRFLogsOBSW.Targets[i] = wingController_DW.TargetPoints[i];

    // End of Outputs for SubSystem: '<Root>/wingController DPG'
  }

  // Outputs for Atomic SubSystem: '<Root>/wingController DPG'
  wingController_Y.PRFLogsOBSW.TargetIndex = static_cast<uint8_t>(rtb_target +
    wingController_P.Bias_Bias_e);
  wingController_Y.PRFLogsOBSW.Heading = rtb_inversion;
  wingController_Y.PRFLogsOBSW.Reference = rtb_direct_distance;
  wingController_Y.PRFLogsOBSW.ServoCommands[0] = wingController_DW.Merge[0];
  wingController_Y.PRFLogsOBSW.Q1[0] = wingController_DW.Q1[0];
  wingController_Y.PRFLogsOBSW.Q2[0] = wingController_DW.Q2[0];
  wingController_Y.PRFLogsOBSW.ServoCommands[1] = wingController_DW.Merge[1];
  wingController_Y.PRFLogsOBSW.Q1[1] = wingController_DW.Q1[1];
  wingController_Y.PRFLogsOBSW.Q2[1] = wingController_DW.Q2[1];

  // End of Outport: '<Root>/PRF Logs OBSW'
  // End of Outputs for SubSystem: '<Root>/wingController DPG'
}

// Model initialize function
void wingController::initialize()
{
  {
    int32_t i;
    wingController_PrevZCX.TargetPointsGeneration_Trig_ZCE = POS_ZCSIG;

    // SystemInitialize for Atomic SubSystem: '<Root>/wingController DPG'
    // InitializeConditions for UnitDelay: '<S6>/Unit Delay1'
    wingController_DW.UnitDelay1_DSTATE =
      wingController_P.UnitDelay1_InitialCondition;

    // InitializeConditions for Memory: '<S4>/Memory1'
    wingController_DW.Memory1_PreviousInput =
      wingController_P.Memory1_InitialCondition;

    // InitializeConditions for Memory: '<S4>/Memory'
    wingController_DW.Memory_PreviousInput =
      wingController_P.Memory_InitialCondition;

    // InitializeConditions for Memory: '<S5>/Memory'
    wingController_DW.Memory_PreviousInput_e =
      wingController_P.Memory_InitialCondition_d;

    // InitializeConditions for Memory: '<S5>/Memory2'
    wingController_DW.Memory2_PreviousInput =
      wingController_P.Memory2_InitialCondition;

    // InitializeConditions for Memory: '<S5>/Memory1'
    wingController_DW.Memory1_PreviousInput_o =
      wingController_P.Memory1_InitialCondition_f;

    // InitializeConditions for Memory: '<S5>/Memory3'
    wingController_DW.Memory3_PreviousInput =
      wingController_P.Memory3_InitialCondition;

    // SystemInitialize for Triggered SubSystem: '<S6>/Target Points Generation' 
    // SystemInitialize for Merge: '<S17>/Merge'
    wingController_DW.Q1[0] = wingController_P.Merge_InitialOutput;
    wingController_DW.Q1[1] = wingController_P.Merge_InitialOutput;
    for (i = 0; i < 6; i++) {
      // SystemInitialize for Math: '<S17>/Transpose1' incorporates:
      //   Outport: '<S17>/Target Points'

      wingController_DW.TargetPoints[i] = wingController_P.TargetPoints_Y0[i];
    }

    // SystemInitialize for Sum: '<S22>/Sum2' incorporates:
    //   Outport: '<S17>/Q2'

    wingController_DW.Q2[0] = wingController_P.Q2_Y0;

    // End of SystemInitialize for SubSystem: '<S6>/Target Points Generation'
    // End of SystemInitialize for SubSystem: '<Root>/wingController DPG'

    // SystemInitialize for Merge: '<S5>/Merge'
    wingController_DW.Merge[0] = wingController_P.Merge_InitialOutput_e;

    // SystemInitialize for Outport: '<Root>/Servo Commands' incorporates:
    //   Merge: '<S5>/Merge'

    wingController_Y.ServoCommands[0] = wingController_P.Merge_InitialOutput_e;

    // SystemInitialize for Atomic SubSystem: '<Root>/wingController DPG'
    // SystemInitialize for Triggered SubSystem: '<S6>/Target Points Generation' 
    // SystemInitialize for Sum: '<S22>/Sum2' incorporates:
    //   Outport: '<S17>/Q2'

    wingController_DW.Q2[1] = wingController_P.Q2_Y0;

    // End of SystemInitialize for SubSystem: '<S6>/Target Points Generation'
    // End of SystemInitialize for SubSystem: '<Root>/wingController DPG'

    // SystemInitialize for Merge: '<S5>/Merge'
    wingController_DW.Merge[1] = wingController_P.Merge_InitialOutput_e;

    // SystemInitialize for Outport: '<Root>/Servo Commands' incorporates:
    //   Merge: '<S5>/Merge'

    wingController_Y.ServoCommands[1] = wingController_P.Merge_InitialOutput_e;
  }
}

// Model terminate function
void wingController::terminate()
{
  // (no terminate code required)
}

// Constructor
wingController::wingController():
  wingController_U(),
  wingController_Y(),
  wingController_DW(),
  wingController_PrevZCX()
{
  // Currently there is no constructor body generated.
}

// Destructor
// Currently there is no destructor body generated.
wingController::~wingController() = default;

//
// File trailer for generated code.
//
// [EOF]
//
