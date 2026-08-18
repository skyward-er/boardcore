//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// File: PRF.cpp
//
// Code generated for Simulink model 'PRF'.
//
// Model version                  : 11.370
// Simulink Coder version         : 24.2 (R2024b) 21-Jun-2024
// C/C++ source code generated on : Tue Aug 18 16:18:50 2026
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
#include <cmath>
#include "PRF_private.h"
#include <stdint.h>
#include <cfloat>
#include <stdbool.h>

namespace PRF 
{


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
  double rtb_Saturation2;
  double rtb_Switch2;
  float rtb_Heading;
  float rtb_Reference;
  float rtb_SinCos_o2;
  float rtb_UnitDelay1;
  float rtb_dEast;
  int32_t tmp;

  // Outputs for Atomic SubSystem: '<Root>/PRF'
  // Outputs for Atomic SubSystem: '<S1>/LLA to NED'
  // Sum: '<S14>/Sum' incorporates:
  //   Constant: '<S14>/Constant2'
  //   Constant: '<S14>/f'

  // Unit Conversion - from: deg to: rad
  // Expression: output = (0.0174533*input) + (0)
  rtb_UnitDelay1 = PRF_P.f_Value - PRF_P.Constant2_Value;

  // Sqrt: '<S14>/sqrt1' incorporates:
  //   Constant: '<S14>/Constant3'
  //   Product: '<S14>/Product5'
  //   Sum: '<S14>/Sum2'

  rtb_UnitDelay1 = std::sqrt(PRF_P.Constant3_Value - rtb_UnitDelay1 *
    rtb_UnitDelay1);

  // UnitConversion: '<S15>/Unit Conversion' incorporates:
  //   Constant: '<S1>/Constant1'

  // Unit Conversion - from: deg to: rad
  // Expression: output = (0.0174533*input) + (0)
  rtb_Heading = 0.0174532924F * PRF_P.Constant1_Value_c[0];

  // Trigonometry: '<S16>/Trigonometric Function1'
  rtb_Reference = std::sin(rtb_Heading);

  // Product: '<S16>/Product1' incorporates:
  //   Product: '<S14>/Product2'

  rtb_UnitDelay1 *= rtb_UnitDelay1;

  // Sum: '<S16>/Sum1' incorporates:
  //   Constant: '<S16>/Constant'
  //   Product: '<S16>/Product1'

  rtb_Reference = PRF_P.Constant_Value_b - rtb_UnitDelay1 * rtb_Reference *
    rtb_Reference;

  // Product: '<S14>/Product1' incorporates:
  //   Constant: '<S14>/Constant1'
  //   Sqrt: '<S14>/sqrt'

  rtb_dEast = PRF_P.Constant1_Value_p / std::sqrt(rtb_Reference);

  // Product: '<S12>/dNorth' incorporates:
  //   Constant: '<S14>/Constant'
  //   Constant: '<S1>/Constant1'
  //   Inport: '<Root>/PRF Reference'
  //   Product: '<S14>/Product3'
  //   Sum: '<S14>/Sum1'
  //   Sum: '<S3>/Sum1'
  //   UnitConversion: '<S13>/Unit Conversion'

  rtb_Reference = (PRF_P.Constant_Value_n - rtb_UnitDelay1) * rtb_dEast /
    rtb_Reference * ((PRF_U.PRFReference_m.TargetPositionLLA[0] -
                      PRF_P.Constant1_Value_c[0]) * 0.0174532924F);

  // Trigonometry: '<S12>/SinCos' incorporates:
  //   Constant: '<S12>/Zero'

  rtb_UnitDelay1 = std::sin(PRF_P.Zero_Value_c);
  rtb_SinCos_o2 = std::cos(PRF_P.Zero_Value_c);

  // Product: '<S12>/dEast' incorporates:
  //   Constant: '<S1>/Constant1'
  //   Inport: '<Root>/PRF Reference'
  //   Product: '<S14>/Product4'
  //   Sum: '<S3>/Sum1'
  //   Trigonometry: '<S14>/Trigonometric Function'
  //   UnitConversion: '<S13>/Unit Conversion'

  rtb_dEast = (PRF_U.PRFReference_m.TargetPositionLLA[1] -
               PRF_P.Constant1_Value_c[1]) * 0.0174532924F * (rtb_dEast * std::
    cos(rtb_Heading));

  // RateTransition: '<S1>/Rate Transition' incorporates:
  //   Product: '<S12>/x*cos'
  //   Product: '<S12>/x*sin'
  //   Product: '<S12>/y*cos'
  //   Product: '<S12>/y*sin'
  //   Sum: '<S12>/Sum2'
  //   Sum: '<S12>/Sum3'

  rtb_Heading = rtb_Reference * rtb_SinCos_o2 + rtb_dEast * rtb_UnitDelay1;
  rtb_Reference = rtb_dEast * rtb_SinCos_o2 - rtb_Reference * rtb_UnitDelay1;

  // End of Outputs for SubSystem: '<S1>/LLA to NED'

  // SignalConversion generated from: '<S1>/Vector Concatenate' incorporates:
  //   Concatenate: '<S1>/Vector Concatenate'
  //   Outport: '<Root>/PRF Logs OBSW'
  //   Sum: '<S2>/Subtract1'

  PRF_Y.PRFLogsOBSW.TerminalTarget[0] = rtb_Heading;
  PRF_Y.PRFLogsOBSW.TerminalTarget[1] = rtb_Reference;

  // Trigonometry: '<S2>/Atan1' incorporates:
  //   Inport: '<Root>/PRF In'
  //   Math: '<S2>/Transpose'
  //   Sum: '<S2>/Subtract1'

  rtb_Reference = std::atan2(rtb_Reference - PRF_U.PRFIn_o.NASDAQPosition[1],
    rtb_Heading - PRF_U.PRFIn_o.NASDAQPosition[0]);

  // Trigonometry: '<S2>/Atan2' incorporates:
  //   Inport: '<Root>/PRF In'

  rtb_Heading = std::atan2(PRF_U.PRFIn_o.NASDAQVelocity[1],
    PRF_U.PRFIn_o.NASDAQVelocity[0]);

  // Sum: '<S2>/Subtract'
  rtb_UnitDelay1 = rtb_Reference - rtb_Heading;

  // Switch: '<S2>/Switch' incorporates:
  //   Abs: '<S2>/Abs'
  //   Bias: '<S2>/Bias'
  //   Switch: '<S9>/Switch'

  if (std::abs(rtb_UnitDelay1) > PRF_P.Switch_Threshold) {
    // Bias: '<S2>/Bias1'
    rtb_UnitDelay1 += PRF_P.Bias1_Bias;

    // Math: '<S9>/Mod' incorporates:
    //   Constant: '<S9>/Constant'

    rtb_dEast = rt_modf(rtb_UnitDelay1, PRF_P.Constant_Value_j);

    // Switch: '<S9>/Switch' incorporates:
    //   Constant: '<S10>/Constant'
    //   Constant: '<S11>/Constant'
    //   Constant: '<S9>/Constant1'
    //   Logic: '<S9>/AND'
    //   RelationalOperator: '<S10>/Compare'
    //   RelationalOperator: '<S11>/Compare'

    if ((rtb_UnitDelay1 > PRF_P.Constant_Value_o) && (rtb_dEast ==
         PRF_P.Constant_Value_e)) {
      rtb_dEast = PRF_P.Constant1_Value;
    }

    rtb_UnitDelay1 = rtb_dEast + PRF_P.Bias_Bias;
  }

  // End of Switch: '<S2>/Switch'

  // Switch: '<S2>/Switch2' incorporates:
  //   Constant: '<S2>/Constant'
  //   Constant: '<S2>/Constant10'
  //   Constant: '<S2>/Constant9'
  //   Product: '<S2>/Product6'
  //   UnitDelay: '<S2>/Unit Delay2'

  if (PRF_DW.UnitDelay2_DSTATE) {
    rtb_Switch2 = PRF_P.Constant_Value;
  } else {
    rtb_Switch2 = PRF_P.Constant9_Value * PRF_P.Constant10_Value *
      rtb_UnitDelay1;
  }

  // Sum: '<S2>/Add3' incorporates:
  //   Switch: '<S2>/Switch2'
  //   UnitDelay: '<S2>/Unit Delay3'

  rtb_Switch2 += PRF_DW.UnitDelay3_DSTATE;

  // Sum: '<S2>/Add2' incorporates:
  //   Constant: '<S2>/Constant6'
  //   Constant: '<S2>/Constant7'
  //   Constant: '<S2>/Constant8'
  //   Product: '<S2>/Product4'
  //   Product: '<S2>/Product5'
  //   Sum: '<S2>/Add'
  //   UnitDelay: '<S2>/Unit Delay1'

  rtb_Saturation2 = (rtb_UnitDelay1 - PRF_DW.UnitDelay1_DSTATE) *
    PRF_P.Constant7_Value / PRF_P.Constant6_Value + (rtb_UnitDelay1 *
    PRF_P.Constant8_Value + rtb_Switch2);

  // Saturate: '<S2>/Saturation2'
  if (rtb_Saturation2 > PRF_P.Saturation2_UpperSat) {
    rtb_Saturation2 = PRF_P.Saturation2_UpperSat;
  } else if (rtb_Saturation2 < PRF_P.Saturation2_LowerSat) {
    rtb_Saturation2 = PRF_P.Saturation2_LowerSat;
  }

  // End of Saturate: '<S2>/Saturation2'

  // Signum: '<S2>/Sign1'
  if (rtb_Saturation2 < 0.0) {
    tmp = -1;
  } else {
    tmp = (rtb_Saturation2 > 0.0);
  }

  // SwitchCase: '<S2>/Switch Case1' incorporates:
  //   Constant: '<S7>/Zero'
  //   Constant: '<S8>/Zero'
  //   Merge: '<S2>/Merge1'
  //   Signum: '<S2>/Sign1'

  switch (tmp) {
   case 1:
    // Abs: '<S2>/Abs2'
    if (rtb_Saturation2 < 0.0) {
      // Outputs for IfAction SubSystem: '<S2>/Servo Right1' incorporates:
      //   ActionPort: '<S8>/Action Port'

      // SignalConversion generated from: '<S8>/Command' incorporates:
      //   Merge: '<S2>/Merge1'

      PRF_DW.Merge1[1] = static_cast<float>(-rtb_Saturation2);

      // End of Outputs for SubSystem: '<S2>/Servo Right1'
    } else {
      // Outputs for IfAction SubSystem: '<S2>/Servo Right1' incorporates:
      //   ActionPort: '<S8>/Action Port'

      // SignalConversion generated from: '<S8>/Command' incorporates:
      //   Merge: '<S2>/Merge1'

      PRF_DW.Merge1[1] = static_cast<float>(rtb_Saturation2);

      // End of Outputs for SubSystem: '<S2>/Servo Right1'
    }

    // Outputs for IfAction SubSystem: '<S2>/Servo Right1' incorporates:
    //   ActionPort: '<S8>/Action Port'

    PRF_DW.Merge1[0] = PRF_P.Zero_Value;

    // End of Outputs for SubSystem: '<S2>/Servo Right1'
    break;

   case 0:
    // Outputs for IfAction SubSystem: '<S2>/No Activation1' incorporates:
    //   ActionPort: '<S4>/Action Port'

    // Merge: '<S2>/Merge1' incorporates:
    //   Constant: '<S4>/Zero'
    //   SignalConversion generated from: '<S4>/Servo'

    PRF_DW.Merge1[0] = PRF_P.Zero_Value_a[0];
    PRF_DW.Merge1[1] = PRF_P.Zero_Value_a[1];

    // End of Outputs for SubSystem: '<S2>/No Activation1'
    break;

   default:
    // Abs: '<S2>/Abs2'
    if (rtb_Saturation2 < 0.0) {
      // Outputs for IfAction SubSystem: '<S2>/Servo Left1' incorporates:
      //   ActionPort: '<S7>/Action Port'

      // SignalConversion generated from: '<S7>/Command' incorporates:
      //   Merge: '<S2>/Merge1'

      PRF_DW.Merge1[0] = static_cast<float>(-rtb_Saturation2);

      // End of Outputs for SubSystem: '<S2>/Servo Left1'
    } else {
      // Outputs for IfAction SubSystem: '<S2>/Servo Left1' incorporates:
      //   ActionPort: '<S7>/Action Port'

      // SignalConversion generated from: '<S7>/Command' incorporates:
      //   Merge: '<S2>/Merge1'

      PRF_DW.Merge1[0] = static_cast<float>(rtb_Saturation2);

      // End of Outputs for SubSystem: '<S2>/Servo Left1'
    }

    // Outputs for IfAction SubSystem: '<S2>/Servo Left1' incorporates:
    //   ActionPort: '<S7>/Action Port'

    PRF_DW.Merge1[1] = PRF_P.Zero_Value_d;

    // End of Outputs for SubSystem: '<S2>/Servo Left1'
    break;
  }

  // End of SwitchCase: '<S2>/Switch Case1'

  // Update for UnitDelay: '<S2>/Unit Delay2' incorporates:
  //   Constant: '<S5>/Constant'
  //   Constant: '<S6>/Constant'
  //   Logic: '<S2>/OR1'
  //   RelationalOperator: '<S5>/Compare'
  //   RelationalOperator: '<S6>/Compare'

  PRF_DW.UnitDelay2_DSTATE = ((rtb_Saturation2 == PRF_P.SaturationCheckUp1_const)
    || (rtb_Saturation2 == PRF_P.SaturationCheckLw1_const));

  // Update for UnitDelay: '<S2>/Unit Delay3'
  PRF_DW.UnitDelay3_DSTATE = rtb_Switch2;

  // Update for UnitDelay: '<S2>/Unit Delay1'
  PRF_DW.UnitDelay1_DSTATE = rtb_UnitDelay1;

  // Outport: '<Root>/PRF Logs OBSW' incorporates:
  //   BusCreator generated from: '<S2>/PRFLogs OBSW_BusCreator'

  PRF_Y.PRFLogsOBSW.TargetIndex = 0U;
  PRF_Y.PRFLogsOBSW.Heading = rtb_Heading;
  PRF_Y.PRFLogsOBSW.Reference = rtb_Reference;

  // RateTransition: '<S1>/Rate Transition1' incorporates:
  //   Concatenate: '<S1>/Vector Concatenate'
  //   Inport: '<Root>/PRF Reference'
  //   Outport: '<Root>/PRF Logs OBSW'

  PRF_Y.PRFLogsOBSW.TerminalTarget[2] = PRF_U.PRFReference_m.TargetPositionLLA[0];

  // End of Outputs for SubSystem: '<Root>/PRF'

  // Outport: '<Root>/Servo Commands'
  PRF_Y.ServoCommands[0] = PRF_DW.Merge1[0];

  // Outputs for Atomic SubSystem: '<Root>/PRF'
  // Outport: '<Root>/PRF Logs OBSW' incorporates:
  //   Outport: '<Root>/Servo Commands'
  //   SignalConversion generated from: '<S2>/PRFLogs OBSW_BusCreator'

  PRF_Y.PRFLogsOBSW.Q1[0] = 0.0F;
  PRF_Y.PRFLogsOBSW.Q2[0] = 0.0F;

  // End of Outputs for SubSystem: '<Root>/PRF'
  PRF_Y.PRFLogsOBSW.ServoCommands[0] = PRF_DW.Merge1[0];

  // Outputs for Atomic SubSystem: '<Root>/PRF'
  // RateTransition: '<S1>/Rate Transition1' incorporates:
  //   Concatenate: '<S1>/Vector Concatenate'
  //   Inport: '<Root>/PRF Reference'
  //   Outport: '<Root>/PRF Logs OBSW'

  PRF_Y.PRFLogsOBSW.TerminalTarget[3] = PRF_U.PRFReference_m.TargetPositionLLA[1];

  // End of Outputs for SubSystem: '<Root>/PRF'

  // Outport: '<Root>/Servo Commands'
  PRF_Y.ServoCommands[1] = PRF_DW.Merge1[1];

  // Outputs for Atomic SubSystem: '<Root>/PRF'
  // Outport: '<Root>/PRF Logs OBSW' incorporates:
  //   Outport: '<Root>/Servo Commands'
  //   SignalConversion generated from: '<S2>/PRFLogs OBSW_BusCreator'

  PRF_Y.PRFLogsOBSW.Q1[1] = 0.0F;
  PRF_Y.PRFLogsOBSW.Q2[1] = 0.0F;

  // End of Outputs for SubSystem: '<Root>/PRF'
  PRF_Y.PRFLogsOBSW.ServoCommands[1] = PRF_DW.Merge1[1];

  // Outputs for Atomic SubSystem: '<Root>/PRF'
  PRF_Y.PRFLogsOBSW.WindHeading = 0.0F;

  // End of Outputs for SubSystem: '<Root>/PRF'
}

// Model initialize function
void PRF::initialize()
{
  // SystemInitialize for Atomic SubSystem: '<Root>/PRF'
  // InitializeConditions for UnitDelay: '<S2>/Unit Delay2'
  PRF_DW.UnitDelay2_DSTATE = PRF_P.UnitDelay2_InitialCondition;

  // InitializeConditions for UnitDelay: '<S2>/Unit Delay3'
  PRF_DW.UnitDelay3_DSTATE = PRF_P.UnitDelay3_InitialCondition;

  // InitializeConditions for UnitDelay: '<S2>/Unit Delay1'
  PRF_DW.UnitDelay1_DSTATE = PRF_P.UnitDelay1_InitialCondition;

  // End of SystemInitialize for SubSystem: '<Root>/PRF'

  // SystemInitialize for Merge: '<S2>/Merge1'
  PRF_DW.Merge1[0] = PRF_P.Merge1_InitialOutput;

  // SystemInitialize for Outport: '<Root>/Servo Commands' incorporates:
  //   Merge: '<S2>/Merge1'

  PRF_Y.ServoCommands[0] = PRF_P.Merge1_InitialOutput;

  // SystemInitialize for Merge: '<S2>/Merge1'
  PRF_DW.Merge1[1] = PRF_P.Merge1_InitialOutput;

  // SystemInitialize for Outport: '<Root>/Servo Commands' incorporates:
  //   Merge: '<S2>/Merge1'

  PRF_Y.ServoCommands[1] = PRF_P.Merge1_InitialOutput;
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
  PRF_DW()
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
