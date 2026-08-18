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
// C/C++ source code generated on : Wed Jul 22 13:17:22 2026
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
  float rtb_Heading;
  float rtb_Memory;
  float rtb_Reference;
  float rtb_Saturation1;
  float rtb_SinCos_o2;
  int32_t tmp;

  // Outputs for Atomic SubSystem: '<Root>/PRF'
  // Outputs for Atomic SubSystem: '<S1>/LLA to NED'
  // Sum: '<S14>/Sum' incorporates:
  //   Constant: '<S14>/Constant2'
  //   Constant: '<S14>/f'

  // Unit Conversion - from: deg to: rad
  // Expression: output = (0.0174533*input) + (0)
  rtb_Saturation1 = PRF_P.f_Value - PRF_P.Constant2_Value;

  // Sqrt: '<S14>/sqrt1' incorporates:
  //   Constant: '<S14>/Constant3'
  //   Product: '<S14>/Product5'
  //   Sum: '<S14>/Sum2'

  rtb_Saturation1 = std::sqrt(PRF_P.Constant3_Value - rtb_Saturation1 *
    rtb_Saturation1);

  // UnitConversion: '<S15>/Unit Conversion' incorporates:
  //   Constant: '<S1>/Constant1'

  // Unit Conversion - from: deg to: rad
  // Expression: output = (0.0174533*input) + (0)
  rtb_Heading = 0.0174532924F * PRF_P.Constant1_Value_i[0];

  // Trigonometry: '<S16>/Trigonometric Function1'
  rtb_Memory = std::sin(rtb_Heading);

  // Product: '<S16>/Product1' incorporates:
  //   Product: '<S14>/Product2'

  rtb_Saturation1 *= rtb_Saturation1;

  // Sum: '<S16>/Sum1' incorporates:
  //   Constant: '<S16>/Constant'
  //   Product: '<S16>/Product1'

  rtb_Memory = PRF_P.Constant_Value_c - rtb_Saturation1 * rtb_Memory *
    rtb_Memory;

  // Product: '<S14>/Product1' incorporates:
  //   Constant: '<S14>/Constant1'
  //   Sqrt: '<S14>/sqrt'

  rtb_Reference = PRF_P.Constant1_Value_a / std::sqrt(rtb_Memory);

  // Product: '<S12>/dNorth' incorporates:
  //   Constant: '<S14>/Constant'
  //   Constant: '<S1>/Constant1'
  //   Inport: '<Root>/PRF Reference'
  //   Product: '<S14>/Product3'
  //   Sum: '<S14>/Sum1'
  //   Sum: '<S3>/Sum1'
  //   UnitConversion: '<S13>/Unit Conversion'

  rtb_Memory = (PRF_P.Constant_Value_a - rtb_Saturation1) * rtb_Reference /
    rtb_Memory * ((PRF_U.PRFReference_a.TargetPositionLLA[0] -
                   PRF_P.Constant1_Value_i[0]) * 0.0174532924F);

  // Trigonometry: '<S12>/SinCos' incorporates:
  //   Constant: '<S12>/Zero'

  rtb_Saturation1 = std::sin(PRF_P.Zero_Value_on);
  rtb_SinCos_o2 = std::cos(PRF_P.Zero_Value_on);

  // Product: '<S12>/dEast' incorporates:
  //   Constant: '<S1>/Constant1'
  //   Inport: '<Root>/PRF Reference'
  //   Product: '<S14>/Product4'
  //   Sum: '<S3>/Sum1'
  //   Trigonometry: '<S14>/Trigonometric Function'
  //   UnitConversion: '<S13>/Unit Conversion'

  rtb_Reference = (PRF_U.PRFReference_a.TargetPositionLLA[1] -
                   PRF_P.Constant1_Value_i[1]) * 0.0174532924F * (rtb_Reference *
    std::cos(rtb_Heading));

  // RateTransition: '<S1>/Rate Transition' incorporates:
  //   Product: '<S12>/x*cos'
  //   Product: '<S12>/x*sin'
  //   Product: '<S12>/y*cos'
  //   Product: '<S12>/y*sin'
  //   Sum: '<S12>/Sum2'
  //   Sum: '<S12>/Sum3'

  rtb_Heading = rtb_Memory * rtb_SinCos_o2 + rtb_Reference * rtb_Saturation1;
  rtb_Memory = rtb_Reference * rtb_SinCos_o2 - rtb_Memory * rtb_Saturation1;

  // End of Outputs for SubSystem: '<S1>/LLA to NED'

  // SignalConversion generated from: '<S1>/Vector Concatenate' incorporates:
  //   Concatenate: '<S1>/Vector Concatenate'
  //   Outport: '<Root>/PRF Logs OBSW'
  //   Sum: '<S2>/Subtract1'

  PRF_Y.PRFLogsOBSW.TerminalTarget[0] = rtb_Heading;
  PRF_Y.PRFLogsOBSW.TerminalTarget[1] = rtb_Memory;

  // Trigonometry: '<S2>/Atan1' incorporates:
  //   Inport: '<Root>/PRF In'
  //   Math: '<S2>/Transpose'
  //   Sum: '<S2>/Subtract1'

  rtb_Reference = std::atan2(rtb_Memory - PRF_U.PRFIn_h.NASDAQPosition[1],
    rtb_Heading - PRF_U.PRFIn_h.NASDAQPosition[0]);

  // Trigonometry: '<S2>/Atan2' incorporates:
  //   Inport: '<Root>/PRF In'

  rtb_Heading = std::atan2(PRF_U.PRFIn_h.NASDAQVelocity[1],
    PRF_U.PRFIn_h.NASDAQVelocity[0]);

  // Sum: '<S2>/Subtract'
  rtb_Memory = rtb_Reference - rtb_Heading;

  // Switch: '<S2>/Switch' incorporates:
  //   Abs: '<S2>/Abs'
  //   Bias: '<S2>/Bias'
  //   Switch: '<S9>/Switch'

  if (std::abs(rtb_Memory) > PRF_P.Switch_Threshold) {
    // Bias: '<S2>/Bias1'
    rtb_Saturation1 = rtb_Memory + PRF_P.Bias1_Bias;

    // Math: '<S9>/Mod' incorporates:
    //   Constant: '<S9>/Constant'

    rtb_Memory = rt_modf(rtb_Saturation1, PRF_P.Constant_Value);

    // Switch: '<S9>/Switch' incorporates:
    //   Constant: '<S10>/Constant'
    //   Constant: '<S11>/Constant'
    //   Constant: '<S9>/Constant1'
    //   Logic: '<S9>/AND'
    //   RelationalOperator: '<S10>/Compare'
    //   RelationalOperator: '<S11>/Compare'

    if ((rtb_Saturation1 > PRF_P.Constant_Value_n) && (rtb_Memory ==
         PRF_P.Constant_Value_nk)) {
      rtb_Memory = PRF_P.Constant1_Value;
    }

    rtb_Memory += PRF_P.Bias_Bias;
  }

  // End of Switch: '<S2>/Switch'

  // Switch: '<S2>/Switch1' incorporates:
  //   Constant: '<S2>/Constant4'
  //   Constant: '<S2>/Constant5'
  //   Memory: '<S2>/Memory'
  //   Memory: '<S2>/Memory2'
  //   Product: '<S2>/Product3'

  if (PRF_DW.Memory2_PreviousInput) {
    rtb_SinCos_o2 = PRF_DW.Memory_PreviousInput;
  } else {
    rtb_SinCos_o2 = PRF_P.Constant4_Value * PRF_P.Constant5_Value * rtb_Memory;
  }

  // End of Switch: '<S2>/Switch1'

  // Sum: '<S2>/Add1' incorporates:
  //   Constant: '<S2>/Constant1'
  //   Constant: '<S2>/Constant2'
  //   Constant: '<S2>/Constant3'
  //   Product: '<S2>/Product1'
  //   Product: '<S2>/Product2'
  //   UnitDelay: '<S2>/Unit Delay'

  rtb_Saturation1 = (rtb_Memory * PRF_P.Constant3_Value_p + rtb_SinCos_o2) +
    PRF_DW.UnitDelay_DSTATE * PRF_P.Constant2_Value_a / PRF_P.Constant1_Value_p;

  // Saturate: '<S2>/Saturation1'
  if (rtb_Saturation1 > PRF_P.Saturation1_UpperSat) {
    rtb_Saturation1 = PRF_P.Saturation1_UpperSat;
  } else if (rtb_Saturation1 < PRF_P.Saturation1_LowerSat) {
    rtb_Saturation1 = PRF_P.Saturation1_LowerSat;
  }

  // End of Saturate: '<S2>/Saturation1'

  // Signum: '<S2>/Sign'
  if (rtb_Saturation1 < 0.0F) {
    tmp = -1;
  } else {
    tmp = (rtb_Saturation1 > 0.0F);
  }

  // SwitchCase: '<S2>/Switch Case' incorporates:
  //   Constant: '<S7>/Zero'
  //   Constant: '<S8>/Zero'
  //   Merge: '<S2>/Merge'
  //   Signum: '<S2>/Sign'

  switch (tmp) {
   case 1:
    // Outputs for IfAction SubSystem: '<S2>/Servo Right' incorporates:
    //   ActionPort: '<S8>/Action Port'

    // SignalConversion generated from: '<S8>/Command' incorporates:
    //   Abs: '<S2>/Abs1'
    //   Merge: '<S2>/Merge'

    PRF_DW.Merge[1] = std::abs(rtb_Saturation1);
    PRF_DW.Merge[0] = PRF_P.Zero_Value;

    // End of Outputs for SubSystem: '<S2>/Servo Right'
    break;

   case 0:
    // Outputs for IfAction SubSystem: '<S2>/No Activation' incorporates:
    //   ActionPort: '<S4>/Action Port'

    // Merge: '<S2>/Merge' incorporates:
    //   Constant: '<S4>/Zero'
    //   SignalConversion generated from: '<S4>/Servo'

    PRF_DW.Merge[0] = PRF_P.Zero_Value_m[0];
    PRF_DW.Merge[1] = PRF_P.Zero_Value_m[1];

    // End of Outputs for SubSystem: '<S2>/No Activation'
    break;

   default:
    // Outputs for IfAction SubSystem: '<S2>/Servo Left' incorporates:
    //   ActionPort: '<S7>/Action Port'

    // SignalConversion generated from: '<S7>/Command' incorporates:
    //   Abs: '<S2>/Abs1'
    //   Merge: '<S2>/Merge'

    PRF_DW.Merge[0] = std::abs(rtb_Saturation1);
    PRF_DW.Merge[1] = PRF_P.Zero_Value_o;

    // End of Outputs for SubSystem: '<S2>/Servo Left'
    break;
  }

  // End of SwitchCase: '<S2>/Switch Case'

  // Update for Memory: '<S2>/Memory'
  PRF_DW.Memory_PreviousInput = rtb_SinCos_o2;

  // Update for Memory: '<S2>/Memory2' incorporates:
  //   Constant: '<S5>/Constant'
  //   Constant: '<S6>/Constant'
  //   Logic: '<S2>/OR2'
  //   RelationalOperator: '<S5>/Compare'
  //   RelationalOperator: '<S6>/Compare'

  PRF_DW.Memory2_PreviousInput = ((rtb_Saturation1 ==
    PRF_P.SaturationCheckUp_const) || (rtb_Saturation1 ==
    PRF_P.SaturationCheckLw_const));

  // Update for UnitDelay: '<S2>/Unit Delay'
  PRF_DW.UnitDelay_DSTATE = rtb_Memory;

  // Outport: '<Root>/PRF Logs OBSW' incorporates:
  //   BusCreator generated from: '<S2>/PRFLogs OBSW_BusCreator'

  PRF_Y.PRFLogsOBSW.TargetIndex = 0U;
  PRF_Y.PRFLogsOBSW.Heading = rtb_Heading;
  PRF_Y.PRFLogsOBSW.Reference = rtb_Reference;

  // RateTransition: '<S1>/Rate Transition1' incorporates:
  //   Concatenate: '<S1>/Vector Concatenate'
  //   Inport: '<Root>/PRF Reference'
  //   Outport: '<Root>/PRF Logs OBSW'

  PRF_Y.PRFLogsOBSW.TerminalTarget[2] = PRF_U.PRFReference_a.TargetPositionLLA[0];

  // End of Outputs for SubSystem: '<Root>/PRF'

  // Outport: '<Root>/Servo Commands'
  PRF_Y.ServoCommands[0] = PRF_DW.Merge[0];

  // Outputs for Atomic SubSystem: '<Root>/PRF'
  // Outport: '<Root>/PRF Logs OBSW' incorporates:
  //   Outport: '<Root>/Servo Commands'
  //   SignalConversion generated from: '<S2>/PRFLogs OBSW_BusCreator'

  PRF_Y.PRFLogsOBSW.Q1[0] = 0.0F;
  PRF_Y.PRFLogsOBSW.Q2[0] = 0.0F;

  // End of Outputs for SubSystem: '<Root>/PRF'
  PRF_Y.PRFLogsOBSW.ServoCommands[0] = PRF_DW.Merge[0];

  // Outputs for Atomic SubSystem: '<Root>/PRF'
  // RateTransition: '<S1>/Rate Transition1' incorporates:
  //   Concatenate: '<S1>/Vector Concatenate'
  //   Inport: '<Root>/PRF Reference'
  //   Outport: '<Root>/PRF Logs OBSW'

  PRF_Y.PRFLogsOBSW.TerminalTarget[3] = PRF_U.PRFReference_a.TargetPositionLLA[1];

  // End of Outputs for SubSystem: '<Root>/PRF'

  // Outport: '<Root>/Servo Commands'
  PRF_Y.ServoCommands[1] = PRF_DW.Merge[1];

  // Outputs for Atomic SubSystem: '<Root>/PRF'
  // Outport: '<Root>/PRF Logs OBSW' incorporates:
  //   Outport: '<Root>/Servo Commands'
  //   SignalConversion generated from: '<S2>/PRFLogs OBSW_BusCreator'

  PRF_Y.PRFLogsOBSW.Q1[1] = 0.0F;
  PRF_Y.PRFLogsOBSW.Q2[1] = 0.0F;

  // End of Outputs for SubSystem: '<Root>/PRF'
  PRF_Y.PRFLogsOBSW.ServoCommands[1] = PRF_DW.Merge[1];

  // Outputs for Atomic SubSystem: '<Root>/PRF'
  PRF_Y.PRFLogsOBSW.WindHeading = 0.0F;

  // End of Outputs for SubSystem: '<Root>/PRF'
}

// Model initialize function
void PRF::initialize()
{
  // SystemInitialize for Atomic SubSystem: '<Root>/PRF'
  // InitializeConditions for Memory: '<S2>/Memory'
  PRF_DW.Memory_PreviousInput = PRF_P.Memory_InitialCondition;

  // InitializeConditions for Memory: '<S2>/Memory2'
  PRF_DW.Memory2_PreviousInput = PRF_P.Memory2_InitialCondition;

  // InitializeConditions for UnitDelay: '<S2>/Unit Delay'
  PRF_DW.UnitDelay_DSTATE = PRF_P.UnitDelay_InitialCondition;

  // End of SystemInitialize for SubSystem: '<Root>/PRF'

  // SystemInitialize for Merge: '<S2>/Merge'
  PRF_DW.Merge[0] = PRF_P.Merge_InitialOutput;

  // SystemInitialize for Outport: '<Root>/Servo Commands' incorporates:
  //   Merge: '<S2>/Merge'

  PRF_Y.ServoCommands[0] = PRF_P.Merge_InitialOutput;

  // SystemInitialize for Merge: '<S2>/Merge'
  PRF_DW.Merge[1] = PRF_P.Merge_InitialOutput;

  // SystemInitialize for Outport: '<Root>/Servo Commands' incorporates:
  //   Merge: '<S2>/Merge'

  PRF_Y.ServoCommands[1] = PRF_P.Merge_InitialOutput;
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
