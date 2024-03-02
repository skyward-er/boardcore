//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// File: ADA_Algorithm0.cpp
//
// Code generated for Simulink model 'ADA_Algorithm0'.
//
// Model version                  : 1.11
// Simulink Coder version         : 23.2 (R2023b) 01-Aug-2023
// C/C++ source code generated on : Wed Nov  1 00:43:29 2023
//
// Target selection: ert.tlc
// Embedded hardware selection: STMicroelectronics->ST10/Super10
// Code generation objectives:
//    1. Execution efficiency
//    2. Safety precaution
// Validation result: Passed (20), Warning (1), Error (0)
//
#include "ADA_Algorithm0.h"
#include <stdint.h>
#include <cmath>

//
// Output and update for action system:
//    '<S8>/No correction'
//    '<S1>/No_correction'
//
void ADA_Algorithm0::ADA_Algorithm0_Nocorrection(const float rtu_x_pred[3],
  const float rtu_P_pred[9], float rty_x_corr[3], float rty_P_corr[9])
{
  // SignalConversion generated from: '<S10>/x_pred'
  rty_x_corr[0] = rtu_x_pred[0];
  rty_x_corr[1] = rtu_x_pred[1];
  rty_x_corr[2] = rtu_x_pred[2];

  // SignalConversion generated from: '<S10>/P_pred'
  for (int16_t i{0}; i < 9; i++) {
    rty_P_corr[i] = rtu_P_pred[i];
  }

  // End of SignalConversion generated from: '<S10>/P_pred'
}

// Model step function
void ADA_Algorithm0::step()
{
  float rtb_P_pred[9];
  float tmp[9];
  float rtb_K[3];
  float rtb_x_pred[3];
  float H_memory_PreviousInput;
  float P_memory_PreviousInput;
  float P_memory_PreviousInput_0;
  float rtb_S;
  int16_t i;
  int16_t i_0;
  int16_t rtb_P_pred_tmp;

  // Outputs for Atomic SubSystem: '<Root>/ADA_Algorithm'
  // Product: '<S4>/Matrix Multiply1' incorporates:
  //   Math: '<S4>/Transpose'
  //   Memory: '<S1>/F_memory'
  //   Memory: '<S1>/P_memory'

  for (i = 0; i < 3; i++) {
    P_memory_PreviousInput = ADA_Algorithm0_DW.P_memory_PreviousInput[i + 3];
    P_memory_PreviousInput_0 = ADA_Algorithm0_DW.P_memory_PreviousInput[i];
    rtb_S = ADA_Algorithm0_DW.P_memory_PreviousInput[i + 6];
    for (i_0 = 0; i_0 < 3; i_0++) {
      tmp[i + 3 * i_0] = (ADA_Algorithm0_DW.F_memory_PreviousInput[i_0 + 3] *
                          P_memory_PreviousInput + P_memory_PreviousInput_0 *
                          ADA_Algorithm0_DW.F_memory_PreviousInput[i_0]) +
        ADA_Algorithm0_DW.F_memory_PreviousInput[i_0 + 6] * rtb_S;
    }
  }

  for (i = 0; i < 3; i++) {
    // Product: '<S4>/Matrix Multiply'
    P_memory_PreviousInput = 0.0F;
    for (i_0 = 0; i_0 < 3; i_0++) {
      // Sum: '<S4>/Add' incorporates:
      //   Memory: '<S1>/F_memory'
      //   Memory: '<S1>/Q_memory'
      //   Product: '<S4>/Matrix Multiply'
      //   Product: '<S4>/Matrix Multiply1'

      rtb_P_pred_tmp = 3 * i_0 + i;
      rtb_P_pred[rtb_P_pred_tmp] = ((tmp[3 * i_0 + 1] *
        ADA_Algorithm0_DW.F_memory_PreviousInput[i + 3] + tmp[3 * i_0] *
        ADA_Algorithm0_DW.F_memory_PreviousInput[i]) + tmp[3 * i_0 + 2] *
        ADA_Algorithm0_DW.F_memory_PreviousInput[i + 6]) +
        ADA_Algorithm0_DW.Q_memory_PreviousInput[rtb_P_pred_tmp];

      // Product: '<S4>/Matrix Multiply' incorporates:
      //   Memory: '<S1>/F_memory'
      //   Memory: '<S1>/x_memory'

      P_memory_PreviousInput +=
        ADA_Algorithm0_DW.F_memory_PreviousInput[rtb_P_pred_tmp] *
        ADA_Algorithm0_DW.x_memory_PreviousInput[i_0];
    }

    // Product: '<S4>/Matrix Multiply'
    rtb_x_pred[i] = P_memory_PreviousInput;
  }

  // If: '<S1>/Check_pressure' incorporates:
  //   Inport: '<Root>/Pressure'

  if (ADA_Algorithm0_U.Pressure > 0.0F) {
    // Outputs for IfAction SubSystem: '<S1>/Correction' incorporates:
    //   ActionPort: '<S2>/Action Port'

    // Product: '<S8>/Matrix Multiply' incorporates:
    //   Memory: '<S2>/H_memory'
    //   Sum: '<S4>/Add'

    P_memory_PreviousInput = 0.0F;
    for (i = 0; i < 3; i++) {
      P_memory_PreviousInput += ((rtb_P_pred[i + 3] *
        ADA_Algorithm0_DW.H_memory_PreviousInput[1] + rtb_P_pred[i] *
        ADA_Algorithm0_DW.H_memory_PreviousInput[0]) + rtb_P_pred[i + 6] *
        ADA_Algorithm0_DW.H_memory_PreviousInput[2]) *
        ADA_Algorithm0_DW.H_memory_PreviousInput[i];
    }

    // Sum: '<S8>/Add' incorporates:
    //   Memory: '<S2>/R_memory'
    //   Product: '<S8>/Matrix Multiply'

    rtb_S = ADA_Algorithm0_DW.R_memory_PreviousInput + P_memory_PreviousInput;

    // If: '<S8>/If'
    if (rtb_S > 0.001F) {
      // Outputs for IfAction SubSystem: '<S8>/Correction' incorporates:
      //   ActionPort: '<S9>/Action Port'

      // Product: '<S9>/K_calculation' incorporates:
      //   Memory: '<S2>/H_memory'
      //   Sum: '<S4>/Add'

      P_memory_PreviousInput = ADA_Algorithm0_DW.H_memory_PreviousInput[0] /
        rtb_S;
      P_memory_PreviousInput_0 = ADA_Algorithm0_DW.H_memory_PreviousInput[1] /
        rtb_S;
      rtb_S = ADA_Algorithm0_DW.H_memory_PreviousInput[2] / rtb_S;
      for (i = 0; i < 3; i++) {
        rtb_K[i] = (rtb_P_pred[i + 3] * P_memory_PreviousInput_0 + rtb_P_pred[i]
                    * P_memory_PreviousInput) + rtb_P_pred[i + 6] * rtb_S;
      }

      // End of Product: '<S9>/K_calculation'

      // Product: '<S9>/Product' incorporates:
      //   Product: '<S9>/K_calculation'

      P_memory_PreviousInput = rtb_K[0];
      P_memory_PreviousInput_0 = rtb_K[1];
      rtb_S = rtb_K[2];

      // Sum: '<S9>/Add' incorporates:
      //   Constant: '<S9>/Identity_matrix'
      //   Memory: '<S2>/H_memory'
      //   Product: '<S9>/K_calculation'
      //   Product: '<S9>/Product'

      for (i = 0; i < 3; i++) {
        // Memory: '<S2>/H_memory' incorporates:
        //   Product: '<S9>/Product'

        H_memory_PreviousInput = ADA_Algorithm0_DW.H_memory_PreviousInput[i];
        tmp[3 * i] = ADA_Algorithm0_P.Identity_matrix_Value[3 * i] -
          P_memory_PreviousInput * H_memory_PreviousInput;
        i_0 = 3 * i + 1;
        tmp[i_0] = ADA_Algorithm0_P.Identity_matrix_Value[i_0] -
          P_memory_PreviousInput_0 * H_memory_PreviousInput;
        i_0 = 3 * i + 2;
        tmp[i_0] = ADA_Algorithm0_P.Identity_matrix_Value[i_0] - rtb_S *
          H_memory_PreviousInput;
      }

      // End of Sum: '<S9>/Add'

      // Product: '<S9>/Product2'
      P_memory_PreviousInput = 0.0F;
      for (i = 0; i < 3; i++) {
        // Product: '<S9>/P_update'
        P_memory_PreviousInput_0 = tmp[i + 3];
        rtb_S = tmp[i];
        H_memory_PreviousInput = tmp[i + 6];
        for (i_0 = 0; i_0 < 3; i_0++) {
          // Product: '<S9>/P_update' incorporates:
          //   Memory: '<S1>/P_memory'
          //   Sum: '<S4>/Add'

          ADA_Algorithm0_DW.P_memory_PreviousInput[i + 3 * i_0] = (rtb_P_pred[3 *
            i_0 + 1] * P_memory_PreviousInput_0 + rtb_P_pred[3 * i_0] * rtb_S) +
            rtb_P_pred[3 * i_0 + 2] * H_memory_PreviousInput;
        }

        // Product: '<S9>/Product2' incorporates:
        //   Memory: '<S2>/H_memory'
        //   Product: '<S4>/Matrix Multiply'

        P_memory_PreviousInput += ADA_Algorithm0_DW.H_memory_PreviousInput[i] *
          rtb_x_pred[i];
      }

      // Sum: '<S9>/Add2' incorporates:
      //   Product: '<S9>/Product2'

      rtb_S = ADA_Algorithm0_U.Pressure - P_memory_PreviousInput;

      // Sum: '<S9>/x_update' incorporates:
      //   Merge: '<S1>/Merge1'
      //   Product: '<S4>/Matrix Multiply'
      //   Product: '<S9>/K_calculation'
      //   Product: '<S9>/Product1'

      rtb_K[0] = rtb_K[0] * rtb_S + rtb_x_pred[0];
      rtb_K[1] = rtb_K[1] * rtb_S + rtb_x_pred[1];
      rtb_K[2] = rtb_K[2] * rtb_S + rtb_x_pred[2];

      // End of Outputs for SubSystem: '<S8>/Correction'
    } else {
      // Outputs for IfAction SubSystem: '<S8>/No correction' incorporates:
      //   ActionPort: '<S10>/Action Port'

      // Update for Memory: '<S1>/P_memory'
      ADA_Algorithm0_Nocorrection(rtb_x_pred, rtb_P_pred, rtb_K,
        ADA_Algorithm0_DW.P_memory_PreviousInput);

      // End of Outputs for SubSystem: '<S8>/No correction'
    }

    // End of If: '<S8>/If'
    // End of Outputs for SubSystem: '<S1>/Correction'
  } else {
    // Outputs for IfAction SubSystem: '<S1>/No_correction' incorporates:
    //   ActionPort: '<S3>/Action Port'

    // Update for Memory: '<S1>/P_memory'
    ADA_Algorithm0_Nocorrection(rtb_x_pred, rtb_P_pred, rtb_K,
      ADA_Algorithm0_DW.P_memory_PreviousInput);

    // End of Outputs for SubSystem: '<S1>/No_correction'
  }

  // End of If: '<S1>/Check_pressure'

  // Product: '<S6>/Product1' incorporates:
  //   Constant: '<S6>/Constant'
  //   Constant: '<S6>/P_ref'
  //   Constant: '<S6>/T_ref'
  //   Constant: '<S6>/a'
  //   Constant: '<S6>/nInv'
  //   Math: '<S6>/pow'
  //   Product: '<S6>/Product'
  //   Product: '<S6>/Product2'
  //   Sum: '<S6>/Add'

  rtb_S = (ADA_Algorithm0_P.Constant_Value - std::pow(rtb_K[0] /
            ADA_Algorithm0_P.P_ref_Value, ADA_Algorithm0_P.nInv_Value)) *
    (ADA_Algorithm0_P.T_ref_Value / ADA_Algorithm0_P.a_Value);

  // Sum: '<S5>/Add' incorporates:
  //   Constant: '<S5>/z0_ref'

  ADA_Algorithm0_Y.ADAState_d.aglAltitude = rtb_S -
    ADA_Algorithm0_P.z0_ref_Value;

  // BusCreator: '<S1>/Bus Creator' incorporates:
  //   Constant: '<S1>/fake_timestamp'
  //   Constant: '<S7>/P_ref'
  //   Constant: '<S7>/T_ref'
  //   Constant: '<S7>/a'
  //   Constant: '<S7>/nInv'
  //   Gain: '<S7>/Gain'
  //   Math: '<S7>/pow'
  //   Outport: '<Root>/ADAState'
  //   Product: '<S7>/Product'
  //   Product: '<S7>/Product1'
  //   Product: '<S7>/Product2'
  //   Product: '<S7>/Product4'

  ADA_Algorithm0_Y.ADAState_d.timestamp = ADA_Algorithm0_P.fake_timestamp_Value;
  ADA_Algorithm0_Y.ADAState_d.mslAltitude = rtb_S;
  ADA_Algorithm0_Y.ADAState_d.verticalSpeed = ADA_Algorithm0_P.T_ref_Value_d *
    rtb_K[1] * std::pow(rtb_K[0] / ADA_Algorithm0_P.P_ref_Value_j,
                        ADA_Algorithm0_P.nInv_Value_g) *
    ADA_Algorithm0_P.Gain_Gain / (ADA_Algorithm0_P.a_Value_e /
    ADA_Algorithm0_P.nInv_Value_g * rtb_K[0]);
  ADA_Algorithm0_Y.ADAState_d.x0 = rtb_K[0];
  ADA_Algorithm0_Y.ADAState_d.x1 = rtb_K[1];
  ADA_Algorithm0_Y.ADAState_d.x2 = rtb_K[2];

  // Update for Memory: '<S1>/x_memory' incorporates:
  //   Merge: '<S1>/Merge1'

  ADA_Algorithm0_DW.x_memory_PreviousInput[0] = rtb_K[0];
  ADA_Algorithm0_DW.x_memory_PreviousInput[1] = rtb_K[1];
  ADA_Algorithm0_DW.x_memory_PreviousInput[2] = rtb_K[2];

  // End of Outputs for SubSystem: '<Root>/ADA_Algorithm'
}

// Model initialize function
void ADA_Algorithm0::initialize()
{
  {
    int16_t i;

    // SystemInitialize for Atomic SubSystem: '<Root>/ADA_Algorithm'
    for (i = 0; i < 9; i++) {
      // InitializeConditions for Memory: '<S1>/F_memory'
      ADA_Algorithm0_DW.F_memory_PreviousInput[i] =
        ADA_Algorithm0_P.F_memory_InitialCondition[i];

      // InitializeConditions for Memory: '<S1>/P_memory'
      ADA_Algorithm0_DW.P_memory_PreviousInput[i] =
        ADA_Algorithm0_P.P_memory_InitialCondition[i];

      // InitializeConditions for Memory: '<S1>/Q_memory'
      ADA_Algorithm0_DW.Q_memory_PreviousInput[i] =
        ADA_Algorithm0_P.Q_memory_InitialCondition[i];
    }

    // InitializeConditions for Memory: '<S1>/x_memory'
    ADA_Algorithm0_DW.x_memory_PreviousInput[0] =
      ADA_Algorithm0_P.x_memory_InitialCondition[0];

    // SystemInitialize for IfAction SubSystem: '<S1>/Correction'
    // InitializeConditions for Memory: '<S2>/H_memory'
    ADA_Algorithm0_DW.H_memory_PreviousInput[0] =
      ADA_Algorithm0_P.H_memory_InitialCondition[0];

    // End of SystemInitialize for SubSystem: '<S1>/Correction'

    // InitializeConditions for Memory: '<S1>/x_memory'
    ADA_Algorithm0_DW.x_memory_PreviousInput[1] =
      ADA_Algorithm0_P.x_memory_InitialCondition[1];

    // SystemInitialize for IfAction SubSystem: '<S1>/Correction'
    // InitializeConditions for Memory: '<S2>/H_memory'
    ADA_Algorithm0_DW.H_memory_PreviousInput[1] =
      ADA_Algorithm0_P.H_memory_InitialCondition[1];

    // End of SystemInitialize for SubSystem: '<S1>/Correction'

    // InitializeConditions for Memory: '<S1>/x_memory'
    ADA_Algorithm0_DW.x_memory_PreviousInput[2] =
      ADA_Algorithm0_P.x_memory_InitialCondition[2];

    // SystemInitialize for IfAction SubSystem: '<S1>/Correction'
    // InitializeConditions for Memory: '<S2>/H_memory'
    ADA_Algorithm0_DW.H_memory_PreviousInput[2] =
      ADA_Algorithm0_P.H_memory_InitialCondition[2];

    // InitializeConditions for Memory: '<S2>/R_memory'
    ADA_Algorithm0_DW.R_memory_PreviousInput =
      ADA_Algorithm0_P.R_memory_InitialCondition;

    // End of SystemInitialize for SubSystem: '<S1>/Correction'
    // End of SystemInitialize for SubSystem: '<Root>/ADA_Algorithm'
  }
}

// Model terminate function
void ADA_Algorithm0::terminate()
{
  // (no terminate code required)
}

// Constructor
ADA_Algorithm0::ADA_Algorithm0():
  ADA_Algorithm0_U(),
  ADA_Algorithm0_Y(),
  ADA_Algorithm0_DW()
{
  // Currently there is no constructor body generated.
}

// Destructor
// Currently there is no destructor body generated.
ADA_Algorithm0::~ADA_Algorithm0() = default;

//
// File trailer for generated code.
//
// [EOF]
//
