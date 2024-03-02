//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// File: ADA_Algorithm0.h
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
#ifndef RTW_HEADER_ADA_Algorithm0_h_
#define RTW_HEADER_ADA_Algorithm0_h_
#include <stdbool.h>
#include <stdint.h>
#include "ADA_Algorithm0_types.h"

// Class declaration for model ADA_Algorithm0
class ADA_Algorithm0 final
{
  // public data and function members
 public:
  // Block signals and states (default storage) for system '<Root>'
  struct DW_ADA_Algorithm0_T {
    float F_memory_PreviousInput[9];   // '<S1>/F_memory'
    float P_memory_PreviousInput[9];   // '<S1>/P_memory'
    float Q_memory_PreviousInput[9];   // '<S1>/Q_memory'
    float x_memory_PreviousInput[3];   // '<S1>/x_memory'
    float H_memory_PreviousInput[3];   // '<S2>/H_memory'
    float R_memory_PreviousInput;      // '<S2>/R_memory'
    float K_calculation_DWORK4;        // '<S9>/K_calculation'
  };

  // External inputs (root inport signals with default storage)
  struct ExtU_ADA_Algorithm0_T {
    float Pressure;                    // '<Root>/Pressure'
  };

  // External outputs (root outports fed by signals with default storage)
  struct ExtY_ADA_Algorithm0_T {
    ADAState ADAState_d;               // '<Root>/ADAState'
  };

  // Parameters (default storage)
  struct P_ADA_Algorithm0_T {
    float Identity_matrix_Value[9]; // Computed Parameter: Identity_matrix_Value
                                       //  Referenced by: '<S9>/Identity_matrix'

    float H_memory_InitialCondition[3];// Expression: H_INIT
                                          //  Referenced by: '<S2>/H_memory'

    float R_memory_InitialCondition;   // Expression: R_INIT
                                          //  Referenced by: '<S2>/R_memory'

    float F_memory_InitialCondition[9];// Expression: F_INIT
                                          //  Referenced by: '<S1>/F_memory'

    float P_memory_InitialCondition[9];// Expression: P_INIT
                                          //  Referenced by: '<S1>/P_memory'

    float Q_memory_InitialCondition[9];// Expression: Q_INIT
                                          //  Referenced by: '<S1>/Q_memory'

    float x_memory_InitialCondition[3];// Expression: x_INIT
                                          //  Referenced by: '<S1>/x_memory'

    float fake_timestamp_Value;      // Computed Parameter: fake_timestamp_Value
                                        //  Referenced by: '<S1>/fake_timestamp'

    float T_ref_Value;                 // Expression: T_ref
                                          //  Referenced by: '<S6>/T_ref'

    float a_Value;                     // Computed Parameter: a_Value
                                          //  Referenced by: '<S6>/a'

    float Constant_Value;              // Computed Parameter: Constant_Value
                                          //  Referenced by: '<S6>/Constant'

    float P_ref_Value;                 // Expression: P_ref
                                          //  Referenced by: '<S6>/P_ref'

    float nInv_Value;                  // Computed Parameter: nInv_Value
                                          //  Referenced by: '<S6>/nInv'

    float z0_ref_Value;                // Expression: z0_ref
                                          //  Referenced by: '<S5>/z0_ref'

    float T_ref_Value_d;               // Expression: T_ref
                                          //  Referenced by: '<S7>/T_ref'

    float P_ref_Value_j;               // Expression: P_ref
                                          //  Referenced by: '<S7>/P_ref'

    float nInv_Value_g;                // Computed Parameter: nInv_Value_g
                                          //  Referenced by: '<S7>/nInv'

    float Gain_Gain;                   // Computed Parameter: Gain_Gain
                                          //  Referenced by: '<S7>/Gain'

    float a_Value_e;                   // Computed Parameter: a_Value_e
                                          //  Referenced by: '<S7>/a'

  };

  // Copy Constructor
  ADA_Algorithm0(ADA_Algorithm0 const&) = delete;

  // Assignment Operator
  ADA_Algorithm0& operator= (ADA_Algorithm0 const&) & = delete;

  // Move Constructor
  ADA_Algorithm0(ADA_Algorithm0 &&) = delete;

  // Move Assignment Operator
  ADA_Algorithm0& operator= (ADA_Algorithm0 &&) = delete;

  // Root inports set method
  void setExternalInputs(const ExtU_ADA_Algorithm0_T *pExtU_ADA_Algorithm0_T)
  {
    ADA_Algorithm0_U = *pExtU_ADA_Algorithm0_T;
  }

  // Root outports get method
  const ExtY_ADA_Algorithm0_T &getExternalOutputs() const
  {
    return ADA_Algorithm0_Y;
  }

  // model initialize function
  void initialize();

  // model step function
  void step();

  // model terminate function
  static void terminate();

  // Constructor
  ADA_Algorithm0();

  // Destructor
  ~ADA_Algorithm0();

  // private data and function members
 private:
  // External inputs
  ExtU_ADA_Algorithm0_T ADA_Algorithm0_U;

  // External outputs
  ExtY_ADA_Algorithm0_T ADA_Algorithm0_Y;

  // Block states
  DW_ADA_Algorithm0_T ADA_Algorithm0_DW;

  // Tunable parameters
  static P_ADA_Algorithm0_T ADA_Algorithm0_P;

  // private member function(s) for subsystem '<S8>/No correction'
  static void ADA_Algorithm0_Nocorrection(const float rtu_x_pred[3], const float
    rtu_P_pred[9], float rty_x_corr[3], float rty_P_corr[9]);
};

//-
//  These blocks were eliminated from the model due to optimizations:
//
//  Block '<S9>/Reshape1' : Reshape block reduction


//-
//  The generated code includes comments that allow you to trace directly
//  back to the appropriate location in the model.  The basic format
//  is <system>/block_name, where system is the system number (uniquely
//  assigned by Simulink) and block_name is the name of the block.
//
//  Note that this particular code originates from a subsystem build,
//  and has its own system numbers different from the parent model.
//  Refer to the system hierarchy for this subsystem below, and use the
//  MATLAB hilite_system command to trace the generated code back
//  to the parent model.  For example,
//
//  hilite_system('complete_ADA_model/ADA_Algorithm')    - opens subsystem complete_ADA_model/ADA_Algorithm
//  hilite_system('complete_ADA_model/ADA_Algorithm/Kp') - opens and selects block Kp
//
//  Here is the system hierarchy for this model
//
//  '<Root>' : 'complete_ADA_model'
//  '<S1>'   : 'complete_ADA_model/ADA_Algorithm'
//  '<S2>'   : 'complete_ADA_model/ADA_Algorithm/Correction'
//  '<S3>'   : 'complete_ADA_model/ADA_Algorithm/No_correction'
//  '<S4>'   : 'complete_ADA_model/ADA_Algorithm/Prediction'
//  '<S5>'   : 'complete_ADA_model/ADA_Algorithm/aglAltitude'
//  '<S6>'   : 'complete_ADA_model/ADA_Algorithm/mslAltitude'
//  '<S7>'   : 'complete_ADA_model/ADA_Algorithm/verticalSpeed'
//  '<S8>'   : 'complete_ADA_model/ADA_Algorithm/Correction/Correction'
//  '<S9>'   : 'complete_ADA_model/ADA_Algorithm/Correction/Correction/Correction'
//  '<S10>'  : 'complete_ADA_model/ADA_Algorithm/Correction/Correction/No correction'

#endif                                 // RTW_HEADER_ADA_Algorithm0_h_

//
// File trailer for generated code.
//
// [EOF]
//
