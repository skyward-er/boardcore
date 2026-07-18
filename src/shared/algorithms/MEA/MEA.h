//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// File: MEA.h
//
// Code generated for Simulink model 'MEA'.
//
// Model version                  : 11.238
// Simulink Coder version         : 24.2 (R2024b) 21-Jun-2024
// C/C++ source code generated on : Mon May 11 01:43:03 2026
//
// Target selection: ert.tlc
// Embedded hardware selection: STMicroelectronics->ST10/Super10
// Code generation objectives:
//    1. Execution efficiency
//    2. Debugging
//    3. RAM efficiency
// Validation result: Not run
//
#ifndef MEA_h_
#define MEA_h_
#include <stdbool.h>
#include <stdint.h>
#include "MEA_types.h"
namespace MEA {
// Class declaration for model MEA
class MEA final
{
  // public data and function members
 public:
  // Block signals and states (default storage) for system '<Root>'
  struct DW_MEA_T {
    uint64_t UnitDelay_DSTATE;         // '<S2>/Unit Delay'
    float I[4];                        // '<S4>/IdentityMatrix'
    float PreviousCovariance_DSTATE[4];// '<S1>/Previous Covariance'
    float PreviousState_DSTATE[2];     // '<S1>/Previous State'
    float UnitDelay2_DSTATE;           // '<S1>/Unit Delay2'
  };

  // External inputs (root inport signals with default storage)
  struct ExtU_MEA_T {
    MEAIn MEAIn_o;                     // '<Root>/MEA In'
  };

  // External outputs (root outports fed by signals with default storage)
  struct ExtY_MEA_T {
    MEAOut MEAOut_k;                   // '<Root>/MEA Out'
    MEALogs MEALogsOBSW;               // '<Root>/MEA Logs OBSW'
  };

  // Parameters (default storage)
  struct P_MEA_T {
    float CompareToConstant_const;    // Mask Parameter: CompareToConstant_const
                                         //  Referenced by: '<S5>/Constant'

    uint64_t UnitDelay_InitialCondition;
                               // Computed Parameter: UnitDelay_InitialCondition
                                  //  Referenced by: '<S2>/Unit Delay'

    float IdentityMatrix_IDMatrixData[4];
                              // Computed Parameter: IdentityMatrix_IDMatrixData
                                 //  Referenced by: '<S4>/IdentityMatrix'

    float Gain4_Gain[2];               // Computed Parameter: Gain4_Gain
                                          //  Referenced by: '<S4>/Gain4'

    float Gain1_Gain[2];               // Computed Parameter: Gain1_Gain
                                          //  Referenced by: '<S4>/Gain1'

    float Constant1_Value;             // Computed Parameter: Constant1_Value
                                          //  Referenced by: '<S4>/Constant1'

    float Gain2_Gain[2];               // Computed Parameter: Gain2_Gain
                                          //  Referenced by: '<S4>/Gain2'

    float PreviousCovariance_InitialCondi[4];
                          // Computed Parameter: PreviousCovariance_InitialCondi
                             //  Referenced by: '<S1>/Previous Covariance'

    float Gain2_Gain_l[4];             // Computed Parameter: Gain2_Gain_l
                                          //  Referenced by: '<S3>/Gain2'

    float Gain3_Gain[4];               // Computed Parameter: Gain3_Gain
                                          //  Referenced by: '<S3>/Gain3'

    float Bias_Bias[4];                // Computed Parameter: Bias_Bias
                                          //  Referenced by: '<S3>/Bias'

    float Gain1_Gain_d[2];             // Computed Parameter: Gain1_Gain_d
                                          //  Referenced by: '<S2>/Gain1'

    float Gain4_Gain_k[2];             // Computed Parameter: Gain4_Gain_k
                                          //  Referenced by: '<S2>/Gain4'

    float Bias_Bias_a;                 // Computed Parameter: Bias_Bias_a
                                          //  Referenced by: '<S2>/Bias'

    float PreviousState_InitialCondition[2];
                           // Computed Parameter: PreviousState_InitialCondition
                              //  Referenced by: '<S1>/Previous State'

    float Gain1_Gain_k[4];             // Computed Parameter: Gain1_Gain_k
                                          //  Referenced by: '<S3>/Gain1'

    float UnitDelay2_InitialCondition;
                              // Computed Parameter: UnitDelay2_InitialCondition
                                 //  Referenced by: '<S1>/Unit Delay2'

    float Gain1_Gain_f[2];             // Computed Parameter: Gain1_Gain_f
                                          //  Referenced by: '<S1>/Gain1'

    float Gain2_Gain_n;                // Computed Parameter: Gain2_Gain_n
                                          //  Referenced by: '<S1>/Gain2'

    float Gain3_Gain_c;                // Computed Parameter: Gain3_Gain_c
                                          //  Referenced by: '<S1>/Gain3'

    float Saturation_UpperSat;        // Computed Parameter: Saturation_UpperSat
                                         //  Referenced by: '<S1>/Saturation'

    float Saturation_LowerSat;        // Computed Parameter: Saturation_LowerSat
                                         //  Referenced by: '<S1>/Saturation'

    uint8_t Gain_Gain[2];              // Computed Parameter: Gain_Gain
                                          //  Referenced by: '<S3>/Gain'

  };

  // Copy Constructor
  MEA(MEA const&) = delete;

  // Assignment Operator
  MEA& operator= (MEA const&) & = delete;

  // Move Constructor
  MEA(MEA &&) = delete;

  // Move Assignment Operator
  MEA& operator= (MEA &&) = delete;

  // Root inport: '<Root>/MEA In' set method
  void setMEA_In(MEAIn localArgInput)
  {
    MEA_U.MEAIn_o = localArgInput;
  }

  // Root outport: '<Root>/MEA Out' get method
  MEAOut getMEA_Out() const
  {
    return MEA_Y.MEAOut_k;
  }

  // Root outport: '<Root>/MEA Logs OBSW' get method
  MEALogs getMEA_Logs_OBSW() const
  {
    return MEA_Y.MEALogsOBSW;
  }

  // Block parameters get method
  const P_MEA_T &getBlockParameters() const
  {
    return MEA_P;
  }

  // Block parameters set method
  void setBlockParameters(const P_MEA_T *pP_MEA_T) const
  {
    MEA_P = *pP_MEA_T;
  }

  // model initialize function
  void initialize();

  // model step function
  void step();

  // model terminate function
  static void terminate();

  // Constructor
  MEA();

  // Destructor
  ~MEA();

  // private data and function members
 private:
  // External inputs
  ExtU_MEA_T MEA_U;

  // External outputs
  ExtY_MEA_T MEA_Y;

  // Block states
  DW_MEA_T MEA_DW;

  // Tunable parameters
  static P_MEA_T MEA_P;
};

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
//  hilite_system('CHADsimulator/Control Units/Control Units SIM/Engine Control Unit (Sim)/Mass Estimation/MEA/MEA II - Autocoding')    - opens subsystem CHADsimulator/Control Units/Control Units SIM/Engine Control Unit (Sim)/Mass Estimation/MEA/MEA II - Autocoding
//  hilite_system('CHADsimulator/Control Units/Control Units SIM/Engine Control Unit (Sim)/Mass Estimation/MEA/MEA II - Autocoding/Kp') - opens and selects block Kp
//
//  Here is the system hierarchy for this model
//
//  '<Root>' : 'CHADsimulator/Control Units/Control Units SIM/Engine Control Unit (Sim)/Mass Estimation/MEA'
//  '<S1>'   : 'CHADsimulator/Control Units/Control Units SIM/Engine Control Unit (Sim)/Mass Estimation/MEA/MEA II - Autocoding'
//  '<S2>'   : 'CHADsimulator/Control Units/Control Units SIM/Engine Control Unit (Sim)/Mass Estimation/MEA/MEA II - Autocoding/CC PT Correction Step'
//  '<S3>'   : 'CHADsimulator/Control Units/Control Units SIM/Engine Control Unit (Sim)/Mass Estimation/MEA/MEA II - Autocoding/Prediction Step'
//  '<S4>'   : 'CHADsimulator/Control Units/Control Units SIM/Engine Control Unit (Sim)/Mass Estimation/MEA/MEA II - Autocoding/CC PT Correction Step/Active PT Correction'
//  '<S5>'   : 'CHADsimulator/Control Units/Control Units SIM/Engine Control Unit (Sim)/Mass Estimation/MEA/MEA II - Autocoding/CC PT Correction Step/Compare To Constant'
//  '<S6>'   : 'CHADsimulator/Control Units/Control Units SIM/Engine Control Unit (Sim)/Mass Estimation/MEA/MEA II - Autocoding/CC PT Correction Step/Non Active PT Correction'


//-
//  Requirements for '<Root>': MEA

}
#endif                                 // MEA_h_

//
// File trailer for generated code.
//
// [EOF]
//
