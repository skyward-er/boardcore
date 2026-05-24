//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// File: ANAS0.h
//
// Code generated for Simulink model 'ANAS0'.
//
// Model version                  : 11.183
// Simulink Coder version         : 24.2 (R2024b) 21-Jun-2024
// C/C++ source code generated on : Thu Apr 16 13:46:31 2026
//
// Target selection: ert.tlc
// Embedded hardware selection: STMicroelectronics->ST10/Super10
// Code generation objectives:
//    1. Execution efficiency
//    2. Debugging
//    3. RAM efficiency
// Validation result: Passed (3), Warning (1), Error (0)
//
#ifndef ANAS0_h_
#define ANAS0_h_
#include <stdbool.h>
#include <stdint.h>
#include "ANAS0_types.h"

// Class declaration for model ANAS0
class ANAS0 final
{
  // public data and function members
 public:
  // External inputs (root inport signals with default storage)
  struct ExtU_ANAS0_T {
    NASIn NASIn_j;                     // '<Root>/NASIn'
  };

  // External outputs (root outports fed by signals with default storage)
  struct ExtY_ANAS0_T {
    NASOut NASOut_m;                   // '<Root>/NASOut'
    NASLogs NASLogs_c;                 // '<Root>/NASLogs'
    NASFinal NASFinal_n;               // '<Root>/NASFinal'
  };

  // Parameters (default storage)
  struct P_ANAS0_T {
    NASFinal Zero2_Value;              // Computed Parameter: Zero2_Value
                                          //  Referenced by: '<S1>/Zero2'

    NASLogs Zero1_Value;               // Computed Parameter: Zero1_Value
                                          //  Referenced by: '<S1>/Zero1'

    NASOut Zero_Value;                 // Computed Parameter: Zero_Value
                                          //  Referenced by: '<S1>/Zero'

  };

  // Copy Constructor
  ANAS0(ANAS0 const&) = delete;

  // Assignment Operator
  ANAS0& operator= (ANAS0 const&) & = delete;

  // Move Constructor
  ANAS0(ANAS0 &&) = delete;

  // Move Assignment Operator
  ANAS0& operator= (ANAS0 &&) = delete;

  // Root inport: '<Root>/NASIn' set method
  void setNASIn(NASIn localArgInput)
  {
    ANAS0_U.NASIn_j = localArgInput;
  }

  // Root outport: '<Root>/NASOut' get method
  NASOut getNASOut() const
  {
    return ANAS0_Y.NASOut_m;
  }

  // Root outport: '<Root>/NASLogs' get method
  NASLogs getNASLogs() const
  {
    return ANAS0_Y.NASLogs_c;
  }

  // Root outport: '<Root>/NASFinal' get method
  NASFinal getNASFinal() const
  {
    return ANAS0_Y.NASFinal_n;
  }

  // Block parameters get method
  const P_ANAS0_T &getBlockParameters() const
  {
    return ANAS0_P;
  }

  // Block parameters set method
  void setBlockParameters(const P_ANAS0_T *pP_ANAS0_T) const
  {
    ANAS0_P = *pP_ANAS0_T;
  }

  // model initialize function
  static void initialize();

  // model step function
  void step();

  // model terminate function
  static void terminate();

  // Constructor
  ANAS0();

  // Destructor
  ~ANAS0();

  // private data and function members
 private:
  // External inputs
  ExtU_ANAS0_T ANAS0_U;

  // External outputs
  ExtY_ANAS0_T ANAS0_Y;

  // Tunable parameters
  static P_ANAS0_T ANAS0_P;
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
//  hilite_system('CHADsimulator/Control Units/Control Units SIM/Main Control Unit (Sim)/NAS/ANAS')    - opens subsystem CHADsimulator/Control Units/Control Units SIM/Main Control Unit (Sim)/NAS/ANAS
//  hilite_system('CHADsimulator/Control Units/Control Units SIM/Main Control Unit (Sim)/NAS/ANAS/Kp') - opens and selects block Kp
//
//  Here is the system hierarchy for this model
//
//  '<Root>' : 'CHADsimulator/Control Units/Control Units SIM/Main Control Unit (Sim)/NAS'
//  '<S1>'   : 'CHADsimulator/Control Units/Control Units SIM/Main Control Unit (Sim)/NAS/ANAS'


//-
//  Requirements for '<Root>': ANAS0


#endif                                 // ANAS0_h_

//
// File trailer for generated code.
//
// [EOF]
//
