//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// File: PRF.h
//
// Code generated for Simulink model 'PRF'.
//
// Model version                  : 11.370
// Simulink Coder version         : 24.2 (R2024b) 21-Jun-2024
// C/C++ source code generated on : Tue Aug 18 23:37:33 2026
//
// Target selection: ert.tlc
// Embedded hardware selection: ARM Compatible->ARM Cortex-M
// Code generation objectives:
//    1. Execution efficiency
//    2. Debugging
//    3. RAM efficiency
// Validation result: Not run
//
#ifndef PRF_h_
#define PRF_h_
#include <stdbool.h>
#include <stdint.h>
#include "PRF_types.h"

namespace PRF 
{


// Class declaration for model PRF
class PRF final
{
  // public data and function members
 public:
  // Block signals and states (default storage) for system '<Root>'
  struct DW_PRF_T {
    float Merge1[2];                   // '<S2>/Merge1'
    float UnitDelay3_DSTATE;           // '<S2>/Unit Delay3'
    float UnitDelay1_DSTATE;           // '<S2>/Unit Delay1'
    bool UnitDelay2_DSTATE;            // '<S2>/Unit Delay2'
  };

  // External inputs (root inport signals with default storage)
  struct ExtU_PRF_T {
    PRFIn PRFIn_o;                     // '<Root>/PRF In'
    PRFReference PRFReference_m;       // '<Root>/PRF Reference'
  };

  // External outputs (root outports fed by signals with default storage)
  struct ExtY_PRF_T {
    float ServoCommands[2];            // '<Root>/Servo Commands'
    PRFLogs PRFLogsOBSW;               // '<Root>/PRF Logs OBSW'
  };

  // Parameters (default storage)
  struct P_PRF_T {
    float SaturationCheckUp1_const;  // Mask Parameter: SaturationCheckUp1_const
                                        //  Referenced by: '<S6>/Constant'

    float SaturationCheckLw1_const;  // Mask Parameter: SaturationCheckLw1_const
                                        //  Referenced by: '<S5>/Constant'

    float Zero_Value;                  // Computed Parameter: Zero_Value
                                          //  Referenced by: '<S8>/Zero'

    float Zero_Value_a[2];             // Computed Parameter: Zero_Value_a
                                          //  Referenced by: '<S4>/Zero'

    float Zero_Value_d;                // Computed Parameter: Zero_Value_d
                                          //  Referenced by: '<S7>/Zero'

    float Constant_Value;              // Computed Parameter: Constant_Value
                                          //  Referenced by: '<S2>/Constant'

    float Constant10_Value;            // Computed Parameter: Constant10_Value
                                          //  Referenced by: '<S2>/Constant10'

    float Constant9_Value;             // Computed Parameter: Constant9_Value
                                          //  Referenced by: '<S2>/Constant9'

    float Constant1_Value;             // Computed Parameter: Constant1_Value
                                          //  Referenced by: '<S9>/Constant1'

    float Constant_Value_j;            // Computed Parameter: Constant_Value_j
                                          //  Referenced by: '<S9>/Constant'

    float Bias1_Bias;                  // Computed Parameter: Bias1_Bias
                                          //  Referenced by: '<S2>/Bias1'

    float Bias_Bias;                   // Computed Parameter: Bias_Bias
                                          //  Referenced by: '<S2>/Bias'

    float Constant_Value_b;            // Computed Parameter: Constant_Value_b
                                          //  Referenced by: '<S16>/Constant'

    float Constant3_Value;             // Computed Parameter: Constant3_Value
                                          //  Referenced by: '<S14>/Constant3'

    float Constant2_Value;             // Computed Parameter: Constant2_Value
                                          //  Referenced by: '<S14>/Constant2'

    float f_Value;                     // Computed Parameter: f_Value
                                          //  Referenced by: '<S14>/f'

    float Constant1_Value_p;           // Computed Parameter: Constant1_Value_p
                                          //  Referenced by: '<S14>/Constant1'

    float Constant_Value_n;            // Computed Parameter: Constant_Value_n
                                          //  Referenced by: '<S14>/Constant'

    float Zero_Value_c;                // Computed Parameter: Zero_Value_c
                                          //  Referenced by: '<S12>/Zero'

    float Constant_Value_o;            // Computed Parameter: Constant_Value_o
                                          //  Referenced by: '<S10>/Constant'

    float Constant_Value_e;            // Computed Parameter: Constant_Value_e
                                          //  Referenced by: '<S11>/Constant'

    float Constant1_Value_c[2];        // Computed Parameter: Constant1_Value_c
                                          //  Referenced by: '<S1>/Constant1'

    float Switch_Threshold;            // Computed Parameter: Switch_Threshold
                                          //  Referenced by: '<S2>/Switch'

    float UnitDelay3_InitialCondition; // Expression: single(0)
                                          //  Referenced by: '<S2>/Unit Delay3'

    float Constant8_Value;             // Computed Parameter: Constant8_Value
                                          //  Referenced by: '<S2>/Constant8'

    float UnitDelay1_InitialCondition; // Expression: single(0)
                                          //  Referenced by: '<S2>/Unit Delay1'

    float Constant7_Value;             // Computed Parameter: Constant7_Value
                                          //  Referenced by: '<S2>/Constant7'

    float Constant6_Value;             // Computed Parameter: Constant6_Value
                                          //  Referenced by: '<S2>/Constant6'

    float Saturation2_UpperSat;        // Expression: single(upSatTE)
                                          //  Referenced by: '<S2>/Saturation2'

    float Saturation2_LowerSat;        // Expression: single(lwSatTE)
                                          //  Referenced by: '<S2>/Saturation2'

    float Merge1_InitialOutput;      // Computed Parameter: Merge1_InitialOutput
                                        //  Referenced by: '<S2>/Merge1'

    bool UnitDelay2_InitialCondition;
                              // Computed Parameter: UnitDelay2_InitialCondition
                                 //  Referenced by: '<S2>/Unit Delay2'

  };

  // Copy Constructor
  PRF(PRF const&) = delete;

  // Assignment Operator
  PRF& operator= (PRF const&) & = delete;

  // Move Constructor
  PRF(PRF &&) = delete;

  // Move Assignment Operator
  PRF& operator= (PRF &&) = delete;

  // Root inport: '<Root>/PRF In' set method
  void setPRF_In(PRFIn localArgInput)
  {
    PRF_U.PRFIn_o = localArgInput;
  }

  // Root inport: '<Root>/PRF Reference' set method
  void setPRF_Reference(PRFReference localArgInput)
  {
    PRF_U.PRFReference_m = localArgInput;
  }

  // Root outport: '<Root>/Servo Commands' get method
  const float *getServo_Commands() const
  {
    return PRF_Y.ServoCommands;
  }

  // Root outport: '<Root>/PRF Logs OBSW' get method
  PRFLogs getPRF_Logs_OBSW() const
  {
    return PRF_Y.PRFLogsOBSW;
  }

  // Block parameters get method
  const P_PRF_T &getBlockParameters() const
  {
    return PRF_P;
  }

  // Block parameters set method
  void setBlockParameters(const P_PRF_T *pP_PRF_T) const
  {
    PRF_P = *pP_PRF_T;
  }

  // model initialize function
  void initialize();

  // model step function
  void step();

  // model terminate function
  static void terminate();

  // Constructor
  PRF();

  // Destructor
  ~PRF();

  // private data and function members
 private:
  // External inputs
  ExtU_PRF_T PRF_U;

  // External outputs
  ExtY_PRF_T PRF_Y;

  // Block states
  DW_PRF_T PRF_DW;

  // Tunable parameters
  static P_PRF_T PRF_P;
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
//  hilite_system('CHADsimulator/Control Units/Control Units SIM/Main Control Unit (Sim)/Parafoil Control/Parafoil/PRF')    - opens subsystem CHADsimulator/Control Units/Control Units SIM/Main Control Unit (Sim)/Parafoil Control/Parafoil/PRF
//  hilite_system('CHADsimulator/Control Units/Control Units SIM/Main Control Unit (Sim)/Parafoil Control/Parafoil/PRF/Kp') - opens and selects block Kp
//
//  Here is the system hierarchy for this model
//
//  '<Root>' : 'CHADsimulator/Control Units/Control Units SIM/Main Control Unit (Sim)/Parafoil Control/Parafoil'
//  '<S1>'   : 'CHADsimulator/Control Units/Control Units SIM/Main Control Unit (Sim)/Parafoil Control/Parafoil/PRF'
//  '<S2>'   : 'CHADsimulator/Control Units/Control Units SIM/Main Control Unit (Sim)/Parafoil Control/Parafoil/PRF/Control'
//  '<S3>'   : 'CHADsimulator/Control Units/Control Units SIM/Main Control Unit (Sim)/Parafoil Control/Parafoil/PRF/LLA to NED'
//  '<S4>'   : 'CHADsimulator/Control Units/Control Units SIM/Main Control Unit (Sim)/Parafoil Control/Parafoil/PRF/Control/No Activation1'
//  '<S5>'   : 'CHADsimulator/Control Units/Control Units SIM/Main Control Unit (Sim)/Parafoil Control/Parafoil/PRF/Control/Saturation Check Lw1'
//  '<S6>'   : 'CHADsimulator/Control Units/Control Units SIM/Main Control Unit (Sim)/Parafoil Control/Parafoil/PRF/Control/Saturation Check Up1'
//  '<S7>'   : 'CHADsimulator/Control Units/Control Units SIM/Main Control Unit (Sim)/Parafoil Control/Parafoil/PRF/Control/Servo Left1'
//  '<S8>'   : 'CHADsimulator/Control Units/Control Units SIM/Main Control Unit (Sim)/Parafoil Control/Parafoil/PRF/Control/Servo Right1'
//  '<S9>'   : 'CHADsimulator/Control Units/Control Units SIM/Main Control Unit (Sim)/Parafoil Control/Parafoil/PRF/Control/Subsystem Reference'
//  '<S10>'  : 'CHADsimulator/Control Units/Control Units SIM/Main Control Unit (Sim)/Parafoil Control/Parafoil/PRF/Control/Subsystem Reference/IsPositive'
//  '<S11>'  : 'CHADsimulator/Control Units/Control Units SIM/Main Control Unit (Sim)/Parafoil Control/Parafoil/PRF/Control/Subsystem Reference/IsZero'
//  '<S12>'  : 'CHADsimulator/Control Units/Control Units SIM/Main Control Unit (Sim)/Parafoil Control/Parafoil/PRF/LLA to NED/Subsystem1'
//  '<S13>'  : 'CHADsimulator/Control Units/Control Units SIM/Main Control Unit (Sim)/Parafoil Control/Parafoil/PRF/LLA to NED/pos_rad'
//  '<S14>'  : 'CHADsimulator/Control Units/Control Units SIM/Main Control Unit (Sim)/Parafoil Control/Parafoil/PRF/LLA to NED/Subsystem1/Subsystem'
//  '<S15>'  : 'CHADsimulator/Control Units/Control Units SIM/Main Control Unit (Sim)/Parafoil Control/Parafoil/PRF/LLA to NED/Subsystem1/Subsystem/Angle Conversion2'
//  '<S16>'  : 'CHADsimulator/Control Units/Control Units SIM/Main Control Unit (Sim)/Parafoil Control/Parafoil/PRF/LLA to NED/Subsystem1/Subsystem/Subsystem'


//-
//  Requirements for '<Root>': PRF


} // fine namespace PRF


#endif                                 // PRF_h_

//
// File trailer for generated code.
//
// [EOF]
//
