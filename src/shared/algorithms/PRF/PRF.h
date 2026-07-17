//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// File: PRF.h
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
#ifndef PRF_h_
#define PRF_h_
#include <stdbool.h>
#include <stdint.h>
#include "PRF_types.h"
#include "zero_crossing_types.h"

namespace PRF 
{


// Class declaration for model PRF
class PRF final
{
  // public data and function members
 public:
  // Block signals and states (default storage) for system '<Root>'
  struct DW_PRF_T {
    PRFLogs PRFLogsOBSW_BusCreator_BusCreat;
    float Merge[2];                    // '<S6>/Merge'
    float Q2[2];                       // '<S21>/Sum2'
    float Q1[2];                       // '<S16>/Merge'
    float TargetPoints[6];             // '<S16>/Transpose1'
    float UnitDelay_DSTATE;            // '<S6>/Unit Delay'
    float Memory1_PreviousInput;       // '<S5>/Memory1'
    float Memory_PreviousInput;        // '<S5>/Memory'
    float Memory_PreviousInput_b;      // '<S6>/Memory'
    uint8_t UnitDelay1_DSTATE;         // '<S7>/Unit Delay1'
    uint8_t is_active_c9_PRF;          // '<S1>/Point Selection'
    uint8_t is_c9_PRF;                 // '<S1>/Point Selection'
    bool Memory2_PreviousInput;        // '<S6>/Memory2'
  };

  // Zero-crossing (trigger) state
  struct PrevZCX_PRF_T {
    ZCSigState TargetPointsGeneration_Trig_ZCE;// '<S7>/Target Points Generation' 
  };

  // External inputs (root inport signals with default storage)
  struct ExtU_PRF_T {
    PRFIn PRFIn_f;                     // '<Root>/PRF In'
    PRFReference PRFReference_i;       // '<Root>/PRF Reference'
  };

  // External outputs (root outports fed by signals with default storage)
  struct ExtY_PRF_T {
    float ServoCommands[2];            // '<Root>/Servo Commands'
    PRFLogs PRFLogsOBSW;               // '<Root>/PRF Logs OBSW'
  };

  // Parameters (default storage)
  struct P_PRF_T {
    float WrapToZero_Threshold;        // Mask Parameter: WrapToZero_Threshold
                                          //  Referenced by: '<S23>/FixPt Switch'

    float Comparetoconstant_const;    // Mask Parameter: Comparetoconstant_const
                                         //  Referenced by: '<S17>/Constant'

    float SaturationCheckUp_const;    // Mask Parameter: SaturationCheckUp_const
                                         //  Referenced by: '<S10>/Constant'

    float SaturationCheckLw_const;    // Mask Parameter: SaturationCheckLw_const
                                         //  Referenced by: '<S9>/Constant'

    PRFLogs PRFLogsOBSW_Y0;            // Computed Parameter: PRFLogsOBSW_Y0
                                          //  Referenced by: '<S16>/PRF Logs OBSW'

    float Zero_Value;                  // Computed Parameter: Zero_Value
                                          //  Referenced by: '<S12>/Zero'

    float Zero_Value_b[2];             // Computed Parameter: Zero_Value_b
                                          //  Referenced by: '<S8>/Zero'

    float Zero_Value_f;                // Computed Parameter: Zero_Value_f
                                          //  Referenced by: '<S11>/Zero'

    float Constant5_Value;             // Computed Parameter: Constant5_Value
                                          //  Referenced by: '<S6>/Constant5'

    float Constant4_Value;             // Computed Parameter: Constant4_Value
                                          //  Referenced by: '<S6>/Constant4'

    float Constant1_Value;             // Computed Parameter: Constant1_Value
                                          //  Referenced by: '<S13>/Constant1'

    float Constant_Value;              // Computed Parameter: Constant_Value
                                          //  Referenced by: '<S13>/Constant'

    float Bias1_Bias;                  // Computed Parameter: Bias1_Bias
                                          //  Referenced by: '<S6>/Bias1'

    float Bias_Bias;                   // Computed Parameter: Bias_Bias
                                          //  Referenced by: '<S6>/Bias'

    float Gain2_Gain;                  // Computed Parameter: Gain2_Gain
                                          //  Referenced by: '<S18>/Gain2'

    float Gain4_Gain;                  // Computed Parameter: Gain4_Gain
                                          //  Referenced by: '<S18>/Gain4'

    float Gain3_Gain;                  // Computed Parameter: Gain3_Gain
                                          //  Referenced by: '<S18>/Gain3'

    float Gain1_Gain;                  // Computed Parameter: Gain1_Gain
                                          //  Referenced by: '<S18>/Gain1'

    float Constant_Value_p;            // Computed Parameter: Constant_Value_p
                                          //  Referenced by: '<S23>/Constant'

    float TargetPoints_Y0;             // Computed Parameter: TargetPoints_Y0
                                          //  Referenced by: '<S16>/Target Points'

    float Q2_Y0;                       // Computed Parameter: Q2_Y0
                                          //  Referenced by: '<S16>/Q2'

    float Gain1_Gain_g;                // Computed Parameter: Gain1_Gain_g
                                          //  Referenced by: '<S22>/Gain1'

    float Constant5_Value_d;           // Computed Parameter: Constant5_Value_d
                                          //  Referenced by: '<S16>/Constant5'

    float glideratio_Value;            // Computed Parameter: glideratio_Value
                                          //  Referenced by: '<S16>/glide ratio'

    float Gain_Gain;                   // Computed Parameter: Gain_Gain
                                          //  Referenced by: '<S20>/Gain'

    float Gain1_Gain_h;                // Computed Parameter: Gain1_Gain_h
                                          //  Referenced by: '<S20>/Gain1'

    float theta_Value;                 // Computed Parameter: theta_Value
                                          //  Referenced by: '<S16>/theta'

    float Merge_InitialOutput;        // Computed Parameter: Merge_InitialOutput
                                         //  Referenced by: '<S16>/Merge'

    float Constant_Value_o;            // Computed Parameter: Constant_Value_o
                                          //  Referenced by: '<S28>/Constant'

    float Constant3_Value;             // Computed Parameter: Constant3_Value
                                          //  Referenced by: '<S26>/Constant3'

    float Constant2_Value;             // Computed Parameter: Constant2_Value
                                          //  Referenced by: '<S26>/Constant2'

    float f_Value;                     // Computed Parameter: f_Value
                                          //  Referenced by: '<S26>/f'

    float Constant1_Value_l;           // Computed Parameter: Constant1_Value_l
                                          //  Referenced by: '<S26>/Constant1'

    float Constant_Value_i;            // Computed Parameter: Constant_Value_i
                                          //  Referenced by: '<S26>/Constant'

    float Zero_Value_m;                // Computed Parameter: Zero_Value_m
                                          //  Referenced by: '<S24>/Zero'

    float Constant_Value_ir;           // Computed Parameter: Constant_Value_ir
                                          //  Referenced by: '<S14>/Constant'

    float Constant_Value_a;            // Computed Parameter: Constant_Value_a
                                          //  Referenced by: '<S15>/Constant'

    float Constant1_Value_h[2];        // Computed Parameter: Constant1_Value_h
                                          //  Referenced by: '<S1>/Constant1'

    float glideratio_Value_o;          // Computed Parameter: glideratio_Value_o
                                          //  Referenced by: '<S5>/glide ratio'

    float PRFControlzThresholdGain_Gain;
                            // Computed Parameter: PRFControlzThresholdGain_Gain
                               //  Referenced by: '<S5>/PRFControl.zThresholdGain'

    float Memory1_InitialCondition;
                                 // Computed Parameter: Memory1_InitialCondition
                                    //  Referenced by: '<S5>/Memory1'

    float Memory_InitialCondition;// Computed Parameter: Memory_InitialCondition
                                     //  Referenced by: '<S5>/Memory'

    float _Value;                      // Computed Parameter: _Value
                                          //  Referenced by: '<S5>/-'

    float Switch_Threshold;            // Computed Parameter: Switch_Threshold
                                          //  Referenced by: '<S6>/Switch'

    float Memory_InitialCondition_b;
                                // Computed Parameter: Memory_InitialCondition_b
                                   //  Referenced by: '<S6>/Memory'

    float Constant3_Value_l;           // Computed Parameter: Constant3_Value_l
                                          //  Referenced by: '<S6>/Constant3'

    float UnitDelay_InitialCondition;
                               // Computed Parameter: UnitDelay_InitialCondition
                                  //  Referenced by: '<S6>/Unit Delay'

    float Constant2_Value_c;           // Computed Parameter: Constant2_Value_c
                                          //  Referenced by: '<S6>/Constant2'

    float Constant1_Value_p;           // Computed Parameter: Constant1_Value_p
                                          //  Referenced by: '<S6>/Constant1'

    float Saturation1_UpperSat;      // Computed Parameter: Saturation1_UpperSat
                                        //  Referenced by: '<S6>/Saturation1'

    float Saturation1_LowerSat;      // Computed Parameter: Saturation1_LowerSat
                                        //  Referenced by: '<S6>/Saturation1'

    float Merge_InitialOutput_g;    // Computed Parameter: Merge_InitialOutput_g
                                       //  Referenced by: '<S6>/Merge'

    bool Memory2_InitialCondition;
                                 // Computed Parameter: Memory2_InitialCondition
                                    //  Referenced by: '<S6>/Memory2'

    uint8_t UnitDelay1_InitialCondition;// Expression: uint8(0)
                                           //  Referenced by: '<S7>/Unit Delay1'

    uint8_t Bias_Bias_i;               // Computed Parameter: Bias_Bias_i
                                          //  Referenced by: '<S7>/Bias'

    uint8_t Bias1_Bias_n;              // Computed Parameter: Bias1_Bias_n
                                          //  Referenced by: '<S7>/Bias1'

    uint8_t Saturation_UpperSat;      // Computed Parameter: Saturation_UpperSat
                                         //  Referenced by: '<S7>/Saturation'

    uint8_t Saturation_LowerSat;      // Computed Parameter: Saturation_LowerSat
                                         //  Referenced by: '<S7>/Saturation'

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
    PRF_U.PRFIn_f = localArgInput;
  }

  // Root inport: '<Root>/PRF Reference' set method
  void setPRF_Reference(PRFReference localArgInput)
  {
    PRF_U.PRFReference_i = localArgInput;
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

  // Triggered events
  PrevZCX_PRF_T PRF_PrevZCX;
};

//-
//  These blocks were eliminated from the model due to optimizations:
//
//  Block '<S23>/FixPt Data Type Duplicate1' : Unused code path elimination


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
//  '<S2>'   : 'CHADsimulator/Control Units/Control Units SIM/Main Control Unit (Sim)/Parafoil Control/Parafoil/PRF/Guidance and Control'
//  '<S3>'   : 'CHADsimulator/Control Units/Control Units SIM/Main Control Unit (Sim)/Parafoil Control/Parafoil/PRF/LLA to NED'
//  '<S4>'   : 'CHADsimulator/Control Units/Control Units SIM/Main Control Unit (Sim)/Parafoil Control/Parafoil/PRF/Point Selection'
//  '<S5>'   : 'CHADsimulator/Control Units/Control Units SIM/Main Control Unit (Sim)/Parafoil Control/Parafoil/PRF/Threshold generation'
//  '<S6>'   : 'CHADsimulator/Control Units/Control Units SIM/Main Control Unit (Sim)/Parafoil Control/Parafoil/PRF/Guidance and Control/Control'
//  '<S7>'   : 'CHADsimulator/Control Units/Control Units SIM/Main Control Unit (Sim)/Parafoil Control/Parafoil/PRF/Guidance and Control/Guidance'
//  '<S8>'   : 'CHADsimulator/Control Units/Control Units SIM/Main Control Unit (Sim)/Parafoil Control/Parafoil/PRF/Guidance and Control/Control/No Activation'
//  '<S9>'   : 'CHADsimulator/Control Units/Control Units SIM/Main Control Unit (Sim)/Parafoil Control/Parafoil/PRF/Guidance and Control/Control/Saturation Check Lw'
//  '<S10>'  : 'CHADsimulator/Control Units/Control Units SIM/Main Control Unit (Sim)/Parafoil Control/Parafoil/PRF/Guidance and Control/Control/Saturation Check Up'
//  '<S11>'  : 'CHADsimulator/Control Units/Control Units SIM/Main Control Unit (Sim)/Parafoil Control/Parafoil/PRF/Guidance and Control/Control/Servo Left'
//  '<S12>'  : 'CHADsimulator/Control Units/Control Units SIM/Main Control Unit (Sim)/Parafoil Control/Parafoil/PRF/Guidance and Control/Control/Servo Right'
//  '<S13>'  : 'CHADsimulator/Control Units/Control Units SIM/Main Control Unit (Sim)/Parafoil Control/Parafoil/PRF/Guidance and Control/Control/Subsystem Reference'
//  '<S14>'  : 'CHADsimulator/Control Units/Control Units SIM/Main Control Unit (Sim)/Parafoil Control/Parafoil/PRF/Guidance and Control/Control/Subsystem Reference/IsPositive'
//  '<S15>'  : 'CHADsimulator/Control Units/Control Units SIM/Main Control Unit (Sim)/Parafoil Control/Parafoil/PRF/Guidance and Control/Control/Subsystem Reference/IsZero'
//  '<S16>'  : 'CHADsimulator/Control Units/Control Units SIM/Main Control Unit (Sim)/Parafoil Control/Parafoil/PRF/Guidance and Control/Guidance/Target Points Generation'
//  '<S17>'  : 'CHADsimulator/Control Units/Control Units SIM/Main Control Unit (Sim)/Parafoil Control/Parafoil/PRF/Guidance and Control/Guidance/Target Points Generation/Compare to constant'
//  '<S18>'  : 'CHADsimulator/Control Units/Control Units SIM/Main Control Unit (Sim)/Parafoil Control/Parafoil/PRF/Guidance and Control/Guidance/Target Points Generation/Enabled Subsystem'
//  '<S19>'  : 'CHADsimulator/Control Units/Control Units SIM/Main Control Unit (Sim)/Parafoil Control/Parafoil/PRF/Guidance and Control/Guidance/Target Points Generation/Enabled Subsystem1'
//  '<S20>'  : 'CHADsimulator/Control Units/Control Units SIM/Main Control Unit (Sim)/Parafoil Control/Parafoil/PRF/Guidance and Control/Guidance/Target Points Generation/Initial Geometry'
//  '<S21>'  : 'CHADsimulator/Control Units/Control Units SIM/Main Control Unit (Sim)/Parafoil Control/Parafoil/PRF/Guidance and Control/Guidance/Target Points Generation/Wind based Q2 generation'
//  '<S22>'  : 'CHADsimulator/Control Units/Control Units SIM/Main Control Unit (Sim)/Parafoil Control/Parafoil/PRF/Guidance and Control/Guidance/Target Points Generation/Wind based Q2 generation/Degrees to Radians'
//  '<S23>'  : 'CHADsimulator/Control Units/Control Units SIM/Main Control Unit (Sim)/Parafoil Control/Parafoil/PRF/Guidance and Control/Guidance/Target Points Generation/Wind based Q2 generation/Wrap To Zero'
//  '<S24>'  : 'CHADsimulator/Control Units/Control Units SIM/Main Control Unit (Sim)/Parafoil Control/Parafoil/PRF/LLA to NED/Subsystem1'
//  '<S25>'  : 'CHADsimulator/Control Units/Control Units SIM/Main Control Unit (Sim)/Parafoil Control/Parafoil/PRF/LLA to NED/pos_rad'
//  '<S26>'  : 'CHADsimulator/Control Units/Control Units SIM/Main Control Unit (Sim)/Parafoil Control/Parafoil/PRF/LLA to NED/Subsystem1/Subsystem'
//  '<S27>'  : 'CHADsimulator/Control Units/Control Units SIM/Main Control Unit (Sim)/Parafoil Control/Parafoil/PRF/LLA to NED/Subsystem1/Subsystem/Angle Conversion2'
//  '<S28>'  : 'CHADsimulator/Control Units/Control Units SIM/Main Control Unit (Sim)/Parafoil Control/Parafoil/PRF/LLA to NED/Subsystem1/Subsystem/Subsystem'


//-
//  Requirements for '<Root>': PRF


#endif                                 // PRF_h_


} // fine namespace PRF

//
// File trailer for generated code.
//
// [EOF]
//
