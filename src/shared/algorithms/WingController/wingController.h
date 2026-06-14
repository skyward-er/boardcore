//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// File: wingController.h
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
#ifndef wingController_h_
#define wingController_h_
#include <stdbool.h>
#include <stdint.h>
#include "wingController_types.h"
#include "zero_crossing_types.h"

// Class declaration for model wingController
class wingController final
{
  // public data and function members
 public:
  // Block signals and states (default storage) for system '<Root>'
  struct DW_wingController_T {
    float Merge[2];                    // '<S5>/Merge'
    float Q2[2];                       // '<S22>/Sum2'
    float Q1[2];                       // '<S17>/Merge'
    float TargetPoints[6];             // '<S17>/Transpose1'
    float Memory1_PreviousInput;       // '<S4>/Memory1'
    float Memory_PreviousInput;        // '<S4>/Memory'
    float Memory_PreviousInput_e;      // '<S5>/Memory'
    float Memory1_PreviousInput_o;     // '<S5>/Memory1'
    uint8_t UnitDelay1_DSTATE;         // '<S6>/Unit Delay1'
    uint8_t is_active_c9_wingController;// '<S1>/Point Selection'
    uint8_t is_c9_wingController;      // '<S1>/Point Selection'
    bool Memory2_PreviousInput;        // '<S5>/Memory2'
    bool Memory3_PreviousInput;        // '<S5>/Memory3'
  };

  // Zero-crossing (trigger) state
  struct PrevZCX_wingController_T {
    ZCSigState TargetPointsGeneration_Trig_ZCE;// '<S6>/Target Points Generation' 
  };

  // External inputs (root inport signals with default storage)
  struct ExtU_wingController_T {
    PRFIn PRFIn_d;                     // '<Root>/PRF In'
  };

  // External outputs (root outports fed by signals with default storage)
  struct ExtY_wingController_T {
    float ServoCommands[2];            // '<Root>/Servo Commands'
    PRFLogs PRFLogsOBSW;               // '<Root>/PRF Logs OBSW'
  };

  // Parameters (default storage)
  struct P_wingController_T {
    float WrapToZero_Threshold;        // Mask Parameter: WrapToZero_Threshold
                                          //  Referenced by: '<S24>/FixPt Switch'

    float Comparetoconstant_const;    // Mask Parameter: Comparetoconstant_const
                                         //  Referenced by: '<S18>/Constant'

    float SaturationCheckUp1_const;  // Mask Parameter: SaturationCheckUp1_const
                                        //  Referenced by: '<S11>/Constant'

    float SaturationCheckLw1_const;  // Mask Parameter: SaturationCheckLw1_const
                                        //  Referenced by: '<S9>/Constant'

    float SaturationCheckUp_const;    // Mask Parameter: SaturationCheckUp_const
                                         //  Referenced by: '<S10>/Constant'

    float SaturationCheckLw_const;    // Mask Parameter: SaturationCheckLw_const
                                         //  Referenced by: '<S8>/Constant'

    float Zero_Value;                  // Computed Parameter: Zero_Value
                                          //  Referenced by: '<S12>/Zero'

    float Zero_Value_d[2];             // Computed Parameter: Zero_Value_d
                                          //  Referenced by: '<S7>/Zero'

    float Zero_Value_p;                // Computed Parameter: Zero_Value_p
                                          //  Referenced by: '<S13>/Zero'

    float Constant5_Value;       // Expression: PRFControl.subsystemSamplingTime
                                    //  Referenced by: '<S5>/Constant5'

    float Constant4_Value;             // Expression: PRFControl.OBSW.ki
                                          //  Referenced by: '<S5>/Constant4'

    float Constant8_Value;       // Expression: PRFControl.subsystemSamplingTime
                                    //  Referenced by: '<S5>/Constant8'

    float Constant2_Value;             // Expression: PRFControl.ki
                                          //  Referenced by: '<S5>/Constant2'

    float Constant7_Value;             // Expression: PRFControl.servoRadius
                                          //  Referenced by: '<S5>/Constant7'

    float Constant6_Value;             // Expression: PRFControl.halfWing
                                          //  Referenced by: '<S5>/Constant6'

    float Gain_Gain;                   // Computed Parameter: Gain_Gain
                                          //  Referenced by: '<S5>/Gain'

    float Gain1_Gain;                  // Computed Parameter: Gain1_Gain
                                          //  Referenced by: '<S5>/Gain1'

    float Gain3_Gain;                  // Computed Parameter: Gain3_Gain
                                          //  Referenced by: '<S5>/Gain3'

    float Constant1_Value;             // Computed Parameter: Constant1_Value
                                          //  Referenced by: '<S14>/Constant1'

    float Constant_Value;              // Computed Parameter: Constant_Value
                                          //  Referenced by: '<S14>/Constant'

    float Bias1_Bias;                  // Computed Parameter: Bias1_Bias
                                          //  Referenced by: '<S5>/Bias1'

    float Bias_Bias;                   // Computed Parameter: Bias_Bias
                                          //  Referenced by: '<S5>/Bias'

    float Gain2_Gain;                  // Computed Parameter: Gain2_Gain
                                          //  Referenced by: '<S19>/Gain2'

    float Gain4_Gain;                // Expression: PRFControl.perpendicularGain
                                        //  Referenced by: '<S19>/Gain4'

    float Gain3_Gain_m;                // Computed Parameter: Gain3_Gain_m
                                          //  Referenced by: '<S19>/Gain3'

    float Gain1_Gain_e;                // Expression: PRFControl.parallelGain
                                          //  Referenced by: '<S19>/Gain1'

    float Constant_Value_a;            // Computed Parameter: Constant_Value_a
                                          //  Referenced by: '<S24>/Constant'

    float TargetPoints_Y0[6];          // Expression: single(1e10*ones(3, 2))
                                          //  Referenced by: '<S17>/Target Points'

    float Q2_Y0;                       // Computed Parameter: Q2_Y0
                                          //  Referenced by: '<S17>/Q2'

    float Constant3_Value[2];          // Expression: PRFControl.target
                                          //  Referenced by: '<S17>/Constant3'

    float Constant1_Value_e;           // Expression: PRFControl.windHeading
                                          //  Referenced by: '<S17>/Constant1'

    float Gain1_Gain_d;                // Computed Parameter: Gain1_Gain_d
                                          //  Referenced by: '<S23>/Gain1'

    float Constant5_Value_h;       // Expression: PRFControl.windAlignmentRadius
                                      //  Referenced by: '<S17>/Constant5'

    float glideratio_Value;            // Expression: PRFControl.glideRatio
                                          //  Referenced by: '<S17>/glide ratio'

    float Gain_Gain_e;                 // Computed Parameter: Gain_Gain_e
                                          //  Referenced by: '<S21>/Gain'

    float Gain1_Gain_dp;               // Computed Parameter: Gain1_Gain_dp
                                          //  Referenced by: '<S21>/Gain1'

    float theta_Value;                 // Expression: PRFControl.DPGAngle
                                          //  Referenced by: '<S17>/theta'

    float Merge_InitialOutput;        // Computed Parameter: Merge_InitialOutput
                                         //  Referenced by: '<S17>/Merge'

    float Constant_Value_o;            // Computed Parameter: Constant_Value_o
                                          //  Referenced by: '<S15>/Constant'

    float Constant_Value_oa;           // Computed Parameter: Constant_Value_oa
                                          //  Referenced by: '<S16>/Constant'

    float Constant1_Value_d[2];        // Expression: PRFControl.target
                                          //  Referenced by: '<S4>/Constant1'

    float glideratio_Value_h;          // Expression: PRFControl.glideRatio
                                          //  Referenced by: '<S4>/glide ratio'

    float PRFControlzThresholdGain_Gain;// Expression: PRFControl.zThresholdGain
                                           //  Referenced by: '<S4>/PRFControl.zThresholdGain'

    float Memory1_InitialCondition;
                                 // Computed Parameter: Memory1_InitialCondition
                                    //  Referenced by: '<S4>/Memory1'

    float Memory_InitialCondition;// Computed Parameter: Memory_InitialCondition
                                     //  Referenced by: '<S4>/Memory'

    float _Value;                      // Expression: PRFControl.QThreshold
                                          //  Referenced by: '<S4>/-'

    float Memory_InitialCondition_d;
                                // Computed Parameter: Memory_InitialCondition_d
                                   //  Referenced by: '<S5>/Memory'

    float Switch_Threshold;            // Computed Parameter: Switch_Threshold
                                          //  Referenced by: '<S5>/Switch'

    float Constant3_Value_d;           // Expression: PRFControl.OBSW.kp
                                          //  Referenced by: '<S5>/Constant3'

    float Saturation1_UpperSat;      // Expression: PRFControl.OBSW.upSaturation
                                        //  Referenced by: '<S5>/Saturation1'

    float Saturation1_LowerSat;      // Expression: PRFControl.OBSW.lwSaturation
                                        //  Referenced by: '<S5>/Saturation1'

    float Memory1_InitialCondition_f;
                               // Computed Parameter: Memory1_InitialCondition_f
                                  //  Referenced by: '<S5>/Memory1'

    float Constant1_Value_m;           // Expression: PRFControl.kp
                                          //  Referenced by: '<S5>/Constant1'

    float Saturation_UpperSat;         // Expression: PRFControl.upSaturation
                                          //  Referenced by: '<S5>/Saturation'

    float Saturation_LowerSat;         // Expression: PRFControl.lwSaturation
                                          //  Referenced by: '<S5>/Saturation'

    float Gain2_Gain_g;                // Expression: PRFControl.scalingFactor
                                          //  Referenced by: '<S5>/Gain2'

    float Merge_InitialOutput_e;    // Computed Parameter: Merge_InitialOutput_e
                                       //  Referenced by: '<S5>/Merge'

    bool Memory2_InitialCondition;
                                 // Computed Parameter: Memory2_InitialCondition
                                    //  Referenced by: '<S5>/Memory2'

    bool Constant9_Value;              // Expression: PRFControl.OBSW.activation
                                          //  Referenced by: '<S5>/Constant9'

    bool Memory3_InitialCondition;
                                 // Computed Parameter: Memory3_InitialCondition
                                    //  Referenced by: '<S5>/Memory3'

    uint8_t UnitDelay1_InitialCondition;// Expression: uint8(0)
                                           //  Referenced by: '<S6>/Unit Delay1'

    uint8_t Bias_Bias_e;               // Computed Parameter: Bias_Bias_e
                                          //  Referenced by: '<S6>/Bias'

    uint8_t Bias1_Bias_b;              // Computed Parameter: Bias1_Bias_b
                                          //  Referenced by: '<S6>/Bias1'

    uint8_t Saturation_UpperSat_f;  // Computed Parameter: Saturation_UpperSat_f
                                       //  Referenced by: '<S6>/Saturation'

    uint8_t Saturation_LowerSat_b;  // Computed Parameter: Saturation_LowerSat_b
                                       //  Referenced by: '<S6>/Saturation'

  };

  // Copy Constructor
  wingController(wingController const&) = delete;

  // Assignment Operator
  wingController& operator= (wingController const&) & = delete;

  // Move Constructor
  wingController(wingController &&) = delete;

  // Move Assignment Operator
  wingController& operator= (wingController &&) = delete;

  // Root inport: '<Root>/PRF In' set method
  void setPRF_In(PRFIn localArgInput)
  {
    wingController_U.PRFIn_d = localArgInput;
  }

  // Root outport: '<Root>/Servo Commands' get method
  const float *getServo_Commands() const
  {
    return wingController_Y.ServoCommands;
  }

  // Root outport: '<Root>/PRF Logs OBSW' get method
  PRFLogs getPRF_Logs_OBSW() const
  {
    return wingController_Y.PRFLogsOBSW;
  }

  // Block parameters get method
  const P_wingController_T &getBlockParameters() const
  {
    return wingController_P;
  }

  // Block parameters set method
  void setBlockParameters(const P_wingController_T *pP_wingController_T) const
  {
    wingController_P = *pP_wingController_T;
  }

  // model initialize function
  void initialize();

  // model step function
  void step();

  // model terminate function
  static void terminate();

  // Constructor
  wingController();

  // Destructor
  ~wingController();

  // private data and function members
 private:
  // External inputs
  ExtU_wingController_T wingController_U;

  // External outputs
  ExtY_wingController_T wingController_Y;

  // Block states
  DW_wingController_T wingController_DW;

  // Tunable parameters
  static P_wingController_T wingController_P;

  // Triggered events
  PrevZCX_wingController_T wingController_PrevZCX;
};

//-
//  These blocks were eliminated from the model due to optimizations:
//
//  Block '<S24>/FixPt Data Type Duplicate1' : Unused code path elimination


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
//  hilite_system('CHADsimulator/Control Units/Control Units SIM/Main Control Unit (Sim)/Parafoil Control/Parafoil /wingController DPG')    - opens subsystem CHADsimulator/Control Units/Control Units SIM/Main Control Unit (Sim)/Parafoil Control/Parafoil /wingController DPG
//  hilite_system('CHADsimulator/Control Units/Control Units SIM/Main Control Unit (Sim)/Parafoil Control/Parafoil /wingController DPG/Kp') - opens and selects block Kp
//
//  Here is the system hierarchy for this model
//
//  '<Root>' : 'CHADsimulator/Control Units/Control Units SIM/Main Control Unit (Sim)/Parafoil Control/Parafoil '
//  '<S1>'   : 'CHADsimulator/Control Units/Control Units SIM/Main Control Unit (Sim)/Parafoil Control/Parafoil /wingController DPG'
//  '<S2>'   : 'CHADsimulator/Control Units/Control Units SIM/Main Control Unit (Sim)/Parafoil Control/Parafoil /wingController DPG/Guidance and Control'
//  '<S3>'   : 'CHADsimulator/Control Units/Control Units SIM/Main Control Unit (Sim)/Parafoil Control/Parafoil /wingController DPG/Point Selection'
//  '<S4>'   : 'CHADsimulator/Control Units/Control Units SIM/Main Control Unit (Sim)/Parafoil Control/Parafoil /wingController DPG/Threshold generation'
//  '<S5>'   : 'CHADsimulator/Control Units/Control Units SIM/Main Control Unit (Sim)/Parafoil Control/Parafoil /wingController DPG/Guidance and Control/Control'
//  '<S6>'   : 'CHADsimulator/Control Units/Control Units SIM/Main Control Unit (Sim)/Parafoil Control/Parafoil /wingController DPG/Guidance and Control/Guidance'
//  '<S7>'   : 'CHADsimulator/Control Units/Control Units SIM/Main Control Unit (Sim)/Parafoil Control/Parafoil /wingController DPG/Guidance and Control/Control/No Activation'
//  '<S8>'   : 'CHADsimulator/Control Units/Control Units SIM/Main Control Unit (Sim)/Parafoil Control/Parafoil /wingController DPG/Guidance and Control/Control/Saturation Check Lw'
//  '<S9>'   : 'CHADsimulator/Control Units/Control Units SIM/Main Control Unit (Sim)/Parafoil Control/Parafoil /wingController DPG/Guidance and Control/Control/Saturation Check Lw1'
//  '<S10>'  : 'CHADsimulator/Control Units/Control Units SIM/Main Control Unit (Sim)/Parafoil Control/Parafoil /wingController DPG/Guidance and Control/Control/Saturation Check Up'
//  '<S11>'  : 'CHADsimulator/Control Units/Control Units SIM/Main Control Unit (Sim)/Parafoil Control/Parafoil /wingController DPG/Guidance and Control/Control/Saturation Check Up1'
//  '<S12>'  : 'CHADsimulator/Control Units/Control Units SIM/Main Control Unit (Sim)/Parafoil Control/Parafoil /wingController DPG/Guidance and Control/Control/Servo Left'
//  '<S13>'  : 'CHADsimulator/Control Units/Control Units SIM/Main Control Unit (Sim)/Parafoil Control/Parafoil /wingController DPG/Guidance and Control/Control/Servo Right'
//  '<S14>'  : 'CHADsimulator/Control Units/Control Units SIM/Main Control Unit (Sim)/Parafoil Control/Parafoil /wingController DPG/Guidance and Control/Control/Subsystem Reference'
//  '<S15>'  : 'CHADsimulator/Control Units/Control Units SIM/Main Control Unit (Sim)/Parafoil Control/Parafoil /wingController DPG/Guidance and Control/Control/Subsystem Reference/IsPositive'
//  '<S16>'  : 'CHADsimulator/Control Units/Control Units SIM/Main Control Unit (Sim)/Parafoil Control/Parafoil /wingController DPG/Guidance and Control/Control/Subsystem Reference/IsZero'
//  '<S17>'  : 'CHADsimulator/Control Units/Control Units SIM/Main Control Unit (Sim)/Parafoil Control/Parafoil /wingController DPG/Guidance and Control/Guidance/Target Points Generation'
//  '<S18>'  : 'CHADsimulator/Control Units/Control Units SIM/Main Control Unit (Sim)/Parafoil Control/Parafoil /wingController DPG/Guidance and Control/Guidance/Target Points Generation/Compare to constant'
//  '<S19>'  : 'CHADsimulator/Control Units/Control Units SIM/Main Control Unit (Sim)/Parafoil Control/Parafoil /wingController DPG/Guidance and Control/Guidance/Target Points Generation/Enabled Subsystem'
//  '<S20>'  : 'CHADsimulator/Control Units/Control Units SIM/Main Control Unit (Sim)/Parafoil Control/Parafoil /wingController DPG/Guidance and Control/Guidance/Target Points Generation/Enabled Subsystem1'
//  '<S21>'  : 'CHADsimulator/Control Units/Control Units SIM/Main Control Unit (Sim)/Parafoil Control/Parafoil /wingController DPG/Guidance and Control/Guidance/Target Points Generation/Initial Geometry'
//  '<S22>'  : 'CHADsimulator/Control Units/Control Units SIM/Main Control Unit (Sim)/Parafoil Control/Parafoil /wingController DPG/Guidance and Control/Guidance/Target Points Generation/Wind based Q2 generation'
//  '<S23>'  : 'CHADsimulator/Control Units/Control Units SIM/Main Control Unit (Sim)/Parafoil Control/Parafoil /wingController DPG/Guidance and Control/Guidance/Target Points Generation/Wind based Q2 generation/Degrees to Radians'
//  '<S24>'  : 'CHADsimulator/Control Units/Control Units SIM/Main Control Unit (Sim)/Parafoil Control/Parafoil /wingController DPG/Guidance and Control/Guidance/Target Points Generation/Wind based Q2 generation/Wrap To Zero'


//-
//  Requirements for '<Root>': wingController


#endif                                 // wingController_h_

//
// File trailer for generated code.
//
// [EOF]
//
