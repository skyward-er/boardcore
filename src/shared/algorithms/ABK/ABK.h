//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// File: ABK.h
//
// Code generated for Simulink model 'ABK'.
//
// Model version                  : 11.369
// Simulink Coder version         : 24.2 (R2024b) 21-Jun-2024
// C/C++ source code generated on : Thu Jul 16 14:05:08 2026
//
// Target selection: ert.tlc
// Embedded hardware selection: ARM Compatible->ARM Cortex-M
// Code generation objectives:
//    1. Execution efficiency
//    2. Debugging
//    3. RAM efficiency
// Validation result: Not run
//
#ifndef ABK_h_
#define ABK_h_
#include <stdbool.h>
#include <stdint.h>
#include "ABK_types.h"
#include "zero_crossing_types.h"

namespace ABK 
{


// Exported data declaration

// Declaration for custom storage class: ExportToFile
extern float abkTraj[19866];       // Referenced by: '<S6>/Trajectories Matrix'

// Class declaration for model ABK
class ABK final
{
  // public data and function members
 public:
  // Block signals and states (default storage) for system '<Root>'
  struct DW_ABK_T {
    ABKLogs BusAssignment1;            // '<S2>/Bus Assignment1'
    ABKLogs ABKLogsOBSW_BusCreator_BusCreat;
    float Selecttrajectories[1505];    // '<S6>/Select trajectories'
    float Selectcolumnz[301];          // '<S6>/Select column z'
    float Memory_PreviousInput[2];     // '<S17>/Memory'
    float Memory3_PreviousInput[2];    // '<S17>/Memory3'
    float Massselector[1806];          // '<S6>/Mass selector'
    float UnitDelay_DSTATE;            // '<S2>/Unit Delay'
    float Memory_PreviousInput_o;      // '<S13>/Memory'
    int8_t If_ActiveSubsystem;         // '<S1>/If'
    int8_t If_ActiveSubsystem_d;       // '<S2>/If'
    uint8_t massIndex;                 // '<S6>/Mass_Prelookup'
    uint8_t UnitDelay1_DSTATE;         // '<S2>/Unit Delay1'
    bool UnitDelay_DSTATE_j;           // '<S16>/Unit Delay'
    bool Memory1_PreviousInput;        // '<S13>/Memory1'
  };

  // Zero-crossing (trigger) state
  struct PrevZCX_ABK_T {
    ZCSigState FunctionCallSubsystem_Trig_ZCE;// '<S2>/Function-Call Subsystem'
  };

  // External inputs (root inport signals with default storage)
  struct ExtU_ABK_T {
    ABKIn ABKIn_e;                     // '<Root>/ABK In'
  };

  // External outputs (root outports fed by signals with default storage)
  struct ExtY_ABK_T {
    float ABKControl;                  // '<Root>/ABK Control'
    ABKLogs ABKLogsOBSW;               // '<Root>/ABK Logs OBSW'
  };

  // Parameters (default storage)
  struct P_ABK_T {
    float FullClose_compare_const;    // Mask Parameter: FullClose_compare_const
                                         //  Referenced by: '<S14>/Constant'

    float FullOpen_compare_const;      // Mask Parameter: FullOpen_compare_const
                                          //  Referenced by: '<S15>/Constant'

    float FullOpen_compare_const_g;  // Mask Parameter: FullOpen_compare_const_g
                                        //  Referenced by: '<S18>/Constant'

    ABKLogs ABKLogsOBSW_Outport_2_Y0;
                                 // Computed Parameter: ABKLogsOBSW_Outport_2_Y0
                                    //  Referenced by:

    ABKLogs ABKLogsOBSW_Y0;            // Computed Parameter: ABKLogsOBSW_Y0
                                          //  Referenced by: '<S2>/ABK Logs OBSW'

    double zColumn_Value[301];         // Expression: abkTrajectoryF(:,1,1)
                                          //  Referenced by: '<S19>/zColumn'

    double SeaLevelTemperature_Value;  // Expression: T0
                                          //  Referenced by: '<S22>/Sea Level  Temperature'

    double Limitaltitudetotroposhere_Upper;// Expression: h_trop
                                              //  Referenced by: '<S22>/Limit  altitude  to troposhere'

    double Limitaltitudetotroposhere_Lower;// Expression: h0
                                              //  Referenced by: '<S22>/Limit  altitude  to troposhere'

    double LapseRate_Gain;             // Expression: L
                                          //  Referenced by: '<S22>/Lapse Rate'

    double gammaR_Gain;                // Expression: gamma*R
                                          //  Referenced by: '<S22>/gamma*R'

    float traj_Y0;                     // Computed Parameter: traj_Y0
                                          //  Referenced by: '<S6>/traj'

    float zVect_Y0;                    // Computed Parameter: zVect_Y0
                                          //  Referenced by: '<S6>/zVect'

    float TrajectoriesMatrix1_Value[11];
                                // Computed Parameter: TrajectoriesMatrix1_Value
                                   //  Referenced by: '<S6>/Trajectories Matrix1'

    float Fullclose_Value;             // Computed Parameter: Fullclose_Value
                                          //  Referenced by: '<S7>/Full close'

    float Fullopen_Value;              // Computed Parameter: Fullopen_Value
                                          //  Referenced by: '<S8>/Full open'

    float Constant_Value;              // Computed Parameter: Constant_Value
                                          //  Referenced by: '<S13>/Constant'

    float SampleTime_Value;            // Computed Parameter: SampleTime_Value
                                          //  Referenced by: '<S13>/Sample Time'

    float refBias1_Bias;               // Computed Parameter: refBias1_Bias
                                          //  Referenced by: '<S13>/refBias1'

    float Memory_InitialCondition;// Computed Parameter: Memory_InitialCondition
                                     //  Referenced by: '<S13>/Memory'

    float Ki_Value;                    // Computed Parameter: Ki_Value
                                          //  Referenced by: '<S13>/Ki'

    float Kd_Value;                    // Computed Parameter: Kd_Value
                                          //  Referenced by: '<S13>/Kd'

    float refBias_Bias;                // Computed Parameter: refBias_Bias
                                          //  Referenced by: '<S13>/refBias'

    float Kp_Value;                    // Computed Parameter: Kp_Value
                                          //  Referenced by: '<S13>/Kp'

    float refBias2_Bias;               // Computed Parameter: refBias2_Bias
                                          //  Referenced by: '<S13>/refBias2'

    float Saturation_UpperSat;        // Computed Parameter: Saturation_UpperSat
                                         //  Referenced by: '<S13>/Saturation'

    float Saturation_LowerSat;        // Computed Parameter: Saturation_LowerSat
                                         //  Referenced by: '<S13>/Saturation'

    float Saturation_UpperSat_l;    // Computed Parameter: Saturation_UpperSat_l
                                       //  Referenced by: '<S17>/Saturation'

    float Saturation_LowerSat_a;    // Computed Parameter: Saturation_LowerSat_a
                                       //  Referenced by: '<S17>/Saturation'

    float Trajectorydifference_Value[3311];
                               // Computed Parameter: Trajectorydifference_Value
                                  //  Referenced by: '<S19>/Trajectory difference'

    float coeffb_Value[3];             // Computed Parameter: coeffb_Value
                                          //  Referenced by: '<S17>/coeff b'

    float Memory_InitialCondition_k;
                                // Computed Parameter: Memory_InitialCondition_k
                                   //  Referenced by: '<S17>/Memory'

    float coeffa_Value[2];             // Computed Parameter: coeffa_Value
                                          //  Referenced by: '<S17>/coeff a'

    float Memory3_InitialCondition;
                                 // Computed Parameter: Memory3_InitialCondition
                                    //  Referenced by: '<S17>/Memory3'

    float T_refGain_Gain;              // Computed Parameter: T_refGain_Gain
                                          //  Referenced by: '<S19>/T_refGain'

    float Saturation2_UpperSat;      // Computed Parameter: Saturation2_UpperSat
                                        //  Referenced by: '<S19>/Saturation2'

    float Saturation2_LowerSat;      // Computed Parameter: Saturation2_LowerSat
                                        //  Referenced by: '<S19>/Saturation2'

    float Bias3_Bias;                  // Computed Parameter: Bias3_Bias
                                          //  Referenced by: '<S19>/Bias3'

    float fullOpen_Value;              // Computed Parameter: fullOpen_Value
                                          //  Referenced by: '<S16>/fullOpen'

    float Extensions_Value[5];         // Computed Parameter: Extensions_Value
                                          //  Referenced by: '<S10>/Extensions'

    float Gain_Gain;                   // Computed Parameter: Gain_Gain
                                          //  Referenced by: '<S2>/Gain'

    float Gain1_Gain;                  // Computed Parameter: Gain1_Gain
                                          //  Referenced by: '<S2>/Gain1'

    float Bypass_vect_Bias[2];         // Computed Parameter: Bypass_vect_Bias
                                          //  Referenced by: '<S2>/Bypass_vect'

    float UnitDelay_InitialCondition;
                               // Computed Parameter: UnitDelay_InitialCondition
                                  //  Referenced by: '<S2>/Unit Delay'

    float Constant_Value_o;            // Computed Parameter: Constant_Value_o
                                          //  Referenced by: '<S4>/Constant'

    float Gain_Gain_p;                 // Computed Parameter: Gain_Gain_p
                                          //  Referenced by: '<S3>/Gain'

    float Constant1_Value;             // Computed Parameter: Constant1_Value
                                          //  Referenced by: '<S3>/Constant1'

    uint32_t Bias_Bias;                // Computed Parameter: Bias_Bias
                                          //  Referenced by: '<S9>/Bias'

    uint16_t Bias_Bias_e;              // Computed Parameter: Bias_Bias_e
                                          //  Referenced by: '<S19>/Bias'

    uint16_t Bias_Bias_o;              // Computed Parameter: Bias_Bias_o
                                          //  Referenced by: '<S10>/Bias'

    bool Memory1_InitialCondition;
                                 // Computed Parameter: Memory1_InitialCondition
                                    //  Referenced by: '<S13>/Memory1'

    bool UnitDelay_InitialCondition_c; // Expression: false
                                          //  Referenced by: '<S16>/Unit Delay'

    uint8_t massIndex_Y0;              // Computed Parameter: massIndex_Y0
                                          //  Referenced by: '<S6>/massIndex'

    uint8_t UnitDelay1_InitialCondition;// Expression: uint8(0)
                                           //  Referenced by: '<S2>/Unit Delay1'

    uint8_t Bias_Bias_c;               // Computed Parameter: Bias_Bias_c
                                          //  Referenced by: '<S2>/Bias'

    uint8_t Saturation_UpperSat_j;  // Computed Parameter: Saturation_UpperSat_j
                                       //  Referenced by: '<S2>/Saturation'

    uint8_t Saturation_LowerSat_h;  // Computed Parameter: Saturation_LowerSat_h
                                       //  Referenced by: '<S2>/Saturation'

  };

  // Copy Constructor
  ABK(ABK const&) = delete;

  // Assignment Operator
  ABK& operator= (ABK const&) & = delete;

  // Move Constructor
  ABK(ABK &&) = delete;

  // Move Assignment Operator
  ABK& operator= (ABK &&) = delete;

  // Root inport: '<Root>/ABK In' set method
  void setABK_In(ABKIn localArgInput)
  {
    ABK_U.ABKIn_e = localArgInput;
  }

  // Root outport: '<Root>/ABK Control' get method
  float getABK_Control() const
  {
    return ABK_Y.ABKControl;
  }

  // Root outport: '<Root>/ABK Logs OBSW' get method
  ABKLogs getABK_Logs_OBSW() const
  {
    return ABK_Y.ABKLogsOBSW;
  }

  // Block parameters get method
  const P_ABK_T &getBlockParameters() const
  {
    return ABK_P;
  }

  // Block parameters set method
  void setBlockParameters(const P_ABK_T *pP_ABK_T) const
  {
    ABK_P = *pP_ABK_T;
  }

  // model initialize function
  void initialize();

  // model step function
  void step();

  // model terminate function
  static void terminate();

  // Constructor
  ABK();

  // Destructor
  ~ABK();

  // private data and function members
 private:
  // External inputs
  ExtU_ABK_T ABK_U;

  // External outputs
  ExtY_ABK_T ABK_Y;

  // Block states
  DW_ABK_T ABK_DW;

  // Tunable parameters
  static P_ABK_T ABK_P;

  // Triggered events
  PrevZCX_ABK_T ABK_PrevZCX;
};

//-
//  These blocks were eliminated from the model due to optimizations:
//
//  Block '<S20>/Add' : Unused code path elimination
//  Block '<S20>/Constant' : Unused code path elimination
//  Block '<S20>/Constant1' : Unused code path elimination
//  Block '<S20>/Data Type Conversion' : Unused code path elimination
//  Block '<S20>/Data Type Conversion2' : Unused code path elimination
//  Block '<S20>/Data Type Conversion3' : Unused code path elimination
//  Block '<S20>/Divide' : Unused code path elimination
//  Block '<S20>/Divide1' : Unused code path elimination
//  Block '<S20>/Gain' : Unused code path elimination
//  Block '<S22>/(T//T0)^(g//LR) ' : Unused code path elimination
//  Block '<S22>/1//T0' : Unused code path elimination
//  Block '<S22>/Altitude of Troposphere' : Unused code path elimination
//  Block '<S22>/Constant' : Unused code path elimination
//  Block '<S22>/Limit  altitude  to Stratosphere' : Unused code path elimination
//  Block '<S22>/P0' : Unused code path elimination
//  Block '<S22>/Product' : Unused code path elimination
//  Block '<S22>/Product1' : Unused code path elimination
//  Block '<S22>/Product2' : Unused code path elimination
//  Block '<S22>/Product3' : Unused code path elimination
//  Block '<S22>/Stratosphere Model' : Unused code path elimination
//  Block '<S22>/Sum' : Unused code path elimination
//  Block '<S22>/g//R' : Unused code path elimination
//  Block '<S22>/rho0' : Unused code path elimination
//  Block '<S20>/Power' : Unused code path elimination
//  Block '<S20>/dynVisc Conversion' : Unused code path elimination
//  Block '<S20>/kineVisc Conversion' : Unused code path elimination


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
//  hilite_system('CHADsimulator/Control Units/Control Units SIM/Main Control Unit (Sim)/Airbrakes Control/ABK/ABK')    - opens subsystem CHADsimulator/Control Units/Control Units SIM/Main Control Unit (Sim)/Airbrakes Control/ABK/ABK
//  hilite_system('CHADsimulator/Control Units/Control Units SIM/Main Control Unit (Sim)/Airbrakes Control/ABK/ABK/Kp') - opens and selects block Kp
//
//  Here is the system hierarchy for this model
//
//  '<Root>' : 'CHADsimulator/Control Units/Control Units SIM/Main Control Unit (Sim)/Airbrakes Control/ABK'
//  '<S1>'   : 'CHADsimulator/Control Units/Control Units SIM/Main Control Unit (Sim)/Airbrakes Control/ABK/ABK'
//  '<S2>'   : 'CHADsimulator/Control Units/Control Units SIM/Main Control Unit (Sim)/Airbrakes Control/ABK/ABK/Algorothm Execution'
//  '<S3>'   : 'CHADsimulator/Control Units/Control Units SIM/Main Control Unit (Sim)/Airbrakes Control/ABK/ABK/Mach Check'
//  '<S4>'   : 'CHADsimulator/Control Units/Control Units SIM/Main Control Unit (Sim)/Airbrakes Control/ABK/ABK/OverMach Protection'
//  '<S5>'   : 'CHADsimulator/Control Units/Control Units SIM/Main Control Unit (Sim)/Airbrakes Control/ABK/ABK/Algorothm Execution/Active control system'
//  '<S6>'   : 'CHADsimulator/Control Units/Control Units SIM/Main Control Unit (Sim)/Airbrakes Control/ABK/ABK/Algorothm Execution/Function-Call Subsystem'
//  '<S7>'   : 'CHADsimulator/Control Units/Control Units SIM/Main Control Unit (Sim)/Airbrakes Control/ABK/ABK/Algorothm Execution/Lower Bypass'
//  '<S8>'   : 'CHADsimulator/Control Units/Control Units SIM/Main Control Unit (Sim)/Airbrakes Control/ABK/ABK/Algorothm Execution/Upper Bypass'
//  '<S9>'   : 'CHADsimulator/Control Units/Control Units SIM/Main Control Unit (Sim)/Airbrakes Control/ABK/ABK/Algorothm Execution/Velocity interpolation'
//  '<S10>'  : 'CHADsimulator/Control Units/Control Units SIM/Main Control Unit (Sim)/Airbrakes Control/ABK/ABK/Algorothm Execution/Active control system/Linear interpolation'
//  '<S11>'  : 'CHADsimulator/Control Units/Control Units SIM/Main Control Unit (Sim)/Airbrakes Control/ABK/ABK/Algorothm Execution/Active control system/PID Controller'
//  '<S12>'  : 'CHADsimulator/Control Units/Control Units SIM/Main Control Unit (Sim)/Airbrakes Control/ABK/ABK/Algorothm Execution/Active control system/Variant Filter'
//  '<S13>'  : 'CHADsimulator/Control Units/Control Units SIM/Main Control Unit (Sim)/Airbrakes Control/ABK/ABK/Algorothm Execution/Active control system/PID Controller/ PID'
//  '<S14>'  : 'CHADsimulator/Control Units/Control Units SIM/Main Control Unit (Sim)/Airbrakes Control/ABK/ABK/Algorothm Execution/Active control system/PID Controller/ PID/FullClose_compare'
//  '<S15>'  : 'CHADsimulator/Control Units/Control Units SIM/Main Control Unit (Sim)/Airbrakes Control/ABK/ABK/Algorothm Execution/Active control system/PID Controller/ PID/FullOpen_compare'
//  '<S16>'  : 'CHADsimulator/Control Units/Control Units SIM/Main Control Unit (Sim)/Airbrakes Control/ABK/ABK/Algorothm Execution/Active control system/Variant Filter/ButterWorth Filter'
//  '<S17>'  : 'CHADsimulator/Control Units/Control Units SIM/Main Control Unit (Sim)/Airbrakes Control/ABK/ABK/Algorothm Execution/Active control system/Variant Filter/ButterWorth Filter/Butterworth filter'
//  '<S18>'  : 'CHADsimulator/Control Units/Control Units SIM/Main Control Unit (Sim)/Airbrakes Control/ABK/ABK/Algorothm Execution/Active control system/Variant Filter/ButterWorth Filter/FullOpen_compare'
//  '<S19>'  : 'CHADsimulator/Control Units/Control Units SIM/Main Control Unit (Sim)/Airbrakes Control/ABK/ABK/Algorothm Execution/Active control system/Variant Filter/ButterWorth Filter/Weight'
//  '<S20>'  : 'CHADsimulator/Control Units/Control Units SIM/Main Control Unit (Sim)/Airbrakes Control/ABK/ABK/Mach Check/ISA Atmosphere Model'
//  '<S21>'  : 'CHADsimulator/Control Units/Control Units SIM/Main Control Unit (Sim)/Airbrakes Control/ABK/ABK/Mach Check/Subsystem Reference'
//  '<S22>'  : 'CHADsimulator/Control Units/Control Units SIM/Main Control Unit (Sim)/Airbrakes Control/ABK/ABK/Mach Check/ISA Atmosphere Model/Modelling'


//-
//  Requirements for '<Root>': ABK


} // fine namespace ABK


#endif                                 // ABK_h_

//
// File trailer for generated code.
//
// [EOF]
//
