//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// File: SDA.h
//
// Code generated for Simulink model 'SDA'.
//
// Model version                  : 11.329
// Simulink Coder version         : 24.2 (R2024b) 21-Jun-2024
// C/C++ source code generated on : Mon Jun 29 14:46:21 2026
//
// Target selection: ert.tlc
// Embedded hardware selection: ARM Compatible->ARM Cortex-M
// Code generation objectives:
//    1. Execution efficiency
//    2. Debugging
//    3. RAM efficiency
// Validation result: Not run
//
#ifndef SDA_h_
#define SDA_h_
#include <stdbool.h>
#include <stdint.h>
#include "SDA_types.h"

namespace SDA 
{


// Exported data declaration

// Declaration for custom storage class: ExportToFile
extern float kriging[5120];            // Referenced by: '<S6>/Kriging Data'

// Class declaration for model SDA
class SDA final
{
  // public data and function members
 public:
  // Block signals and states (default storage) for system '<Root>'
  struct DW_SDA_T {
    float prediction_samples_DSTATE[30];// '<S9>/prediction_samples'
    float time_samples_DSTATE[30];     // '<S9>/time_samples'
    float Power[1600];                 // '<S6>/Power'
    float time_samples1_DSTATE;        // '<S9>/time_samples1'
    uint8_t UnitDelay_DSTATE;          // '<S1>/Unit Delay'
  };

  // External inputs (root inport signals with default storage)
  struct ExtU_SDA_T {
    SDAIn SDAIN;                       // '<Root>/SDA IN'
  };

  // External outputs (root outports fed by signals with default storage)
  struct ExtY_SDA_T {
    bool SDAShutdown;                  // '<Root>/SDA Shutdown'
    SDALogs SDALogsOBSW;               // '<Root>/SDA Logs OBSW'
  };

  // Parameters (default storage)
  struct P_SDA_T {
    float CompareToConstant2_const;  // Mask Parameter: CompareToConstant2_const
                                        //  Referenced by: '<S2>/Constant'

    uint8_t CompareToConstant3_const;// Mask Parameter: CompareToConstant3_const
                                        //  Referenced by: '<S3>/Constant'

    float prediction_samples_InitialCondi[30];
                                     // Expression: single(zeros(predSamples,1))
                                        //  Referenced by: '<S9>/prediction_samples'

    float time_samples_InitialCondition[30];
                                     // Expression: single(zeros(predSamples,1))
                                        //  Referenced by: '<S9>/time_samples'

    float time_samples1_InitialCondition;// Expression: single(0)
                                            //  Referenced by: '<S9>/time_samples1'

    float Gain2_Gain;                  // Computed Parameter: Gain2_Gain
                                          //  Referenced by: '<S9>/Gain2'

    float Gain1_Gain;                  // Computed Parameter: Gain1_Gain
                                          //  Referenced by: '<S9>/Gain1'

    float Gain_Gain;                   // Computed Parameter: Gain_Gain
                                          //  Referenced by: '<S9>/Gain'

    float Constant15_Value;            // Computed Parameter: Constant15_Value
                                          //  Referenced by: '<S7>/Constant15'

    float Constant_Value;              // Computed Parameter: Constant_Value
                                          //  Referenced by: '<S9>/Constant'

    float Zero_Value;                  // Computed Parameter: Zero_Value
                                          //  Referenced by: '<S4>/Zero'

    float Constant_Value_d[10];        // Computed Parameter: Constant_Value_d
                                          //  Referenced by: '<S1>/Constant'

    float Gain_Gain_m;                 // Computed Parameter: Gain_Gain_m
                                          //  Referenced by: '<S6>/Gain'

    float Bias_Bias;                   // Computed Parameter: Bias_Bias
                                          //  Referenced by: '<S6>/Bias'

    float Gain2_Gain_i;                // Computed Parameter: Gain2_Gain_i
                                          //  Referenced by: '<S6>/Gain2'

    float Bias1_Bias;                  // Computed Parameter: Bias1_Bias
                                          //  Referenced by: '<S6>/Bias1'

    bool Constant2_Value;              // Expression: transFlag
                                          //  Referenced by: '<S1>/Constant2'

    int8_t One_Value;                  // Computed Parameter: One_Value
                                          //  Referenced by: '<S1>/One'

    int8_t Gain1_Gain_g;               // Computed Parameter: Gain1_Gain_g
                                          //  Referenced by: '<S1>/Gain1'

    uint8_t Gain3_Gain;                // Computed Parameter: Gain3_Gain
                                          //  Referenced by: '<S7>/Gain3'

    uint8_t Constant12_Value;          // Computed Parameter: Constant12_Value
                                          //  Referenced by: '<S7>/Constant12'

    uint8_t Constant11_Value;          // Computed Parameter: Constant11_Value
                                          //  Referenced by: '<S7>/Constant11'

    uint8_t Switch4_Threshold;         // Computed Parameter: Switch4_Threshold
                                          //  Referenced by: '<S7>/Switch4'

    uint8_t UnitDelay_InitialCondition;
                               // Computed Parameter: UnitDelay_InitialCondition
                                  //  Referenced by: '<S1>/Unit Delay'

    uint8_t Saturation_UpperSat;      // Computed Parameter: Saturation_UpperSat
                                         //  Referenced by: '<S1>/Saturation'

    uint8_t Saturation_LowerSat;      // Computed Parameter: Saturation_LowerSat
                                         //  Referenced by: '<S1>/Saturation'

  };

  // Copy Constructor
  SDA(SDA const&) = delete;

  // Assignment Operator
  SDA& operator= (SDA const&) & = delete;

  // Move Constructor
  SDA(SDA &&) = delete;

  // Move Assignment Operator
  SDA& operator= (SDA &&) = delete;

  // Root inport: '<Root>/SDA IN' set method
  void setSDA_IN(SDAIn localArgInput)
  {
    SDA_U.SDAIN = localArgInput;
  }

  // Root outport: '<Root>/SDA Shutdown' get method
  bool getSDA_Shutdown() const
  {
    return SDA_Y.SDAShutdown;
  }

  // Root outport: '<Root>/SDA Logs OBSW' get method
  SDALogs getSDA_Logs_OBSW() const
  {
    return SDA_Y.SDALogsOBSW;
  }

  // Block parameters get method
  const P_SDA_T &getBlockParameters() const
  {
    return SDA_P;
  }

  // Block parameters set method
  void setBlockParameters(const P_SDA_T *pP_SDA_T) const
  {
    SDA_P = *pP_SDA_T;
  }

  // model initialize function
  void initialize();

  // model step function
  void step();

  // model terminate function
  static void terminate();

  // Constructor
  SDA();

  // Destructor
  ~SDA();

  // private data and function members
 private:
  // External inputs
  ExtU_SDA_T SDA_U;

  // External outputs
  ExtY_SDA_T SDA_Y;

  // Block states
  DW_SDA_T SDA_DW;

  // Tunable parameters
  static P_SDA_T SDA_P;
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
//  hilite_system('CHADsimulator/Control Units/Control Units SIM/Main Control Unit (Sim)/Engine Shutdown/SDA/SDA')    - opens subsystem CHADsimulator/Control Units/Control Units SIM/Main Control Unit (Sim)/Engine Shutdown/SDA/SDA
//  hilite_system('CHADsimulator/Control Units/Control Units SIM/Main Control Unit (Sim)/Engine Shutdown/SDA/SDA/Kp') - opens and selects block Kp
//
//  Here is the system hierarchy for this model
//
//  '<Root>' : 'CHADsimulator/Control Units/Control Units SIM/Main Control Unit (Sim)/Engine Shutdown/SDA'
//  '<S1>'   : 'CHADsimulator/Control Units/Control Units SIM/Main Control Unit (Sim)/Engine Shutdown/SDA/SDA'
//  '<S2>'   : 'CHADsimulator/Control Units/Control Units SIM/Main Control Unit (Sim)/Engine Shutdown/SDA/SDA/Compare To Constant2'
//  '<S3>'   : 'CHADsimulator/Control Units/Control Units SIM/Main Control Unit (Sim)/Engine Shutdown/SDA/SDA/Compare To Constant3'
//  '<S4>'   : 'CHADsimulator/Control Units/Control Units SIM/Main Control Unit (Sim)/Engine Shutdown/SDA/SDA/No Propagation'
//  '<S5>'   : 'CHADsimulator/Control Units/Control Units SIM/Main Control Unit (Sim)/Engine Shutdown/SDA/SDA/Physical To Unitary Normalization'
//  '<S6>'   : 'CHADsimulator/Control Units/Control Units SIM/Main Control Unit (Sim)/Engine Shutdown/SDA/SDA/Prediction'
//  '<S7>'   : 'CHADsimulator/Control Units/Control Units SIM/Main Control Unit (Sim)/Engine Shutdown/SDA/SDA/Propagation'
//  '<S8>'   : 'CHADsimulator/Control Units/Control Units SIM/Main Control Unit (Sim)/Engine Shutdown/SDA/SDA/Subsystem Reference'
//  '<S9>'   : 'CHADsimulator/Control Units/Control Units SIM/Main Control Unit (Sim)/Engine Shutdown/SDA/SDA/Propagation/Interpolation'


//-
//  Requirements for '<Root>': SDA


#endif                                 // SDA_h_


} // fine namespace SDA

//
// File trailer for generated code.
//
// [EOF]
//
