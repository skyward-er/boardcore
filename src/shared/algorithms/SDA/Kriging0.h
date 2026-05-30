//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// File: Kriging0.h
//
// Code generated for Simulink model 'Kriging0'.
//
// Model version                  : 11.257
// Simulink Coder version         : 24.2 (R2024b) 21-Jun-2024
// C/C++ source code generated on : Sun May 24 16:00:49 2026
//
// Target selection: ert.tlc
// Embedded hardware selection: ARM Compatible->ARM Cortex-M
// Code generation objectives:
//    1. Execution efficiency
//    2. Debugging
//    3. RAM efficiency
// Validation result: Not run
//
#ifndef Kriging0_h_
#define Kriging0_h_
#include <stdbool.h>
#include <stdint.h>
#include "Kriging0_types.h"

// Exported data declaration

// Declaration for custom storage class: ExportToFile
extern struct_6Rnfmp5FIUwEnZoyXFhchB kriging;// Referenced by:
                                                //  '<S1>/Constant'
                                                //  '<S6>/Bias'
                                                //  '<S6>/Bias1'
                                                //  '<S6>/Constant3'
                                                //  '<S6>/Constant4'
                                                //  '<S6>/Constant6'
                                                //  '<S6>/Gain1'
                                                //  '<S6>/Gain2'


// Class declaration for model Kriging0
class Kriging0 final
{
  // public data and function members
 public:
  // Block signals and states (default storage) for system '<Root>'
  struct DW_Kriging0_T {
    float Gain1[1600];                 // '<S6>/Gain1'
    uint8_t UnitDelay_DSTATE;          // '<S1>/Unit Delay'
  };

  // External inputs (root inport signals with default storage)
  struct ExtU_Kriging0_T {
    SDAIn SDAIN;                       // '<Root>/SDA IN'
  };

  // External outputs (root outports fed by signals with default storage)
  struct ExtY_Kriging0_T {
    bool SDAShutdown;                  // '<Root>/SDA Shutdown'
    SDALogs SDALogsOBSW;               // '<Root>/SDA Logs OBSW'
  };

  // Parameters (default storage)
  struct P_Kriging0_T {
    float CompareToConstant2_const;  // Mask Parameter: CompareToConstant2_const
                                        //  Referenced by: '<S2>/Constant'

    uint8_t CompareToConstant3_const;// Mask Parameter: CompareToConstant3_const
                                        //  Referenced by: '<S3>/Constant'

    float Constant12_Value;            // Computed Parameter: Constant12_Value
                                          //  Referenced by: '<S7>/Constant12'

    float Constant11_Value;            // Computed Parameter: Constant11_Value
                                          //  Referenced by: '<S7>/Constant11'

    float Switch4_Threshold;           // Computed Parameter: Switch4_Threshold
                                          //  Referenced by: '<S7>/Switch4'

    float Gain3_Gain;                  // Computed Parameter: Gain3_Gain
                                          //  Referenced by: '<S7>/Gain3'

    float Constant15_Value;            // Computed Parameter: Constant15_Value
                                          //  Referenced by: '<S7>/Constant15'

    float Gain_Gain;                   // Computed Parameter: Gain_Gain
                                          //  Referenced by: '<S6>/Gain'

    float Constant2_Value;             // Computed Parameter: Constant2_Value
                                          //  Referenced by: '<S1>/Constant2'

    int8_t One_Value;                  // Computed Parameter: One_Value
                                          //  Referenced by: '<S1>/One'

    int8_t Gain1_Gain;                 // Computed Parameter: Gain1_Gain
                                          //  Referenced by: '<S1>/Gain1'

    uint8_t UnitDelay_InitialCondition;
                               // Computed Parameter: UnitDelay_InitialCondition
                                  //  Referenced by: '<S1>/Unit Delay'

    uint8_t Saturation_UpperSat;      // Computed Parameter: Saturation_UpperSat
                                         //  Referenced by: '<S1>/Saturation'

    uint8_t Saturation_LowerSat;      // Computed Parameter: Saturation_LowerSat
                                         //  Referenced by: '<S1>/Saturation'

  };

  // Copy Constructor
  Kriging0(Kriging0 const&) = delete;

  // Assignment Operator
  Kriging0& operator= (Kriging0 const&) & = delete;

  // Move Constructor
  Kriging0(Kriging0 &&) = delete;

  // Move Assignment Operator
  Kriging0& operator= (Kriging0 &&) = delete;

  // Root inport: '<Root>/SDA IN' set method
  void setSDA_IN(SDAIn localArgInput)
  {
    Kriging0_U.SDAIN = localArgInput;
  }

  // Root outport: '<Root>/SDA Shutdown' get method
  bool getSDA_Shutdown() const
  {
    return Kriging0_Y.SDAShutdown;
  }

  // Root outport: '<Root>/SDA Logs OBSW' get method
  SDALogs getSDA_Logs_OBSW() const
  {
    return Kriging0_Y.SDALogsOBSW;
  }

  // Block parameters get method
  const P_Kriging0_T &getBlockParameters() const
  {
    return Kriging0_P;
  }

  // Block parameters set method
  void setBlockParameters(const P_Kriging0_T *pP_Kriging0_T) const
  {
    Kriging0_P = *pP_Kriging0_T;
  }

  // model initialize function
  void initialize();

  // model step function
  void step();

  // model terminate function
  static void terminate();

  // Constructor
  Kriging0();

  // Destructor
  ~Kriging0();

  // private data and function members
 private:
  // External inputs
  ExtU_Kriging0_T Kriging0_U;

  // External outputs
  ExtY_Kriging0_T Kriging0_Y;

  // Block states
  DW_Kriging0_T Kriging0_DW;

  // Tunable parameters
  static P_Kriging0_T Kriging0_P;
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
//  hilite_system('CHADsimulator/Control Units/Control Units SIM/Main Control Unit (Sim)/Engine Shutdown/SDA/Kriging')    - opens subsystem CHADsimulator/Control Units/Control Units SIM/Main Control Unit (Sim)/Engine Shutdown/SDA/Kriging
//  hilite_system('CHADsimulator/Control Units/Control Units SIM/Main Control Unit (Sim)/Engine Shutdown/SDA/Kriging/Kp') - opens and selects block Kp
//
//  Here is the system hierarchy for this model
//
//  '<Root>' : 'CHADsimulator/Control Units/Control Units SIM/Main Control Unit (Sim)/Engine Shutdown/SDA'
//  '<S1>'   : 'CHADsimulator/Control Units/Control Units SIM/Main Control Unit (Sim)/Engine Shutdown/SDA/Kriging'
//  '<S2>'   : 'CHADsimulator/Control Units/Control Units SIM/Main Control Unit (Sim)/Engine Shutdown/SDA/Kriging/Compare To Constant2'
//  '<S3>'   : 'CHADsimulator/Control Units/Control Units SIM/Main Control Unit (Sim)/Engine Shutdown/SDA/Kriging/Compare To Constant3'
//  '<S4>'   : 'CHADsimulator/Control Units/Control Units SIM/Main Control Unit (Sim)/Engine Shutdown/SDA/Kriging/No Propagation'
//  '<S5>'   : 'CHADsimulator/Control Units/Control Units SIM/Main Control Unit (Sim)/Engine Shutdown/SDA/Kriging/Physical To Unitary Normalization'
//  '<S6>'   : 'CHADsimulator/Control Units/Control Units SIM/Main Control Unit (Sim)/Engine Shutdown/SDA/Kriging/Prediction'
//  '<S7>'   : 'CHADsimulator/Control Units/Control Units SIM/Main Control Unit (Sim)/Engine Shutdown/SDA/Kriging/Propagation'
//  '<S8>'   : 'CHADsimulator/Control Units/Control Units SIM/Main Control Unit (Sim)/Engine Shutdown/SDA/Kriging/Subsystem Reference'


//-
//  Requirements for '<Root>': Kriging0


#endif                                 // Kriging0_h_

//
// File trailer for generated code.
//
// [EOF]
//
