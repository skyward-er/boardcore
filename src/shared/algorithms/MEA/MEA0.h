//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// File: MEA0.h
//
// Code generated for Simulink model 'MEA0'.
//
// Model version                  : 11.125
// Simulink Coder version         : 24.2 (R2024b) 21-Jun-2024
// C/C++ source code generated on : Sun Feb 22 11:31:50 2026
//
// Target selection: ert.tlc
// Embedded hardware selection: ARM Compatible->ARM Cortex-M
// Code generation objective: RAM efficiency
// Validation result: Passed (3), Warning (1), Error (1)
//
#ifndef MEA0_h_
#define MEA0_h_
#include <stdbool.h>
#include <stdint.h>
#include "MEA0_types.h"
#include "multiword_types.h"

// Custom Skyward Includes
#include <drivers/timer/TimestampTimer.h>
#include <utils/AeroUtils/AeroUtils.h>

// Class declaration for model MEA0
class MEA0 final
{
  // public data and function members
 public:

  // Block signals (default storage)
  struct B_MEA0_T {
    uint64m_T Memory;                  // '<S1>/Memory'
    float Constant[2];                 // '<S1>/Constant'
    float Constant1;                   // '<S1>/Constant1'
    float Constant1_a[2];              // '<S4>/Constant1'
    float Bu[2];                       // '<S4>/Matrix Multiply2'
    float Constant_i[4];               // '<S4>/Constant'
    float x_memory[2];                 // '<S1>/x_memory'
    float Ax[2];                       // '<S4>/Matrix Multiply'
    float AxBu[2];                     // '<S4>/Add1'
    float P_memory[4];                 // '<S1>/P_memory'
    float Transpose[4];                // '<S4>/Transpose'
    float APA[4];                      // '<S4>/Matrix Multiply1'
    float Constant2[4];                // '<S4>/Constant2'
    float APAQ[4];                     // '<S4>/Add'
    float x_est[2];                    // '<S1>/Merge'
    float MatrixMultiply;              // '<S1>/Matrix Multiply'
    float x_est_c[4];                  // '<S1>/Merge1'
    float m_dot;                       // '<S1>/Product'
    float Product1;                    // '<S1>/Product1'
    float m_old;                       // '<S1>/mass_memory'
    float m_new;                       // '<S1>/Subtract'
    float Saturation;                  // '<S1>/Saturation'
    float Constant_o[2];               // '<S3>/Constant'
    float Transpose_g[2];              // '<S3>/Transpose'
    float CPC;                         // '<S3>/Matrix Multiply'
    float R;                           // '<S3>/Constant1'
    float CPCR;                        // '<S3>/Sum'
    float I[4];                        // '<S6>/IdentityMatrix'
    float Constant_ow[2];              // '<S6>/Constant'
    float Transpose_a[2];              // '<S6>/Transpose'
    float PC[2];                       // '<S6>/Matrix Multiply'
    float K[2];                        // '<S6>/Divide'
    float KC[4];                       // '<S6>/Matrix Multiply1'
    float IKC[4];                      // '<S6>/Add'
    float Cx;                          // '<S6>/Matrix Multiply3'
    float e;                           // '<S6>/Add1'
    float Ke[2];                       // '<S6>/Product'
    float R_k;                         // '<S6>/Constant1'
    float K_d[2];                      // '<S6>/Transpose2'
    float KRK[4];                      // '<S6>/Matrix Multiply4'
    float IKC_f[4];                    // '<S6>/Transpose1'
    float IKCPIKC[4];                  // '<S6>/Matrix Multiply2'
    bool RelationalOperator;           // '<S1>/Relational Operator'
    bool NOT;                          // '<S1>/NOT'
    bool Compare;                      // '<S5>/Compare'
    bool NOT_i;                        // '<S3>/NOT'
  };

  // Block states (default storage) for system '<Root>'
  struct DW_MEA0_T {
    uint64m_T Memory_PreviousInput;    // '<S1>/Memory'
    float x_memory_PreviousInput[2];   // '<S1>/x_memory'
    float P_memory_PreviousInput[4];   // '<S1>/P_memory'
    float MatrixMultiply1_DWORK1[4];   // '<S4>/Matrix Multiply1'
    float mass_memory_PreviousInput;   // '<S1>/mass_memory'
  };

  // External inputs (root inport signals with default storage)
  struct ExtU_MEA0_T {
    bool MainValvePosition;            // '<Root>/Main Valve Position'
    Bus_MEA_CC CombustionChamberBus;   // '<Root>/Combustion Chamber Bus'
  };

  // External outputs (root outports fed by signals with default storage)
  struct ExtY_MEA0_T {
    float Mass;                        // '<Root>/Mass'
    float Sates[2];                    // '<Root>/Sates'
    float Pressure;                    // '<Root>/Pressure'
  };

  // Parameters (default storage)
  struct P_MEA0_T {
    float CompareToConstant_const;    // Mask Parameter: CompareToConstant_const
                                         //  Referenced by: '<S5>/Constant'

    uint64m_T Memory_InitialCondition;
                                  // Computed Parameter: Memory_InitialCondition
                                     //  Referenced by: '<S1>/Memory'

    float IdentityMatrix_IDMatrixData[4];
                              // Computed Parameter: IdentityMatrix_IDMatrixData
                                 //  Referenced by: '<S6>/IdentityMatrix'

    float Constant_Value[2];           // Expression: mea.CObservable
                                          //  Referenced by: '<S6>/Constant'

    float Constant1_Value;             // Expression: mea.baroR
                                          //  Referenced by: '<S6>/Constant1'

    float Constant_Value_p[2];         // Expression: mea.CObservable
                                          //  Referenced by: '<S3>/Constant'

    float Constant1_Value_k;           // Expression: mea.baroR
                                          //  Referenced by: '<S3>/Constant1'

    float Constant_Value_o[2];         // Expression: mea.CObservable
                                          //  Referenced by: '<S1>/Constant'

    float Constant1_Value_g;           // Expression: mea.massCoeff
                                          //  Referenced by: '<S1>/Constant1'

    float Constant2_Value;             // Computed Parameter: Constant2_Value
                                          //  Referenced by: '<S1>/Constant2'

    float Constant1_Value_m[2];        // Expression: mea.BObservable
                                          //  Referenced by: '<S4>/Constant1'

    float Constant_Value_i[4];         // Expression: mea.AObservable
                                          //  Referenced by: '<S4>/Constant'

    float x_memory_InitialCondition[2];
                                // Computed Parameter: x_memory_InitialCondition
                                   //  Referenced by: '<S1>/x_memory'

    float P_memory_InitialCondition[4];// Expression: mea.P0Observable
                                          //  Referenced by: '<S1>/P_memory'

    float Constant2_Value_j[4];        // Expression: mea.QObservable
                                          //  Referenced by: '<S4>/Constant2'

    float mass_memory_InitialCondition;
                             // Computed Parameter: mass_memory_InitialCondition
                                //  Referenced by: '<S1>/mass_memory'

    float Saturation_UpperSat;         // Expression: mea.maxMass
                                          //  Referenced by: '<S1>/Saturation'

    float Saturation_LowerSat;         // Expression: mea.minMass
                                          //  Referenced by: '<S1>/Saturation'

  };

  // Real-time Model Data Structure
  struct RT_MODEL_MEA0_T {
    //
    //  Timing:
    //  The following substructure contains information regarding
    //  the timing information for the model.

    struct {
      struct {
        uint8_t TID[2];
      } TaskCounters;
    } Timing;
  };

  // Copy Constructor
  MEA0(MEA0 const&) = delete;

  // Assignment Operator
  MEA0& operator= (MEA0 const&) & = delete;

  // Move Constructor
  MEA0(MEA0 &&) = delete;

  // Move Assignment Operator
  MEA0& operator= (MEA0 &&) = delete;

  // Real-Time Model get method
  MEA0::RT_MODEL_MEA0_T * getRTM();

  // Root inports set method
  void setExternalInputs(const ExtU_MEA0_T *pExtU_MEA0_T)
  {
    MEA0_U = *pExtU_MEA0_T;
  }

  // Root outports get method
  const ExtY_MEA0_T &getExternalOutputs() const
  {
    return MEA0_Y;
  }

  // model initialize function
  void initialize();

  // Custom step method
  void step(float verticalSpeed, float mslAltitude);
  
  // Custom additions
  MEAState getState();
  float computeForce(float verticalSpeed, float mslAltitude);
  float computeApogee(float verticalSpeed, float mslAltitude);

  // model terminate function
  static void terminate();

  // Constructor
  MEA0();

  // Destructor
  ~MEA0();

  // private data and function members
 private:
  // Custom fields
  float lastVerticalSpeed = 0.0f;
  float lastMslAltitude = 0.0f;

  // External inputs
  ExtU_MEA0_T MEA0_U;

  // External outputs
  ExtY_MEA0_T MEA0_Y;

  // Block signals
  B_MEA0_T MEA0_B;

  // Block states
  DW_MEA0_T MEA0_DW;

  // Tunable parameters
  static P_MEA0_T MEA0_P;

  // Real-Time Model
  RT_MODEL_MEA0_T MEA0_M;
};

//-
//  These blocks were eliminated from the model due to optimizations:
//
//  Block '<S1>/Reshape' : Reshape block reduction
//  Block '<S3>/Reshape' : Reshape block reduction
//  Block '<S6>/Reshape' : Reshape block reduction
//  Block '<S6>/Reshape1' : Reshape block reduction


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
//  hilite_system('CHADsimulator/Control Units/Control Units SIM/Main Control Unit (Sim)/MEA - SDA/MEA/MEA - Biliquid')    - opens subsystem CHADsimulator/Control Units/Control Units SIM/Main Control Unit (Sim)/MEA - SDA/MEA/MEA - Biliquid
//  hilite_system('CHADsimulator/Control Units/Control Units SIM/Main Control Unit (Sim)/MEA - SDA/MEA/MEA - Biliquid/Kp') - opens and selects block Kp
//
//  Here is the system hierarchy for this model
//
//  '<Root>' : 'CHADsimulator/Control Units/Control Units SIM/Main Control Unit (Sim)/MEA - SDA/MEA'
//  '<S1>'   : 'CHADsimulator/Control Units/Control Units SIM/Main Control Unit (Sim)/MEA - SDA/MEA/MEA - Biliquid'
//  '<S2>'   : 'CHADsimulator/Control Units/Control Units SIM/Main Control Unit (Sim)/MEA - SDA/MEA/MEA - Biliquid/No Correction Step'
//  '<S3>'   : 'CHADsimulator/Control Units/Control Units SIM/Main Control Unit (Sim)/MEA - SDA/MEA/MEA - Biliquid/cc baro correction'
//  '<S4>'   : 'CHADsimulator/Control Units/Control Units SIM/Main Control Unit (Sim)/MEA - SDA/MEA/MEA - Biliquid/prediction'
//  '<S5>'   : 'CHADsimulator/Control Units/Control Units SIM/Main Control Unit (Sim)/MEA - SDA/MEA/MEA - Biliquid/cc baro correction/Compare To Constant'
//  '<S6>'   : 'CHADsimulator/Control Units/Control Units SIM/Main Control Unit (Sim)/MEA - SDA/MEA/MEA - Biliquid/cc baro correction/Subsystem'
//  '<S7>'   : 'CHADsimulator/Control Units/Control Units SIM/Main Control Unit (Sim)/MEA - SDA/MEA/MEA - Biliquid/cc baro correction/Subsystem1'


//-
//  Requirements for '<Root>': MEA0


#endif                                 

// MEA0_h_
//
// File trailer for generated code.
//
// [EOF]
//