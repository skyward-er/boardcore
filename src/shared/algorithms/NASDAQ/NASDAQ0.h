//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// File: NASDAQ0.h
//
// Code generated for Simulink model 'NASDAQ0'.
//
// Model version                  : 11.240
// Simulink Coder version         : 24.2 (R2024b) 21-Jun-2024
// C/C++ source code generated on : Tue May 12 11:57:35 2026
//
// Target selection: ert.tlc
// Embedded hardware selection: STMicroelectronics->ST10/Super10
// Code generation objectives:
//    1. Execution efficiency
//    2. Debugging
//    3. RAM efficiency
// Validation result: Not run
//
#ifndef NASDAQ0_h_
#define NASDAQ0_h_
#include <stdbool.h>
#include <stdint.h>
#include "NASDAQ0_types.h"

// Class declaration for model NASDAQ0
class NASDAQ0 final
{
  // public data and function members
 public:
  // Block signals and states (default storage) for system '<S11>/Correction'
  struct DW_Correction_NASDAQ0_T {
    float MatrixDivide_DWORK4;         // '<S14>/Matrix Divide'
  };

  // Block signals and states (default storage) for system '<Root>'
  struct DW_NASDAQ0_T {
    DW_Correction_NASDAQ0_T Correction_b;// '<S24>/Correction'
    DW_Correction_NASDAQ0_T Correction;// '<S11>/Correction'
    ISAReference UnitDelay1_DSTATE;    // '<S19>/Unit Delay1'
    uint64_t Memory_PreviousInput;     // '<S34>/Memory'
    uint64_t Memory_PreviousInput_d;   // '<S20>/Memory'
    uint64_t Memory_PreviousInput_a;   // '<S8>/Memory'
    float VectorConcatenate[6];        // '<S1>/Vector Concatenate'
    float IdentityMatrix[16];          // '<S46>/Identity Matrix'
    float Merge[36];                   // '<S4>/Merge'
    float Merge1[6];                   // '<S4>/Merge1'
    float UnitDelay_DSTATE[36];        // '<S1>/Unit Delay'
    float UnitDelay1_DSTATE_h[6];      // '<S1>/Unit Delay1'
    float MatrixDivide_DWORK4[16];     // '<S42>/Matrix Divide'
    float IdentityMatrix_h;            // '<S31>/Identity Matrix'
    float Gain;                        // '<S4>/Gain'
    float Switch;                      // '<S7>/Switch'
    float IdentityMatrix_b;            // '<S18>/Identity Matrix'
    uint8_t UnitDelay3_DSTATE;         // '<S1>/Unit Delay3'
    uint8_t UnitDelay2_DSTATE;         // '<S1>/Unit Delay2'
    uint8_t UnitDelay_DSTATE_j;        // '<S19>/Unit Delay'
  };

  // External inputs (root inport signals with default storage)
  struct ExtU_NASDAQ0_T {
    NASDAQInSensors NASDAQInSensors_i; // '<Root>/NASDAQ In Sensors'
    NASDAQInADA NASDAQInADA_i;         // '<Root>/NASDAQ In ADA'
    ANAS_NASDAQ NASDAQInANAS;          // '<Root>/NASDAQ In ANAS'
    ISAReference NASDAQInReference;    // '<Root>/NASDAQ In Reference'
  };

  // External outputs (root outports fed by signals with default storage)
  struct ExtY_NASDAQ0_T {
    NASDAQOut NASDAQOut_l;             // '<Root>/NASDAQ Out'
    NASDAQLogs NASDAQLogsOBSW;         // '<Root>/NASDAQ Logs OBSW'
  };

  // Parameters for system: '<S11>/No correction'
  struct P_Nocorrection_NASDAQ0_T {
    float Gain_Gain;                   // Computed Parameter: Gain_Gain
                                          //  Referenced by: '<S15>/Gain'

  };

  // Parameters (default storage)
  struct P_NASDAQ0_T {
    ISAReference UnitDelay1_InitialCondition;
                              // Computed Parameter: UnitDelay1_InitialCondition
                                 //  Referenced by: '<S19>/Unit Delay1'

    double NASCondLim_Value;           // Expression: minCond
                                          //  Referenced by: '<S11>/NASCondLim'

    double Constant_Value;             // Expression: flagADA
                                          //  Referenced by: '<S8>/Constant'

    double NASCondLim_Value_l;         // Expression: minCond
                                          //  Referenced by: '<S24>/NASCondLim'

    double NASCondLim_Value_o;         // Expression: minCond
                                          //  Referenced by: '<S38>/NASCondLim'

    double Constant_Value_a;           // Expression: flagGPS
                                          //  Referenced by: '<S34>/Constant'

    uint64_t Memory_InitialCondition;
                                  // Computed Parameter: Memory_InitialCondition
                                     //  Referenced by: '<S8>/Memory'

    uint64_t Memory_InitialCondition_m;
                                // Computed Parameter: Memory_InitialCondition_m
                                   //  Referenced by: '<S20>/Memory'

    uint64_t Memory_InitialCondition_o;
                                // Computed Parameter: Memory_InitialCondition_o
                                   //  Referenced by: '<S34>/Memory'

    float Constant_Value_g;            // Computed Parameter: Constant_Value_g
                                          //  Referenced by: '<S16>/Constant'

    float IdentityMatrix_IDMatrixData;
                              // Computed Parameter: IdentityMatrix_IDMatrixData
                                 //  Referenced by: '<S18>/Identity Matrix'

    float Constant_Value_an;           // Computed Parameter: Constant_Value_an
                                          //  Referenced by: '<S7>/Constant'

    float Constant_Value_f[36];        // Computed Parameter: Constant_Value_f
                                          //  Referenced by: '<S13>/Constant'

    float Gain_Gain;                   // Computed Parameter: Gain_Gain
                                          //  Referenced by: '<S4>/Gain'

    float Constant_Value_h;            // Computed Parameter: Constant_Value_h
                                          //  Referenced by: '<S29>/Constant'

    float IdentityMatrix_IDMatrixData_b;
                            // Computed Parameter: IdentityMatrix_IDMatrixData_b
                               //  Referenced by: '<S31>/Identity Matrix'

    float Constant_Value_c[36];        // Computed Parameter: Constant_Value_c
                                          //  Referenced by: '<S26>/Constant'

    float Constant_Value_n[2];         // Computed Parameter: Constant_Value_n
                                          //  Referenced by: '<S23>/Constant'

    float Gain3_Gain;                  // Computed Parameter: Gain3_Gain
                                          //  Referenced by: '<S23>/Gain3'

    float Gain4_Gain;                  // Computed Parameter: Gain4_Gain
                                          //  Referenced by: '<S23>/Gain4'

    float gR_Value;                    // Computed Parameter: gR_Value
                                          //  Referenced by: '<S23>/g R'

    float Constant1_Value[3];          // Computed Parameter: Constant1_Value
                                          //  Referenced by: '<S23>/Constant1'

    float Constant_Value_p;            // Computed Parameter: Constant_Value_p
                                          //  Referenced by: '<S19>/Constant'

    float Gain_Gain_d;                 // Computed Parameter: Gain_Gain_d
                                          //  Referenced by: '<S25>/Gain'

    float HeightTemperatureGradient_Value;
                          // Computed Parameter: HeightTemperatureGradient_Value
                             //  Referenced by: '<S32>/HeightTemperatureGradient'

    float gravity_Value;               // Computed Parameter: gravity_Value
                                          //  Referenced by: '<S32>/gravity'

    float Rair_Value;                  // Computed Parameter: Rair_Value
                                          //  Referenced by: '<S32>/R air'

    float Gain_Gain_dp;                // Computed Parameter: Gain_Gain_dp
                                          //  Referenced by: '<S43>/Gain'

    float Constant_Value_l;            // Computed Parameter: Constant_Value_l
                                          //  Referenced by: '<S44>/Constant'

    float IdentityMatrix_IDMatrixData_a[16];
                            // Computed Parameter: IdentityMatrix_IDMatrixData_a
                               //  Referenced by: '<S46>/Identity Matrix'

    float Constant_Value_p1[36];       // Computed Parameter: Constant_Value_p1
                                          //  Referenced by: '<S40>/Constant'

    float Constant_Value_fg;           // Computed Parameter: Constant_Value_fg
                                          //  Referenced by: '<S37>/Constant'

    float Constant1_Value_f;           // Computed Parameter: Constant1_Value_f
                                          //  Referenced by: '<S37>/Constant1'

    float Gain_Gain_e;                 // Computed Parameter: Gain_Gain_e
                                          //  Referenced by: '<S37>/Gain'

    float Bias_Bias;                   // Computed Parameter: Bias_Bias
                                          //  Referenced by: '<S37>/Bias'

    float Gain1_Gain;                  // Computed Parameter: Gain1_Gain
                                          //  Referenced by: '<S41>/Gain1'

    float Constant3_Value;             // Computed Parameter: Constant3_Value
                                          //  Referenced by: '<S37>/Constant3'

    float Constant4_Value;             // Computed Parameter: Constant4_Value
                                          //  Referenced by: '<S37>/Constant4'

    float Gain1_Gain_n;                // Computed Parameter: Gain1_Gain_n
                                          //  Referenced by: '<S37>/Gain1'

    float Gain3_Gain_k;                // Computed Parameter: Gain3_Gain_k
                                          //  Referenced by: '<S37>/Gain3'

    float Constant5_Value[8];          // Computed Parameter: Constant5_Value
                                          //  Referenced by: '<S37>/Constant5'

    float Constant6_Value[12];         // Computed Parameter: Constant6_Value
                                          //  Referenced by: '<S37>/Constant6'

    float Constant_Value_o[16];        // Computed Parameter: Constant_Value_o
                                          //  Referenced by: '<S33>/Constant'

    float Gain3_Gain_h;                // Computed Parameter: Gain3_Gain_h
                                          //  Referenced by: '<S39>/Gain3'

    float Bias_Bias_l;                 // Computed Parameter: Bias_Bias_l
                                          //  Referenced by: '<S39>/Bias'

    float Gain1_Gain_d;                // Computed Parameter: Gain1_Gain_d
                                          //  Referenced by: '<S47>/Gain1'

    float Gain4_Gain_l;                // Computed Parameter: Gain4_Gain_l
                                          //  Referenced by: '<S39>/Gain4'

    float Bias1_Bias;                  // Computed Parameter: Bias1_Bias
                                          //  Referenced by: '<S39>/Bias1'

    float Gain_Gain_l[4];              // Computed Parameter: Gain_Gain_l
                                          //  Referenced by: '<S39>/Gain'

    float Constant1_Value_j[36];       // Computed Parameter: Constant1_Value_j
                                          //  Referenced by: '<S3>/Constant1'

    float Gain1_Gain_h;                // Computed Parameter: Gain1_Gain_h
                                          //  Referenced by: '<S3>/Gain1'

    float Bias_Bias_d[36];             // Computed Parameter: Bias_Bias_d
                                          //  Referenced by: '<S3>/Bias'

    float Bias1_Bias_h[36];            // Computed Parameter: Bias1_Bias_h
                                          //  Referenced by: '<S3>/Bias1'

    float Gain_Gain_i;                 // Computed Parameter: Gain_Gain_i
                                          //  Referenced by: '<S3>/Gain'

    float UnitDelay_InitialCondition;
                               // Computed Parameter: UnitDelay_InitialCondition
                                  //  Referenced by: '<S1>/Unit Delay'

    float UnitDelay1_InitialCondition_o;
                            // Computed Parameter: UnitDelay1_InitialCondition_o
                               //  Referenced by: '<S1>/Unit Delay1'

    bool Constant1_Value_f5;           // Computed Parameter: Constant1_Value_f5
                                          //  Referenced by: '<S7>/Constant1'

    bool Constant_Value_o1;            // Computed Parameter: Constant_Value_o1
                                          //  Referenced by: '<S20>/Constant'

    uint8_t HMatrix_Value[6];          // Computed Parameter: HMatrix_Value
                                          //  Referenced by: '<S7>/H Matrix'

    uint8_t UnitDelay_InitialCondition_n;
                             // Computed Parameter: UnitDelay_InitialCondition_n
                                //  Referenced by: '<S19>/Unit Delay'

    uint8_t Switch_Threshold;          // Computed Parameter: Switch_Threshold
                                          //  Referenced by: '<S19>/Switch'

    uint8_t Bias_Bias_m;               // Computed Parameter: Bias_Bias_m
                                          //  Referenced by: '<S19>/Bias'

    uint8_t Saturation_UpperSat;      // Computed Parameter: Saturation_UpperSat
                                         //  Referenced by: '<S19>/Saturation'

    uint8_t Saturation_LowerSat;      // Computed Parameter: Saturation_LowerSat
                                         //  Referenced by: '<S19>/Saturation'

    uint8_t UnitDelay3_InitialCondition;
                              // Computed Parameter: UnitDelay3_InitialCondition
                                 //  Referenced by: '<S1>/Unit Delay3'

    uint8_t UnitDelay2_InitialCondition;
                              // Computed Parameter: UnitDelay2_InitialCondition
                                 //  Referenced by: '<S1>/Unit Delay2'

    uint8_t Bias1_Bias_l;              // Computed Parameter: Bias1_Bias_l
                                          //  Referenced by: '<S1>/Bias1'

    uint8_t Saturation1_UpperSat;    // Computed Parameter: Saturation1_UpperSat
                                        //  Referenced by: '<S1>/Saturation1'

    uint8_t Saturation1_LowerSat;    // Computed Parameter: Saturation1_LowerSat
                                        //  Referenced by: '<S1>/Saturation1'

    uint8_t Bias2_Bias;                // Computed Parameter: Bias2_Bias
                                          //  Referenced by: '<S1>/Bias2'

    uint8_t Saturation2_UpperSat;    // Computed Parameter: Saturation2_UpperSat
                                        //  Referenced by: '<S1>/Saturation2'

    uint8_t Saturation2_LowerSat;    // Computed Parameter: Saturation2_LowerSat
                                        //  Referenced by: '<S1>/Saturation2'

    P_Nocorrection_NASDAQ0_T Nocorrection_h;// '<S24>/No correction'
    P_Nocorrection_NASDAQ0_T Nocorrection;// '<S11>/No correction'
  };

  // Real-time Model Data Structure
  struct RT_MODEL_NASDAQ0_T {
    //
    //  Timing:
    //  The following substructure contains information regarding
    //  the timing information for the model.

    struct {
      struct {
        uint8_t TID[3];
      } TaskCounters;
    } Timing;
  };

  // Copy Constructor
  NASDAQ0(NASDAQ0 const&) = delete;

  // Assignment Operator
  NASDAQ0& operator= (NASDAQ0 const&) & = delete;

  // Move Constructor
  NASDAQ0(NASDAQ0 &&) = delete;

  // Move Assignment Operator
  NASDAQ0& operator= (NASDAQ0 &&) = delete;

  // Real-Time Model get method
  NASDAQ0::RT_MODEL_NASDAQ0_T * getRTM();

  // Root inport: '<Root>/NASDAQ In Sensors' set method
  void setNASDAQ_In_Sensors(NASDAQInSensors localArgInput)
  {
    NASDAQ0_U.NASDAQInSensors_i = localArgInput;
  }

  // Root inport: '<Root>/NASDAQ In ADA' set method
  void setNASDAQ_In_ADA(NASDAQInADA localArgInput)
  {
    NASDAQ0_U.NASDAQInADA_i = localArgInput;
  }

  // Root inport: '<Root>/NASDAQ In ANAS' set method
  void setNASDAQ_In_ANAS(ANAS_NASDAQ localArgInput)
  {
    NASDAQ0_U.NASDAQInANAS = localArgInput;
  }

  // Root inport: '<Root>/NASDAQ In Reference' set method
  void setNASDAQ_In_Reference(ISAReference localArgInput)
  {
    NASDAQ0_U.NASDAQInReference = localArgInput;
  }

  // Root outport: '<Root>/NASDAQ Out' get method
  NASDAQOut getNASDAQ_Out() const
  {
    return NASDAQ0_Y.NASDAQOut_l;
  }

  // Root outport: '<Root>/NASDAQ Logs OBSW' get method
  NASDAQLogs getNASDAQ_Logs_OBSW() const
  {
    return NASDAQ0_Y.NASDAQLogsOBSW;
  }

  // Block parameters get method
  const P_NASDAQ0_T &getBlockParameters() const
  {
    return NASDAQ0_P;
  }

  // Block parameters set method
  void setBlockParameters(const P_NASDAQ0_T *pP_NASDAQ0_T) const
  {
    NASDAQ0_P = *pP_NASDAQ0_T;
  }

  // model initialize function
  void initialize();

  // model step function
  void step();

  // model terminate function
  static void terminate();

  // Constructor
  NASDAQ0();

  // Destructor
  ~NASDAQ0();

  // private data and function members
 private:
  // External inputs
  ExtU_NASDAQ0_T NASDAQ0_U;

  // External outputs
  ExtY_NASDAQ0_T NASDAQ0_Y;

  // Block states
  DW_NASDAQ0_T NASDAQ0_DW;

  // Tunable parameters
  static P_NASDAQ0_T NASDAQ0_P;

  // private member function(s) for subsystem '<S11>/Correction'
  static void NASDAQ0_Correction(float rtu_Sprev, const float rtu_PH_prev[6],
    float rty_K[6], float *rty_S);

  // private member function(s) for subsystem '<S11>/No correction'
  static void NASDAQ0_Nocorrection(float rtu_R, const float rtu_PH_prev[6],
    float rty_K[6], float *rty_S, P_Nocorrection_NASDAQ0_T *localP);

  // private member function(s) for subsystem '<S4>/No Correction Step'
  static void NASDAQ0_NoCorrectionStep(bool rtu_Enable, const float
    rtu_nextLinearState[6], const float rtu_nextLinearCov[36], float rty_State[6],
    float rty_Covariance[36]);

  // Real-Time Model
  RT_MODEL_NASDAQ0_T NASDAQ0_M;
};

//-
//  These blocks were eliminated from the model due to optimizations:
//
//  Block '<S17>/Check Signal Attributes' : Unused code path elimination
//  Block '<S30>/Check Signal Attributes' : Unused code path elimination
//  Block '<S45>/Check Signal Attributes' : Unused code path elimination
//  Block '<S7>/Reshape1' : Reshape block reduction
//  Block '<S23>/Reshape' : Reshape block reduction
//  Block '<S37>/Reshape' : Reshape block reduction
//  Block '<S37>/Reshape1' : Reshape block reduction


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
//  hilite_system('CHADsimulator/Control Units/Control Units SIM/Main Control Unit (Sim)/Descent Navigation /Descent NAS/NASDAQ - Autocoding')    - opens subsystem CHADsimulator/Control Units/Control Units SIM/Main Control Unit (Sim)/Descent Navigation /Descent NAS/NASDAQ - Autocoding
//  hilite_system('CHADsimulator/Control Units/Control Units SIM/Main Control Unit (Sim)/Descent Navigation /Descent NAS/NASDAQ - Autocoding/Kp') - opens and selects block Kp
//
//  Here is the system hierarchy for this model
//
//  '<Root>' : 'CHADsimulator/Control Units/Control Units SIM/Main Control Unit (Sim)/Descent Navigation /Descent NAS'
//  '<S1>'   : 'CHADsimulator/Control Units/Control Units SIM/Main Control Unit (Sim)/Descent Navigation /Descent NAS/NASDAQ - Autocoding'
//  '<S2>'   : 'CHADsimulator/Control Units/Control Units SIM/Main Control Unit (Sim)/Descent Navigation /Descent NAS/NASDAQ - Autocoding/Correction Step'
//  '<S3>'   : 'CHADsimulator/Control Units/Control Units SIM/Main Control Unit (Sim)/Descent Navigation /Descent NAS/NASDAQ - Autocoding/Prediction Step'
//  '<S4>'   : 'CHADsimulator/Control Units/Control Units SIM/Main Control Unit (Sim)/Descent Navigation /Descent NAS/NASDAQ - Autocoding/Correction Step/ADA Correction'
//  '<S5>'   : 'CHADsimulator/Control Units/Control Units SIM/Main Control Unit (Sim)/Descent Navigation /Descent NAS/NASDAQ - Autocoding/Correction Step/Baro Correction'
//  '<S6>'   : 'CHADsimulator/Control Units/Control Units SIM/Main Control Unit (Sim)/Descent Navigation /Descent NAS/NASDAQ - Autocoding/Correction Step/GPS Correction'
//  '<S7>'   : 'CHADsimulator/Control Units/Control Units SIM/Main Control Unit (Sim)/Descent Navigation /Descent NAS/NASDAQ - Autocoding/Correction Step/ADA Correction/Active Correction Step'
//  '<S8>'   : 'CHADsimulator/Control Units/Control Units SIM/Main Control Unit (Sim)/Descent Navigation /Descent NAS/NASDAQ - Autocoding/Correction Step/ADA Correction/Correction Controller'
//  '<S9>'   : 'CHADsimulator/Control Units/Control Units SIM/Main Control Unit (Sim)/Descent Navigation /Descent NAS/NASDAQ - Autocoding/Correction Step/ADA Correction/No Correction Step'
//  '<S10>'  : 'CHADsimulator/Control Units/Control Units SIM/Main Control Unit (Sim)/Descent Navigation /Descent NAS/NASDAQ - Autocoding/Correction Step/ADA Correction/Active Correction Step/Covariance - Joseph Formula'
//  '<S11>'  : 'CHADsimulator/Control Units/Control Units SIM/Main Control Unit (Sim)/Descent Navigation /Descent NAS/NASDAQ - Autocoding/Correction Step/ADA Correction/Active Correction Step/Kalman Gain'
//  '<S12>'  : 'CHADsimulator/Control Units/Control Units SIM/Main Control Unit (Sim)/Descent Navigation /Descent NAS/NASDAQ - Autocoding/Correction Step/ADA Correction/Active Correction Step/Residual - ADA Correction '
//  '<S13>'  : 'CHADsimulator/Control Units/Control Units SIM/Main Control Unit (Sim)/Descent Navigation /Descent NAS/NASDAQ - Autocoding/Correction Step/ADA Correction/Active Correction Step/Covariance - Joseph Formula/F'
//  '<S14>'  : 'CHADsimulator/Control Units/Control Units SIM/Main Control Unit (Sim)/Descent Navigation /Descent NAS/NASDAQ - Autocoding/Correction Step/ADA Correction/Active Correction Step/Kalman Gain/Correction'
//  '<S15>'  : 'CHADsimulator/Control Units/Control Units SIM/Main Control Unit (Sim)/Descent Navigation /Descent NAS/NASDAQ - Autocoding/Correction Step/ADA Correction/Active Correction Step/Kalman Gain/No correction'
//  '<S16>'  : 'CHADsimulator/Control Units/Control Units SIM/Main Control Unit (Sim)/Descent Navigation /Descent NAS/NASDAQ - Autocoding/Correction Step/ADA Correction/Active Correction Step/Kalman Gain/Reciprocal Condition'
//  '<S17>'  : 'CHADsimulator/Control Units/Control Units SIM/Main Control Unit (Sim)/Descent Navigation /Descent NAS/NASDAQ - Autocoding/Correction Step/ADA Correction/Active Correction Step/Kalman Gain/Reciprocal Condition/Error if not floating-point'
//  '<S18>'  : 'CHADsimulator/Control Units/Control Units SIM/Main Control Unit (Sim)/Descent Navigation /Descent NAS/NASDAQ - Autocoding/Correction Step/ADA Correction/Active Correction Step/Kalman Gain/Reciprocal Condition/LU invert & Check Singularity'
//  '<S19>'  : 'CHADsimulator/Control Units/Control Units SIM/Main Control Unit (Sim)/Descent Navigation /Descent NAS/NASDAQ - Autocoding/Correction Step/Baro Correction/Active Correction Step'
//  '<S20>'  : 'CHADsimulator/Control Units/Control Units SIM/Main Control Unit (Sim)/Descent Navigation /Descent NAS/NASDAQ - Autocoding/Correction Step/Baro Correction/Correction Controller'
//  '<S21>'  : 'CHADsimulator/Control Units/Control Units SIM/Main Control Unit (Sim)/Descent Navigation /Descent NAS/NASDAQ - Autocoding/Correction Step/Baro Correction/No Correction Step'
//  '<S22>'  : 'CHADsimulator/Control Units/Control Units SIM/Main Control Unit (Sim)/Descent Navigation /Descent NAS/NASDAQ - Autocoding/Correction Step/Baro Correction/Active Correction Step/Covariance - Joseph Formula'
//  '<S23>'  : 'CHADsimulator/Control Units/Control Units SIM/Main Control Unit (Sim)/Descent Navigation /Descent NAS/NASDAQ - Autocoding/Correction Step/Baro Correction/Active Correction Step/H Matrix - Baro Correction'
//  '<S24>'  : 'CHADsimulator/Control Units/Control Units SIM/Main Control Unit (Sim)/Descent Navigation /Descent NAS/NASDAQ - Autocoding/Correction Step/Baro Correction/Active Correction Step/Kalman Gain'
//  '<S25>'  : 'CHADsimulator/Control Units/Control Units SIM/Main Control Unit (Sim)/Descent Navigation /Descent NAS/NASDAQ - Autocoding/Correction Step/Baro Correction/Active Correction Step/Residual - Baro Correction '
//  '<S26>'  : 'CHADsimulator/Control Units/Control Units SIM/Main Control Unit (Sim)/Descent Navigation /Descent NAS/NASDAQ - Autocoding/Correction Step/Baro Correction/Active Correction Step/Covariance - Joseph Formula/F'
//  '<S27>'  : 'CHADsimulator/Control Units/Control Units SIM/Main Control Unit (Sim)/Descent Navigation /Descent NAS/NASDAQ - Autocoding/Correction Step/Baro Correction/Active Correction Step/Kalman Gain/Correction'
//  '<S28>'  : 'CHADsimulator/Control Units/Control Units SIM/Main Control Unit (Sim)/Descent Navigation /Descent NAS/NASDAQ - Autocoding/Correction Step/Baro Correction/Active Correction Step/Kalman Gain/No correction'
//  '<S29>'  : 'CHADsimulator/Control Units/Control Units SIM/Main Control Unit (Sim)/Descent Navigation /Descent NAS/NASDAQ - Autocoding/Correction Step/Baro Correction/Active Correction Step/Kalman Gain/Reciprocal Condition'
//  '<S30>'  : 'CHADsimulator/Control Units/Control Units SIM/Main Control Unit (Sim)/Descent Navigation /Descent NAS/NASDAQ - Autocoding/Correction Step/Baro Correction/Active Correction Step/Kalman Gain/Reciprocal Condition/Error if not floating-point'
//  '<S31>'  : 'CHADsimulator/Control Units/Control Units SIM/Main Control Unit (Sim)/Descent Navigation /Descent NAS/NASDAQ - Autocoding/Correction Step/Baro Correction/Active Correction Step/Kalman Gain/Reciprocal Condition/LU invert & Check Singularity'
//  '<S32>'  : 'CHADsimulator/Control Units/Control Units SIM/Main Control Unit (Sim)/Descent Navigation /Descent NAS/NASDAQ - Autocoding/Correction Step/Baro Correction/Active Correction Step/Residual - Baro Correction /Subsystem'
//  '<S33>'  : 'CHADsimulator/Control Units/Control Units SIM/Main Control Unit (Sim)/Descent Navigation /Descent NAS/NASDAQ - Autocoding/Correction Step/GPS Correction/Active Correction Step'
//  '<S34>'  : 'CHADsimulator/Control Units/Control Units SIM/Main Control Unit (Sim)/Descent Navigation /Descent NAS/NASDAQ - Autocoding/Correction Step/GPS Correction/Correction Controller'
//  '<S35>'  : 'CHADsimulator/Control Units/Control Units SIM/Main Control Unit (Sim)/Descent Navigation /Descent NAS/NASDAQ - Autocoding/Correction Step/GPS Correction/No Correction Step'
//  '<S36>'  : 'CHADsimulator/Control Units/Control Units SIM/Main Control Unit (Sim)/Descent Navigation /Descent NAS/NASDAQ - Autocoding/Correction Step/GPS Correction/Active Correction Step/Covariance - Joseph formula'
//  '<S37>'  : 'CHADsimulator/Control Units/Control Units SIM/Main Control Unit (Sim)/Descent Navigation /Descent NAS/NASDAQ - Autocoding/Correction Step/GPS Correction/Active Correction Step/H Matrix - GPS Correction'
//  '<S38>'  : 'CHADsimulator/Control Units/Control Units SIM/Main Control Unit (Sim)/Descent Navigation /Descent NAS/NASDAQ - Autocoding/Correction Step/GPS Correction/Active Correction Step/Kalman Gain'
//  '<S39>'  : 'CHADsimulator/Control Units/Control Units SIM/Main Control Unit (Sim)/Descent Navigation /Descent NAS/NASDAQ - Autocoding/Correction Step/GPS Correction/Active Correction Step/Residual - GPS Correction'
//  '<S40>'  : 'CHADsimulator/Control Units/Control Units SIM/Main Control Unit (Sim)/Descent Navigation /Descent NAS/NASDAQ - Autocoding/Correction Step/GPS Correction/Active Correction Step/Covariance - Joseph formula/F'
//  '<S41>'  : 'CHADsimulator/Control Units/Control Units SIM/Main Control Unit (Sim)/Descent Navigation /Descent NAS/NASDAQ - Autocoding/Correction Step/GPS Correction/Active Correction Step/H Matrix - GPS Correction/Degrees to Radians'
//  '<S42>'  : 'CHADsimulator/Control Units/Control Units SIM/Main Control Unit (Sim)/Descent Navigation /Descent NAS/NASDAQ - Autocoding/Correction Step/GPS Correction/Active Correction Step/Kalman Gain/Correction'
//  '<S43>'  : 'CHADsimulator/Control Units/Control Units SIM/Main Control Unit (Sim)/Descent Navigation /Descent NAS/NASDAQ - Autocoding/Correction Step/GPS Correction/Active Correction Step/Kalman Gain/No correction'
//  '<S44>'  : 'CHADsimulator/Control Units/Control Units SIM/Main Control Unit (Sim)/Descent Navigation /Descent NAS/NASDAQ - Autocoding/Correction Step/GPS Correction/Active Correction Step/Kalman Gain/Reciprocal Condition'
//  '<S45>'  : 'CHADsimulator/Control Units/Control Units SIM/Main Control Unit (Sim)/Descent Navigation /Descent NAS/NASDAQ - Autocoding/Correction Step/GPS Correction/Active Correction Step/Kalman Gain/Reciprocal Condition/Error if not floating-point'
//  '<S46>'  : 'CHADsimulator/Control Units/Control Units SIM/Main Control Unit (Sim)/Descent Navigation /Descent NAS/NASDAQ - Autocoding/Correction Step/GPS Correction/Active Correction Step/Kalman Gain/Reciprocal Condition/LU invert & Check Singularity'
//  '<S47>'  : 'CHADsimulator/Control Units/Control Units SIM/Main Control Unit (Sim)/Descent Navigation /Descent NAS/NASDAQ - Autocoding/Correction Step/GPS Correction/Active Correction Step/Residual - GPS Correction/Degrees to Radians1'


//-
//  Requirements for '<Root>': NASDAQ0


#endif                                 // NASDAQ0_h_

//
// File trailer for generated code.
//
// [EOF]
//
