//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// File: NASDAQ0.h
//
// Code generated for Simulink model 'NASDAQ0'.
//
// Model version                  : 11.128
// Simulink Coder version         : 24.2 (R2024b) 21-Jun-2024
// C/C++ source code generated on : Tue Mar 24 12:53:07 2026
//
// Target selection: ert.tlc
// Embedded hardware selection: STMicroelectronics->ST10/Super10
// Code generation objectives:
//    1. Execution efficiency
//    2. Debugging
//    3. RAM efficiency
// Validation result: Passed (3), Warning (1), Error (0)
//
#ifndef NASDAQ0_h_
#define NASDAQ0_h_
#include <stdbool.h>
#include <stdint.h>
#include "NASDAQ0_types.h"
#include "NASDAQState.h"

// Class declaration for model NASDAQ0
class NASDAQ0 final
{
  // public data and function members
 public:
  // Block signals and states (default storage) for system '<Root>'
  struct DW_NASDAQ0_T {
    uint64_t Memory_PreviousInput;     // '<S24>/Memory'
    uint64_t Memory_PreviousInput_m;   // '<S15>/Memory'
    float NASVarianceInterface_PreviousIn[36];// '<S1>/NAS Variance Interface'
    float NASStateInterface_PreviousInput[6];// '<S1>/NAS State Interface'
    float MatrixDivide_DWORK4[16];     // '<S28>/Matrix Divide'
    float MatrixDivide_DWORK4_k;       // '<S19>/Matrix Divide'
    float Memory_PreviousInput_mp;     // '<S8>/Memory'
    float MatrixDivide_DWORK4_d;       // '<S11>/Matrix Divide'
  };

  // External inputs (root inport signals with default storage)
  struct ExtU_NASDAQ0_T {
    Bus_AdaState ADAStates;            // '<Root>/ADA States'
    Bus_GPS GPS;                       // '<Root>/GPS'
    Bus_Baro Baro;                     // '<Root>/Baro'
  };

  // External outputs (root outports fed by signals with default storage)
  struct ExtY_NASDAQ0_T {
    float Position[3];                 // '<Root>/Position'
    float Velocity[3];                 // '<Root>/Velocity'
    float Covariance[6];               // '<Root>/Covariance'
    float Timestamp;                   // '<Root>/Timestamp'
  };

  // Parameters (default storage)
  struct P_NASDAQ0_T {
    struct_UyXIlhvkjCvDDDjTjsnNlD nasdaq;// Variable: nasdaq
                                            //  Referenced by:
                                            //    '<S8>/Constant'
                                            //    '<S24>/Constant'

    uint64_t Memory_InitialCondition;
                                  // Computed Parameter: Memory_InitialCondition
                                     //  Referenced by: '<S15>/Memory'

    uint64_t Memory_InitialCondition_b;
                                // Computed Parameter: Memory_InitialCondition_b
                                   //  Referenced by: '<S24>/Memory'

    float Constant_Value;              // Expression: nasdaq.sigma.ADA
                                          //  Referenced by: '<S7>/Constant'

    float Constant_Value_g[36];        // Computed Parameter: Constant_Value_g
                                          //  Referenced by: '<S13>/Constant'

    float Gain_Gain;                   // Computed Parameter: Gain_Gain
                                          //  Referenced by: '<S4>/Gain'

    float Memory_InitialCondition_l;
                                // Computed Parameter: Memory_InitialCondition_l
                                   //  Referenced by: '<S8>/Memory'

    float Constant_Value_h[36];        // Computed Parameter: Constant_Value_h
                                          //  Referenced by: '<S21>/Constant'

    float Constant_Value_j[2];         // Computed Parameter: Constant_Value_j
                                          //  Referenced by: '<S18>/Constant'

    float StandardAirPressureP0_Value;
                              // Computed Parameter: StandardAirPressureP0_Value
                                 //  Referenced by: '<S18>/StandardAirPressure P0'

    float Gain3_Gain;                  // Computed Parameter: Gain3_Gain
                                          //  Referenced by: '<S18>/Gain3'

    float Gain4_Gain;                  // Computed Parameter: Gain4_Gain
                                          //  Referenced by: '<S18>/Gain4'

    float StandardAirTemperatureT0_Value;
                           // Computed Parameter: StandardAirTemperatureT0_Value
                              //  Referenced by: '<S18>/StandardAirTemperature T0'

    float gR_Value;                    // Computed Parameter: gR_Value
                                          //  Referenced by: '<S18>/g R'

    float Constant1_Value[3];          // Computed Parameter: Constant1_Value
                                          //  Referenced by: '<S18>/Constant1'

    float Constant_Value_jb;           // Expression: nasdaq.sigma.baro
                                          //  Referenced by: '<S14>/Constant'

    float StandardAirPressureP0_Value_a;
                            // Computed Parameter: StandardAirPressureP0_Value_a
                               //  Referenced by: '<S22>/StandardAirPressure P0'

    float StandardAirTemperatureT0_Valu_o;
                          // Computed Parameter: StandardAirTemperatureT0_Valu_o
                             //  Referenced by: '<S22>/StandardAirTemperature T0'

    float Gain_Gain_j;                 // Computed Parameter: Gain_Gain_j
                                          //  Referenced by: '<S20>/Gain'

    float HeightTemperatureGradient_Value;// Expression: nasdaq.baro.a
                                             //  Referenced by: '<S22>/HeightTemperatureGradient'

    float gravity_Value;               // Computed Parameter: gravity_Value
                                          //  Referenced by: '<S22>/gravity'

    float Rair_Value;                  // Computed Parameter: Rair_Value
                                          //  Referenced by: '<S22>/R air'

    float Constant_Value_a[36];        // Computed Parameter: Constant_Value_a
                                          //  Referenced by: '<S30>/Constant'

    float Constant_Value_c;            // Computed Parameter: Constant_Value_c
                                          //  Referenced by: '<S27>/Constant'

    float Constant1_Value_e;           // Computed Parameter: Constant1_Value_e
                                          //  Referenced by: '<S27>/Constant1'

    float Gain_Gain_a;                 // Computed Parameter: Gain_Gain_a
                                          //  Referenced by: '<S27>/Gain'

    float Bias_Bias;                   // Expression: nasdaq.gps.lat0
                                          //  Referenced by: '<S27>/Bias'

    float Gain1_Gain;                  // Computed Parameter: Gain1_Gain
                                          //  Referenced by: '<S31>/Gain1'

    float Constant3_Value;             // Computed Parameter: Constant3_Value
                                          //  Referenced by: '<S27>/Constant3'

    float Constant4_Value;             // Computed Parameter: Constant4_Value
                                          //  Referenced by: '<S27>/Constant4'

    float Gain1_Gain_m;                // Expression: nasdaq.gps.b
                                          //  Referenced by: '<S27>/Gain1'

    float Gain3_Gain_b;                // Computed Parameter: Gain3_Gain_b
                                          //  Referenced by: '<S27>/Gain3'

    float Constant5_Value[8];          // Computed Parameter: Constant5_Value
                                          //  Referenced by: '<S27>/Constant5'

    float Constant6_Value[12];         // Computed Parameter: Constant6_Value
                                          //  Referenced by: '<S27>/Constant6'

    float Constant_Value_n[16];        // Expression: nasdaq.sigma.GPS
                                          //  Referenced by: '<S23>/Constant'

    float Gain3_Gain_g;                // Computed Parameter: Gain3_Gain_g
                                          //  Referenced by: '<S29>/Gain3'

    float Bias_Bias_g;                 // Expression: nasdaq.gps.lat0
                                          //  Referenced by: '<S29>/Bias'

    float Gain1_Gain_g;                // Computed Parameter: Gain1_Gain_g
                                          //  Referenced by: '<S32>/Gain1'

    float Gain4_Gain_o;                // Expression: nasdaq.gps.b
                                          //  Referenced by: '<S29>/Gain4'

    float Bias1_Bias;                  // Expression: nasdaq.gps.lon0
                                          //  Referenced by: '<S29>/Bias1'

    float Gain_Gain_jx[4];             // Computed Parameter: Gain_Gain_jx
                                          //  Referenced by: '<S29>/Gain'

    float Constant1_Value_o[36];       // Computed Parameter: Constant1_Value_o
                                          //  Referenced by: '<S3>/Constant1'

    float Gain1_Gain_g2;               // Computed Parameter: Gain1_Gain_g2
                                          //  Referenced by: '<S3>/Gain1'

    float Bias_Bias_l[36];             // Computed Parameter: Bias_Bias_l
                                          //  Referenced by: '<S3>/Bias'

    float Bias1_Bias_m[36];            // Expression: nasdaq.initMatrix.Q
                                          //  Referenced by: '<S3>/Bias1'

    float Gain_Gain_b;                 // Computed Parameter: Gain_Gain_b
                                          //  Referenced by: '<S3>/Gain'

    float NASVarianceInterface_InitialCon[36];// Expression: init.NASVariance
                                                 //  Referenced by: '<S1>/NAS Variance Interface'

    float NASStateInterface_InitialCondit[6];
                          // Computed Parameter: NASStateInterface_InitialCondit
                             //  Referenced by: '<S1>/NAS State Interface'

    bool Constant1_Value_m;            // Expression: nasdaq.flags.ADAdynamicR
                                          //  Referenced by: '<S7>/Constant1'

    bool Constant_Value_n0;            // Expression: nasdaq.flags.baro
                                          //  Referenced by: '<S15>/Constant'

    uint8_t HMatrix_Value[6];          // Computed Parameter: HMatrix_Value
                                          //  Referenced by: '<S7>/H Matrix'

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

  // Root inport: '<Root>/ADA States' set method
  void setADA_States(Bus_AdaState localArgInput)
  {
    NASDAQ0_U.ADAStates = localArgInput;
  }

  // Root inport: '<Root>/GPS' set method
  void setGPS(Bus_GPS localArgInput)
  {
    NASDAQ0_U.GPS = localArgInput;
  }

  // Root inport: '<Root>/Baro' set method
  void setBaro(Bus_Baro localArgInput)
  {
    NASDAQ0_U.Baro = localArgInput;
  }

  // Root outports get method
  const ExtY_NASDAQ0_T &getExternalOutputs() const
  {
    return NASDAQ0_Y;
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
//  Block '<S7>/Reshape1' : Reshape block reduction
//  Block '<S18>/Reshape' : Reshape block reduction
//  Block '<S15>/Rate Transition' : Eliminated since input and output rates are identical
//  Block '<S27>/Reshape' : Reshape block reduction
//  Block '<S27>/Reshape1' : Reshape block reduction


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
//  hilite_system('CHADsimulator/Control Units/Control Units SIM/Parafoil Control Unit (Sim)/NAS/NASDAQ')    - opens subsystem CHADsimulator/Control Units/Control Units SIM/Parafoil Control Unit (Sim)/NAS/NASDAQ
//  hilite_system('CHADsimulator/Control Units/Control Units SIM/Parafoil Control Unit (Sim)/NAS/NASDAQ/Kp') - opens and selects block Kp
//
//  Here is the system hierarchy for this model
//
//  '<Root>' : 'CHADsimulator/Control Units/Control Units SIM/Parafoil Control Unit (Sim)/NAS'
//  '<S1>'   : 'CHADsimulator/Control Units/Control Units SIM/Parafoil Control Unit (Sim)/NAS/NASDAQ'
//  '<S2>'   : 'CHADsimulator/Control Units/Control Units SIM/Parafoil Control Unit (Sim)/NAS/NASDAQ/Correction Step'
//  '<S3>'   : 'CHADsimulator/Control Units/Control Units SIM/Parafoil Control Unit (Sim)/NAS/NASDAQ/Prediction Step'
//  '<S4>'   : 'CHADsimulator/Control Units/Control Units SIM/Parafoil Control Unit (Sim)/NAS/NASDAQ/Correction Step/ADA Correction'
//  '<S5>'   : 'CHADsimulator/Control Units/Control Units SIM/Parafoil Control Unit (Sim)/NAS/NASDAQ/Correction Step/Baro Correction'
//  '<S6>'   : 'CHADsimulator/Control Units/Control Units SIM/Parafoil Control Unit (Sim)/NAS/NASDAQ/Correction Step/GPS Correction'
//  '<S7>'   : 'CHADsimulator/Control Units/Control Units SIM/Parafoil Control Unit (Sim)/NAS/NASDAQ/Correction Step/ADA Correction/Active Correction Step'
//  '<S8>'   : 'CHADsimulator/Control Units/Control Units SIM/Parafoil Control Unit (Sim)/NAS/NASDAQ/Correction Step/ADA Correction/Correction Controller'
//  '<S9>'   : 'CHADsimulator/Control Units/Control Units SIM/Parafoil Control Unit (Sim)/NAS/NASDAQ/Correction Step/ADA Correction/No Correction Step'
//  '<S10>'  : 'CHADsimulator/Control Units/Control Units SIM/Parafoil Control Unit (Sim)/NAS/NASDAQ/Correction Step/ADA Correction/Active Correction Step/Covariance - Joseph Formula'
//  '<S11>'  : 'CHADsimulator/Control Units/Control Units SIM/Parafoil Control Unit (Sim)/NAS/NASDAQ/Correction Step/ADA Correction/Active Correction Step/Kalman Gain'
//  '<S12>'  : 'CHADsimulator/Control Units/Control Units SIM/Parafoil Control Unit (Sim)/NAS/NASDAQ/Correction Step/ADA Correction/Active Correction Step/Residual - ADA Correction '
//  '<S13>'  : 'CHADsimulator/Control Units/Control Units SIM/Parafoil Control Unit (Sim)/NAS/NASDAQ/Correction Step/ADA Correction/Active Correction Step/Covariance - Joseph Formula/F'
//  '<S14>'  : 'CHADsimulator/Control Units/Control Units SIM/Parafoil Control Unit (Sim)/NAS/NASDAQ/Correction Step/Baro Correction/Active Correction Step'
//  '<S15>'  : 'CHADsimulator/Control Units/Control Units SIM/Parafoil Control Unit (Sim)/NAS/NASDAQ/Correction Step/Baro Correction/Correction Controller'
//  '<S16>'  : 'CHADsimulator/Control Units/Control Units SIM/Parafoil Control Unit (Sim)/NAS/NASDAQ/Correction Step/Baro Correction/No Correction Step'
//  '<S17>'  : 'CHADsimulator/Control Units/Control Units SIM/Parafoil Control Unit (Sim)/NAS/NASDAQ/Correction Step/Baro Correction/Active Correction Step/Covariance - Joseph Formula'
//  '<S18>'  : 'CHADsimulator/Control Units/Control Units SIM/Parafoil Control Unit (Sim)/NAS/NASDAQ/Correction Step/Baro Correction/Active Correction Step/H Matrix - Baro Correction'
//  '<S19>'  : 'CHADsimulator/Control Units/Control Units SIM/Parafoil Control Unit (Sim)/NAS/NASDAQ/Correction Step/Baro Correction/Active Correction Step/Kalman Gain'
//  '<S20>'  : 'CHADsimulator/Control Units/Control Units SIM/Parafoil Control Unit (Sim)/NAS/NASDAQ/Correction Step/Baro Correction/Active Correction Step/Residual - Baro Correction '
//  '<S21>'  : 'CHADsimulator/Control Units/Control Units SIM/Parafoil Control Unit (Sim)/NAS/NASDAQ/Correction Step/Baro Correction/Active Correction Step/Covariance - Joseph Formula/F'
//  '<S22>'  : 'CHADsimulator/Control Units/Control Units SIM/Parafoil Control Unit (Sim)/NAS/NASDAQ/Correction Step/Baro Correction/Active Correction Step/Residual - Baro Correction /Subsystem'
//  '<S23>'  : 'CHADsimulator/Control Units/Control Units SIM/Parafoil Control Unit (Sim)/NAS/NASDAQ/Correction Step/GPS Correction/Active Correction Step'
//  '<S24>'  : 'CHADsimulator/Control Units/Control Units SIM/Parafoil Control Unit (Sim)/NAS/NASDAQ/Correction Step/GPS Correction/Correction Controller'
//  '<S25>'  : 'CHADsimulator/Control Units/Control Units SIM/Parafoil Control Unit (Sim)/NAS/NASDAQ/Correction Step/GPS Correction/No Correction Step'
//  '<S26>'  : 'CHADsimulator/Control Units/Control Units SIM/Parafoil Control Unit (Sim)/NAS/NASDAQ/Correction Step/GPS Correction/Active Correction Step/Covariance - Joseph formula'
//  '<S27>'  : 'CHADsimulator/Control Units/Control Units SIM/Parafoil Control Unit (Sim)/NAS/NASDAQ/Correction Step/GPS Correction/Active Correction Step/H Matrix - GPS Correction'
//  '<S28>'  : 'CHADsimulator/Control Units/Control Units SIM/Parafoil Control Unit (Sim)/NAS/NASDAQ/Correction Step/GPS Correction/Active Correction Step/Kalman Gain'
//  '<S29>'  : 'CHADsimulator/Control Units/Control Units SIM/Parafoil Control Unit (Sim)/NAS/NASDAQ/Correction Step/GPS Correction/Active Correction Step/Residual - GPS Correction'
//  '<S30>'  : 'CHADsimulator/Control Units/Control Units SIM/Parafoil Control Unit (Sim)/NAS/NASDAQ/Correction Step/GPS Correction/Active Correction Step/Covariance - Joseph formula/F'
//  '<S31>'  : 'CHADsimulator/Control Units/Control Units SIM/Parafoil Control Unit (Sim)/NAS/NASDAQ/Correction Step/GPS Correction/Active Correction Step/H Matrix - GPS Correction/Degrees to Radians'
//  '<S32>'  : 'CHADsimulator/Control Units/Control Units SIM/Parafoil Control Unit (Sim)/NAS/NASDAQ/Correction Step/GPS Correction/Active Correction Step/Residual - GPS Correction/Degrees to Radians1'


//-
//  Requirements for '<Root>': NASDAQ0


#endif                                 // NASDAQ0_h_

//
// File trailer for generated code.
//
// [EOF]
//
