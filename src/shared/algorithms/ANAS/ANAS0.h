//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// File: ANAS0.h
//
// Code generated for Simulink model 'ANAS0'.
//
// Model version                  : 11.296
// Simulink Coder version         : 24.2 (R2024b) 21-Jun-2024
// C/C++ source code generated on : Mon Jun  8 17:58:37 2026
//
// Target selection: ert.tlc
// Embedded hardware selection: ARM Compatible->ARM Cortex-M
// Code generation objectives:
//    1. Execution efficiency
//    2. Debugging
//    3. RAM efficiency
// Validation result: Not run
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
  // Block signals and states (default storage) for system '<S14>/Correction'
  struct DW_Correction_ANAS0_T {
    float MatrixDivide_DWORK4[9];      // '<S18>/Matrix Divide'
  };

  // Block signals and states (default storage) for system '<S107>/Correction'
  struct DW_Correction_ANAS0_d_T {
    float MatrixDivide_DWORK4[4];      // '<S111>/Matrix Divide'
  };

  // Block signals and states (default storage) for system '<S165>/Correction'
  struct DW_Correction_ANAS0_i_T {
    float MatrixDivide_DWORK4;         // '<S168>/Matrix Divide'
  };

  // Block signals and states (default storage) for system '<Root>'
  struct DW_ANAS0_T {
    DW_Correction_ANAS0_i_T Correction_d;// '<S178>/Correction'
    DW_Correction_ANAS0_i_T Correction_m;// '<S165>/Correction'
    DW_Correction_ANAS0_d_T Correction_no;// '<S145>/Correction'
    DW_Correction_ANAS0_d_T Correction_f;// '<S125>/Correction'
    DW_Correction_ANAS0_d_T Correction_i;// '<S107>/Correction'
    DW_Correction_ANAS0_T Correction_n;// '<S54>/Correction'
    DW_Correction_ANAS0_T Correction;  // '<S14>/Correction'
    uint64_t RateTransition;           // '<S203>/Rate Transition'
    uint64_t RateTransition_c;         // '<S103>/Rate Transition'
    uint64_t RateTransition1;          // '<S103>/Rate Transition1'
    uint64_t RateTransition2;          // '<S103>/Rate Transition2'
    uint64_t RateTransition_i;         // '<S11>/Rate Transition'
    uint64_t RateTransition_o;         // '<S51>/Rate Transition'
    uint64_t UD_DSTATE;                // '<S219>/UD'
    uint64_t UD_DSTATE_a;              // '<S89>/UD'
    uint64_t Memory_PreviousInput;     // '<S203>/Memory'
    uint64_t Memory3_PreviousInput;    // '<S103>/Memory3'
    uint64_t Memory2_PreviousInput;    // '<S103>/Memory2'
    uint64_t Memory1_PreviousInput;    // '<S103>/Memory1'
    uint64_t Memory_PreviousInput_k;   // '<S11>/Memory'
    uint64_t Memory_PreviousInput_kq;  // '<S51>/Memory'
    float Merge[6];                    // '<S96>/Merge'
    float Merge1[36];                  // '<S96>/Merge1'
    float IdentityMatrix[16];          // '<S215>/Identity Matrix'
    float IdentityMatrix_n[4];         // '<S156>/Identity Matrix'
    float IdentityMatrix_g[4];         // '<S136>/Identity Matrix'
    float IdentityMatrix_eg[4];        // '<S115>/Identity Matrix'
    float Merge_l[4];                  // '<S9>/Merge'
    float Merge1_e[9];                 // '<S9>/Merge1'
    float Merge_h[4];                  // '<S8>/Merge'
    float fromGGausstonTTesla[3];      // '<S56>/from G (Gauss) to nT (Tesla)'
    float IdentityMatrix_j[9];         // '<S62>/Identity Matrix'
    float IdentityMatrix_h[9];         // '<S22>/Identity Matrix'
    float UnitDelay6_DSTATE[4];        // '<S7>/Unit Delay6'
    float UnitDelay7_DSTATE[4];        // '<S7>/Unit Delay7'
    float UnitDelay4_DSTATE[6];        // '<S7>/Unit Delay4'
    float UnitDelay8_DSTATE[6];        // '<S7>/Unit Delay8'
    float UnitDelay_DSTATE[36];        // '<S7>/Unit Delay'
    float UnitDelay1_DSTATE[36];       // '<S7>/Unit Delay1'
    float UnitDelay2_DSTATE[9];        // '<S7>/Unit Delay2'
    float UnitDelay3_DSTATE[9];        // '<S7>/Unit Delay3'
    float MatrixDivide_DWORK4[16];     // '<S211>/Matrix Divide'
    float IdentityMatrix_i;            // '<S185>/Identity Matrix'
    float IdentityMatrix_e;            // '<S172>/Identity Matrix'
    float DiscreteFilter_states;       // '<S116>/Discrete Filter'
    float UD_DSTATE_h;                 // '<S120>/UD'
    uint8_t Merge_k;                   // '<S103>/Merge'
    uint8_t UnitDelay3_DSTATE_h;       // '<S5>/Unit Delay3'
    uint8_t UnitDelay3_DSTATE_o;       // '<S3>/Unit Delay3'
  };

  // External inputs (root inport signals with default storage)
  struct ExtU_ANAS0_T {
    ANASIn ANASIn_c;                   // '<Root>/ANAS In'
    ANASReference ANASReference_f;     // '<Root>/ANAS Reference'
  };

  // External outputs (root outports fed by signals with default storage)
  struct ExtY_ANAS0_T {
    ANASOut ANASOut_h;                 // '<Root>/ANAS Out'
    ANASLogs ANASLogsOBSW;             // '<Root>/ANAS Logs OBSW'
    ANAS_NASDAQ NASDAQInitialState;    // '<Root>/NASDAQ Initial State'
  };

  // Parameters for system: '<S14>/No correction'
  struct P_Nocorrection_ANAS0_T {
    float Gain_Gain;                   // Computed Parameter: Gain_Gain
                                          //  Referenced by: '<S19>/Gain'

  };

  // Parameters for system: '<S107>/No correction'
  struct P_Nocorrection_ANAS0_f_T {
    float Gain_Gain;                   // Computed Parameter: Gain_Gain
                                          //  Referenced by: '<S112>/Gain'

  };

  // Parameters for system: '<S165>/No correction'
  struct P_Nocorrection_ANAS0_i_T {
    float Gain_Gain;                   // Computed Parameter: Gain_Gain
                                          //  Referenced by: '<S169>/Gain'

  };

  // Parameters (default storage)
  struct P_ANAS0_T {
    uint64_t Difference_ICPrevInput;   // Mask Parameter: Difference_ICPrevInput
                                          //  Referenced by: '<S89>/UD'

    uint64_t Difference_ICPrevInput_j;
                                     // Mask Parameter: Difference_ICPrevInput_j
                                        //  Referenced by: '<S219>/UD'

    float Difference_ICPrevInput_m;  // Mask Parameter: Difference_ICPrevInput_m
                                        //  Referenced by: '<S120>/UD'

    float GPSAccelerationLimit_const;
                                   // Mask Parameter: GPSAccelerationLimit_const
                                      //  Referenced by: '<S217>/Constant'

    float PitotMinMach_const;          // Mask Parameter: PitotMinMach_const
                                          //  Referenced by: '<S196>/Constant'

    float g02accg02_lowlimit;          // Mask Parameter: g02accg02_lowlimit
                                          //  Referenced by: '<S49>/Lower Limit'

    float g02accg02_uplimit;           // Mask Parameter: g02accg02_uplimit
                                          //  Referenced by: '<S49>/Upper Limit'

    uint64_t Memory_InitialCondition;
                                  // Computed Parameter: Memory_InitialCondition
                                     //  Referenced by: '<S11>/Memory'

    uint64_t Memory_InitialCondition_n;
                                // Computed Parameter: Memory_InitialCondition_n
                                   //  Referenced by: '<S51>/Memory'

    uint64_t Memory_InitialCondition_k;
                                // Computed Parameter: Memory_InitialCondition_k
                                   //  Referenced by: '<S203>/Memory'

    uint64_t Memory3_InitialCondition;
                                 // Computed Parameter: Memory3_InitialCondition
                                    //  Referenced by: '<S103>/Memory3'

    uint64_t Memory2_InitialCondition;
                                 // Computed Parameter: Memory2_InitialCondition
                                    //  Referenced by: '<S103>/Memory2'

    uint64_t Memory1_InitialCondition;
                                 // Computed Parameter: Memory1_InitialCondition
                                    //  Referenced by: '<S103>/Memory1'

    float Constant_Value;              // Computed Parameter: Constant_Value
                                          //  Referenced by: '<S20>/Constant'

    float IdentityMatrix_IDMatrixData[9];
                              // Computed Parameter: IdentityMatrix_IDMatrixData
                                 //  Referenced by: '<S22>/Identity Matrix'

    float AccSigma_Value[9];           // Computed Parameter: AccSigma_Value
                                          //  Referenced by: '<S10>/AccSigma'

    float Constant_Value_k;            // Computed Parameter: Constant_Value_k
                                          //  Referenced by: '<S33>/Constant'

    float Gain_Gain;                   // Computed Parameter: Gain_Gain
                                          //  Referenced by: '<S37>/Gain'

    float Gain_Gain_o;                 // Computed Parameter: Gain_Gain_o
                                          //  Referenced by: '<S40>/Gain'

    float Gain_Gain_i;                 // Computed Parameter: Gain_Gain_i
                                          //  Referenced by: '<S35>/Gain'

    float Gain_Gain_b;                 // Computed Parameter: Gain_Gain_b
                                          //  Referenced by: '<S41>/Gain'

    float Gain_Gain_h;                 // Computed Parameter: Gain_Gain_h
                                          //  Referenced by: '<S36>/Gain'

    float Gain_Gain_g;                 // Computed Parameter: Gain_Gain_g
                                          //  Referenced by: '<S39>/Gain'

    float LocalGravity_Value[3];       // Computed Parameter: LocalGravity_Value
                                          //  Referenced by: '<S16>/Local Gravity'

    float Gain1_Gain;                  // Computed Parameter: Gain1_Gain
                                          //  Referenced by: '<S33>/Gain1'

    float Gain2_Gain;                  // Computed Parameter: Gain2_Gain
                                          //  Referenced by: '<S33>/Gain2'

    float Gain_Gain_j;                 // Computed Parameter: Gain_Gain_j
                                          //  Referenced by: '<S33>/Gain'

    float NASCondLim_Value;            // Computed Parameter: NASCondLim_Value
                                          //  Referenced by: '<S14>/NASCondLim'

    float Constant_Value_b[9];         // Computed Parameter: Constant_Value_b
                                          //  Referenced by: '<S17>/Constant'

    float Gain1_Gain_o;                // Computed Parameter: Gain1_Gain_o
                                          //  Referenced by: '<S15>/Gain1'

    float Gain_Gain_h5;                // Computed Parameter: Gain_Gain_h5
                                          //  Referenced by: '<S15>/Gain'

    float Bias_Bias;                   // Computed Parameter: Bias_Bias
                                          //  Referenced by: '<S15>/Bias'

    float Constant_Value_c;            // Computed Parameter: Constant_Value_c
                                          //  Referenced by: '<S60>/Constant'

    float IdentityMatrix_IDMatrixData_a[9];
                            // Computed Parameter: IdentityMatrix_IDMatrixData_a
                               //  Referenced by: '<S62>/Identity Matrix'

    float sxMatrix_Value[9];           // Computed Parameter: sxMatrix_Value
                                          //  Referenced by: '<S74>/sxMatrix'

    float dxMatrix_Value[9];           // Computed Parameter: dxMatrix_Value
                                          //  Referenced by: '<S74>/dxMatrix'

    float Constant_Value_l[9];         // Computed Parameter: Constant_Value_l
                                          //  Referenced by: '<S57>/Constant'

    float Constant_Value_p;            // Computed Parameter: Constant_Value_p
                                          //  Referenced by: '<S73>/Constant'

    float Gain_Gain_hq;                // Computed Parameter: Gain_Gain_hq
                                          //  Referenced by: '<S78>/Gain'

    float Gain_Gain_gr;                // Computed Parameter: Gain_Gain_gr
                                          //  Referenced by: '<S81>/Gain'

    float Gain_Gain_hf;                // Computed Parameter: Gain_Gain_hf
                                          //  Referenced by: '<S76>/Gain'

    float Gain_Gain_m;                 // Computed Parameter: Gain_Gain_m
                                          //  Referenced by: '<S82>/Gain'

    float Gain_Gain_d;                 // Computed Parameter: Gain_Gain_d
                                          //  Referenced by: '<S77>/Gain'

    float Gain_Gain_h1;                // Computed Parameter: Gain_Gain_h1
                                          //  Referenced by: '<S80>/Gain'

    float InitMagnField_Value[3];     // Computed Parameter: InitMagnField_Value
                                         //  Referenced by: '<S56>/InitMagnField'

    float Gain1_Gain_m;                // Computed Parameter: Gain1_Gain_m
                                          //  Referenced by: '<S73>/Gain1'

    float Gain2_Gain_g;                // Computed Parameter: Gain2_Gain_g
                                          //  Referenced by: '<S73>/Gain2'

    float Gain_Gain_gl;                // Computed Parameter: Gain_Gain_gl
                                          //  Referenced by: '<S73>/Gain'

    float MagSigma_Value[9];           // Computed Parameter: MagSigma_Value
                                          //  Referenced by: '<S50>/MagSigma'

    float NASCondLim_Value_n;          // Computed Parameter: NASCondLim_Value_n
                                          //  Referenced by: '<S54>/NASCondLim'

    float fromGGausstonTTesla_Gain;
                                 // Computed Parameter: fromGGausstonTTesla_Gain
                                    //  Referenced by: '<S56>/from G (Gauss) to nT (Tesla)'

    float Gain1_Gain_j;                // Computed Parameter: Gain1_Gain_j
                                          //  Referenced by: '<S55>/Gain1'

    float Gain_Gain_p;                 // Computed Parameter: Gain_Gain_p
                                          //  Referenced by: '<S55>/Gain'

    float Bias_Bias_d;                 // Computed Parameter: Bias_Bias_d
                                          //  Referenced by: '<S55>/Bias'

    float Merge_InitialOutput;        // Computed Parameter: Merge_InitialOutput
                                         //  Referenced by: '<S9>/Merge'

    float Merge_InitialOutput_d;    // Computed Parameter: Merge_InitialOutput_d
                                       //  Referenced by: '<S8>/Merge'

    float Gain1_Gain_h;                // Computed Parameter: Gain1_Gain_h
                                          //  Referenced by: '<S3>/Gain1'

    float Constant_Value_cv;           // Computed Parameter: Constant_Value_cv
                                          //  Referenced by: '<S3>/Constant'

    float Constant1_Value[16];         // Computed Parameter: Constant1_Value
                                          //  Referenced by: '<S3>/Constant1'

    float Constant_Value_j;            // Computed Parameter: Constant_Value_j
                                          //  Referenced by: '<S92>/Constant'

    float Gain1_Gain_oh;               // Computed Parameter: Gain1_Gain_oh
                                          //  Referenced by: '<S92>/Gain1'

    float Gain2_Gain_d;                // Computed Parameter: Gain2_Gain_d
                                          //  Referenced by: '<S92>/Gain2'

    float Gain_Gain_e;                 // Computed Parameter: Gain_Gain_e
                                          //  Referenced by: '<S92>/Gain'

    float Gain_Gain_eg;                // Computed Parameter: Gain_Gain_eg
                                          //  Referenced by: '<S90>/Gain'

    float Gain1_Gain_k;                // Computed Parameter: Gain1_Gain_k
                                          //  Referenced by: '<S90>/Gain1'

    float Constant_Value_h;            // Computed Parameter: Constant_Value_h
                                          //  Referenced by: '<S90>/Constant'

    float Gain_Gain_ow;                // Computed Parameter: Gain_Gain_ow
                                          //  Referenced by: '<S3>/Gain'

    float Constant2_Value[9];          // Computed Parameter: Constant2_Value
                                          //  Referenced by: '<S3>/Constant2'

    float AngularQ1_3_Value[9];        // Computed Parameter: AngularQ1_3_Value
                                          //  Referenced by: '<S3>/AngularQ1_3'

    float Constant_Value_e;            // Computed Parameter: Constant_Value_e
                                          //  Referenced by: '<S113>/Constant'

    float IdentityMatrix_IDMatrixData_i[4];
                            // Computed Parameter: IdentityMatrix_IDMatrixData_i
                               //  Referenced by: '<S115>/Identity Matrix'

    float Constant_Value_n[36];        // Computed Parameter: Constant_Value_n
                                          //  Referenced by: '<S109>/Constant'

    float Constant3_Value[2];          // Computed Parameter: Constant3_Value
                                          //  Referenced by: '<S106>/Constant3'

    float Baroa_Value;                 // Computed Parameter: Baroa_Value
                                          //  Referenced by: '<S106>/Baroa'

    float BarogR_Value;                // Computed Parameter: BarogR_Value
                                          //  Referenced by: '<S106>/Barog//R'

    float Constant2_Value_l;           // Computed Parameter: Constant2_Value_l
                                          //  Referenced by: '<S106>/Constant2'

    float Constant1_Value_f;           // Computed Parameter: Constant1_Value_f
                                          //  Referenced by: '<S106>/Constant1'

    float Constant9_Value[3];          // Computed Parameter: Constant9_Value
                                          //  Referenced by: '<S106>/Constant9'

    float Constant5_Value[2];          // Computed Parameter: Constant5_Value
                                          //  Referenced by: '<S106>/Constant5'

    float Gain1_Gain_a;                // Computed Parameter: Gain1_Gain_a
                                          //  Referenced by: '<S106>/Gain1'

    float RAir1_Value;                 // Computed Parameter: RAir1_Value
                                          //  Referenced by: '<S110>/RAir1'

    float LocalGravity_Value_j;      // Computed Parameter: LocalGravity_Value_j
                                        //  Referenced by: '<S110>/LocalGravity'

    float RAir_Value;                  // Computed Parameter: RAir_Value
                                          //  Referenced by: '<S110>/RAir'

    float Gain2_Gain_j;                // Computed Parameter: Gain2_Gain_j
                                          //  Referenced by: '<S106>/Gain2'

    float Constant6_Value[2];          // Computed Parameter: Constant6_Value
                                          //  Referenced by: '<S106>/Constant6'

    float BaroVaroSigma_Value[4];     // Computed Parameter: BaroVaroSigma_Value
                                         //  Referenced by: '<S98>/BaroVaroSigma'

    float NASCondLim_Value_a;          // Computed Parameter: NASCondLim_Value_a
                                          //  Referenced by: '<S107>/NASCondLim'

    float DiscreteFilter_NumCoef[2];
                                   // Computed Parameter: DiscreteFilter_NumCoef
                                      //  Referenced by: '<S116>/Discrete Filter'

    float DiscreteFilter_DenCoef[2];
                                   // Computed Parameter: DiscreteFilter_DenCoef
                                      //  Referenced by: '<S116>/Discrete Filter'

    float DiscreteFilter_InitialStates;
                             // Computed Parameter: DiscreteFilter_InitialStates
                                //  Referenced by: '<S116>/Discrete Filter'

    float Gain_Gain_c;                 // Computed Parameter: Gain_Gain_c
                                          //  Referenced by: '<S108>/Gain'

    float RAir1_Value_c;               // Computed Parameter: RAir1_Value_c
                                          //  Referenced by: '<S117>/RAir1'

    float LocalGravity_Value_h;      // Computed Parameter: LocalGravity_Value_h
                                        //  Referenced by: '<S117>/LocalGravity'

    float RAir_Value_o;                // Computed Parameter: RAir_Value_o
                                          //  Referenced by: '<S117>/RAir'

    float Baroa_Value_g;               // Computed Parameter: Baroa_Value_g
                                          //  Referenced by: '<S119>/Baroa'

    float BarogR_Value_i;              // Computed Parameter: BarogR_Value_i
                                          //  Referenced by: '<S119>/Barog//R'

    float Constant2_Value_lk;          // Computed Parameter: Constant2_Value_lk
                                          //  Referenced by: '<S122>/Constant2'

    float Constant1_Value_p;           // Computed Parameter: Constant1_Value_p
                                          //  Referenced by: '<S122>/Constant1'

    float frequencyPrediction_Value;
                                // Computed Parameter: frequencyPrediction_Value
                                   //  Referenced by: '<S116>/frequencyPrediction'

    float Constant_Value_ez;           // Computed Parameter: Constant_Value_ez
                                          //  Referenced by: '<S134>/Constant'

    float IdentityMatrix_IDMatrixData_n[4];
                            // Computed Parameter: IdentityMatrix_IDMatrixData_n
                               //  Referenced by: '<S136>/Identity Matrix'

    float Constant_Value_k5[36];       // Computed Parameter: Constant_Value_k5
                                          //  Referenced by: '<S127>/Constant'

    float Constant1_Value_h[2];        // Computed Parameter: Constant1_Value_h
                                          //  Referenced by: '<S124>/Constant1'

    float Constant8_Value;             // Computed Parameter: Constant8_Value
                                          //  Referenced by: '<S124>/Constant8'

    float BarogR_Value_b;              // Computed Parameter: BarogR_Value_b
                                          //  Referenced by: '<S124>/Barog//R'

    float Constant2_Value_n;           // Computed Parameter: Constant2_Value_n
                                          //  Referenced by: '<S130>/Constant2'

    float Constant1_Value_j;           // Computed Parameter: Constant1_Value_j
                                          //  Referenced by: '<S130>/Constant1'

    float Constant11_Value;            // Computed Parameter: Constant11_Value
                                          //  Referenced by: '<S129>/Constant11'

    float Constant10_Value;            // Computed Parameter: Constant10_Value
                                          //  Referenced by: '<S129>/Constant10'

    float Constant6_Value_f;           // Computed Parameter: Constant6_Value_f
                                          //  Referenced by: '<S129>/Constant6'

    float Gain2_Gain_h;                // Computed Parameter: Gain2_Gain_h
                                          //  Referenced by: '<S129>/Gain2'

    float Bias2_Bias;                  // Computed Parameter: Bias2_Bias
                                          //  Referenced by: '<S129>/Bias2'

    float IsaGamma_Value;              // Computed Parameter: IsaGamma_Value
                                          //  Referenced by: '<S124>/IsaGamma'

    float Bias1_Bias;                  // Computed Parameter: Bias1_Bias
                                          //  Referenced by: '<S124>/Bias1'

    float Bias4_Bias;                  // Computed Parameter: Bias4_Bias
                                          //  Referenced by: '<S124>/Bias4'

    float Gain1_Gain_b;                // Computed Parameter: Gain1_Gain_b
                                          //  Referenced by: '<S124>/Gain1'

    float RAir1_Value_cc;              // Computed Parameter: RAir1_Value_cc
                                          //  Referenced by: '<S128>/RAir1'

    float LocalGravity_Value_p;      // Computed Parameter: LocalGravity_Value_p
                                        //  Referenced by: '<S128>/LocalGravity'

    float RAir_Value_p;                // Computed Parameter: RAir_Value_p
                                          //  Referenced by: '<S128>/RAir'

    float Bias2_Bias_l;                // Computed Parameter: Bias2_Bias_l
                                          //  Referenced by: '<S124>/Bias2'

    float Baro05R_Gain;                // Computed Parameter: Baro05R_Gain
                                          //  Referenced by: '<S124>/Baro0.5//R'

    float Bias3_Bias;                  // Computed Parameter: Bias3_Bias
                                          //  Referenced by: '<S124>/Bias3'

    float Baro1R_Gain;                 // Computed Parameter: Baro1R_Gain
                                          //  Referenced by: '<S124>/Baro1//R'

    float Constant3_Value_a[2];        // Computed Parameter: Constant3_Value_a
                                          //  Referenced by: '<S124>/Constant3'

    float Constant9_Value_o[3];        // Computed Parameter: Constant9_Value_o
                                          //  Referenced by: '<S124>/Constant9'

    float PitotDBaroSigma_Value[4]; // Computed Parameter: PitotDBaroSigma_Value
                                       //  Referenced by: '<S99>/PitotDBaroSigma'

    float NASCondLim_Value_g;          // Computed Parameter: NASCondLim_Value_g
                                          //  Referenced by: '<S125>/NASCondLim'

    float Gain_Gain_l;                 // Computed Parameter: Gain_Gain_l
                                          //  Referenced by: '<S137>/Gain'

    float RAir1_Value_m;               // Computed Parameter: RAir1_Value_m
                                          //  Referenced by: '<S139>/RAir1'

    float LocalGravity_Value_e;      // Computed Parameter: LocalGravity_Value_e
                                        //  Referenced by: '<S139>/LocalGravity'

    float RAir_Value_a;                // Computed Parameter: RAir_Value_a
                                          //  Referenced by: '<S139>/RAir'

    float Constant_Value_pb;           // Computed Parameter: Constant_Value_pb
                                          //  Referenced by: '<S137>/Constant'

    float IsaGamma1_Value;             // Computed Parameter: IsaGamma1_Value
                                          //  Referenced by: '<S140>/IsaGamma-1'

    float AirR_Value;                  // Computed Parameter: AirR_Value
                                          //  Referenced by: '<S140>/AirR'

    float IsaGamma_Value_i;            // Computed Parameter: IsaGamma_Value_i
                                          //  Referenced by: '<S140>/IsaGamma'

    float Baroa_Value_a;               // Computed Parameter: Baroa_Value_a
                                          //  Referenced by: '<S137>/Baroa'

    float Gain2_Gain_k;                // Computed Parameter: Gain2_Gain_k
                                          //  Referenced by: '<S140>/Gain2'

    float Bias2_Bias_b;                // Computed Parameter: Bias2_Bias_b
                                          //  Referenced by: '<S140>/Bias2'

    float IsaGamma_Value_iz;           // Computed Parameter: IsaGamma_Value_iz
                                          //  Referenced by: '<S137>/IsaGamma'

    float Bias_Bias_p;                 // Computed Parameter: Bias_Bias_p
                                          //  Referenced by: '<S137>/Bias'

    float Constant_Value_j3;           // Computed Parameter: Constant_Value_j3
                                          //  Referenced by: '<S154>/Constant'

    float IdentityMatrix_IDMatrixData_p[4];
                            // Computed Parameter: IdentityMatrix_IDMatrixData_p
                               //  Referenced by: '<S156>/Identity Matrix'

    float Constant_Value_i[36];        // Computed Parameter: Constant_Value_i
                                          //  Referenced by: '<S147>/Constant'

    float Constant1_Value_k[2];        // Computed Parameter: Constant1_Value_k
                                          //  Referenced by: '<S144>/Constant1'

    float Constant8_Value_c;           // Computed Parameter: Constant8_Value_c
                                          //  Referenced by: '<S144>/Constant8'

    float Constant4_Value;             // Computed Parameter: Constant4_Value
                                          //  Referenced by: '<S144>/Constant4'

    float Constant7_Value;             // Computed Parameter: Constant7_Value
                                          //  Referenced by: '<S144>/Constant7'

    float Constant2_Value_d;           // Computed Parameter: Constant2_Value_d
                                          //  Referenced by: '<S149>/Constant2'

    float Constant1_Value_n;           // Computed Parameter: Constant1_Value_n
                                          //  Referenced by: '<S149>/Constant1'

    float Constant11_Value_k;          // Computed Parameter: Constant11_Value_k
                                          //  Referenced by: '<S148>/Constant11'

    float Constant10_Value_i;          // Computed Parameter: Constant10_Value_i
                                          //  Referenced by: '<S148>/Constant10'

    float Constant6_Value_j;           // Computed Parameter: Constant6_Value_j
                                          //  Referenced by: '<S148>/Constant6'

    float Gain2_Gain_n;                // Computed Parameter: Gain2_Gain_n
                                          //  Referenced by: '<S148>/Gain2'

    float Bias2_Bias_f;                // Computed Parameter: Bias2_Bias_f
                                          //  Referenced by: '<S148>/Bias2'

    float Constant2_Value_f;           // Computed Parameter: Constant2_Value_f
                                          //  Referenced by: '<S144>/Constant2'

    float Bias1_Bias_c;                // Computed Parameter: Bias1_Bias_c
                                          //  Referenced by: '<S144>/Bias1'

    float Bias4_Bias_f;                // Computed Parameter: Bias4_Bias_f
                                          //  Referenced by: '<S144>/Bias4'

    float Gain1_Gain_n;                // Computed Parameter: Gain1_Gain_n
                                          //  Referenced by: '<S144>/Gain1'

    float RAir1_Value_d;               // Computed Parameter: RAir1_Value_d
                                          //  Referenced by: '<S150>/RAir1'

    float LocalGravity_Value_o;      // Computed Parameter: LocalGravity_Value_o
                                        //  Referenced by: '<S150>/LocalGravity'

    float RAir_Value_k;                // Computed Parameter: RAir_Value_k
                                          //  Referenced by: '<S150>/RAir'

    float Bias2_Bias_j;                // Computed Parameter: Bias2_Bias_j
                                          //  Referenced by: '<S144>/Bias2'

    float Gain_Gain_c1;                // Computed Parameter: Gain_Gain_c1
                                          //  Referenced by: '<S144>/Gain'

    float Bias3_Bias_f;                // Computed Parameter: Bias3_Bias_f
                                          //  Referenced by: '<S144>/Bias3'

    float Gain2_Gain_f;                // Computed Parameter: Gain2_Gain_f
                                          //  Referenced by: '<S144>/Gain2'

    float Constant3_Value_l[2];        // Computed Parameter: Constant3_Value_l
                                          //  Referenced by: '<S144>/Constant3'

    float Constant9_Value_m[3];        // Computed Parameter: Constant9_Value_m
                                          //  Referenced by: '<S144>/Constant9'

    float PitotDSSigma_Value[4];       // Computed Parameter: PitotDSSigma_Value
                                          //  Referenced by: '<S100>/PitotDSSigma'

    float NASCondLim_Value_i;          // Computed Parameter: NASCondLim_Value_i
                                          //  Referenced by: '<S145>/NASCondLim'

    float Gain_Gain_f;                 // Computed Parameter: Gain_Gain_f
                                          //  Referenced by: '<S157>/Gain'

    float RAir1_Value_b;               // Computed Parameter: RAir1_Value_b
                                          //  Referenced by: '<S159>/RAir1'

    float LocalGravity_Value_d;      // Computed Parameter: LocalGravity_Value_d
                                        //  Referenced by: '<S159>/LocalGravity'

    float RAir_Value_j;                // Computed Parameter: RAir_Value_j
                                          //  Referenced by: '<S159>/RAir'

    float Constant_Value_nu;           // Computed Parameter: Constant_Value_nu
                                          //  Referenced by: '<S157>/Constant'

    float IsaGamma1_Value_e;           // Computed Parameter: IsaGamma1_Value_e
                                          //  Referenced by: '<S160>/IsaGamma-1'

    float AirR_Value_m;                // Computed Parameter: AirR_Value_m
                                          //  Referenced by: '<S160>/AirR'

    float IsaGamma_Value_m;            // Computed Parameter: IsaGamma_Value_m
                                          //  Referenced by: '<S160>/IsaGamma'

    float Baroa_Value_p;               // Computed Parameter: Baroa_Value_p
                                          //  Referenced by: '<S157>/Baroa'

    float Gain2_Gain_e;                // Computed Parameter: Gain2_Gain_e
                                          //  Referenced by: '<S160>/Gain2'

    float Bias2_Bias_lw;               // Computed Parameter: Bias2_Bias_lw
                                          //  Referenced by: '<S160>/Bias2'

    float IsaGamma_Value_j;            // Computed Parameter: IsaGamma_Value_j
                                          //  Referenced by: '<S157>/IsaGamma'

    float Bias_Bias_h;                 // Computed Parameter: Bias_Bias_h
                                          //  Referenced by: '<S157>/Bias'

    float Constant_Value_is;           // Computed Parameter: Constant_Value_is
                                          //  Referenced by: '<S170>/Constant'

    float IdentityMatrix_IDMatrixData_p4;
                           // Computed Parameter: IdentityMatrix_IDMatrixData_p4
                              //  Referenced by: '<S172>/Identity Matrix'

    float Constant_Value_px[36];       // Computed Parameter: Constant_Value_px
                                          //  Referenced by: '<S167>/Constant'

    float Constant3_Value_g[2];        // Computed Parameter: Constant3_Value_g
                                          //  Referenced by: '<S164>/Constant3'

    float Baroa_Value_h;               // Computed Parameter: Baroa_Value_h
                                          //  Referenced by: '<S164>/Baroa'

    float BarogR_Value_k;              // Computed Parameter: BarogR_Value_k
                                          //  Referenced by: '<S164>/Barog//R'

    float Constant2_Value_l1;          // Computed Parameter: Constant2_Value_l1
                                          //  Referenced by: '<S164>/Constant2'

    float Constant1_Value_l;           // Computed Parameter: Constant1_Value_l
                                          //  Referenced by: '<S164>/Constant1'

    float Constant9_Value_b[3];        // Computed Parameter: Constant9_Value_b
                                          //  Referenced by: '<S164>/Constant9'

    float BaroSigma_Value;             // Computed Parameter: BaroSigma_Value
                                          //  Referenced by: '<S101>/BaroSigma'

    float NASCondLim_Value_i1;        // Computed Parameter: NASCondLim_Value_i1
                                         //  Referenced by: '<S165>/NASCondLim'

    float Gain_Gain_e5;                // Computed Parameter: Gain_Gain_e5
                                          //  Referenced by: '<S166>/Gain'

    float RAir1_Value_ba;              // Computed Parameter: RAir1_Value_ba
                                          //  Referenced by: '<S173>/RAir1'

    float LocalGravity_Value_dq;    // Computed Parameter: LocalGravity_Value_dq
                                       //  Referenced by: '<S173>/LocalGravity'

    float RAir_Value_oc;               // Computed Parameter: RAir_Value_oc
                                          //  Referenced by: '<S173>/RAir'

    float Constant_Value_m;            // Computed Parameter: Constant_Value_m
                                          //  Referenced by: '<S183>/Constant'

    float IdentityMatrix_IDMatrixData_iu;
                           // Computed Parameter: IdentityMatrix_IDMatrixData_iu
                              //  Referenced by: '<S185>/Identity Matrix'

    float Constant_Value_o[36];        // Computed Parameter: Constant_Value_o
                                          //  Referenced by: '<S180>/Constant'

    float Constant3_Value_gt[2];       // Computed Parameter: Constant3_Value_gt
                                          //  Referenced by: '<S177>/Constant3'

    float Baroa_Value_n;               // Computed Parameter: Baroa_Value_n
                                          //  Referenced by: '<S177>/Baroa'

    float BarogR_Value_h;              // Computed Parameter: BarogR_Value_h
                                          //  Referenced by: '<S177>/Barog//R'

    float Constant2_Value_o;           // Computed Parameter: Constant2_Value_o
                                          //  Referenced by: '<S177>/Constant2'

    float Constant1_Value_c;           // Computed Parameter: Constant1_Value_c
                                          //  Referenced by: '<S177>/Constant1'

    float Constant9_Value_g[3];        // Computed Parameter: Constant9_Value_g
                                          //  Referenced by: '<S177>/Constant9'

    float PitotSigma_Value;            // Computed Parameter: PitotSigma_Value
                                          //  Referenced by: '<S102>/PitotSigma'

    float NASCondLim_Value_ii;        // Computed Parameter: NASCondLim_Value_ii
                                         //  Referenced by: '<S178>/NASCondLim'

    float Gain_Gain_n;                 // Computed Parameter: Gain_Gain_n
                                          //  Referenced by: '<S179>/Gain'

    float RAir1_Value_g;               // Computed Parameter: RAir1_Value_g
                                          //  Referenced by: '<S186>/RAir1'

    float LocalGravity_Value_g;      // Computed Parameter: LocalGravity_Value_g
                                        //  Referenced by: '<S186>/LocalGravity'

    float RAir_Value_c;                // Computed Parameter: RAir_Value_c
                                          //  Referenced by: '<S186>/RAir'

    float Gain_Gain_n3;                // Computed Parameter: Gain_Gain_n3
                                          //  Referenced by: '<S212>/Gain'

    float Constant_Value_p2;           // Computed Parameter: Constant_Value_p2
                                          //  Referenced by: '<S213>/Constant'

    float IdentityMatrix_IDMatrixData_m[16];
                            // Computed Parameter: IdentityMatrix_IDMatrixData_m
                               //  Referenced by: '<S215>/Identity Matrix'

    float Constant_Value_a[36];        // Computed Parameter: Constant_Value_a
                                          //  Referenced by: '<S209>/Constant'

    float GPS1a_Value;                 // Computed Parameter: GPS1a_Value
                                          //  Referenced by: '<S206>/GPS1//a'

    float Constant1_Value_i;           // Computed Parameter: Constant1_Value_i
                                          //  Referenced by: '<S206>/Constant1'

    float GPS1a1_Gain;                 // Computed Parameter: GPS1a1_Gain
                                          //  Referenced by: '<S206>/GPS1//a 1'

    float GPSlat0_Bias;                // Computed Parameter: GPSlat0_Bias
                                          //  Referenced by: '<S206>/GPSlat0'

    float Gain1_Gain_ke;               // Computed Parameter: Gain1_Gain_ke
                                          //  Referenced by: '<S210>/Gain1'

    float GPSb1_Gain;                  // Computed Parameter: GPSb1_Gain
                                          //  Referenced by: '<S206>/GPSb1'

    float GPSb_Gain;                   // Computed Parameter: GPSb_Gain
                                          //  Referenced by: '<S206>/GPSb'

    float Constant2_Value_g;           // Computed Parameter: Constant2_Value_g
                                          //  Referenced by: '<S206>/Constant2'

    float Constant5_Value_e[8];        // Computed Parameter: Constant5_Value_e
                                          //  Referenced by: '<S206>/Constant5'

    float Constant6_Value_k[12];       // Computed Parameter: Constant6_Value_k
                                          //  Referenced by: '<S206>/Constant6'

    float GPSSigma_Value[16];          // Computed Parameter: GPSSigma_Value
                                          //  Referenced by: '<S202>/GPSSigma'

    float NASCondLim_Value_d;          // Computed Parameter: NASCondLim_Value_d
                                          //  Referenced by: '<S207>/NASCondLim'

    float GPS1a_Gain;                  // Computed Parameter: GPS1a_Gain
                                          //  Referenced by: '<S208>/GPS1//a'

    float GPSlat0_Bias_g;              // Computed Parameter: GPSlat0_Bias_g
                                          //  Referenced by: '<S208>/GPSlat0'

    float Gain1_Gain_af;               // Computed Parameter: Gain1_Gain_af
                                          //  Referenced by: '<S216>/Gain1'

    float GPS1b_Gain;                  // Computed Parameter: GPS1b_Gain
                                          //  Referenced by: '<S208>/GPS1//b'

    float GPSlon0_Bias;                // Computed Parameter: GPSlon0_Bias
                                          //  Referenced by: '<S208>/GPSlon0'

    float AirR_Value_i;                // Computed Parameter: AirR_Value_i
                                          //  Referenced by: '<S200>/AirR'

    float IsaGamma_Value_b;            // Computed Parameter: IsaGamma_Value_b
                                          //  Referenced by: '<S200>/IsaGamma'

    float Baroa_Value_f;               // Computed Parameter: Baroa_Value_f
                                          //  Referenced by: '<S198>/Baroa'

    float Merge_InitialOutput_f;    // Computed Parameter: Merge_InitialOutput_f
                                       //  Referenced by: '<S96>/Merge'

    float Merge1_InitialOutput;      // Computed Parameter: Merge1_InitialOutput
                                        //  Referenced by: '<S96>/Merge1'

    float Gain1_Gain_e;                // Computed Parameter: Gain1_Gain_e
                                          //  Referenced by: '<S5>/Gain1'

    float Constant_Value_p5;           // Computed Parameter: Constant_Value_p5
                                          //  Referenced by: '<S5>/Constant'

    float Constant1_Value_hg[36];      // Computed Parameter: Constant1_Value_hg
                                          //  Referenced by: '<S5>/Constant1'

    float Bias2_Bias_m[36];            // Computed Parameter: Bias2_Bias_m
                                          //  Referenced by: '<S5>/Bias2'

    float LinearQ_Bias[36];            // Computed Parameter: LinearQ_Bias
                                          //  Referenced by: '<S5>/Linear Q'

    float Gain_Gain_ie;                // Computed Parameter: Gain_Gain_ie
                                          //  Referenced by: '<S225>/Gain'

    float Gain_Gain_ob;                // Computed Parameter: Gain_Gain_ob
                                          //  Referenced by: '<S228>/Gain'

    float Gain_Gain_hm;                // Computed Parameter: Gain_Gain_hm
                                          //  Referenced by: '<S223>/Gain'

    float Gain_Gain_bt;                // Computed Parameter: Gain_Gain_bt
                                          //  Referenced by: '<S229>/Gain'

    float Gain_Gain_p3;                // Computed Parameter: Gain_Gain_p3
                                          //  Referenced by: '<S224>/Gain'

    float Gain_Gain_a;                 // Computed Parameter: Gain_Gain_a
                                          //  Referenced by: '<S227>/Gain'

    float LocalGravity_Value_jy[3]; // Computed Parameter: LocalGravity_Value_jy
                                       //  Referenced by: '<S220>/Local Gravity'

    float AccPropagationFlag_Gain;// Computed Parameter: AccPropagationFlag_Gain
                                     //  Referenced by: '<S5>/AccPropagationFlag'

    float Zero_Value[81];              // Computed Parameter: Zero_Value
                                          //  Referenced by: '<S6>/Zero'

    float UnitDelay6_InitialCondition;
                              // Computed Parameter: UnitDelay6_InitialCondition
                                 //  Referenced by: '<S7>/Unit Delay6'

    float UnitDelay7_InitialCondition;
                              // Computed Parameter: UnitDelay7_InitialCondition
                                 //  Referenced by: '<S7>/Unit Delay7'

    float UnitDelay4_InitialCondition;
                              // Computed Parameter: UnitDelay4_InitialCondition
                                 //  Referenced by: '<S7>/Unit Delay4'

    float UnitDelay8_InitialCondition;
                              // Computed Parameter: UnitDelay8_InitialCondition
                                 //  Referenced by: '<S7>/Unit Delay8'

    float UnitDelay_InitialCondition;
                               // Computed Parameter: UnitDelay_InitialCondition
                                  //  Referenced by: '<S7>/Unit Delay'

    float UnitDelay1_InitialCondition;
                              // Computed Parameter: UnitDelay1_InitialCondition
                                 //  Referenced by: '<S7>/Unit Delay1'

    float UnitDelay2_InitialCondition;
                              // Computed Parameter: UnitDelay2_InitialCondition
                                 //  Referenced by: '<S7>/Unit Delay2'

    float UnitDelay3_InitialCondition;
                              // Computed Parameter: UnitDelay3_InitialCondition
                                 //  Referenced by: '<S7>/Unit Delay3'

    bool Constant_Value_at;            // Expression: magLeft
                                          //  Referenced by: '<S74>/Constant'

    bool AccCorrFlag_Value;            // Expression: flagAccCorrectionANAS
                                          //  Referenced by: '<S11>/AccCorrFlag'

    bool MagCorrFlag_Value;            // Expression: flagMagnetometerANAS
                                          //  Referenced by: '<S51>/MagCorrFlag'

    bool GPSCorrectionFlag_Value;      // Expression: flagGPSANAS
                                          //  Referenced by: '<S203>/GPSCorrectionFlag'

    bool PitotFlag_Value;              // Expression: flagPitotANAS
                                          //  Referenced by: '<S103>/PitotFlag'

    bool BaroFlag_Value;               // Expression: flagBaroANAS
                                          //  Referenced by: '<S103>/BaroFlag'

    bool StaticPitotFlag1_Value;       // Expression: flagUsePitotStatic
                                          //  Referenced by: '<S103>/StaticPitotFlag1'

    bool VaroFlag_Value;               // Expression: flagVaroANAS
                                          //  Referenced by: '<S103>/VaroFlag'

    bool StaticPitotFlag_Value;        // Expression: flagUsePitotStatic
                                          //  Referenced by: '<S103>/StaticPitotFlag'

    uint8_t UnitDelay3_InitialCondition_c;
                            // Computed Parameter: UnitDelay3_InitialCondition_c
                               //  Referenced by: '<S3>/Unit Delay3'

    uint8_t Switch_Threshold;          // Computed Parameter: Switch_Threshold
                                          //  Referenced by: '<S3>/Switch'

    uint8_t Bias1_Bias_k;              // Computed Parameter: Bias1_Bias_k
                                          //  Referenced by: '<S3>/Bias1'

    uint8_t Saturation2_UpperSat;    // Computed Parameter: Saturation2_UpperSat
                                        //  Referenced by: '<S3>/Saturation2'

    uint8_t Saturation2_LowerSat;    // Computed Parameter: Saturation2_LowerSat
                                        //  Referenced by: '<S3>/Saturation2'

    uint8_t Zero_Value_c;              // Computed Parameter: Zero_Value_c
                                          //  Referenced by: '<S193>/Zero'

    uint8_t Zero_Value_e;              // Computed Parameter: Zero_Value_e
                                          //  Referenced by: '<S190>/Zero'

    uint8_t Zero_Value_i;              // Computed Parameter: Zero_Value_i
                                          //  Referenced by: '<S191>/Zero'

    uint8_t Zero_Value_o;              // Computed Parameter: Zero_Value_o
                                          //  Referenced by: '<S195>/Zero'

    uint8_t Zero_Value_d;              // Computed Parameter: Zero_Value_d
                                          //  Referenced by: '<S192>/Zero'

    uint8_t Zero_Value_dv;             // Computed Parameter: Zero_Value_dv
                                          //  Referenced by: '<S194>/Zero'

    uint8_t Merge_InitialOutput_k;  // Computed Parameter: Merge_InitialOutput_k
                                       //  Referenced by: '<S103>/Merge'

    uint8_t UnitDelay3_InitialCondition_l;
                            // Computed Parameter: UnitDelay3_InitialCondition_l
                               //  Referenced by: '<S5>/Unit Delay3'

    uint8_t Bias1_Bias_o;              // Computed Parameter: Bias1_Bias_o
                                          //  Referenced by: '<S5>/Bias1'

    uint8_t Saturation2_UpperSat_c;// Computed Parameter: Saturation2_UpperSat_c
                                      //  Referenced by: '<S5>/Saturation2'

    uint8_t Saturation2_LowerSat_n;// Computed Parameter: Saturation2_LowerSat_n
                                      //  Referenced by: '<S5>/Saturation2'

    P_Nocorrection_ANAS0_i_T Nocorrection_h;// '<S178>/No correction'
    P_Nocorrection_ANAS0_i_T Nocorrection_d;// '<S165>/No correction'
    P_Nocorrection_ANAS0_f_T Nocorrection_g4;// '<S145>/No correction'
    P_Nocorrection_ANAS0_f_T Nocorrection_g;// '<S125>/No correction'
    P_Nocorrection_ANAS0_f_T Nocorrection_o;// '<S107>/No correction'
    P_Nocorrection_ANAS0_T Nocorrection_k;// '<S54>/No correction'
    P_Nocorrection_ANAS0_T Nocorrection;// '<S14>/No correction'
  };

  // Real-time Model Data Structure
  struct RT_MODEL_ANAS0_T {
    //
    //  Timing:
    //  The following substructure contains information regarding
    //  the timing information for the model.

    struct {
      struct {
        uint8_t TID[4];
      } TaskCounters;
    } Timing;
  };

  // Copy Constructor
  ANAS0(ANAS0 const&) = delete;

  // Assignment Operator
  ANAS0& operator= (ANAS0 const&) & = delete;

  // Move Constructor
  ANAS0(ANAS0 &&) = delete;

  // Move Assignment Operator
  ANAS0& operator= (ANAS0 &&) = delete;

  // Real-Time Model get method
  ANAS0::RT_MODEL_ANAS0_T * getRTM();

  // Root inport: '<Root>/ANAS In' set method
  void setANAS_In(ANASIn localArgInput)
  {
    ANAS0_U.ANASIn_c = localArgInput;
  }

  // Root inport: '<Root>/ANAS Reference' set method
  void setANAS_Reference(ANASReference localArgInput)
  {
    ANAS0_U.ANASReference_f = localArgInput;
  }

  // Root outport: '<Root>/ANAS Out' get method
  ANASOut getANAS_Out() const
  {
    return ANAS0_Y.ANASOut_h;
  }

  // Root outport: '<Root>/ANAS Logs OBSW' get method
  ANASLogs getANAS_Logs_OBSW() const
  {
    return ANAS0_Y.ANASLogsOBSW;
  }

  // Root outport: '<Root>/NASDAQ Initial State' get method
  ANAS_NASDAQ getNASDAQ_Initial_State() const
  {
    return ANAS0_Y.NASDAQInitialState;
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
  void initialize();

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

  // Block states
  DW_ANAS0_T ANAS0_DW;

  // Tunable parameters
  static P_ANAS0_T ANAS0_P;

  // private member function(s) for subsystem '<S14>/Correction'
  void ANAS0_Correction(float rty_S[9], const float rtu_PH_prev[9], float rty_K
                        [9]);

  // private member function(s) for subsystem '<S14>/No correction'
  static void ANAS0_Nocorrection(const float rtu_PH_prev[9], float rty_K[9],
    P_Nocorrection_ANAS0_T *localP);

  // private member function(s) for subsystem '<S107>/Correction'
  void ANAS0_Correction_i(float rty_S[4], const float rtu_PH_prev[12], float
    rty_K[12]);

  // private member function(s) for subsystem '<S107>/No correction'
  static void ANAS0_Nocorrection_o(const float rtu_PH_prev[12], float rty_K[12],
    P_Nocorrection_ANAS0_f_T *localP);

  // private member function(s) for subsystem '<S165>/Correction'
  static void ANAS0_Correction_m(float rtu_Sprev, const float rtu_PH_prev[6],
    float rty_K[6], float *rty_S);

  // private member function(s) for subsystem '<S165>/No correction'
  static void ANAS0_Nocorrection_d(float rtu_R, const float rtu_PH_prev[6],
    float rty_K[6], float *rty_S, P_Nocorrection_ANAS0_i_T *localP);

  // Real-Time Model
  RT_MODEL_ANAS0_T ANAS0_M;
};

//-
//  These blocks were eliminated from the model due to optimizations:
//
//  Block '<S21>/Check Signal Attributes' : Unused code path elimination
//  Block '<S49>/FixPt Data Type Duplicate' : Unused code path elimination
//  Block '<S61>/Check Signal Attributes' : Unused code path elimination
//  Block '<S114>/Check Signal Attributes' : Unused code path elimination
//  Block '<S129>/Sqrt' : Unused code path elimination
//  Block '<S135>/Check Signal Attributes' : Unused code path elimination
//  Block '<S137>/AirR' : Unused code path elimination
//  Block '<S137>/Divide3' : Unused code path elimination
//  Block '<S137>/Gain1' : Unused code path elimination
//  Block '<S137>/Product' : Unused code path elimination
//  Block '<S140>/Sqrt' : Unused code path elimination
//  Block '<S148>/Sqrt' : Unused code path elimination
//  Block '<S155>/Check Signal Attributes' : Unused code path elimination
//  Block '<S157>/AirR' : Unused code path elimination
//  Block '<S157>/Divide3' : Unused code path elimination
//  Block '<S157>/Gain1' : Unused code path elimination
//  Block '<S157>/Product' : Unused code path elimination
//  Block '<S160>/Sqrt' : Unused code path elimination
//  Block '<S171>/Check Signal Attributes' : Unused code path elimination
//  Block '<S184>/Check Signal Attributes' : Unused code path elimination
//  Block '<S198>/Add3' : Unused code path elimination
//  Block '<S198>/AirR' : Unused code path elimination
//  Block '<S198>/Bias' : Unused code path elimination
//  Block '<S198>/Constant' : Unused code path elimination
//  Block '<S198>/Divide' : Unused code path elimination
//  Block '<S198>/Divide1' : Unused code path elimination
//  Block '<S198>/Divide3' : Unused code path elimination
//  Block '<S198>/Gain' : Unused code path elimination
//  Block '<S198>/Gain1' : Unused code path elimination
//  Block '<S198>/IsaGamma' : Unused code path elimination
//  Block '<S198>/Power' : Unused code path elimination
//  Block '<S198>/Product' : Unused code path elimination
//  Block '<S199>/Divide' : Unused code path elimination
//  Block '<S199>/Divide1' : Unused code path elimination
//  Block '<S199>/LocalGravity' : Unused code path elimination
//  Block '<S199>/Power' : Unused code path elimination
//  Block '<S199>/Product' : Unused code path elimination
//  Block '<S199>/Product2' : Unused code path elimination
//  Block '<S199>/Product3' : Unused code path elimination
//  Block '<S199>/RAir' : Unused code path elimination
//  Block '<S199>/RAir1' : Unused code path elimination
//  Block '<S199>/Subtract' : Unused code path elimination
//  Block '<S200>/Bias2' : Unused code path elimination
//  Block '<S200>/Divide7' : Unused code path elimination
//  Block '<S200>/Gain2' : Unused code path elimination
//  Block '<S200>/IsaGamma-1' : Unused code path elimination
//  Block '<S214>/Check Signal Attributes' : Unused code path elimination
//  Block '<S15>/Reshape' : Reshape block reduction
//  Block '<S43>/Reshape (9) to [3x3] column-major' : Reshape block reduction
//  Block '<S47>/Reshape (9) to [3x3] column-major' : Reshape block reduction
//  Block '<S55>/Reshape' : Reshape block reduction
//  Block '<S84>/Reshape (9) to [3x3] column-major' : Reshape block reduction
//  Block '<S88>/Reshape (9) to [3x3] column-major' : Reshape block reduction
//  Block '<S93>/Reshape (9) to [3x3] column-major' : Reshape block reduction
//  Block '<S3>/Rate Transition' : Eliminated since input and output rates are identical
//  Block '<S108>/Reshape' : Reshape block reduction
//  Block '<S108>/Reshape1' : Reshape block reduction
//  Block '<S126>/Reshape' : Reshape block reduction
//  Block '<S146>/Reshape' : Reshape block reduction
//  Block '<S208>/Reshape' : Reshape block reduction
//  Block '<S231>/Reshape (9) to [3x3] column-major' : Reshape block reduction


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
//  hilite_system('CHADsimulator/Control Units/Control Units SIM/Main Control Unit (Sim)/Ascent Navigation/Ascent NAS/ANAS')    - opens subsystem CHADsimulator/Control Units/Control Units SIM/Main Control Unit (Sim)/Ascent Navigation/Ascent NAS/ANAS
//  hilite_system('CHADsimulator/Control Units/Control Units SIM/Main Control Unit (Sim)/Ascent Navigation/Ascent NAS/ANAS/Kp') - opens and selects block Kp
//
//  Here is the system hierarchy for this model
//
//  '<Root>' : 'CHADsimulator/Control Units/Control Units SIM/Main Control Unit (Sim)/Ascent Navigation/Ascent NAS'
//  '<S1>'   : 'CHADsimulator/Control Units/Control Units SIM/Main Control Unit (Sim)/Ascent Navigation/Ascent NAS/ANAS'
//  '<S2>'   : 'CHADsimulator/Control Units/Control Units SIM/Main Control Unit (Sim)/Ascent Navigation/Ascent NAS/ANAS/Angular States Corrections'
//  '<S3>'   : 'CHADsimulator/Control Units/Control Units SIM/Main Control Unit (Sim)/Ascent Navigation/Ascent NAS/ANAS/Angular States Prediction'
//  '<S4>'   : 'CHADsimulator/Control Units/Control Units SIM/Main Control Unit (Sim)/Ascent Navigation/Ascent NAS/ANAS/Linear States Corrections'
//  '<S5>'   : 'CHADsimulator/Control Units/Control Units SIM/Main Control Unit (Sim)/Ascent Navigation/Ascent NAS/ANAS/Linear States Prediction'
//  '<S6>'   : 'CHADsimulator/Control Units/Control Units SIM/Main Control Unit (Sim)/Ascent Navigation/Ascent NAS/ANAS/Logging and Output'
//  '<S7>'   : 'CHADsimulator/Control Units/Control Units SIM/Main Control Unit (Sim)/Ascent Navigation/Ascent NAS/ANAS/Memory Management'
//  '<S8>'   : 'CHADsimulator/Control Units/Control Units SIM/Main Control Unit (Sim)/Ascent Navigation/Ascent NAS/ANAS/Angular States Corrections/Accelerometer Correction'
//  '<S9>'   : 'CHADsimulator/Control Units/Control Units SIM/Main Control Unit (Sim)/Ascent Navigation/Ascent NAS/ANAS/Angular States Corrections/Magnetometer Correction'
//  '<S10>'  : 'CHADsimulator/Control Units/Control Units SIM/Main Control Unit (Sim)/Ascent Navigation/Ascent NAS/ANAS/Angular States Corrections/Accelerometer Correction/Active Correction Step Accelerometer'
//  '<S11>'  : 'CHADsimulator/Control Units/Control Units SIM/Main Control Unit (Sim)/Ascent Navigation/Ascent NAS/ANAS/Angular States Corrections/Accelerometer Correction/Correction Check'
//  '<S12>'  : 'CHADsimulator/Control Units/Control Units SIM/Main Control Unit (Sim)/Ascent Navigation/Ascent NAS/ANAS/Angular States Corrections/Accelerometer Correction/No Correction Step'
//  '<S13>'  : 'CHADsimulator/Control Units/Control Units SIM/Main Control Unit (Sim)/Ascent Navigation/Ascent NAS/ANAS/Angular States Corrections/Accelerometer Correction/Active Correction Step Accelerometer/Joseph Formula for Attitude Correction'
//  '<S14>'  : 'CHADsimulator/Control Units/Control Units SIM/Main Control Unit (Sim)/Ascent Navigation/Ascent NAS/ANAS/Angular States Corrections/Accelerometer Correction/Active Correction Step Accelerometer/Kalman Gain'
//  '<S15>'  : 'CHADsimulator/Control Units/Control Units SIM/Main Control Unit (Sim)/Ascent Navigation/Ascent NAS/ANAS/Angular States Corrections/Accelerometer Correction/Active Correction Step Accelerometer/Multiplicative Correction'
//  '<S16>'  : 'CHADsimulator/Control Units/Control Units SIM/Main Control Unit (Sim)/Ascent Navigation/Ascent NAS/ANAS/Angular States Corrections/Accelerometer Correction/Active Correction Step Accelerometer/Residual and H computation'
//  '<S17>'  : 'CHADsimulator/Control Units/Control Units SIM/Main Control Unit (Sim)/Ascent Navigation/Ascent NAS/ANAS/Angular States Corrections/Accelerometer Correction/Active Correction Step Accelerometer/Joseph Formula for Attitude Correction/F'
//  '<S18>'  : 'CHADsimulator/Control Units/Control Units SIM/Main Control Unit (Sim)/Ascent Navigation/Ascent NAS/ANAS/Angular States Corrections/Accelerometer Correction/Active Correction Step Accelerometer/Kalman Gain/Correction'
//  '<S19>'  : 'CHADsimulator/Control Units/Control Units SIM/Main Control Unit (Sim)/Ascent Navigation/Ascent NAS/ANAS/Angular States Corrections/Accelerometer Correction/Active Correction Step Accelerometer/Kalman Gain/No correction'
//  '<S20>'  : 'CHADsimulator/Control Units/Control Units SIM/Main Control Unit (Sim)/Ascent Navigation/Ascent NAS/ANAS/Angular States Corrections/Accelerometer Correction/Active Correction Step Accelerometer/Kalman Gain/Reciprocal Condition'
//  '<S21>'  : 'CHADsimulator/Control Units/Control Units SIM/Main Control Unit (Sim)/Ascent Navigation/Ascent NAS/ANAS/Angular States Corrections/Accelerometer Correction/Active Correction Step Accelerometer/Kalman Gain/Reciprocal Condition/Error if not floating-point'
//  '<S22>'  : 'CHADsimulator/Control Units/Control Units SIM/Main Control Unit (Sim)/Ascent Navigation/Ascent NAS/ANAS/Angular States Corrections/Accelerometer Correction/Active Correction Step Accelerometer/Kalman Gain/Reciprocal Condition/LU invert & Check Singularity'
//  '<S23>'  : 'CHADsimulator/Control Units/Control Units SIM/Main Control Unit (Sim)/Ascent Navigation/Ascent NAS/ANAS/Angular States Corrections/Accelerometer Correction/Active Correction Step Accelerometer/Multiplicative Correction/Quaternion Multiplication'
//  '<S24>'  : 'CHADsimulator/Control Units/Control Units SIM/Main Control Unit (Sim)/Ascent Navigation/Ascent NAS/ANAS/Angular States Corrections/Accelerometer Correction/Active Correction Step Accelerometer/Multiplicative Correction/Quaternion Normalize'
//  '<S25>'  : 'CHADsimulator/Control Units/Control Units SIM/Main Control Unit (Sim)/Ascent Navigation/Ascent NAS/ANAS/Angular States Corrections/Accelerometer Correction/Active Correction Step Accelerometer/Multiplicative Correction/Subsystem Reference'
//  '<S26>'  : 'CHADsimulator/Control Units/Control Units SIM/Main Control Unit (Sim)/Ascent Navigation/Ascent NAS/ANAS/Angular States Corrections/Accelerometer Correction/Active Correction Step Accelerometer/Multiplicative Correction/Quaternion Multiplication/q0'
//  '<S27>'  : 'CHADsimulator/Control Units/Control Units SIM/Main Control Unit (Sim)/Ascent Navigation/Ascent NAS/ANAS/Angular States Corrections/Accelerometer Correction/Active Correction Step Accelerometer/Multiplicative Correction/Quaternion Multiplication/q1'
//  '<S28>'  : 'CHADsimulator/Control Units/Control Units SIM/Main Control Unit (Sim)/Ascent Navigation/Ascent NAS/ANAS/Angular States Corrections/Accelerometer Correction/Active Correction Step Accelerometer/Multiplicative Correction/Quaternion Multiplication/q2'
//  '<S29>'  : 'CHADsimulator/Control Units/Control Units SIM/Main Control Unit (Sim)/Ascent Navigation/Ascent NAS/ANAS/Angular States Corrections/Accelerometer Correction/Active Correction Step Accelerometer/Multiplicative Correction/Quaternion Multiplication/q3'
//  '<S30>'  : 'CHADsimulator/Control Units/Control Units SIM/Main Control Unit (Sim)/Ascent Navigation/Ascent NAS/ANAS/Angular States Corrections/Accelerometer Correction/Active Correction Step Accelerometer/Multiplicative Correction/Quaternion Normalize/Quaternion Modulus'
//  '<S31>'  : 'CHADsimulator/Control Units/Control Units SIM/Main Control Unit (Sim)/Ascent Navigation/Ascent NAS/ANAS/Angular States Corrections/Accelerometer Correction/Active Correction Step Accelerometer/Multiplicative Correction/Quaternion Normalize/Quaternion Modulus/Quaternion Norm'
//  '<S32>'  : 'CHADsimulator/Control Units/Control Units SIM/Main Control Unit (Sim)/Ascent Navigation/Ascent NAS/ANAS/Angular States Corrections/Accelerometer Correction/Active Correction Step Accelerometer/Residual and H computation/Quaternions to  Direction Cosine Matrix1'
//  '<S33>'  : 'CHADsimulator/Control Units/Control Units SIM/Main Control Unit (Sim)/Ascent Navigation/Ascent NAS/ANAS/Angular States Corrections/Accelerometer Correction/Active Correction Step Accelerometer/Residual and H computation/Skew'
//  '<S34>'  : 'CHADsimulator/Control Units/Control Units SIM/Main Control Unit (Sim)/Ascent Navigation/Ascent NAS/ANAS/Angular States Corrections/Accelerometer Correction/Active Correction Step Accelerometer/Residual and H computation/Quaternions to  Direction Cosine Matrix1/A11'
//  '<S35>'  : 'CHADsimulator/Control Units/Control Units SIM/Main Control Unit (Sim)/Ascent Navigation/Ascent NAS/ANAS/Angular States Corrections/Accelerometer Correction/Active Correction Step Accelerometer/Residual and H computation/Quaternions to  Direction Cosine Matrix1/A12'
//  '<S36>'  : 'CHADsimulator/Control Units/Control Units SIM/Main Control Unit (Sim)/Ascent Navigation/Ascent NAS/ANAS/Angular States Corrections/Accelerometer Correction/Active Correction Step Accelerometer/Residual and H computation/Quaternions to  Direction Cosine Matrix1/A13'
//  '<S37>'  : 'CHADsimulator/Control Units/Control Units SIM/Main Control Unit (Sim)/Ascent Navigation/Ascent NAS/ANAS/Angular States Corrections/Accelerometer Correction/Active Correction Step Accelerometer/Residual and H computation/Quaternions to  Direction Cosine Matrix1/A21'
//  '<S38>'  : 'CHADsimulator/Control Units/Control Units SIM/Main Control Unit (Sim)/Ascent Navigation/Ascent NAS/ANAS/Angular States Corrections/Accelerometer Correction/Active Correction Step Accelerometer/Residual and H computation/Quaternions to  Direction Cosine Matrix1/A22'
//  '<S39>'  : 'CHADsimulator/Control Units/Control Units SIM/Main Control Unit (Sim)/Ascent Navigation/Ascent NAS/ANAS/Angular States Corrections/Accelerometer Correction/Active Correction Step Accelerometer/Residual and H computation/Quaternions to  Direction Cosine Matrix1/A23'
//  '<S40>'  : 'CHADsimulator/Control Units/Control Units SIM/Main Control Unit (Sim)/Ascent Navigation/Ascent NAS/ANAS/Angular States Corrections/Accelerometer Correction/Active Correction Step Accelerometer/Residual and H computation/Quaternions to  Direction Cosine Matrix1/A31'
//  '<S41>'  : 'CHADsimulator/Control Units/Control Units SIM/Main Control Unit (Sim)/Ascent Navigation/Ascent NAS/ANAS/Angular States Corrections/Accelerometer Correction/Active Correction Step Accelerometer/Residual and H computation/Quaternions to  Direction Cosine Matrix1/A32'
//  '<S42>'  : 'CHADsimulator/Control Units/Control Units SIM/Main Control Unit (Sim)/Ascent Navigation/Ascent NAS/ANAS/Angular States Corrections/Accelerometer Correction/Active Correction Step Accelerometer/Residual and H computation/Quaternions to  Direction Cosine Matrix1/A33'
//  '<S43>'  : 'CHADsimulator/Control Units/Control Units SIM/Main Control Unit (Sim)/Ascent Navigation/Ascent NAS/ANAS/Angular States Corrections/Accelerometer Correction/Active Correction Step Accelerometer/Residual and H computation/Quaternions to  Direction Cosine Matrix1/Create 3x3 Matrix'
//  '<S44>'  : 'CHADsimulator/Control Units/Control Units SIM/Main Control Unit (Sim)/Ascent Navigation/Ascent NAS/ANAS/Angular States Corrections/Accelerometer Correction/Active Correction Step Accelerometer/Residual and H computation/Quaternions to  Direction Cosine Matrix1/Quaternion Normalize'
//  '<S45>'  : 'CHADsimulator/Control Units/Control Units SIM/Main Control Unit (Sim)/Ascent Navigation/Ascent NAS/ANAS/Angular States Corrections/Accelerometer Correction/Active Correction Step Accelerometer/Residual and H computation/Quaternions to  Direction Cosine Matrix1/Quaternion Normalize/Quaternion Modulus'
//  '<S46>'  : 'CHADsimulator/Control Units/Control Units SIM/Main Control Unit (Sim)/Ascent Navigation/Ascent NAS/ANAS/Angular States Corrections/Accelerometer Correction/Active Correction Step Accelerometer/Residual and H computation/Quaternions to  Direction Cosine Matrix1/Quaternion Normalize/Quaternion Modulus/Quaternion Norm'
//  '<S47>'  : 'CHADsimulator/Control Units/Control Units SIM/Main Control Unit (Sim)/Ascent Navigation/Ascent NAS/ANAS/Angular States Corrections/Accelerometer Correction/Active Correction Step Accelerometer/Residual and H computation/Skew/Create 3x3 Matrix'
//  '<S48>'  : 'CHADsimulator/Control Units/Control Units SIM/Main Control Unit (Sim)/Ascent Navigation/Ascent NAS/ANAS/Angular States Corrections/Accelerometer Correction/Correction Check/Subsystem Reference'
//  '<S49>'  : 'CHADsimulator/Control Units/Control Units SIM/Main Control Unit (Sim)/Ascent Navigation/Ascent NAS/ANAS/Angular States Corrections/Accelerometer Correction/Correction Check/g-0.2<acc<g+0.2'
//  '<S50>'  : 'CHADsimulator/Control Units/Control Units SIM/Main Control Unit (Sim)/Ascent Navigation/Ascent NAS/ANAS/Angular States Corrections/Magnetometer Correction/Active Correction Step Magnetometer'
//  '<S51>'  : 'CHADsimulator/Control Units/Control Units SIM/Main Control Unit (Sim)/Ascent Navigation/Ascent NAS/ANAS/Angular States Corrections/Magnetometer Correction/Correction Check'
//  '<S52>'  : 'CHADsimulator/Control Units/Control Units SIM/Main Control Unit (Sim)/Ascent Navigation/Ascent NAS/ANAS/Angular States Corrections/Magnetometer Correction/No Correction Step'
//  '<S53>'  : 'CHADsimulator/Control Units/Control Units SIM/Main Control Unit (Sim)/Ascent Navigation/Ascent NAS/ANAS/Angular States Corrections/Magnetometer Correction/Active Correction Step Magnetometer/Joseph Formula for Attitude Correction'
//  '<S54>'  : 'CHADsimulator/Control Units/Control Units SIM/Main Control Unit (Sim)/Ascent Navigation/Ascent NAS/ANAS/Angular States Corrections/Magnetometer Correction/Active Correction Step Magnetometer/Kalman Gain'
//  '<S55>'  : 'CHADsimulator/Control Units/Control Units SIM/Main Control Unit (Sim)/Ascent Navigation/Ascent NAS/ANAS/Angular States Corrections/Magnetometer Correction/Active Correction Step Magnetometer/Multiplicative Correction'
//  '<S56>'  : 'CHADsimulator/Control Units/Control Units SIM/Main Control Unit (Sim)/Ascent Navigation/Ascent NAS/ANAS/Angular States Corrections/Magnetometer Correction/Active Correction Step Magnetometer/Residual and H computation'
//  '<S57>'  : 'CHADsimulator/Control Units/Control Units SIM/Main Control Unit (Sim)/Ascent Navigation/Ascent NAS/ANAS/Angular States Corrections/Magnetometer Correction/Active Correction Step Magnetometer/Joseph Formula for Attitude Correction/F'
//  '<S58>'  : 'CHADsimulator/Control Units/Control Units SIM/Main Control Unit (Sim)/Ascent Navigation/Ascent NAS/ANAS/Angular States Corrections/Magnetometer Correction/Active Correction Step Magnetometer/Kalman Gain/Correction'
//  '<S59>'  : 'CHADsimulator/Control Units/Control Units SIM/Main Control Unit (Sim)/Ascent Navigation/Ascent NAS/ANAS/Angular States Corrections/Magnetometer Correction/Active Correction Step Magnetometer/Kalman Gain/No correction'
//  '<S60>'  : 'CHADsimulator/Control Units/Control Units SIM/Main Control Unit (Sim)/Ascent Navigation/Ascent NAS/ANAS/Angular States Corrections/Magnetometer Correction/Active Correction Step Magnetometer/Kalman Gain/Reciprocal Condition'
//  '<S61>'  : 'CHADsimulator/Control Units/Control Units SIM/Main Control Unit (Sim)/Ascent Navigation/Ascent NAS/ANAS/Angular States Corrections/Magnetometer Correction/Active Correction Step Magnetometer/Kalman Gain/Reciprocal Condition/Error if not floating-point'
//  '<S62>'  : 'CHADsimulator/Control Units/Control Units SIM/Main Control Unit (Sim)/Ascent Navigation/Ascent NAS/ANAS/Angular States Corrections/Magnetometer Correction/Active Correction Step Magnetometer/Kalman Gain/Reciprocal Condition/LU invert & Check Singularity'
//  '<S63>'  : 'CHADsimulator/Control Units/Control Units SIM/Main Control Unit (Sim)/Ascent Navigation/Ascent NAS/ANAS/Angular States Corrections/Magnetometer Correction/Active Correction Step Magnetometer/Multiplicative Correction/Quaternion Multiplication'
//  '<S64>'  : 'CHADsimulator/Control Units/Control Units SIM/Main Control Unit (Sim)/Ascent Navigation/Ascent NAS/ANAS/Angular States Corrections/Magnetometer Correction/Active Correction Step Magnetometer/Multiplicative Correction/Quaternion Normalize'
//  '<S65>'  : 'CHADsimulator/Control Units/Control Units SIM/Main Control Unit (Sim)/Ascent Navigation/Ascent NAS/ANAS/Angular States Corrections/Magnetometer Correction/Active Correction Step Magnetometer/Multiplicative Correction/Subsystem Reference'
//  '<S66>'  : 'CHADsimulator/Control Units/Control Units SIM/Main Control Unit (Sim)/Ascent Navigation/Ascent NAS/ANAS/Angular States Corrections/Magnetometer Correction/Active Correction Step Magnetometer/Multiplicative Correction/Quaternion Multiplication/q0'
//  '<S67>'  : 'CHADsimulator/Control Units/Control Units SIM/Main Control Unit (Sim)/Ascent Navigation/Ascent NAS/ANAS/Angular States Corrections/Magnetometer Correction/Active Correction Step Magnetometer/Multiplicative Correction/Quaternion Multiplication/q1'
//  '<S68>'  : 'CHADsimulator/Control Units/Control Units SIM/Main Control Unit (Sim)/Ascent Navigation/Ascent NAS/ANAS/Angular States Corrections/Magnetometer Correction/Active Correction Step Magnetometer/Multiplicative Correction/Quaternion Multiplication/q2'
//  '<S69>'  : 'CHADsimulator/Control Units/Control Units SIM/Main Control Unit (Sim)/Ascent Navigation/Ascent NAS/ANAS/Angular States Corrections/Magnetometer Correction/Active Correction Step Magnetometer/Multiplicative Correction/Quaternion Multiplication/q3'
//  '<S70>'  : 'CHADsimulator/Control Units/Control Units SIM/Main Control Unit (Sim)/Ascent Navigation/Ascent NAS/ANAS/Angular States Corrections/Magnetometer Correction/Active Correction Step Magnetometer/Multiplicative Correction/Quaternion Normalize/Quaternion Modulus'
//  '<S71>'  : 'CHADsimulator/Control Units/Control Units SIM/Main Control Unit (Sim)/Ascent Navigation/Ascent NAS/ANAS/Angular States Corrections/Magnetometer Correction/Active Correction Step Magnetometer/Multiplicative Correction/Quaternion Normalize/Quaternion Modulus/Quaternion Norm'
//  '<S72>'  : 'CHADsimulator/Control Units/Control Units SIM/Main Control Unit (Sim)/Ascent Navigation/Ascent NAS/ANAS/Angular States Corrections/Magnetometer Correction/Active Correction Step Magnetometer/Residual and H computation/Quaternions to  Direction Cosine Matrix1'
//  '<S73>'  : 'CHADsimulator/Control Units/Control Units SIM/Main Control Unit (Sim)/Ascent Navigation/Ascent NAS/ANAS/Angular States Corrections/Magnetometer Correction/Active Correction Step Magnetometer/Residual and H computation/Skew'
//  '<S74>'  : 'CHADsimulator/Control Units/Control Units SIM/Main Control Unit (Sim)/Ascent Navigation/Ascent NAS/ANAS/Angular States Corrections/Magnetometer Correction/Active Correction Step Magnetometer/Residual and H computation/changeReferenceSystem'
//  '<S75>'  : 'CHADsimulator/Control Units/Control Units SIM/Main Control Unit (Sim)/Ascent Navigation/Ascent NAS/ANAS/Angular States Corrections/Magnetometer Correction/Active Correction Step Magnetometer/Residual and H computation/Quaternions to  Direction Cosine Matrix1/A11'
//  '<S76>'  : 'CHADsimulator/Control Units/Control Units SIM/Main Control Unit (Sim)/Ascent Navigation/Ascent NAS/ANAS/Angular States Corrections/Magnetometer Correction/Active Correction Step Magnetometer/Residual and H computation/Quaternions to  Direction Cosine Matrix1/A12'
//  '<S77>'  : 'CHADsimulator/Control Units/Control Units SIM/Main Control Unit (Sim)/Ascent Navigation/Ascent NAS/ANAS/Angular States Corrections/Magnetometer Correction/Active Correction Step Magnetometer/Residual and H computation/Quaternions to  Direction Cosine Matrix1/A13'
//  '<S78>'  : 'CHADsimulator/Control Units/Control Units SIM/Main Control Unit (Sim)/Ascent Navigation/Ascent NAS/ANAS/Angular States Corrections/Magnetometer Correction/Active Correction Step Magnetometer/Residual and H computation/Quaternions to  Direction Cosine Matrix1/A21'
//  '<S79>'  : 'CHADsimulator/Control Units/Control Units SIM/Main Control Unit (Sim)/Ascent Navigation/Ascent NAS/ANAS/Angular States Corrections/Magnetometer Correction/Active Correction Step Magnetometer/Residual and H computation/Quaternions to  Direction Cosine Matrix1/A22'
//  '<S80>'  : 'CHADsimulator/Control Units/Control Units SIM/Main Control Unit (Sim)/Ascent Navigation/Ascent NAS/ANAS/Angular States Corrections/Magnetometer Correction/Active Correction Step Magnetometer/Residual and H computation/Quaternions to  Direction Cosine Matrix1/A23'
//  '<S81>'  : 'CHADsimulator/Control Units/Control Units SIM/Main Control Unit (Sim)/Ascent Navigation/Ascent NAS/ANAS/Angular States Corrections/Magnetometer Correction/Active Correction Step Magnetometer/Residual and H computation/Quaternions to  Direction Cosine Matrix1/A31'
//  '<S82>'  : 'CHADsimulator/Control Units/Control Units SIM/Main Control Unit (Sim)/Ascent Navigation/Ascent NAS/ANAS/Angular States Corrections/Magnetometer Correction/Active Correction Step Magnetometer/Residual and H computation/Quaternions to  Direction Cosine Matrix1/A32'
//  '<S83>'  : 'CHADsimulator/Control Units/Control Units SIM/Main Control Unit (Sim)/Ascent Navigation/Ascent NAS/ANAS/Angular States Corrections/Magnetometer Correction/Active Correction Step Magnetometer/Residual and H computation/Quaternions to  Direction Cosine Matrix1/A33'
//  '<S84>'  : 'CHADsimulator/Control Units/Control Units SIM/Main Control Unit (Sim)/Ascent Navigation/Ascent NAS/ANAS/Angular States Corrections/Magnetometer Correction/Active Correction Step Magnetometer/Residual and H computation/Quaternions to  Direction Cosine Matrix1/Create 3x3 Matrix'
//  '<S85>'  : 'CHADsimulator/Control Units/Control Units SIM/Main Control Unit (Sim)/Ascent Navigation/Ascent NAS/ANAS/Angular States Corrections/Magnetometer Correction/Active Correction Step Magnetometer/Residual and H computation/Quaternions to  Direction Cosine Matrix1/Quaternion Normalize'
//  '<S86>'  : 'CHADsimulator/Control Units/Control Units SIM/Main Control Unit (Sim)/Ascent Navigation/Ascent NAS/ANAS/Angular States Corrections/Magnetometer Correction/Active Correction Step Magnetometer/Residual and H computation/Quaternions to  Direction Cosine Matrix1/Quaternion Normalize/Quaternion Modulus'
//  '<S87>'  : 'CHADsimulator/Control Units/Control Units SIM/Main Control Unit (Sim)/Ascent Navigation/Ascent NAS/ANAS/Angular States Corrections/Magnetometer Correction/Active Correction Step Magnetometer/Residual and H computation/Quaternions to  Direction Cosine Matrix1/Quaternion Normalize/Quaternion Modulus/Quaternion Norm'
//  '<S88>'  : 'CHADsimulator/Control Units/Control Units SIM/Main Control Unit (Sim)/Ascent Navigation/Ascent NAS/ANAS/Angular States Corrections/Magnetometer Correction/Active Correction Step Magnetometer/Residual and H computation/Skew/Create 3x3 Matrix'
//  '<S89>'  : 'CHADsimulator/Control Units/Control Units SIM/Main Control Unit (Sim)/Ascent Navigation/Ascent NAS/ANAS/Angular States Prediction/Difference'
//  '<S90>'  : 'CHADsimulator/Control Units/Control Units SIM/Main Control Unit (Sim)/Ascent Navigation/Ascent NAS/ANAS/Angular States Prediction/Omega'
//  '<S91>'  : 'CHADsimulator/Control Units/Control Units SIM/Main Control Unit (Sim)/Ascent Navigation/Ascent NAS/ANAS/Angular States Prediction/Quaternion Normalize'
//  '<S92>'  : 'CHADsimulator/Control Units/Control Units SIM/Main Control Unit (Sim)/Ascent Navigation/Ascent NAS/ANAS/Angular States Prediction/Omega/Skew'
//  '<S93>'  : 'CHADsimulator/Control Units/Control Units SIM/Main Control Unit (Sim)/Ascent Navigation/Ascent NAS/ANAS/Angular States Prediction/Omega/Skew/Create 3x3 Matrix'
//  '<S94>'  : 'CHADsimulator/Control Units/Control Units SIM/Main Control Unit (Sim)/Ascent Navigation/Ascent NAS/ANAS/Angular States Prediction/Quaternion Normalize/Quaternion Modulus'
//  '<S95>'  : 'CHADsimulator/Control Units/Control Units SIM/Main Control Unit (Sim)/Ascent Navigation/Ascent NAS/ANAS/Angular States Prediction/Quaternion Normalize/Quaternion Modulus/Quaternion Norm'
//  '<S96>'  : 'CHADsimulator/Control Units/Control Units SIM/Main Control Unit (Sim)/Ascent Navigation/Ascent NAS/ANAS/Linear States Corrections/Barometric Corrections'
//  '<S97>'  : 'CHADsimulator/Control Units/Control Units SIM/Main Control Unit (Sim)/Ascent Navigation/Ascent NAS/ANAS/Linear States Corrections/GPS Correction'
//  '<S98>'  : 'CHADsimulator/Control Units/Control Units SIM/Main Control Unit (Sim)/Ascent Navigation/Ascent NAS/ANAS/Linear States Corrections/Barometric Corrections/Active Correction Step Baro & Variometer '
//  '<S99>'  : 'CHADsimulator/Control Units/Control Units SIM/Main Control Unit (Sim)/Ascent Navigation/Ascent NAS/ANAS/Linear States Corrections/Barometric Corrections/Active Correction Step Main Static & Pitot'
//  '<S100>' : 'CHADsimulator/Control Units/Control Units SIM/Main Control Unit (Sim)/Ascent Navigation/Ascent NAS/ANAS/Linear States Corrections/Barometric Corrections/Active Correction Step Pitot Static & Differential'
//  '<S101>' : 'CHADsimulator/Control Units/Control Units SIM/Main Control Unit (Sim)/Ascent Navigation/Ascent NAS/ANAS/Linear States Corrections/Barometric Corrections/Active Correction Step Static Main'
//  '<S102>' : 'CHADsimulator/Control Units/Control Units SIM/Main Control Unit (Sim)/Ascent Navigation/Ascent NAS/ANAS/Linear States Corrections/Barometric Corrections/Active Correction Step Static Pitot'
//  '<S103>' : 'CHADsimulator/Control Units/Control Units SIM/Main Control Unit (Sim)/Ascent Navigation/Ascent NAS/ANAS/Linear States Corrections/Barometric Corrections/Correction Check Baro Pitot'
//  '<S104>' : 'CHADsimulator/Control Units/Control Units SIM/Main Control Unit (Sim)/Ascent Navigation/Ascent NAS/ANAS/Linear States Corrections/Barometric Corrections/No Correction Step'
//  '<S105>' : 'CHADsimulator/Control Units/Control Units SIM/Main Control Unit (Sim)/Ascent Navigation/Ascent NAS/ANAS/Linear States Corrections/Barometric Corrections/Active Correction Step Baro & Variometer /Covariance - Joseph formula'
//  '<S106>' : 'CHADsimulator/Control Units/Control Units SIM/Main Control Unit (Sim)/Ascent Navigation/Ascent NAS/ANAS/Linear States Corrections/Barometric Corrections/Active Correction Step Baro & Variometer /H Matrix'
//  '<S107>' : 'CHADsimulator/Control Units/Control Units SIM/Main Control Unit (Sim)/Ascent Navigation/Ascent NAS/ANAS/Linear States Corrections/Barometric Corrections/Active Correction Step Baro & Variometer /Kalman Gain'
//  '<S108>' : 'CHADsimulator/Control Units/Control Units SIM/Main Control Unit (Sim)/Ascent Navigation/Ascent NAS/ANAS/Linear States Corrections/Barometric Corrections/Active Correction Step Baro & Variometer /Residual'
//  '<S109>' : 'CHADsimulator/Control Units/Control Units SIM/Main Control Unit (Sim)/Ascent Navigation/Ascent NAS/ANAS/Linear States Corrections/Barometric Corrections/Active Correction Step Baro & Variometer /Covariance - Joseph formula/F'
//  '<S110>' : 'CHADsimulator/Control Units/Control Units SIM/Main Control Unit (Sim)/Ascent Navigation/Ascent NAS/ANAS/Linear States Corrections/Barometric Corrections/Active Correction Step Baro & Variometer /H Matrix/Subsystem4'
//  '<S111>' : 'CHADsimulator/Control Units/Control Units SIM/Main Control Unit (Sim)/Ascent Navigation/Ascent NAS/ANAS/Linear States Corrections/Barometric Corrections/Active Correction Step Baro & Variometer /Kalman Gain/Correction'
//  '<S112>' : 'CHADsimulator/Control Units/Control Units SIM/Main Control Unit (Sim)/Ascent Navigation/Ascent NAS/ANAS/Linear States Corrections/Barometric Corrections/Active Correction Step Baro & Variometer /Kalman Gain/No correction'
//  '<S113>' : 'CHADsimulator/Control Units/Control Units SIM/Main Control Unit (Sim)/Ascent Navigation/Ascent NAS/ANAS/Linear States Corrections/Barometric Corrections/Active Correction Step Baro & Variometer /Kalman Gain/Reciprocal Condition'
//  '<S114>' : 'CHADsimulator/Control Units/Control Units SIM/Main Control Unit (Sim)/Ascent Navigation/Ascent NAS/ANAS/Linear States Corrections/Barometric Corrections/Active Correction Step Baro & Variometer /Kalman Gain/Reciprocal Condition/Error if not floating-point'
//  '<S115>' : 'CHADsimulator/Control Units/Control Units SIM/Main Control Unit (Sim)/Ascent Navigation/Ascent NAS/ANAS/Linear States Corrections/Barometric Corrections/Active Correction Step Baro & Variometer /Kalman Gain/Reciprocal Condition/LU invert & Check Singularity'
//  '<S116>' : 'CHADsimulator/Control Units/Control Units SIM/Main Control Unit (Sim)/Ascent Navigation/Ascent NAS/ANAS/Linear States Corrections/Barometric Corrections/Active Correction Step Baro & Variometer /Residual/Fake Variometer Measure'
//  '<S117>' : 'CHADsimulator/Control Units/Control Units SIM/Main Control Unit (Sim)/Ascent Navigation/Ascent NAS/ANAS/Linear States Corrections/Barometric Corrections/Active Correction Step Baro & Variometer /Residual/Subsystem4'
//  '<S118>' : 'CHADsimulator/Control Units/Control Units SIM/Main Control Unit (Sim)/Ascent Navigation/Ascent NAS/ANAS/Linear States Corrections/Barometric Corrections/Active Correction Step Baro & Variometer /Residual/Variant Subsystem'
//  '<S119>' : 'CHADsimulator/Control Units/Control Units SIM/Main Control Unit (Sim)/Ascent Navigation/Ascent NAS/ANAS/Linear States Corrections/Barometric Corrections/Active Correction Step Baro & Variometer /Residual/Variometer Estimation'
//  '<S120>' : 'CHADsimulator/Control Units/Control Units SIM/Main Control Unit (Sim)/Ascent Navigation/Ascent NAS/ANAS/Linear States Corrections/Barometric Corrections/Active Correction Step Baro & Variometer /Residual/Fake Variometer Measure/Difference'
//  '<S121>' : 'CHADsimulator/Control Units/Control Units SIM/Main Control Unit (Sim)/Ascent Navigation/Ascent NAS/ANAS/Linear States Corrections/Barometric Corrections/Active Correction Step Baro & Variometer /Residual/Variant Subsystem/Raw Measure'
//  '<S122>' : 'CHADsimulator/Control Units/Control Units SIM/Main Control Unit (Sim)/Ascent Navigation/Ascent NAS/ANAS/Linear States Corrections/Barometric Corrections/Active Correction Step Baro & Variometer /Residual/Variometer Estimation/Subsystem2'
//  '<S123>' : 'CHADsimulator/Control Units/Control Units SIM/Main Control Unit (Sim)/Ascent Navigation/Ascent NAS/ANAS/Linear States Corrections/Barometric Corrections/Active Correction Step Main Static & Pitot/Covariance - Joseph formula'
//  '<S124>' : 'CHADsimulator/Control Units/Control Units SIM/Main Control Unit (Sim)/Ascent Navigation/Ascent NAS/ANAS/Linear States Corrections/Barometric Corrections/Active Correction Step Main Static & Pitot/H Matrix'
//  '<S125>' : 'CHADsimulator/Control Units/Control Units SIM/Main Control Unit (Sim)/Ascent Navigation/Ascent NAS/ANAS/Linear States Corrections/Barometric Corrections/Active Correction Step Main Static & Pitot/Kalman Gain'
//  '<S126>' : 'CHADsimulator/Control Units/Control Units SIM/Main Control Unit (Sim)/Ascent Navigation/Ascent NAS/ANAS/Linear States Corrections/Barometric Corrections/Active Correction Step Main Static & Pitot/Residual computation'
//  '<S127>' : 'CHADsimulator/Control Units/Control Units SIM/Main Control Unit (Sim)/Ascent Navigation/Ascent NAS/ANAS/Linear States Corrections/Barometric Corrections/Active Correction Step Main Static & Pitot/Covariance - Joseph formula/F'
//  '<S128>' : 'CHADsimulator/Control Units/Control Units SIM/Main Control Unit (Sim)/Ascent Navigation/Ascent NAS/ANAS/Linear States Corrections/Barometric Corrections/Active Correction Step Main Static & Pitot/H Matrix/Subsystem Reference'
//  '<S129>' : 'CHADsimulator/Control Units/Control Units SIM/Main Control Unit (Sim)/Ascent Navigation/Ascent NAS/ANAS/Linear States Corrections/Barometric Corrections/Active Correction Step Main Static & Pitot/H Matrix/Subsystem Reference1'
//  '<S130>' : 'CHADsimulator/Control Units/Control Units SIM/Main Control Unit (Sim)/Ascent Navigation/Ascent NAS/ANAS/Linear States Corrections/Barometric Corrections/Active Correction Step Main Static & Pitot/H Matrix/Subsystem2'
//  '<S131>' : 'CHADsimulator/Control Units/Control Units SIM/Main Control Unit (Sim)/Ascent Navigation/Ascent NAS/ANAS/Linear States Corrections/Barometric Corrections/Active Correction Step Main Static & Pitot/H Matrix/Subsystem Reference1/Subsystem Reference'
//  '<S132>' : 'CHADsimulator/Control Units/Control Units SIM/Main Control Unit (Sim)/Ascent Navigation/Ascent NAS/ANAS/Linear States Corrections/Barometric Corrections/Active Correction Step Main Static & Pitot/Kalman Gain/Correction'
//  '<S133>' : 'CHADsimulator/Control Units/Control Units SIM/Main Control Unit (Sim)/Ascent Navigation/Ascent NAS/ANAS/Linear States Corrections/Barometric Corrections/Active Correction Step Main Static & Pitot/Kalman Gain/No correction'
//  '<S134>' : 'CHADsimulator/Control Units/Control Units SIM/Main Control Unit (Sim)/Ascent Navigation/Ascent NAS/ANAS/Linear States Corrections/Barometric Corrections/Active Correction Step Main Static & Pitot/Kalman Gain/Reciprocal Condition'
//  '<S135>' : 'CHADsimulator/Control Units/Control Units SIM/Main Control Unit (Sim)/Ascent Navigation/Ascent NAS/ANAS/Linear States Corrections/Barometric Corrections/Active Correction Step Main Static & Pitot/Kalman Gain/Reciprocal Condition/Error if not floating-point'
//  '<S136>' : 'CHADsimulator/Control Units/Control Units SIM/Main Control Unit (Sim)/Ascent Navigation/Ascent NAS/ANAS/Linear States Corrections/Barometric Corrections/Active Correction Step Main Static & Pitot/Kalman Gain/Reciprocal Condition/LU invert & Check Singularity'
//  '<S137>' : 'CHADsimulator/Control Units/Control Units SIM/Main Control Unit (Sim)/Ascent Navigation/Ascent NAS/ANAS/Linear States Corrections/Barometric Corrections/Active Correction Step Main Static & Pitot/Residual computation/Subsystem2'
//  '<S138>' : 'CHADsimulator/Control Units/Control Units SIM/Main Control Unit (Sim)/Ascent Navigation/Ascent NAS/ANAS/Linear States Corrections/Barometric Corrections/Active Correction Step Main Static & Pitot/Residual computation/Variant Subsystem'
//  '<S139>' : 'CHADsimulator/Control Units/Control Units SIM/Main Control Unit (Sim)/Ascent Navigation/Ascent NAS/ANAS/Linear States Corrections/Barometric Corrections/Active Correction Step Main Static & Pitot/Residual computation/Subsystem2/Subsystem Reference'
//  '<S140>' : 'CHADsimulator/Control Units/Control Units SIM/Main Control Unit (Sim)/Ascent Navigation/Ascent NAS/ANAS/Linear States Corrections/Barometric Corrections/Active Correction Step Main Static & Pitot/Residual computation/Subsystem2/Subsystem Reference1'
//  '<S141>' : 'CHADsimulator/Control Units/Control Units SIM/Main Control Unit (Sim)/Ascent Navigation/Ascent NAS/ANAS/Linear States Corrections/Barometric Corrections/Active Correction Step Main Static & Pitot/Residual computation/Subsystem2/Subsystem Reference1/Subsystem Reference'
//  '<S142>' : 'CHADsimulator/Control Units/Control Units SIM/Main Control Unit (Sim)/Ascent Navigation/Ascent NAS/ANAS/Linear States Corrections/Barometric Corrections/Active Correction Step Main Static & Pitot/Residual computation/Variant Subsystem/Raw Measure'
//  '<S143>' : 'CHADsimulator/Control Units/Control Units SIM/Main Control Unit (Sim)/Ascent Navigation/Ascent NAS/ANAS/Linear States Corrections/Barometric Corrections/Active Correction Step Pitot Static & Differential/Covariance - Joseph formula'
//  '<S144>' : 'CHADsimulator/Control Units/Control Units SIM/Main Control Unit (Sim)/Ascent Navigation/Ascent NAS/ANAS/Linear States Corrections/Barometric Corrections/Active Correction Step Pitot Static & Differential/H matrix computation'
//  '<S145>' : 'CHADsimulator/Control Units/Control Units SIM/Main Control Unit (Sim)/Ascent Navigation/Ascent NAS/ANAS/Linear States Corrections/Barometric Corrections/Active Correction Step Pitot Static & Differential/Kalman Gain'
//  '<S146>' : 'CHADsimulator/Control Units/Control Units SIM/Main Control Unit (Sim)/Ascent Navigation/Ascent NAS/ANAS/Linear States Corrections/Barometric Corrections/Active Correction Step Pitot Static & Differential/Residual computation'
//  '<S147>' : 'CHADsimulator/Control Units/Control Units SIM/Main Control Unit (Sim)/Ascent Navigation/Ascent NAS/ANAS/Linear States Corrections/Barometric Corrections/Active Correction Step Pitot Static & Differential/Covariance - Joseph formula/F'
//  '<S148>' : 'CHADsimulator/Control Units/Control Units SIM/Main Control Unit (Sim)/Ascent Navigation/Ascent NAS/ANAS/Linear States Corrections/Barometric Corrections/Active Correction Step Pitot Static & Differential/H matrix computation/Subsystem Reference1'
//  '<S149>' : 'CHADsimulator/Control Units/Control Units SIM/Main Control Unit (Sim)/Ascent Navigation/Ascent NAS/ANAS/Linear States Corrections/Barometric Corrections/Active Correction Step Pitot Static & Differential/H matrix computation/Subsystem2'
//  '<S150>' : 'CHADsimulator/Control Units/Control Units SIM/Main Control Unit (Sim)/Ascent Navigation/Ascent NAS/ANAS/Linear States Corrections/Barometric Corrections/Active Correction Step Pitot Static & Differential/H matrix computation/Subsystem4'
//  '<S151>' : 'CHADsimulator/Control Units/Control Units SIM/Main Control Unit (Sim)/Ascent Navigation/Ascent NAS/ANAS/Linear States Corrections/Barometric Corrections/Active Correction Step Pitot Static & Differential/H matrix computation/Subsystem Reference1/Subsystem Reference'
//  '<S152>' : 'CHADsimulator/Control Units/Control Units SIM/Main Control Unit (Sim)/Ascent Navigation/Ascent NAS/ANAS/Linear States Corrections/Barometric Corrections/Active Correction Step Pitot Static & Differential/Kalman Gain/Correction'
//  '<S153>' : 'CHADsimulator/Control Units/Control Units SIM/Main Control Unit (Sim)/Ascent Navigation/Ascent NAS/ANAS/Linear States Corrections/Barometric Corrections/Active Correction Step Pitot Static & Differential/Kalman Gain/No correction'
//  '<S154>' : 'CHADsimulator/Control Units/Control Units SIM/Main Control Unit (Sim)/Ascent Navigation/Ascent NAS/ANAS/Linear States Corrections/Barometric Corrections/Active Correction Step Pitot Static & Differential/Kalman Gain/Reciprocal Condition'
//  '<S155>' : 'CHADsimulator/Control Units/Control Units SIM/Main Control Unit (Sim)/Ascent Navigation/Ascent NAS/ANAS/Linear States Corrections/Barometric Corrections/Active Correction Step Pitot Static & Differential/Kalman Gain/Reciprocal Condition/Error if not floating-point'
//  '<S156>' : 'CHADsimulator/Control Units/Control Units SIM/Main Control Unit (Sim)/Ascent Navigation/Ascent NAS/ANAS/Linear States Corrections/Barometric Corrections/Active Correction Step Pitot Static & Differential/Kalman Gain/Reciprocal Condition/LU invert & Check Singularity'
//  '<S157>' : 'CHADsimulator/Control Units/Control Units SIM/Main Control Unit (Sim)/Ascent Navigation/Ascent NAS/ANAS/Linear States Corrections/Barometric Corrections/Active Correction Step Pitot Static & Differential/Residual computation/Subsystem2'
//  '<S158>' : 'CHADsimulator/Control Units/Control Units SIM/Main Control Unit (Sim)/Ascent Navigation/Ascent NAS/ANAS/Linear States Corrections/Barometric Corrections/Active Correction Step Pitot Static & Differential/Residual computation/Variant Subsystem'
//  '<S159>' : 'CHADsimulator/Control Units/Control Units SIM/Main Control Unit (Sim)/Ascent Navigation/Ascent NAS/ANAS/Linear States Corrections/Barometric Corrections/Active Correction Step Pitot Static & Differential/Residual computation/Subsystem2/Subsystem Reference'
//  '<S160>' : 'CHADsimulator/Control Units/Control Units SIM/Main Control Unit (Sim)/Ascent Navigation/Ascent NAS/ANAS/Linear States Corrections/Barometric Corrections/Active Correction Step Pitot Static & Differential/Residual computation/Subsystem2/Subsystem Reference1'
//  '<S161>' : 'CHADsimulator/Control Units/Control Units SIM/Main Control Unit (Sim)/Ascent Navigation/Ascent NAS/ANAS/Linear States Corrections/Barometric Corrections/Active Correction Step Pitot Static & Differential/Residual computation/Subsystem2/Subsystem Reference1/Subsystem Reference'
//  '<S162>' : 'CHADsimulator/Control Units/Control Units SIM/Main Control Unit (Sim)/Ascent Navigation/Ascent NAS/ANAS/Linear States Corrections/Barometric Corrections/Active Correction Step Pitot Static & Differential/Residual computation/Variant Subsystem/Raw Measure'
//  '<S163>' : 'CHADsimulator/Control Units/Control Units SIM/Main Control Unit (Sim)/Ascent Navigation/Ascent NAS/ANAS/Linear States Corrections/Barometric Corrections/Active Correction Step Static Main/Covariance - Joseph formula'
//  '<S164>' : 'CHADsimulator/Control Units/Control Units SIM/Main Control Unit (Sim)/Ascent Navigation/Ascent NAS/ANAS/Linear States Corrections/Barometric Corrections/Active Correction Step Static Main/H Matrix'
//  '<S165>' : 'CHADsimulator/Control Units/Control Units SIM/Main Control Unit (Sim)/Ascent Navigation/Ascent NAS/ANAS/Linear States Corrections/Barometric Corrections/Active Correction Step Static Main/Kalman Gain'
//  '<S166>' : 'CHADsimulator/Control Units/Control Units SIM/Main Control Unit (Sim)/Ascent Navigation/Ascent NAS/ANAS/Linear States Corrections/Barometric Corrections/Active Correction Step Static Main/Residual'
//  '<S167>' : 'CHADsimulator/Control Units/Control Units SIM/Main Control Unit (Sim)/Ascent Navigation/Ascent NAS/ANAS/Linear States Corrections/Barometric Corrections/Active Correction Step Static Main/Covariance - Joseph formula/F'
//  '<S168>' : 'CHADsimulator/Control Units/Control Units SIM/Main Control Unit (Sim)/Ascent Navigation/Ascent NAS/ANAS/Linear States Corrections/Barometric Corrections/Active Correction Step Static Main/Kalman Gain/Correction'
//  '<S169>' : 'CHADsimulator/Control Units/Control Units SIM/Main Control Unit (Sim)/Ascent Navigation/Ascent NAS/ANAS/Linear States Corrections/Barometric Corrections/Active Correction Step Static Main/Kalman Gain/No correction'
//  '<S170>' : 'CHADsimulator/Control Units/Control Units SIM/Main Control Unit (Sim)/Ascent Navigation/Ascent NAS/ANAS/Linear States Corrections/Barometric Corrections/Active Correction Step Static Main/Kalman Gain/Reciprocal Condition'
//  '<S171>' : 'CHADsimulator/Control Units/Control Units SIM/Main Control Unit (Sim)/Ascent Navigation/Ascent NAS/ANAS/Linear States Corrections/Barometric Corrections/Active Correction Step Static Main/Kalman Gain/Reciprocal Condition/Error if not floating-point'
//  '<S172>' : 'CHADsimulator/Control Units/Control Units SIM/Main Control Unit (Sim)/Ascent Navigation/Ascent NAS/ANAS/Linear States Corrections/Barometric Corrections/Active Correction Step Static Main/Kalman Gain/Reciprocal Condition/LU invert & Check Singularity'
//  '<S173>' : 'CHADsimulator/Control Units/Control Units SIM/Main Control Unit (Sim)/Ascent Navigation/Ascent NAS/ANAS/Linear States Corrections/Barometric Corrections/Active Correction Step Static Main/Residual/Subsystem4'
//  '<S174>' : 'CHADsimulator/Control Units/Control Units SIM/Main Control Unit (Sim)/Ascent Navigation/Ascent NAS/ANAS/Linear States Corrections/Barometric Corrections/Active Correction Step Static Main/Residual/Variant Subsystem'
//  '<S175>' : 'CHADsimulator/Control Units/Control Units SIM/Main Control Unit (Sim)/Ascent Navigation/Ascent NAS/ANAS/Linear States Corrections/Barometric Corrections/Active Correction Step Static Main/Residual/Variant Subsystem/Raw Measure'
//  '<S176>' : 'CHADsimulator/Control Units/Control Units SIM/Main Control Unit (Sim)/Ascent Navigation/Ascent NAS/ANAS/Linear States Corrections/Barometric Corrections/Active Correction Step Static Pitot/Covariance - Joseph formula'
//  '<S177>' : 'CHADsimulator/Control Units/Control Units SIM/Main Control Unit (Sim)/Ascent Navigation/Ascent NAS/ANAS/Linear States Corrections/Barometric Corrections/Active Correction Step Static Pitot/H matrix computation'
//  '<S178>' : 'CHADsimulator/Control Units/Control Units SIM/Main Control Unit (Sim)/Ascent Navigation/Ascent NAS/ANAS/Linear States Corrections/Barometric Corrections/Active Correction Step Static Pitot/Kalman Gain'
//  '<S179>' : 'CHADsimulator/Control Units/Control Units SIM/Main Control Unit (Sim)/Ascent Navigation/Ascent NAS/ANAS/Linear States Corrections/Barometric Corrections/Active Correction Step Static Pitot/Residual computation'
//  '<S180>' : 'CHADsimulator/Control Units/Control Units SIM/Main Control Unit (Sim)/Ascent Navigation/Ascent NAS/ANAS/Linear States Corrections/Barometric Corrections/Active Correction Step Static Pitot/Covariance - Joseph formula/F'
//  '<S181>' : 'CHADsimulator/Control Units/Control Units SIM/Main Control Unit (Sim)/Ascent Navigation/Ascent NAS/ANAS/Linear States Corrections/Barometric Corrections/Active Correction Step Static Pitot/Kalman Gain/Correction'
//  '<S182>' : 'CHADsimulator/Control Units/Control Units SIM/Main Control Unit (Sim)/Ascent Navigation/Ascent NAS/ANAS/Linear States Corrections/Barometric Corrections/Active Correction Step Static Pitot/Kalman Gain/No correction'
//  '<S183>' : 'CHADsimulator/Control Units/Control Units SIM/Main Control Unit (Sim)/Ascent Navigation/Ascent NAS/ANAS/Linear States Corrections/Barometric Corrections/Active Correction Step Static Pitot/Kalman Gain/Reciprocal Condition'
//  '<S184>' : 'CHADsimulator/Control Units/Control Units SIM/Main Control Unit (Sim)/Ascent Navigation/Ascent NAS/ANAS/Linear States Corrections/Barometric Corrections/Active Correction Step Static Pitot/Kalman Gain/Reciprocal Condition/Error if not floating-point'
//  '<S185>' : 'CHADsimulator/Control Units/Control Units SIM/Main Control Unit (Sim)/Ascent Navigation/Ascent NAS/ANAS/Linear States Corrections/Barometric Corrections/Active Correction Step Static Pitot/Kalman Gain/Reciprocal Condition/LU invert & Check Singularity'
//  '<S186>' : 'CHADsimulator/Control Units/Control Units SIM/Main Control Unit (Sim)/Ascent Navigation/Ascent NAS/ANAS/Linear States Corrections/Barometric Corrections/Active Correction Step Static Pitot/Residual computation/Subsystem4'
//  '<S187>' : 'CHADsimulator/Control Units/Control Units SIM/Main Control Unit (Sim)/Ascent Navigation/Ascent NAS/ANAS/Linear States Corrections/Barometric Corrections/Active Correction Step Static Pitot/Residual computation/Variant Subsystem1'
//  '<S188>' : 'CHADsimulator/Control Units/Control Units SIM/Main Control Unit (Sim)/Ascent Navigation/Ascent NAS/ANAS/Linear States Corrections/Barometric Corrections/Active Correction Step Static Pitot/Residual computation/Variant Subsystem1/Raw Measure'
//  '<S189>' : 'CHADsimulator/Control Units/Control Units SIM/Main Control Unit (Sim)/Ascent Navigation/Ascent NAS/ANAS/Linear States Corrections/Barometric Corrections/Correction Check Baro Pitot/Check if mach is > than minimum1'
//  '<S190>' : 'CHADsimulator/Control Units/Control Units SIM/Main Control Unit (Sim)/Ascent Navigation/Ascent NAS/ANAS/Linear States Corrections/Barometric Corrections/Correction Check Baro Pitot/If Action Subsystem1'
//  '<S191>' : 'CHADsimulator/Control Units/Control Units SIM/Main Control Unit (Sim)/Ascent Navigation/Ascent NAS/ANAS/Linear States Corrections/Barometric Corrections/Correction Check Baro Pitot/If Action Subsystem2'
//  '<S192>' : 'CHADsimulator/Control Units/Control Units SIM/Main Control Unit (Sim)/Ascent Navigation/Ascent NAS/ANAS/Linear States Corrections/Barometric Corrections/Correction Check Baro Pitot/If Action Subsystem3'
//  '<S193>' : 'CHADsimulator/Control Units/Control Units SIM/Main Control Unit (Sim)/Ascent Navigation/Ascent NAS/ANAS/Linear States Corrections/Barometric Corrections/Correction Check Baro Pitot/If Action Subsystem4'
//  '<S194>' : 'CHADsimulator/Control Units/Control Units SIM/Main Control Unit (Sim)/Ascent Navigation/Ascent NAS/ANAS/Linear States Corrections/Barometric Corrections/Correction Check Baro Pitot/If Action Subsystem5'
//  '<S195>' : 'CHADsimulator/Control Units/Control Units SIM/Main Control Unit (Sim)/Ascent Navigation/Ascent NAS/ANAS/Linear States Corrections/Barometric Corrections/Correction Check Baro Pitot/If Action Subsystem6'
//  '<S196>' : 'CHADsimulator/Control Units/Control Units SIM/Main Control Unit (Sim)/Ascent Navigation/Ascent NAS/ANAS/Linear States Corrections/Barometric Corrections/Correction Check Baro Pitot/Check if mach is > than minimum1/PitotMinMach'
//  '<S197>' : 'CHADsimulator/Control Units/Control Units SIM/Main Control Unit (Sim)/Ascent Navigation/Ascent NAS/ANAS/Linear States Corrections/Barometric Corrections/Correction Check Baro Pitot/Check if mach is > than minimum1/Subsystem Reference'
//  '<S198>' : 'CHADsimulator/Control Units/Control Units SIM/Main Control Unit (Sim)/Ascent Navigation/Ascent NAS/ANAS/Linear States Corrections/Barometric Corrections/Correction Check Baro Pitot/Check if mach is > than minimum1/Subsystem Reference1'
//  '<S199>' : 'CHADsimulator/Control Units/Control Units SIM/Main Control Unit (Sim)/Ascent Navigation/Ascent NAS/ANAS/Linear States Corrections/Barometric Corrections/Correction Check Baro Pitot/Check if mach is > than minimum1/Subsystem Reference1/Subsystem Reference'
//  '<S200>' : 'CHADsimulator/Control Units/Control Units SIM/Main Control Unit (Sim)/Ascent Navigation/Ascent NAS/ANAS/Linear States Corrections/Barometric Corrections/Correction Check Baro Pitot/Check if mach is > than minimum1/Subsystem Reference1/Subsystem Reference1'
//  '<S201>' : 'CHADsimulator/Control Units/Control Units SIM/Main Control Unit (Sim)/Ascent Navigation/Ascent NAS/ANAS/Linear States Corrections/Barometric Corrections/Correction Check Baro Pitot/Check if mach is > than minimum1/Subsystem Reference1/Subsystem Reference1/Subsystem Reference'
//  '<S202>' : 'CHADsimulator/Control Units/Control Units SIM/Main Control Unit (Sim)/Ascent Navigation/Ascent NAS/ANAS/Linear States Corrections/GPS Correction/Active Correction Step GPS'
//  '<S203>' : 'CHADsimulator/Control Units/Control Units SIM/Main Control Unit (Sim)/Ascent Navigation/Ascent NAS/ANAS/Linear States Corrections/GPS Correction/Correction Check'
//  '<S204>' : 'CHADsimulator/Control Units/Control Units SIM/Main Control Unit (Sim)/Ascent Navigation/Ascent NAS/ANAS/Linear States Corrections/GPS Correction/No Correction Step'
//  '<S205>' : 'CHADsimulator/Control Units/Control Units SIM/Main Control Unit (Sim)/Ascent Navigation/Ascent NAS/ANAS/Linear States Corrections/GPS Correction/Active Correction Step GPS/Covariance - Joseph formula'
//  '<S206>' : 'CHADsimulator/Control Units/Control Units SIM/Main Control Unit (Sim)/Ascent Navigation/Ascent NAS/ANAS/Linear States Corrections/GPS Correction/Active Correction Step GPS/H Matrix'
//  '<S207>' : 'CHADsimulator/Control Units/Control Units SIM/Main Control Unit (Sim)/Ascent Navigation/Ascent NAS/ANAS/Linear States Corrections/GPS Correction/Active Correction Step GPS/Kalman Gain'
//  '<S208>' : 'CHADsimulator/Control Units/Control Units SIM/Main Control Unit (Sim)/Ascent Navigation/Ascent NAS/ANAS/Linear States Corrections/GPS Correction/Active Correction Step GPS/Residual'
//  '<S209>' : 'CHADsimulator/Control Units/Control Units SIM/Main Control Unit (Sim)/Ascent Navigation/Ascent NAS/ANAS/Linear States Corrections/GPS Correction/Active Correction Step GPS/Covariance - Joseph formula/F'
//  '<S210>' : 'CHADsimulator/Control Units/Control Units SIM/Main Control Unit (Sim)/Ascent Navigation/Ascent NAS/ANAS/Linear States Corrections/GPS Correction/Active Correction Step GPS/H Matrix/Degrees to Radians'
//  '<S211>' : 'CHADsimulator/Control Units/Control Units SIM/Main Control Unit (Sim)/Ascent Navigation/Ascent NAS/ANAS/Linear States Corrections/GPS Correction/Active Correction Step GPS/Kalman Gain/Correction'
//  '<S212>' : 'CHADsimulator/Control Units/Control Units SIM/Main Control Unit (Sim)/Ascent Navigation/Ascent NAS/ANAS/Linear States Corrections/GPS Correction/Active Correction Step GPS/Kalman Gain/No correction'
//  '<S213>' : 'CHADsimulator/Control Units/Control Units SIM/Main Control Unit (Sim)/Ascent Navigation/Ascent NAS/ANAS/Linear States Corrections/GPS Correction/Active Correction Step GPS/Kalman Gain/Reciprocal Condition'
//  '<S214>' : 'CHADsimulator/Control Units/Control Units SIM/Main Control Unit (Sim)/Ascent Navigation/Ascent NAS/ANAS/Linear States Corrections/GPS Correction/Active Correction Step GPS/Kalman Gain/Reciprocal Condition/Error if not floating-point'
//  '<S215>' : 'CHADsimulator/Control Units/Control Units SIM/Main Control Unit (Sim)/Ascent Navigation/Ascent NAS/ANAS/Linear States Corrections/GPS Correction/Active Correction Step GPS/Kalman Gain/Reciprocal Condition/LU invert & Check Singularity'
//  '<S216>' : 'CHADsimulator/Control Units/Control Units SIM/Main Control Unit (Sim)/Ascent Navigation/Ascent NAS/ANAS/Linear States Corrections/GPS Correction/Active Correction Step GPS/Residual/Degrees to Radians'
//  '<S217>' : 'CHADsimulator/Control Units/Control Units SIM/Main Control Unit (Sim)/Ascent Navigation/Ascent NAS/ANAS/Linear States Corrections/GPS Correction/Correction Check/GPSAccelerationLimit'
//  '<S218>' : 'CHADsimulator/Control Units/Control Units SIM/Main Control Unit (Sim)/Ascent Navigation/Ascent NAS/ANAS/Linear States Corrections/GPS Correction/Correction Check/Subsystem Reference'
//  '<S219>' : 'CHADsimulator/Control Units/Control Units SIM/Main Control Unit (Sim)/Ascent Navigation/Ascent NAS/ANAS/Linear States Prediction/Difference'
//  '<S220>' : 'CHADsimulator/Control Units/Control Units SIM/Main Control Unit (Sim)/Ascent Navigation/Ascent NAS/ANAS/Linear States Prediction/Rotate acceleration  from body to NED'
//  '<S221>' : 'CHADsimulator/Control Units/Control Units SIM/Main Control Unit (Sim)/Ascent Navigation/Ascent NAS/ANAS/Linear States Prediction/Rotate acceleration  from body to NED/Quaternions to  Direction Cosine Matrix'
//  '<S222>' : 'CHADsimulator/Control Units/Control Units SIM/Main Control Unit (Sim)/Ascent Navigation/Ascent NAS/ANAS/Linear States Prediction/Rotate acceleration  from body to NED/Quaternions to  Direction Cosine Matrix/A11'
//  '<S223>' : 'CHADsimulator/Control Units/Control Units SIM/Main Control Unit (Sim)/Ascent Navigation/Ascent NAS/ANAS/Linear States Prediction/Rotate acceleration  from body to NED/Quaternions to  Direction Cosine Matrix/A12'
//  '<S224>' : 'CHADsimulator/Control Units/Control Units SIM/Main Control Unit (Sim)/Ascent Navigation/Ascent NAS/ANAS/Linear States Prediction/Rotate acceleration  from body to NED/Quaternions to  Direction Cosine Matrix/A13'
//  '<S225>' : 'CHADsimulator/Control Units/Control Units SIM/Main Control Unit (Sim)/Ascent Navigation/Ascent NAS/ANAS/Linear States Prediction/Rotate acceleration  from body to NED/Quaternions to  Direction Cosine Matrix/A21'
//  '<S226>' : 'CHADsimulator/Control Units/Control Units SIM/Main Control Unit (Sim)/Ascent Navigation/Ascent NAS/ANAS/Linear States Prediction/Rotate acceleration  from body to NED/Quaternions to  Direction Cosine Matrix/A22'
//  '<S227>' : 'CHADsimulator/Control Units/Control Units SIM/Main Control Unit (Sim)/Ascent Navigation/Ascent NAS/ANAS/Linear States Prediction/Rotate acceleration  from body to NED/Quaternions to  Direction Cosine Matrix/A23'
//  '<S228>' : 'CHADsimulator/Control Units/Control Units SIM/Main Control Unit (Sim)/Ascent Navigation/Ascent NAS/ANAS/Linear States Prediction/Rotate acceleration  from body to NED/Quaternions to  Direction Cosine Matrix/A31'
//  '<S229>' : 'CHADsimulator/Control Units/Control Units SIM/Main Control Unit (Sim)/Ascent Navigation/Ascent NAS/ANAS/Linear States Prediction/Rotate acceleration  from body to NED/Quaternions to  Direction Cosine Matrix/A32'
//  '<S230>' : 'CHADsimulator/Control Units/Control Units SIM/Main Control Unit (Sim)/Ascent Navigation/Ascent NAS/ANAS/Linear States Prediction/Rotate acceleration  from body to NED/Quaternions to  Direction Cosine Matrix/A33'
//  '<S231>' : 'CHADsimulator/Control Units/Control Units SIM/Main Control Unit (Sim)/Ascent Navigation/Ascent NAS/ANAS/Linear States Prediction/Rotate acceleration  from body to NED/Quaternions to  Direction Cosine Matrix/Create 3x3 Matrix'
//  '<S232>' : 'CHADsimulator/Control Units/Control Units SIM/Main Control Unit (Sim)/Ascent Navigation/Ascent NAS/ANAS/Linear States Prediction/Rotate acceleration  from body to NED/Quaternions to  Direction Cosine Matrix/Quaternion Normalize'
//  '<S233>' : 'CHADsimulator/Control Units/Control Units SIM/Main Control Unit (Sim)/Ascent Navigation/Ascent NAS/ANAS/Linear States Prediction/Rotate acceleration  from body to NED/Quaternions to  Direction Cosine Matrix/Quaternion Normalize/Quaternion Modulus'
//  '<S234>' : 'CHADsimulator/Control Units/Control Units SIM/Main Control Unit (Sim)/Ascent Navigation/Ascent NAS/ANAS/Linear States Prediction/Rotate acceleration  from body to NED/Quaternions to  Direction Cosine Matrix/Quaternion Normalize/Quaternion Modulus/Quaternion Norm'


//-
//  Requirements for '<Root>': ANAS0


#endif                                 // ANAS0_h_

//
// File trailer for generated code.
//
// [EOF]
//
