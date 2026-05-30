//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// File: ANAS0.h
//
// Code generated for Simulink model 'ANAS0'.
//
// Model version                  : 11.276
// Simulink Coder version         : 24.2 (R2024b) 21-Jun-2024
// C/C++ source code generated on : Fri May 29 18:43:15 2026
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
#include "rtw_continuous.h"
#include "rtw_solver.h"
#include "ANAS0_types.h"

// Class declaration for model ANAS0
class ANAS0 final
{
  // public data and function members
 public:
  // Block signals and states (default storage) for system '<Root>'
  struct DW_ANAS0_T {
    double ProbeSampleTime[2];         // '<S10>/Probe Sample Time'
    double Merge2[7];                  // '<S2>/Merge2'
    double Merge3[36];                 // '<S2>/Merge3'
    double ProbeSampleTime1[2];        // '<S64>/Probe Sample Time1'
    double Merge[6];                   // '<S60>/Merge'
    double Merge1[36];                 // '<S60>/Merge1'
    double Merge_l[36];                // '<S59>/Merge'
    double Merge1_a[6];                // '<S59>/Merge1'
    double ProbeSampleTime1_b[2];      // '<S76>/Probe Sample Time1'
    double ProbeSampleTime1_i[2];      // '<S90>/Probe Sample Time1'
    double Merge_j[36];                // '<S61>/Merge'
    double Merge1_b[6];                // '<S61>/Merge1'
    double ProbeSampleTime_i[2];       // '<S6>/Probe Sample Time'
    double ProbeSampleTime_k[2];       // '<S7>/Probe Sample Time'
    double Sum1[36];                   // '<S106>/Sum1'
    double Sum[3];                     // '<S107>/Sum'
    double Sum1_j[3];                  // '<S107>/Sum1'
    double Sum1_a[36];                 // '<S46>/Sum1'
    double prevAngularState[7];        // '<S3>/prevAngularState'
    double UnitDelay_DSTATE[7];        // '<S1>/Unit Delay'
    double UnitDelay1_DSTATE[36];      // '<S1>/Unit Delay1'
    double UnitDelay2_DSTATE[6];       // '<S1>/Unit Delay2'
    double UnitDelay5_DSTATE[36];      // '<S1>/Unit Delay5'
    double MatrixDivide_DWORK4[4];     // '<S93>/Matrix Divide'
    double MatrixDivide_DWORK4_b[16];  // '<S81>/Matrix Divide'
    double MatrixDivide_DWORK4_n[9];   // '<S15>/Matrix Divide'
    double Product;                    // '<S54>/Product'
    double Product1;                   // '<S54>/Product1'
    double Product2;                   // '<S54>/Product2'
    double Product3;                   // '<S54>/Product3'
    double A_b;                        // '<S76>/Data Store Memory'
    double A_g;                        // '<S6>/Data Store Memory'
    double A_gb;                       // '<S7>/Data Store Memory'
    double MatrixDivide_DWORK4_a;      // '<S67>/Matrix Divide'
    uint8_t UnitDelay3_DSTATE;         // '<S1>/Unit Delay3'
    uint8_t UnitDelay4_DSTATE;         // '<S1>/Unit Delay4'
    bool NOT;                          // '<S90>/NOT'
  };

  // External inputs (root inport signals with default storage)
  struct ExtU_ANAS0_T {
    ANASIn ANASIn_j;                   // '<Root>/ANAS In'
    ANASReference ANASReferenceIn;     // '<Root>/ANAS Reference In'
  };

  // External outputs (root outports fed by signals with default storage)
  struct ExtY_ANAS0_T {
    ANASOut ANASOut_p;                 // '<Root>/ANAS Out'
    ANAS_NASDAQ NASDAQInitialState;    // '<Root>/NASDAQ Initial State'
    ANASLogs ANASOBSWLogs;             // '<Root>/ANAS OBSW Logs'
  };

  // Parameters (default storage)
  struct P_ANAS0_T {
    double CompareToConstant1_const; // Mask Parameter: CompareToConstant1_const
                                        //  Referenced by: '<S43>/Constant'

    double CompareToConstant1_const_e;
                                   // Mask Parameter: CompareToConstant1_const_e
                                      //  Referenced by: '<S124>/Constant'

    double CompareToConstant1_const_n;
                                   // Mask Parameter: CompareToConstant1_const_n
                                      //  Referenced by: '<S71>/Constant'

    double CompareToConstant1_const_k;
                                   // Mask Parameter: CompareToConstant1_const_k
                                      //  Referenced by: '<S127>/Constant'

    double CompareToConstant1_const_p;
                                   // Mask Parameter: CompareToConstant1_const_p
                                      //  Referenced by: '<S85>/Constant'

    double CompareToConstant1_const_ky;
                                  // Mask Parameter: CompareToConstant1_const_ky
                                     //  Referenced by: '<S98>/Constant'

    double CompareToConstant_const;   // Mask Parameter: CompareToConstant_const
                                         //  Referenced by: '<S102>/Constant'

    double Constant_Value[9];          // Expression: sigmaMag.^2
                                          //  Referenced by: '<S11>/Constant'

    double Constant_Value_c[36];       // Expression: eye(6,6)
                                          //  Referenced by: '<S13>/Constant'

    double Constant_Value_h;           // Expression: 0
                                          //  Referenced by: '<S17>/Constant'

    double Gain_Gain;                  // Expression: 2
                                          //  Referenced by: '<S33>/Gain'

    double Gain_Gain_d;                // Expression: 2
                                          //  Referenced by: '<S36>/Gain'

    double Gain_Gain_k;                // Expression: 2
                                          //  Referenced by: '<S31>/Gain'

    double Gain_Gain_o;                // Expression: 2
                                          //  Referenced by: '<S37>/Gain'

    double Gain_Gain_g;                // Expression: 2
                                          //  Referenced by: '<S32>/Gain'

    double Gain_Gain_m;                // Expression: 2
                                          //  Referenced by: '<S35>/Gain'

    double knownmagdirection_Value[3];
                            // Expression: initialMagnetic/norm(initialMagnetic)
                               //  Referenced by: '<S12>/known mag direction'

    double Gain1_Gain;                 // Expression: -1
                                          //  Referenced by: '<S17>/Gain1'

    double Gain2_Gain;                 // Expression: -1
                                          //  Referenced by: '<S17>/Gain2'

    double Gain_Gain_ok;               // Expression: -1
                                          //  Referenced by: '<S17>/Gain'

    double Constant_Value_d[9];        // Expression: zeros(3,3)
                                          //  Referenced by: '<S12>/Constant'

    double Constant_Value_he[9];       // Expression: sigmaMag.^2
                                          //  Referenced by: '<S15>/Constant'

    double Constant_Value_p;           // Expression: 1
                                          //  Referenced by: '<S18>/Constant'

    double Gain1_Gain_b;               // Expression: 1/4
                                          //  Referenced by: '<S18>/Gain1'

    double Gain_Gain_p;                // Expression: 1/2
                                          //  Referenced by: '<S18>/Gain'

    double nextAngularState_Y0;       // Computed Parameter: nextAngularState_Y0
                                         //  Referenced by: '<S3>/nextAngularState'

    double nextAngularCov_Y0;          // Computed Parameter: nextAngularCov_Y0
                                          //  Referenced by: '<S3>/nextAngularCov'

    double Constant_Value_l;        // Expression: 1/frequencyPredictionAttitude
                                       //  Referenced by: '<S3>/Constant'

    double Constant_Value_hu[36];      // Expression: angQ
                                          //  Referenced by: '<S46>/Constant'

    double Constant_Value_n[9];        // Expression: eye(3,3)
                                          //  Referenced by: '<S48>/Constant'

    double Gain_Gain_e;                // Expression: -1
                                          //  Referenced by: '<S48>/Gain'

    double Constant2_Value[9];         // Expression: eye(3,3)
                                          //  Referenced by: '<S48>/Constant2'

    double Constant3_Value[9];         // Expression: zeros(3,3)
                                          //  Referenced by: '<S48>/Constant3'

    double Constant3_Value_j[9];       // Expression: zeros(3,3)
                                          //  Referenced by: '<S50>/Constant3'

    double Constant_Value_m[9];        // Expression: eye(3,3)
                                          //  Referenced by: '<S50>/Constant'

    double Constant2_Value_i[9];       // Expression: eye(3,3)
                                          //  Referenced by: '<S50>/Constant2'

    double Gain_Gain_l;                // Expression: -1
                                          //  Referenced by: '<S50>/Gain'

    double Constant_Value_ly[9];       // Expression: eye(3,3)
                                          //  Referenced by: '<S49>/Constant'

    double Gain_Gain_c;                // Expression: -1
                                          //  Referenced by: '<S49>/Gain'

    double Constant_Value_g;           // Expression: 0
                                          //  Referenced by: '<S51>/Constant'

    double Gain1_Gain_k;               // Expression: -1
                                          //  Referenced by: '<S51>/Gain1'

    double Gain2_Gain_a;               // Expression: -1
                                          //  Referenced by: '<S51>/Gain2'

    double Gain_Gain_go;               // Expression: -1
                                          //  Referenced by: '<S51>/Gain'

    double Gain1_Gain_e;               // Expression: -1
                                          //  Referenced by: '<S49>/Gain1'

    double Constant3_Value_b[9];       // Expression: zeros(3,3)
                                          //  Referenced by: '<S49>/Constant3'

    double Constant_Value_k[16];       // Expression: eye(4,4)
                                          //  Referenced by: '<S47>/Constant'

    double Constant_Value_c3;          // Expression: 0
                                          //  Referenced by: '<S55>/Constant'

    double gyrobiases_Value[3];
      // Expression: [-0.001907000000000; -0.011470000000000; 0.002193000000000]
         //  Referenced by: '<S47>/gyro biases'

    double Gain1_Gain_p;               // Expression: -1
                                          //  Referenced by: '<S55>/Gain1'

    double Gain2_Gain_h;               // Expression: -1
                                          //  Referenced by: '<S55>/Gain2'

    double Gain_Gain_f;                // Expression: -1
                                          //  Referenced by: '<S55>/Gain'

    double Gain_Gain_h;                // Expression: -1
                                          //  Referenced by: '<S53>/Gain'

    double Constant_Value_b[36];       // Expression: eye(6,6)
                                          //  Referenced by: '<S70>/Constant'

    double Constant_Value_bd[2];       // Expression: [0;0]
                                          //  Referenced by: '<S66>/Constant'

    double Gain4_Gain;                 // Expression: tempGradient
                                          //  Referenced by: '<S66>/Gain4'

    double gR_Value;                   // Expression: localGravity/isaR
                                          //  Referenced by: '<S66>/g R'

    double Constant1_Value[3];         // Expression: [0;0;0]
                                          //  Referenced by: '<S66>/Constant1'

    double Constant_Value_lw;          // Expression: sigmaBaro
                                          //  Referenced by: '<S68>/Constant'

    double Gain_Gain_mn;               // Expression: -1
                                          //  Referenced by: '<S62>/Gain'

    double HeightTemperatureGradient_Value;// Expression: tempGradient
                                              //  Referenced by: '<S69>/HeightTemperatureGradient'

    double gravity_Value;              // Expression: localGravity
                                          //  Referenced by: '<S69>/gravity'

    double Rair_Value;                 // Expression: isaR
                                          //  Referenced by: '<S69>/R air'

    double Constant1_Value_e;          // Expression: initialLong
                                          //  Referenced by: '<S74>/Constant1'

    double Constant2_Value_m;          // Expression: initialLat
                                          //  Referenced by: '<S74>/Constant2'

    double Gain2_Gain_g;               // Expression: 1/gpsA
                                          //  Referenced by: '<S74>/Gain2'

    double Gain1_Gain_bu;              // Expression: pi/180
                                          //  Referenced by: '<S79>/Gain1'

    double Constant_Value_f[36];       // Expression: eye(6,6)
                                          //  Referenced by: '<S83>/Constant'

    double Constant_Value_fh;          // Expression: 1/gpsA
                                          //  Referenced by: '<S80>/Constant'

    double Constant1_Value_p;          // Expression: 0
                                          //  Referenced by: '<S80>/Constant1'

    double Constant2_Value_o;          // Expression: initialLat
                                          //  Referenced by: '<S80>/Constant2'

    double Gain_Gain_c4;               // Expression: 1/gpsA
                                          //  Referenced by: '<S80>/Gain'

    double Gain1_Gain_i;               // Expression: pi/180
                                          //  Referenced by: '<S84>/Gain1'

    double Constant3_Value_e;          // Expression: gpsA
                                          //  Referenced by: '<S80>/Constant3'

    double Gain2_Gain_j;               // Expression: gpsB
                                          //  Referenced by: '<S80>/Gain2'

    double Constant4_Value;            // Expression: 1
                                          //  Referenced by: '<S80>/Constant4'

    double Gain1_Gain_g;               // Expression: gpsB
                                          //  Referenced by: '<S80>/Gain1'

    double Gain3_Gain;                 // Expression: 1000
                                          //  Referenced by: '<S80>/Gain3'

    double Constant5_Value[8];         // Expression: [0 0 0 0; 0 0 0 0]
                                          //  Referenced by: '<S80>/Constant5'

    double Constant6_Value[12];        // Expression: [0 0 0 1 0 0; 0 0 0 0 1 0]
                                          //  Referenced by: '<S80>/Constant6'

    double Constant_Value_m2[16];      // Expression: sigmaGPS
                                          //  Referenced by: '<S82>/Constant'

    double Gain1_Gain_j;               // Expression: gpsB
                                          //  Referenced by: '<S74>/Gain1'

    double Gain_Gain_on[4];            // Expression: [1000; 1000; 1; 1]
                                          //  Referenced by: '<S74>/Gain'

    double Constant_Value_ki[36];      // Expression: eye(6,6)
                                          //  Referenced by: '<S95>/Constant'

    double Constant_Value_h4[2];       // Expression: [0;0]
                                          //  Referenced by: '<S92>/Constant'

    double Constant5_Value_p;          // Expression: 1
                                          //  Referenced by: '<S92>/Constant5'

    double lambda_Value;               // Expression: tempGradient
                                          //  Referenced by: '<S92>/lambda'

    double Constant9_Value;            // Expression: z0
                                          //  Referenced by: '<S92>/Constant9'

    double Constant3_Value_ji;         // Expression: localGravity
                                          //  Referenced by: '<S92>/Constant3'

    double R_Value;                    // Expression: isaR
                                          //  Referenced by: '<S92>/R'

    double Gain_Gain_pa;               // Expression: -1
                                          //  Referenced by: '<S92>/Gain'

    double AddConstant_Bias;           // Expression: -1
                                          //  Referenced by: '<S92>/Add Constant'

    double Constant1_Value_m[3];       // Expression: [0;0;0]
                                          //  Referenced by: '<S92>/Constant1'

    double Gain_Gain_f4;               // Expression: 2
                                          //  Referenced by: '<S97>/Gain'

    double Gain1_Gain_d;               // Expression: 2
                                          //  Referenced by: '<S97>/Gain1'

    double Gain3_Gain_e;               // Expression: -1
                                          //  Referenced by: '<S92>/Gain3'

    double gamma_Value;                // Expression: gamma
                                          //  Referenced by: '<S92>/gamma'

    double AddConstant1_Bias;          // Expression: -1
                                          //  Referenced by: '<S92>/Add Constant1'

    double Gain1_Gain_c;               // Expression: 1/2
                                          //  Referenced by: '<S92>/Gain1'

    double AddConstant3_Bias;          // Expression: +1
                                          //  Referenced by: '<S92>/Add Constant3'

    double AddConstant2_Bias;          // Expression: -1
                                          //  Referenced by: '<S92>/Add Constant2'

    double Constant7_Value[2];         // Expression: [0;0]
                                          //  Referenced by: '<S92>/Constant7'

    double Gain2_Gain_gz;              // Expression: -1
                                          //  Referenced by: '<S92>/Gain2'

    double Constant8_Value[3];         // Expression: [0;0;0]
                                          //  Referenced by: '<S92>/Constant8'

    double Constant6_Value_e;          // Expression: 2
                                          //  Referenced by: '<S92>/Constant6'

    double dstates_Value[18];          // Expression: [zeros(3, 3),eye(3)]
                                          //  Referenced by: '<S92>/dstates'

    double Bias_Bias;                  // Expression: 1
                                          //  Referenced by: '<S92>/Bias'

    double Constant_Value_fb[4];
                             // Expression: [sigmaPitotS^2, 0; 0, sigmaPitotD^2]
                                //  Referenced by: '<S94>/Constant'

    double AddConstant_Bias_e;         // Expression: -1
                                          //  Referenced by: '<S96>/Add Constant'

    double Gain_Gain_i;                // Expression: 1/2
                                          //  Referenced by: '<S96>/Gain'

    double AddConstant1_Bias_i;        // Expression: +1
                                          //  Referenced by: '<S96>/Add Constant1'

    double AddConstant2_Bias_b;        // Expression: -1
                                          //  Referenced by: '<S96>/Add Constant2'

    double nextLinearState_Y0;         // Computed Parameter: nextLinearState_Y0
                                          //  Referenced by: '<S5>/nextLinearState'

    double nextLinearCov_Y0;           // Computed Parameter: nextLinearCov_Y0
                                          //  Referenced by: '<S5>/nextLinearCov'

    double Constant_Value_px;         // Expression: 1/frequencyPredictionLinear
                                         //  Referenced by: '<S5>/Constant'

    double Constant_Value_o[36];       // Expression: linQ
                                          //  Referenced by: '<S106>/Constant'

    double Constant_Value_g0[36];
    // Expression: [0 0 0 1 0 0; 0 0 0 0 1 0; 0 0 0 0 0 1; 0 0 0 0 0 0; 0 0 0 0 0 0; 0 0 0 0 0 0]
       //  Referenced by: '<S108>/Constant'

    double Gain_Gain_im;               // Expression: 2
                                          //  Referenced by: '<S114>/Gain'

    double Gain_Gain_n;                // Expression: 2
                                          //  Referenced by: '<S117>/Gain'

    double Gain_Gain_no;               // Expression: 2
                                          //  Referenced by: '<S112>/Gain'

    double Gain_Gain_ec;               // Expression: 2
                                          //  Referenced by: '<S118>/Gain'

    double Gain_Gain_ij;               // Expression: 2
                                          //  Referenced by: '<S113>/Gain'

    double Gain_Gain_ih;               // Expression: 2
                                          //  Referenced by: '<S116>/Gain'

    double Constant_Value_dm[3];       // Expression: [0;0; localGravity]
                                          //  Referenced by: '<S109>/Constant'

    double Zero5_Value;                // Expression: 1
                                          //  Referenced by: '<S1>/Zero5'

    double Zero2_Value;                // Expression: 0
                                          //  Referenced by: '<S1>/Zero2'

    double Constant_Value_i;           // Expression: 0
                                          //  Referenced by: '<S44>/Constant'

    double Constant_Value_dl;          // Expression: 0
                                          //  Referenced by: '<S45>/Constant'

    double Constant_Value_b2;          // Expression: 0
                                          //  Referenced by: '<S72>/Constant'

    double Constant_Value_bh;          // Expression: 0
                                          //  Referenced by: '<S73>/Constant'

    double Constant_Value_fs;          // Expression: 0
                                          //  Referenced by: '<S86>/Constant'

    double Constant_Value_j;           // Expression: 0
                                          //  Referenced by: '<S87>/Constant'

    double Constant_Value_ct;          // Expression: 0
                                          //  Referenced by: '<S99>/Constant'

    double Constant_Value_hx;          // Expression: 0
                                          //  Referenced by: '<S100>/Constant'

    double Constant_Value_e;           // Expression: 0
                                          //  Referenced by: '<S125>/Constant'

    double Constant_Value_mg;          // Expression: 0
                                          //  Referenced by: '<S126>/Constant'

    double Constant_Value_pf;          // Expression: 0
                                          //  Referenced by: '<S128>/Constant'

    double Constant_Value_f0;          // Expression: 0
                                          //  Referenced by: '<S129>/Constant'

    double Constant4_Value_c;          // Expression: 1/frequencyMagnetometer
                                          //  Referenced by: '<S10>/Constant4'

    double Constant_Value_bm;          // Expression: 1
                                          //  Referenced by: '<S10>/Constant'

    double Constant1_Value_n;          // Expression: 0
                                          //  Referenced by: '<S10>/Constant1'

    double Constant2_Value_b;          // Expression: 1
                                          //  Referenced by: '<S10>/Constant2'

    double Constant3_Value_p;          // Expression: 0
                                          //  Referenced by: '<S10>/Constant3'

    double Gain_Gain_gl;               // Expression: flagMagnetometer
                                          //  Referenced by: '<S10>/Gain'

    double DataStoreMemory_InitialValue;// Expression: 0
                                           //  Referenced by: '<S10>/Data Store Memory'

    double UnitDelay_InitialCondition; // Expression: 0
                                          //  Referenced by: '<S1>/Unit Delay'

    double UnitDelay1_InitialCondition[36];// Expression: angP0
                                              //  Referenced by: '<S1>/Unit Delay1'

    double Constant_Value_ez;          // Expression: 1
                                          //  Referenced by: '<S6>/Constant'

    double Constant1_Value_o;          // Expression: 0
                                          //  Referenced by: '<S6>/Constant1'

    double Constant2_Value_c;          // Expression: 1
                                          //  Referenced by: '<S6>/Constant2'

    double Constant4_Value_h;       // Expression: 1/frequencyPredictionAttitude
                                       //  Referenced by: '<S6>/Constant4'

    double Constant3_Value_j4;         // Expression: 0
                                          //  Referenced by: '<S6>/Constant3'

    double Merge2_InitialOutput;     // Computed Parameter: Merge2_InitialOutput
                                        //  Referenced by: '<S2>/Merge2'

    double Merge3_InitialOutput;     // Computed Parameter: Merge3_InitialOutput
                                        //  Referenced by: '<S2>/Merge3'

    double Constant4_Value_d;          // Expression: 1/frequencyBarometer
                                          //  Referenced by: '<S64>/Constant4'

    double Constant_Value_es;          // Expression: 1
                                          //  Referenced by: '<S64>/Constant'

    double Constant1_Value_nf;         // Expression: 0
                                          //  Referenced by: '<S64>/Constant1'

    double Constant2_Value_j;          // Expression: 1
                                          //  Referenced by: '<S64>/Constant2'

    double Constant3_Value_i;          // Expression: 0
                                          //  Referenced by: '<S64>/Constant3'

    double Gain_Gain_j;                // Expression: flagBaro
                                          //  Referenced by: '<S64>/Gain'

    double DataStoreMemory_InitialValue_o;// Expression: 0
                                             //  Referenced by: '<S64>/Data Store Memory'

    double UnitDelay2_InitialCondition;// Expression: 0
                                          //  Referenced by: '<S1>/Unit Delay2'

    double UnitDelay5_InitialCondition[36];// Expression: linP0
                                              //  Referenced by: '<S1>/Unit Delay5'

    double Constant_Value_a;           // Expression: 1
                                          //  Referenced by: '<S7>/Constant'

    double Constant1_Value_j;          // Expression: 0
                                          //  Referenced by: '<S7>/Constant1'

    double Constant2_Value_d;          // Expression: 1
                                          //  Referenced by: '<S7>/Constant2'

    double Constant4_Value_n;         // Expression: 1/frequencyPredictionLinear
                                         //  Referenced by: '<S7>/Constant4'

    double Constant3_Value_i5;         // Expression: 0
                                          //  Referenced by: '<S7>/Constant3'

    double Constant_Value_gf;          // Expression: 1
                                          //  Referenced by: '<S76>/Constant'

    double Constant1_Value_or;         // Expression: 0
                                          //  Referenced by: '<S76>/Constant1'

    double Constant2_Value_i0;         // Expression: 1
                                          //  Referenced by: '<S76>/Constant2'

    double Constant4_Value_b;          // Expression: 1/frequencyGPS
                                          //  Referenced by: '<S76>/Constant4'

    double Constant3_Value_d;          // Expression: 0
                                          //  Referenced by: '<S76>/Constant3'

    double Gain_Gain_en;               // Expression: flagGPS
                                          //  Referenced by: '<S76>/Gain'

    double Merge_InitialOutput;       // Computed Parameter: Merge_InitialOutput
                                         //  Referenced by: '<S60>/Merge'

    double Merge1_InitialOutput;     // Computed Parameter: Merge1_InitialOutput
                                        //  Referenced by: '<S60>/Merge1'

    double Merge_InitialOutput_o;   // Computed Parameter: Merge_InitialOutput_o
                                       //  Referenced by: '<S59>/Merge'

    double Merge1_InitialOutput_n; // Computed Parameter: Merge1_InitialOutput_n
                                      //  Referenced by: '<S59>/Merge1'

    double DataStoreMemory_InitialValue_c;// Expression: 0
                                             //  Referenced by: '<S76>/Data Store Memory'

    double Constant4_Value_h1;         // Expression: 1/frequencyPitot
                                          //  Referenced by: '<S90>/Constant4'

    double Constant_Value_ij;          // Expression: 1
                                          //  Referenced by: '<S90>/Constant'

    double Constant1_Value_pt;         // Expression: 0
                                          //  Referenced by: '<S90>/Constant1'

    double Constant2_Value_bb;         // Expression: 1
                                          //  Referenced by: '<S90>/Constant2'

    double Constant3_Value_dl;         // Expression: 0
                                          //  Referenced by: '<S90>/Constant3'

    double Constant5_Value_b;          // Expression: 0
                                          //  Referenced by: '<S90>/Constant5'

    double SeaLevelTemperature_Value;  // Expression: T0
                                          //  Referenced by: '<S105>/Sea Level  Temperature'

    double Gain_Gain_e3;               // Expression: -1
                                          //  Referenced by: '<S101>/Gain'

    double Bias_Bias_j;                // Expression: z0
                                          //  Referenced by: '<S101>/Bias'

    double Limitaltitudetotroposhere_Upper;// Expression: h_trop
                                              //  Referenced by: '<S105>/Limit  altitude  to troposhere'

    double Limitaltitudetotroposhere_Lower;// Expression: h0
                                              //  Referenced by: '<S105>/Limit  altitude  to troposhere'

    double LapseRate_Gain;             // Expression: L
                                          //  Referenced by: '<S105>/Lapse Rate'

    double gammaR_Gain;                // Expression: gamma*R
                                          //  Referenced by: '<S105>/gamma*R'

    double Gain_Gain_mw;               // Expression: flagPitot
                                          //  Referenced by: '<S90>/Gain'

    double DataStoreMemory_InitialValue_h;// Expression: 0
                                             //  Referenced by: '<S90>/Data Store Memory'

    double Merge_InitialOutput_a;   // Computed Parameter: Merge_InitialOutput_a
                                       //  Referenced by: '<S61>/Merge'

    double Merge1_InitialOutput_k; // Computed Parameter: Merge1_InitialOutput_k
                                      //  Referenced by: '<S61>/Merge1'

    double DataStoreMemory_InitialValue_n;// Expression: 0
                                             //  Referenced by: '<S6>/Data Store Memory'

    double DataStoreMemory_InitialValue_f;// Expression: 0
                                             //  Referenced by: '<S7>/Data Store Memory'

    float Switch_Threshold;            // Computed Parameter: Switch_Threshold
                                          //  Referenced by: '<S19>/Switch'

    float Gain1_Gain_ed;               // Computed Parameter: Gain1_Gain_ed
                                          //  Referenced by: '<S53>/Gain1'

    float Constant_Value_dx;           // Computed Parameter: Constant_Value_dx
                                          //  Referenced by: '<S53>/Constant'

    float Gain_Gain_nq;                // Computed Parameter: Gain_Gain_nq
                                          //  Referenced by: '<S47>/Gain'

    float Gain3_Gain_l;                // Computed Parameter: Gain3_Gain_l
                                          //  Referenced by: '<S66>/Gain3'

    float Constant1_Value_a[36];       // Computed Parameter: Constant1_Value_a
                                          //  Referenced by: '<S108>/Constant1'

    float Constant_Value_i5[3];        // Computed Parameter: Constant_Value_i5
                                          //  Referenced by: '<S1>/Constant'

    float RateTransition_InitialCondition;
                          // Computed Parameter: RateTransition_InitialCondition
                             //  Referenced by: '<S10>/Rate Transition'

    float RateTransition1_InitialConditio;
                          // Computed Parameter: RateTransition1_InitialConditio
                             //  Referenced by: '<S64>/Rate Transition1'

    float RateTransition1_InitialCondit_a;
                          // Computed Parameter: RateTransition1_InitialCondit_a
                             //  Referenced by: '<S90>/Rate Transition1'

    float RateTransition_InitialConditi_p;
                          // Computed Parameter: RateTransition_InitialConditi_p
                             //  Referenced by: '<S6>/Rate Transition'

    float RateTransition_InitialConditi_h;
                          // Computed Parameter: RateTransition_InitialConditi_h
                             //  Referenced by: '<S7>/Rate Transition'

    bool Zero1_Value;                  // Computed Parameter: Zero1_Value
                                          //  Referenced by: '<S1>/Zero1'

    uint8_t Zero3_Value;               // Computed Parameter: Zero3_Value
                                          //  Referenced by: '<S1>/Zero3'

    uint8_t Zero4_Value;               // Computed Parameter: Zero4_Value
                                          //  Referenced by: '<S1>/Zero4'

    uint8_t UnitDelay3_InitialCondition;
                              // Computed Parameter: UnitDelay3_InitialCondition
                                 //  Referenced by: '<S1>/Unit Delay3'

    uint8_t UnitDelay4_InitialCondition;
                              // Computed Parameter: UnitDelay4_InitialCondition
                                 //  Referenced by: '<S1>/Unit Delay4'

    uint8_t Bias1_Bias;                // Computed Parameter: Bias1_Bias
                                          //  Referenced by: '<S1>/Bias1'

    uint8_t Bias2_Bias;                // Computed Parameter: Bias2_Bias
                                          //  Referenced by: '<S1>/Bias2'

    uint8_t Saturation1_UpperSat;    // Computed Parameter: Saturation1_UpperSat
                                        //  Referenced by: '<S1>/Saturation1'

    uint8_t Saturation1_LowerSat;    // Computed Parameter: Saturation1_LowerSat
                                        //  Referenced by: '<S1>/Saturation1'

    uint8_t Saturation2_UpperSat;    // Computed Parameter: Saturation2_UpperSat
                                        //  Referenced by: '<S1>/Saturation2'

    uint8_t Saturation2_LowerSat;    // Computed Parameter: Saturation2_LowerSat
                                        //  Referenced by: '<S1>/Saturation2'

  };

  // Real-time Model Data Structure
  struct RT_MODEL_ANAS0_T {
    const char *errorStatus;
    RTWSolverInfo solverInfo;

    //
    //  Timing:
    //  The following substructure contains information regarding
    //  the timing information for the model.

    struct {
      uint32_t clockTick0;
      double stepSize0;
      uint32_t clockTick1;
      struct {
        uint8_t TID[4];
      } TaskCounters;

      SimTimeStep simTimeStep;
      double *t;
      double tArray[4];
    } Timing;

    double** getTPtrPtr();
    const char* getErrorStatus() const;
    void setErrorStatus(const char* const aErrorStatus);
    double* getTPtr() const;
    void setTPtr(double* aTPtr);
    const char** getErrorStatusPtr();
    bool isMajorTimeStep() const;
    bool isMinorTimeStep() const;
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
    ANAS0_U.ANASIn_j = localArgInput;
  }

  // Root inport: '<Root>/ANAS Reference In' set method
  void setANAS_Reference_In(ANASReference localArgInput)
  {
    ANAS0_U.ANASReferenceIn = localArgInput;
  }

  // Root outport: '<Root>/ANAS Out' get method
  ANASOut getANAS_Out() const
  {
    return ANAS0_Y.ANASOut_p;
  }

  // Root outport: '<Root>/NASDAQ Initial State' get method
  ANAS_NASDAQ getNASDAQ_Initial_State() const
  {
    return ANAS0_Y.NASDAQInitialState;
  }

  // Root outport: '<Root>/ANAS OBSW Logs' get method
  ANASLogs getANAS_OBSW_Logs() const
  {
    return ANAS0_Y.ANASOBSWLogs;
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

  // private member function(s) for subsystem '<S59>/Subsystem1'
  static void ANAS0_Subsystem1(bool rtu_Enable, const double
    rtu_nextLinearState[6], const double rtu_nextLinearCov[36], double
    rty_State[6], double rty_Covariance[36]);

  // Real-Time Model
  RT_MODEL_ANAS0_T ANAS0_M;
};

//-
//  These blocks were eliminated from the model due to optimizations:
//
//  Block '<S49>/Matrix Concatenate2' : Unused code path elimination
//  Block '<S53>/Scope' : Unused code path elimination
//  Block '<S77>/Constant' : Unused code path elimination
//  Block '<S77>/Matrix Multiply3' : Unused code path elimination
//  Block '<S77>/Matrix Multiply4' : Unused code path elimination
//  Block '<S77>/Sum5' : Unused code path elimination
//  Block '<S78>/Scope' : Unused code path elimination
//  Block '<S78>/Scope1' : Unused code path elimination
//  Block '<S78>/Scope2' : Unused code path elimination
//  Block '<S78>/Scope3' : Unused code path elimination
//  Block '<S103>/Add' : Unused code path elimination
//  Block '<S103>/Constant' : Unused code path elimination
//  Block '<S103>/Constant1' : Unused code path elimination
//  Block '<S103>/Data Type Conversion' : Unused code path elimination
//  Block '<S103>/Data Type Conversion2' : Unused code path elimination
//  Block '<S103>/Data Type Conversion3' : Unused code path elimination
//  Block '<S103>/Divide' : Unused code path elimination
//  Block '<S103>/Divide1' : Unused code path elimination
//  Block '<S103>/Gain' : Unused code path elimination
//  Block '<S105>/(T//T0)^(g//LR) ' : Unused code path elimination
//  Block '<S105>/1//T0' : Unused code path elimination
//  Block '<S105>/Altitude of Troposphere' : Unused code path elimination
//  Block '<S105>/Constant' : Unused code path elimination
//  Block '<S105>/Limit  altitude  to Stratosphere' : Unused code path elimination
//  Block '<S105>/P0' : Unused code path elimination
//  Block '<S105>/Product' : Unused code path elimination
//  Block '<S105>/Product1' : Unused code path elimination
//  Block '<S105>/Product2' : Unused code path elimination
//  Block '<S105>/Product3' : Unused code path elimination
//  Block '<S105>/Stratosphere Model' : Unused code path elimination
//  Block '<S105>/Sum' : Unused code path elimination
//  Block '<S105>/g//R' : Unused code path elimination
//  Block '<S105>/rho0' : Unused code path elimination
//  Block '<S103>/Power' : Unused code path elimination
//  Block '<S103>/dynVisc Conversion' : Unused code path elimination
//  Block '<S103>/kineVisc Conversion' : Unused code path elimination
//  Block '<S12>/Reshape' : Reshape block reduction
//  Block '<S27>/Reshape (9) to [3x3] column-major' : Reshape block reduction
//  Block '<S39>/Reshape (9) to [3x3] column-major' : Reshape block reduction
//  Block '<S52>/Reshape (9) to [3x3] column-major' : Reshape block reduction
//  Block '<S53>/Reshape' : Reshape block reduction
//  Block '<S56>/Reshape (9) to [3x3] column-major' : Reshape block reduction
//  Block '<S66>/Reshape' : Reshape block reduction
//  Block '<S80>/Reshape' : Reshape block reduction
//  Block '<S80>/Reshape1' : Reshape block reduction
//  Block '<S92>/Reshape' : Reshape block reduction
//  Block '<S92>/Reshape1' : Reshape block reduction
//  Block '<S97>/Reshape' : Reshape block reduction
//  Block '<S103>/Cast To Double' : Eliminate redundant data type conversion
//  Block '<S103>/Data Type Conversion1' : Eliminate redundant data type conversion
//  Block '<S120>/Reshape (9) to [3x3] column-major' : Reshape block reduction


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
//  hilite_system('CHADsimulator/Control Units/Control Units SIM/Main Control Unit (Sim)/Ascent Navigation/Ascent NAS/ANAS - Autocoding')    - opens subsystem CHADsimulator/Control Units/Control Units SIM/Main Control Unit (Sim)/Ascent Navigation/Ascent NAS/ANAS - Autocoding
//  hilite_system('CHADsimulator/Control Units/Control Units SIM/Main Control Unit (Sim)/Ascent Navigation/Ascent NAS/ANAS - Autocoding/Kp') - opens and selects block Kp
//
//  Here is the system hierarchy for this model
//
//  '<Root>' : 'CHADsimulator/Control Units/Control Units SIM/Main Control Unit (Sim)/Ascent Navigation/Ascent NAS'
//  '<S1>'   : 'CHADsimulator/Control Units/Control Units SIM/Main Control Unit (Sim)/Ascent Navigation/Ascent NAS/ANAS - Autocoding'
//  '<S2>'   : 'CHADsimulator/Control Units/Control Units SIM/Main Control Unit (Sim)/Ascent Navigation/Ascent NAS/ANAS - Autocoding/Attitude corrector'
//  '<S3>'   : 'CHADsimulator/Control Units/Control Units SIM/Main Control Unit (Sim)/Ascent Navigation/Ascent NAS/ANAS - Autocoding/Attitude predictor'
//  '<S4>'   : 'CHADsimulator/Control Units/Control Units SIM/Main Control Unit (Sim)/Ascent Navigation/Ascent NAS/ANAS - Autocoding/Linear corrector'
//  '<S5>'   : 'CHADsimulator/Control Units/Control Units SIM/Main Control Unit (Sim)/Ascent Navigation/Ascent NAS/ANAS - Autocoding/Linear predictor'
//  '<S6>'   : 'CHADsimulator/Control Units/Control Units SIM/Main Control Unit (Sim)/Ascent Navigation/Ascent NAS/ANAS - Autocoding/Sampling Check Angular'
//  '<S7>'   : 'CHADsimulator/Control Units/Control Units SIM/Main Control Unit (Sim)/Ascent Navigation/Ascent NAS/ANAS - Autocoding/Sampling Check Linear'
//  '<S8>'   : 'CHADsimulator/Control Units/Control Units SIM/Main Control Unit (Sim)/Ascent Navigation/Ascent NAS/ANAS - Autocoding/Attitude corrector/Attitude corrector'
//  '<S9>'   : 'CHADsimulator/Control Units/Control Units SIM/Main Control Unit (Sim)/Ascent Navigation/Ascent NAS/ANAS - Autocoding/Attitude corrector/Subsystem1'
//  '<S10>'  : 'CHADsimulator/Control Units/Control Units SIM/Main Control Unit (Sim)/Ascent Navigation/Ascent NAS/ANAS - Autocoding/Attitude corrector/Subsystem5'
//  '<S11>'  : 'CHADsimulator/Control Units/Control Units SIM/Main Control Unit (Sim)/Ascent Navigation/Ascent NAS/ANAS - Autocoding/Attitude corrector/Attitude corrector/Covariance'
//  '<S12>'  : 'CHADsimulator/Control Units/Control Units SIM/Main Control Unit (Sim)/Ascent Navigation/Ascent NAS/ANAS - Autocoding/Attitude corrector/Attitude corrector/State'
//  '<S13>'  : 'CHADsimulator/Control Units/Control Units SIM/Main Control Unit (Sim)/Ascent Navigation/Ascent NAS/ANAS - Autocoding/Attitude corrector/Attitude corrector/Covariance/F'
//  '<S14>'  : 'CHADsimulator/Control Units/Control Units SIM/Main Control Unit (Sim)/Ascent Navigation/Ascent NAS/ANAS - Autocoding/Attitude corrector/Attitude corrector/State/Hamiltonian product'
//  '<S15>'  : 'CHADsimulator/Control Units/Control Units SIM/Main Control Unit (Sim)/Ascent Navigation/Ascent NAS/ANAS - Autocoding/Attitude corrector/Attitude corrector/State/Kalman gain'
//  '<S16>'  : 'CHADsimulator/Control Units/Control Units SIM/Main Control Unit (Sim)/Ascent Navigation/Ascent NAS/ANAS - Autocoding/Attitude corrector/Attitude corrector/State/Quaternion Normalize'
//  '<S17>'  : 'CHADsimulator/Control Units/Control Units SIM/Main Control Unit (Sim)/Ascent Navigation/Ascent NAS/ANAS - Autocoding/Attitude corrector/Attitude corrector/State/Skew'
//  '<S18>'  : 'CHADsimulator/Control Units/Control Units SIM/Main Control Unit (Sim)/Ascent Navigation/Ascent NAS/ANAS - Autocoding/Attitude corrector/Attitude corrector/State/Subsystem'
//  '<S19>'  : 'CHADsimulator/Control Units/Control Units SIM/Main Control Unit (Sim)/Ascent Navigation/Ascent NAS/ANAS - Autocoding/Attitude corrector/Attitude corrector/State/Subsystem2'
//  '<S20>'  : 'CHADsimulator/Control Units/Control Units SIM/Main Control Unit (Sim)/Ascent Navigation/Ascent NAS/ANAS - Autocoding/Attitude corrector/Attitude corrector/State/quaternion to DCM'
//  '<S21>'  : 'CHADsimulator/Control Units/Control Units SIM/Main Control Unit (Sim)/Ascent Navigation/Ascent NAS/ANAS - Autocoding/Attitude corrector/Attitude corrector/State/Hamiltonian product/Cross Product'
//  '<S22>'  : 'CHADsimulator/Control Units/Control Units SIM/Main Control Unit (Sim)/Ascent Navigation/Ascent NAS/ANAS - Autocoding/Attitude corrector/Attitude corrector/State/Hamiltonian product/Quaternion Normalize'
//  '<S23>'  : 'CHADsimulator/Control Units/Control Units SIM/Main Control Unit (Sim)/Ascent Navigation/Ascent NAS/ANAS - Autocoding/Attitude corrector/Attitude corrector/State/Hamiltonian product/Quaternion Normalize/Quaternion Modulus'
//  '<S24>'  : 'CHADsimulator/Control Units/Control Units SIM/Main Control Unit (Sim)/Ascent Navigation/Ascent NAS/ANAS - Autocoding/Attitude corrector/Attitude corrector/State/Hamiltonian product/Quaternion Normalize/Quaternion Modulus/Quaternion Norm'
//  '<S25>'  : 'CHADsimulator/Control Units/Control Units SIM/Main Control Unit (Sim)/Ascent Navigation/Ascent NAS/ANAS - Autocoding/Attitude corrector/Attitude corrector/State/Quaternion Normalize/Quaternion Modulus'
//  '<S26>'  : 'CHADsimulator/Control Units/Control Units SIM/Main Control Unit (Sim)/Ascent Navigation/Ascent NAS/ANAS - Autocoding/Attitude corrector/Attitude corrector/State/Quaternion Normalize/Quaternion Modulus/Quaternion Norm'
//  '<S27>'  : 'CHADsimulator/Control Units/Control Units SIM/Main Control Unit (Sim)/Ascent Navigation/Ascent NAS/ANAS - Autocoding/Attitude corrector/Attitude corrector/State/Skew/Create 3x3 Matrix1'
//  '<S28>'  : 'CHADsimulator/Control Units/Control Units SIM/Main Control Unit (Sim)/Ascent Navigation/Ascent NAS/ANAS - Autocoding/Attitude corrector/Attitude corrector/State/Subsystem2/Norm'
//  '<S29>'  : 'CHADsimulator/Control Units/Control Units SIM/Main Control Unit (Sim)/Ascent Navigation/Ascent NAS/ANAS - Autocoding/Attitude corrector/Attitude corrector/State/quaternion to DCM/Quaternions to  Direction Cosine Matrix'
//  '<S30>'  : 'CHADsimulator/Control Units/Control Units SIM/Main Control Unit (Sim)/Ascent Navigation/Ascent NAS/ANAS - Autocoding/Attitude corrector/Attitude corrector/State/quaternion to DCM/Quaternions to  Direction Cosine Matrix/A11'
//  '<S31>'  : 'CHADsimulator/Control Units/Control Units SIM/Main Control Unit (Sim)/Ascent Navigation/Ascent NAS/ANAS - Autocoding/Attitude corrector/Attitude corrector/State/quaternion to DCM/Quaternions to  Direction Cosine Matrix/A12'
//  '<S32>'  : 'CHADsimulator/Control Units/Control Units SIM/Main Control Unit (Sim)/Ascent Navigation/Ascent NAS/ANAS - Autocoding/Attitude corrector/Attitude corrector/State/quaternion to DCM/Quaternions to  Direction Cosine Matrix/A13'
//  '<S33>'  : 'CHADsimulator/Control Units/Control Units SIM/Main Control Unit (Sim)/Ascent Navigation/Ascent NAS/ANAS - Autocoding/Attitude corrector/Attitude corrector/State/quaternion to DCM/Quaternions to  Direction Cosine Matrix/A21'
//  '<S34>'  : 'CHADsimulator/Control Units/Control Units SIM/Main Control Unit (Sim)/Ascent Navigation/Ascent NAS/ANAS - Autocoding/Attitude corrector/Attitude corrector/State/quaternion to DCM/Quaternions to  Direction Cosine Matrix/A22'
//  '<S35>'  : 'CHADsimulator/Control Units/Control Units SIM/Main Control Unit (Sim)/Ascent Navigation/Ascent NAS/ANAS - Autocoding/Attitude corrector/Attitude corrector/State/quaternion to DCM/Quaternions to  Direction Cosine Matrix/A23'
//  '<S36>'  : 'CHADsimulator/Control Units/Control Units SIM/Main Control Unit (Sim)/Ascent Navigation/Ascent NAS/ANAS - Autocoding/Attitude corrector/Attitude corrector/State/quaternion to DCM/Quaternions to  Direction Cosine Matrix/A31'
//  '<S37>'  : 'CHADsimulator/Control Units/Control Units SIM/Main Control Unit (Sim)/Ascent Navigation/Ascent NAS/ANAS - Autocoding/Attitude corrector/Attitude corrector/State/quaternion to DCM/Quaternions to  Direction Cosine Matrix/A32'
//  '<S38>'  : 'CHADsimulator/Control Units/Control Units SIM/Main Control Unit (Sim)/Ascent Navigation/Ascent NAS/ANAS - Autocoding/Attitude corrector/Attitude corrector/State/quaternion to DCM/Quaternions to  Direction Cosine Matrix/A33'
//  '<S39>'  : 'CHADsimulator/Control Units/Control Units SIM/Main Control Unit (Sim)/Ascent Navigation/Ascent NAS/ANAS - Autocoding/Attitude corrector/Attitude corrector/State/quaternion to DCM/Quaternions to  Direction Cosine Matrix/Create 3x3 Matrix'
//  '<S40>'  : 'CHADsimulator/Control Units/Control Units SIM/Main Control Unit (Sim)/Ascent Navigation/Ascent NAS/ANAS - Autocoding/Attitude corrector/Attitude corrector/State/quaternion to DCM/Quaternions to  Direction Cosine Matrix/Quaternion Normalize'
//  '<S41>'  : 'CHADsimulator/Control Units/Control Units SIM/Main Control Unit (Sim)/Ascent Navigation/Ascent NAS/ANAS - Autocoding/Attitude corrector/Attitude corrector/State/quaternion to DCM/Quaternions to  Direction Cosine Matrix/Quaternion Normalize/Quaternion Modulus'
//  '<S42>'  : 'CHADsimulator/Control Units/Control Units SIM/Main Control Unit (Sim)/Ascent Navigation/Ascent NAS/ANAS - Autocoding/Attitude corrector/Attitude corrector/State/quaternion to DCM/Quaternions to  Direction Cosine Matrix/Quaternion Normalize/Quaternion Modulus/Quaternion Norm'
//  '<S43>'  : 'CHADsimulator/Control Units/Control Units SIM/Main Control Unit (Sim)/Ascent Navigation/Ascent NAS/ANAS - Autocoding/Attitude corrector/Subsystem5/Compare To Constant1'
//  '<S44>'  : 'CHADsimulator/Control Units/Control Units SIM/Main Control Unit (Sim)/Ascent Navigation/Ascent NAS/ANAS - Autocoding/Attitude corrector/Subsystem5/Compare To Zero1'
//  '<S45>'  : 'CHADsimulator/Control Units/Control Units SIM/Main Control Unit (Sim)/Ascent Navigation/Ascent NAS/ANAS - Autocoding/Attitude corrector/Subsystem5/Compare To Zero2'
//  '<S46>'  : 'CHADsimulator/Control Units/Control Units SIM/Main Control Unit (Sim)/Ascent Navigation/Ascent NAS/ANAS - Autocoding/Attitude predictor/Covariance'
//  '<S47>'  : 'CHADsimulator/Control Units/Control Units SIM/Main Control Unit (Sim)/Ascent Navigation/Ascent NAS/ANAS - Autocoding/Attitude predictor/State'
//  '<S48>'  : 'CHADsimulator/Control Units/Control Units SIM/Main Control Unit (Sim)/Ascent Navigation/Ascent NAS/ANAS - Autocoding/Attitude predictor/Covariance/F'
//  '<S49>'  : 'CHADsimulator/Control Units/Control Units SIM/Main Control Unit (Sim)/Ascent Navigation/Ascent NAS/ANAS - Autocoding/Attitude predictor/Covariance/F1'
//  '<S50>'  : 'CHADsimulator/Control Units/Control Units SIM/Main Control Unit (Sim)/Ascent Navigation/Ascent NAS/ANAS - Autocoding/Attitude predictor/Covariance/G'
//  '<S51>'  : 'CHADsimulator/Control Units/Control Units SIM/Main Control Unit (Sim)/Ascent Navigation/Ascent NAS/ANAS - Autocoding/Attitude predictor/Covariance/F1/Skew'
//  '<S52>'  : 'CHADsimulator/Control Units/Control Units SIM/Main Control Unit (Sim)/Ascent Navigation/Ascent NAS/ANAS - Autocoding/Attitude predictor/Covariance/F1/Skew/Create 3x3 Matrix'
//  '<S53>'  : 'CHADsimulator/Control Units/Control Units SIM/Main Control Unit (Sim)/Ascent Navigation/Ascent NAS/ANAS - Autocoding/Attitude predictor/State/Omega'
//  '<S54>'  : 'CHADsimulator/Control Units/Control Units SIM/Main Control Unit (Sim)/Ascent Navigation/Ascent NAS/ANAS - Autocoding/Attitude predictor/State/Quaternion Normalize'
//  '<S55>'  : 'CHADsimulator/Control Units/Control Units SIM/Main Control Unit (Sim)/Ascent Navigation/Ascent NAS/ANAS - Autocoding/Attitude predictor/State/Omega/Skew'
//  '<S56>'  : 'CHADsimulator/Control Units/Control Units SIM/Main Control Unit (Sim)/Ascent Navigation/Ascent NAS/ANAS - Autocoding/Attitude predictor/State/Omega/Skew/Create 3x3 Matrix'
//  '<S57>'  : 'CHADsimulator/Control Units/Control Units SIM/Main Control Unit (Sim)/Ascent Navigation/Ascent NAS/ANAS - Autocoding/Attitude predictor/State/Quaternion Normalize/Quaternion Modulus'
//  '<S58>'  : 'CHADsimulator/Control Units/Control Units SIM/Main Control Unit (Sim)/Ascent Navigation/Ascent NAS/ANAS - Autocoding/Attitude predictor/State/Quaternion Normalize/Quaternion Modulus/Quaternion Norm'
//  '<S59>'  : 'CHADsimulator/Control Units/Control Units SIM/Main Control Unit (Sim)/Ascent Navigation/Ascent NAS/ANAS - Autocoding/Linear corrector/Baro Correction'
//  '<S60>'  : 'CHADsimulator/Control Units/Control Units SIM/Main Control Unit (Sim)/Ascent Navigation/Ascent NAS/ANAS - Autocoding/Linear corrector/GPS Correction'
//  '<S61>'  : 'CHADsimulator/Control Units/Control Units SIM/Main Control Unit (Sim)/Ascent Navigation/Ascent NAS/ANAS - Autocoding/Linear corrector/Pitot Correction'
//  '<S62>'  : 'CHADsimulator/Control Units/Control Units SIM/Main Control Unit (Sim)/Ascent Navigation/Ascent NAS/ANAS - Autocoding/Linear corrector/Baro Correction/State and Covariance'
//  '<S63>'  : 'CHADsimulator/Control Units/Control Units SIM/Main Control Unit (Sim)/Ascent Navigation/Ascent NAS/ANAS - Autocoding/Linear corrector/Baro Correction/Subsystem1'
//  '<S64>'  : 'CHADsimulator/Control Units/Control Units SIM/Main Control Unit (Sim)/Ascent Navigation/Ascent NAS/ANAS - Autocoding/Linear corrector/Baro Correction/Subsystem4'
//  '<S65>'  : 'CHADsimulator/Control Units/Control Units SIM/Main Control Unit (Sim)/Ascent Navigation/Ascent NAS/ANAS - Autocoding/Linear corrector/Baro Correction/State and Covariance/Covariance w// Joseph formula'
//  '<S66>'  : 'CHADsimulator/Control Units/Control Units SIM/Main Control Unit (Sim)/Ascent Navigation/Ascent NAS/ANAS - Autocoding/Linear corrector/Baro Correction/State and Covariance/H Matrix'
//  '<S67>'  : 'CHADsimulator/Control Units/Control Units SIM/Main Control Unit (Sim)/Ascent Navigation/Ascent NAS/ANAS - Autocoding/Linear corrector/Baro Correction/State and Covariance/Kalman Gain'
//  '<S68>'  : 'CHADsimulator/Control Units/Control Units SIM/Main Control Unit (Sim)/Ascent Navigation/Ascent NAS/ANAS - Autocoding/Linear corrector/Baro Correction/State and Covariance/R Matrix'
//  '<S69>'  : 'CHADsimulator/Control Units/Control Units SIM/Main Control Unit (Sim)/Ascent Navigation/Ascent NAS/ANAS - Autocoding/Linear corrector/Baro Correction/State and Covariance/Subsystem'
//  '<S70>'  : 'CHADsimulator/Control Units/Control Units SIM/Main Control Unit (Sim)/Ascent Navigation/Ascent NAS/ANAS - Autocoding/Linear corrector/Baro Correction/State and Covariance/Covariance w// Joseph formula/F'
//  '<S71>'  : 'CHADsimulator/Control Units/Control Units SIM/Main Control Unit (Sim)/Ascent Navigation/Ascent NAS/ANAS - Autocoding/Linear corrector/Baro Correction/Subsystem4/Compare To Constant1'
//  '<S72>'  : 'CHADsimulator/Control Units/Control Units SIM/Main Control Unit (Sim)/Ascent Navigation/Ascent NAS/ANAS - Autocoding/Linear corrector/Baro Correction/Subsystem4/Compare To Zero1'
//  '<S73>'  : 'CHADsimulator/Control Units/Control Units SIM/Main Control Unit (Sim)/Ascent Navigation/Ascent NAS/ANAS - Autocoding/Linear corrector/Baro Correction/Subsystem4/Compare To Zero2'
//  '<S74>'  : 'CHADsimulator/Control Units/Control Units SIM/Main Control Unit (Sim)/Ascent Navigation/Ascent NAS/ANAS - Autocoding/Linear corrector/GPS Correction/State and Covariance'
//  '<S75>'  : 'CHADsimulator/Control Units/Control Units SIM/Main Control Unit (Sim)/Ascent Navigation/Ascent NAS/ANAS - Autocoding/Linear corrector/GPS Correction/Subsystem1'
//  '<S76>'  : 'CHADsimulator/Control Units/Control Units SIM/Main Control Unit (Sim)/Ascent Navigation/Ascent NAS/ANAS - Autocoding/Linear corrector/GPS Correction/Subsystem4'
//  '<S77>'  : 'CHADsimulator/Control Units/Control Units SIM/Main Control Unit (Sim)/Ascent Navigation/Ascent NAS/ANAS - Autocoding/Linear corrector/GPS Correction/State and Covariance/Covariance'
//  '<S78>'  : 'CHADsimulator/Control Units/Control Units SIM/Main Control Unit (Sim)/Ascent Navigation/Ascent NAS/ANAS - Autocoding/Linear corrector/GPS Correction/State and Covariance/Covariance w// Joseph formula'
//  '<S79>'  : 'CHADsimulator/Control Units/Control Units SIM/Main Control Unit (Sim)/Ascent Navigation/Ascent NAS/ANAS - Autocoding/Linear corrector/GPS Correction/State and Covariance/Degrees to Radians'
//  '<S80>'  : 'CHADsimulator/Control Units/Control Units SIM/Main Control Unit (Sim)/Ascent Navigation/Ascent NAS/ANAS - Autocoding/Linear corrector/GPS Correction/State and Covariance/H Matrix'
//  '<S81>'  : 'CHADsimulator/Control Units/Control Units SIM/Main Control Unit (Sim)/Ascent Navigation/Ascent NAS/ANAS - Autocoding/Linear corrector/GPS Correction/State and Covariance/Kalman Gain'
//  '<S82>'  : 'CHADsimulator/Control Units/Control Units SIM/Main Control Unit (Sim)/Ascent Navigation/Ascent NAS/ANAS - Autocoding/Linear corrector/GPS Correction/State and Covariance/R Matrix'
//  '<S83>'  : 'CHADsimulator/Control Units/Control Units SIM/Main Control Unit (Sim)/Ascent Navigation/Ascent NAS/ANAS - Autocoding/Linear corrector/GPS Correction/State and Covariance/Covariance w// Joseph formula/F'
//  '<S84>'  : 'CHADsimulator/Control Units/Control Units SIM/Main Control Unit (Sim)/Ascent Navigation/Ascent NAS/ANAS - Autocoding/Linear corrector/GPS Correction/State and Covariance/H Matrix/Degrees to Radians'
//  '<S85>'  : 'CHADsimulator/Control Units/Control Units SIM/Main Control Unit (Sim)/Ascent Navigation/Ascent NAS/ANAS - Autocoding/Linear corrector/GPS Correction/Subsystem4/Compare To Constant1'
//  '<S86>'  : 'CHADsimulator/Control Units/Control Units SIM/Main Control Unit (Sim)/Ascent Navigation/Ascent NAS/ANAS - Autocoding/Linear corrector/GPS Correction/Subsystem4/Compare To Zero1'
//  '<S87>'  : 'CHADsimulator/Control Units/Control Units SIM/Main Control Unit (Sim)/Ascent Navigation/Ascent NAS/ANAS - Autocoding/Linear corrector/GPS Correction/Subsystem4/Compare To Zero2'
//  '<S88>'  : 'CHADsimulator/Control Units/Control Units SIM/Main Control Unit (Sim)/Ascent Navigation/Ascent NAS/ANAS - Autocoding/Linear corrector/Pitot Correction/State and Covariance'
//  '<S89>'  : 'CHADsimulator/Control Units/Control Units SIM/Main Control Unit (Sim)/Ascent Navigation/Ascent NAS/ANAS - Autocoding/Linear corrector/Pitot Correction/Subsystem1'
//  '<S90>'  : 'CHADsimulator/Control Units/Control Units SIM/Main Control Unit (Sim)/Ascent Navigation/Ascent NAS/ANAS - Autocoding/Linear corrector/Pitot Correction/Subsystem4'
//  '<S91>'  : 'CHADsimulator/Control Units/Control Units SIM/Main Control Unit (Sim)/Ascent Navigation/Ascent NAS/ANAS - Autocoding/Linear corrector/Pitot Correction/State and Covariance/Covariance w// Joseph formula'
//  '<S92>'  : 'CHADsimulator/Control Units/Control Units SIM/Main Control Unit (Sim)/Ascent Navigation/Ascent NAS/ANAS - Autocoding/Linear corrector/Pitot Correction/State and Covariance/H Matrix'
//  '<S93>'  : 'CHADsimulator/Control Units/Control Units SIM/Main Control Unit (Sim)/Ascent Navigation/Ascent NAS/ANAS - Autocoding/Linear corrector/Pitot Correction/State and Covariance/Kalman Gain'
//  '<S94>'  : 'CHADsimulator/Control Units/Control Units SIM/Main Control Unit (Sim)/Ascent Navigation/Ascent NAS/ANAS - Autocoding/Linear corrector/Pitot Correction/State and Covariance/R Matrix'
//  '<S95>'  : 'CHADsimulator/Control Units/Control Units SIM/Main Control Unit (Sim)/Ascent Navigation/Ascent NAS/ANAS - Autocoding/Linear corrector/Pitot Correction/State and Covariance/Covariance w// Joseph formula/F'
//  '<S96>'  : 'CHADsimulator/Control Units/Control Units SIM/Main Control Unit (Sim)/Ascent Navigation/Ascent NAS/ANAS - Autocoding/Linear corrector/Pitot Correction/State and Covariance/H Matrix/Subsystem'
//  '<S97>'  : 'CHADsimulator/Control Units/Control Units SIM/Main Control Unit (Sim)/Ascent Navigation/Ascent NAS/ANAS - Autocoding/Linear corrector/Pitot Correction/State and Covariance/H Matrix/quat2rot'
//  '<S98>'  : 'CHADsimulator/Control Units/Control Units SIM/Main Control Unit (Sim)/Ascent Navigation/Ascent NAS/ANAS - Autocoding/Linear corrector/Pitot Correction/Subsystem4/Compare To Constant1'
//  '<S99>'  : 'CHADsimulator/Control Units/Control Units SIM/Main Control Unit (Sim)/Ascent Navigation/Ascent NAS/ANAS - Autocoding/Linear corrector/Pitot Correction/Subsystem4/Compare To Zero1'
//  '<S100>' : 'CHADsimulator/Control Units/Control Units SIM/Main Control Unit (Sim)/Ascent Navigation/Ascent NAS/ANAS - Autocoding/Linear corrector/Pitot Correction/Subsystem4/Compare To Zero2'
//  '<S101>' : 'CHADsimulator/Control Units/Control Units SIM/Main Control Unit (Sim)/Ascent Navigation/Ascent NAS/ANAS - Autocoding/Linear corrector/Pitot Correction/Subsystem4/Min Mach Number Check'
//  '<S102>' : 'CHADsimulator/Control Units/Control Units SIM/Main Control Unit (Sim)/Ascent Navigation/Ascent NAS/ANAS - Autocoding/Linear corrector/Pitot Correction/Subsystem4/Min Mach Number Check/Compare To Constant'
//  '<S103>' : 'CHADsimulator/Control Units/Control Units SIM/Main Control Unit (Sim)/Ascent Navigation/Ascent NAS/ANAS - Autocoding/Linear corrector/Pitot Correction/Subsystem4/Min Mach Number Check/ISA Atmosphere Model'
//  '<S104>' : 'CHADsimulator/Control Units/Control Units SIM/Main Control Unit (Sim)/Ascent Navigation/Ascent NAS/ANAS - Autocoding/Linear corrector/Pitot Correction/Subsystem4/Min Mach Number Check/Subsystem Reference'
//  '<S105>' : 'CHADsimulator/Control Units/Control Units SIM/Main Control Unit (Sim)/Ascent Navigation/Ascent NAS/ANAS - Autocoding/Linear corrector/Pitot Correction/Subsystem4/Min Mach Number Check/ISA Atmosphere Model/Modelling'
//  '<S106>' : 'CHADsimulator/Control Units/Control Units SIM/Main Control Unit (Sim)/Ascent Navigation/Ascent NAS/ANAS - Autocoding/Linear predictor/Covariance'
//  '<S107>' : 'CHADsimulator/Control Units/Control Units SIM/Main Control Unit (Sim)/Ascent Navigation/Ascent NAS/ANAS - Autocoding/Linear predictor/State'
//  '<S108>' : 'CHADsimulator/Control Units/Control Units SIM/Main Control Unit (Sim)/Ascent Navigation/Ascent NAS/ANAS - Autocoding/Linear predictor/Covariance/F'
//  '<S109>' : 'CHADsimulator/Control Units/Control Units SIM/Main Control Unit (Sim)/Ascent Navigation/Ascent NAS/ANAS - Autocoding/Linear predictor/State/Subsystem'
//  '<S110>' : 'CHADsimulator/Control Units/Control Units SIM/Main Control Unit (Sim)/Ascent Navigation/Ascent NAS/ANAS - Autocoding/Linear predictor/State/Subsystem/Quaternions to  Direction Cosine Matrix'
//  '<S111>' : 'CHADsimulator/Control Units/Control Units SIM/Main Control Unit (Sim)/Ascent Navigation/Ascent NAS/ANAS - Autocoding/Linear predictor/State/Subsystem/Quaternions to  Direction Cosine Matrix/A11'
//  '<S112>' : 'CHADsimulator/Control Units/Control Units SIM/Main Control Unit (Sim)/Ascent Navigation/Ascent NAS/ANAS - Autocoding/Linear predictor/State/Subsystem/Quaternions to  Direction Cosine Matrix/A12'
//  '<S113>' : 'CHADsimulator/Control Units/Control Units SIM/Main Control Unit (Sim)/Ascent Navigation/Ascent NAS/ANAS - Autocoding/Linear predictor/State/Subsystem/Quaternions to  Direction Cosine Matrix/A13'
//  '<S114>' : 'CHADsimulator/Control Units/Control Units SIM/Main Control Unit (Sim)/Ascent Navigation/Ascent NAS/ANAS - Autocoding/Linear predictor/State/Subsystem/Quaternions to  Direction Cosine Matrix/A21'
//  '<S115>' : 'CHADsimulator/Control Units/Control Units SIM/Main Control Unit (Sim)/Ascent Navigation/Ascent NAS/ANAS - Autocoding/Linear predictor/State/Subsystem/Quaternions to  Direction Cosine Matrix/A22'
//  '<S116>' : 'CHADsimulator/Control Units/Control Units SIM/Main Control Unit (Sim)/Ascent Navigation/Ascent NAS/ANAS - Autocoding/Linear predictor/State/Subsystem/Quaternions to  Direction Cosine Matrix/A23'
//  '<S117>' : 'CHADsimulator/Control Units/Control Units SIM/Main Control Unit (Sim)/Ascent Navigation/Ascent NAS/ANAS - Autocoding/Linear predictor/State/Subsystem/Quaternions to  Direction Cosine Matrix/A31'
//  '<S118>' : 'CHADsimulator/Control Units/Control Units SIM/Main Control Unit (Sim)/Ascent Navigation/Ascent NAS/ANAS - Autocoding/Linear predictor/State/Subsystem/Quaternions to  Direction Cosine Matrix/A32'
//  '<S119>' : 'CHADsimulator/Control Units/Control Units SIM/Main Control Unit (Sim)/Ascent Navigation/Ascent NAS/ANAS - Autocoding/Linear predictor/State/Subsystem/Quaternions to  Direction Cosine Matrix/A33'
//  '<S120>' : 'CHADsimulator/Control Units/Control Units SIM/Main Control Unit (Sim)/Ascent Navigation/Ascent NAS/ANAS - Autocoding/Linear predictor/State/Subsystem/Quaternions to  Direction Cosine Matrix/Create 3x3 Matrix'
//  '<S121>' : 'CHADsimulator/Control Units/Control Units SIM/Main Control Unit (Sim)/Ascent Navigation/Ascent NAS/ANAS - Autocoding/Linear predictor/State/Subsystem/Quaternions to  Direction Cosine Matrix/Quaternion Normalize'
//  '<S122>' : 'CHADsimulator/Control Units/Control Units SIM/Main Control Unit (Sim)/Ascent Navigation/Ascent NAS/ANAS - Autocoding/Linear predictor/State/Subsystem/Quaternions to  Direction Cosine Matrix/Quaternion Normalize/Quaternion Modulus'
//  '<S123>' : 'CHADsimulator/Control Units/Control Units SIM/Main Control Unit (Sim)/Ascent Navigation/Ascent NAS/ANAS - Autocoding/Linear predictor/State/Subsystem/Quaternions to  Direction Cosine Matrix/Quaternion Normalize/Quaternion Modulus/Quaternion Norm'
//  '<S124>' : 'CHADsimulator/Control Units/Control Units SIM/Main Control Unit (Sim)/Ascent Navigation/Ascent NAS/ANAS - Autocoding/Sampling Check Angular/Compare To Constant1'
//  '<S125>' : 'CHADsimulator/Control Units/Control Units SIM/Main Control Unit (Sim)/Ascent Navigation/Ascent NAS/ANAS - Autocoding/Sampling Check Angular/Compare To Zero1'
//  '<S126>' : 'CHADsimulator/Control Units/Control Units SIM/Main Control Unit (Sim)/Ascent Navigation/Ascent NAS/ANAS - Autocoding/Sampling Check Angular/Compare To Zero2'
//  '<S127>' : 'CHADsimulator/Control Units/Control Units SIM/Main Control Unit (Sim)/Ascent Navigation/Ascent NAS/ANAS - Autocoding/Sampling Check Linear/Compare To Constant1'
//  '<S128>' : 'CHADsimulator/Control Units/Control Units SIM/Main Control Unit (Sim)/Ascent Navigation/Ascent NAS/ANAS - Autocoding/Sampling Check Linear/Compare To Zero1'
//  '<S129>' : 'CHADsimulator/Control Units/Control Units SIM/Main Control Unit (Sim)/Ascent Navigation/Ascent NAS/ANAS - Autocoding/Sampling Check Linear/Compare To Zero2'


//-
//  Requirements for '<Root>': ANAS0


#endif                                 // ANAS0_h_

//
// File trailer for generated code.
//
// [EOF]
//
