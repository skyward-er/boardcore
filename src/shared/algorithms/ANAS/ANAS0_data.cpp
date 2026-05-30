//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// File: ANAS0_data.cpp
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
#include "ANAS0.h"

// Block parameters (default storage)
ANAS0::P_ANAS0_T ANAS0::ANAS0_P{
  // Mask Parameter: CompareToConstant1_const
  //  Referenced by: '<S43>/Constant'

  1.0E+6,

  // Mask Parameter: CompareToConstant1_const_e
  //  Referenced by: '<S124>/Constant'

  0.02,

  // Mask Parameter: CompareToConstant1_const_n
  //  Referenced by: '<S71>/Constant'

  0.02,

  // Mask Parameter: CompareToConstant1_const_k
  //  Referenced by: '<S127>/Constant'

  0.02,

  // Mask Parameter: CompareToConstant1_const_p
  //  Referenced by: '<S85>/Constant'

  0.1,

  // Mask Parameter: CompareToConstant1_const_ky
  //  Referenced by: '<S98>/Constant'

  0.04,

  // Mask Parameter: CompareToConstant_const
  //  Referenced by: '<S102>/Constant'

  0.35,

  // Expression: sigmaMag.^2
  //  Referenced by: '<S11>/Constant'

  { 810000.0, 0.0, 0.0, 0.0, 810000.0, 0.0, 0.0, 0.0, 810000.0 },

  // Expression: eye(6,6)
  //  Referenced by: '<S13>/Constant'

  { 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 1.0 },

  // Expression: 0
  //  Referenced by: '<S17>/Constant'

  0.0,

  // Expression: 2
  //  Referenced by: '<S33>/Gain'

  2.0,

  // Expression: 2
  //  Referenced by: '<S36>/Gain'

  2.0,

  // Expression: 2
  //  Referenced by: '<S31>/Gain'

  2.0,

  // Expression: 2
  //  Referenced by: '<S37>/Gain'

  2.0,

  // Expression: 2
  //  Referenced by: '<S32>/Gain'

  2.0,

  // Expression: 2
  //  Referenced by: '<S35>/Gain'

  2.0,

  // Expression: initialMagnetic/norm(initialMagnetic)
  //  Referenced by: '<S12>/known mag direction'

  { 0.59419050246366623, -0.010917626777941402, 0.80425024228004649 },

  // Expression: -1
  //  Referenced by: '<S17>/Gain1'

  -1.0,

  // Expression: -1
  //  Referenced by: '<S17>/Gain2'

  -1.0,

  // Expression: -1
  //  Referenced by: '<S17>/Gain'

  -1.0,

  // Expression: zeros(3,3)
  //  Referenced by: '<S12>/Constant'

  { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 },

  // Expression: sigmaMag.^2
  //  Referenced by: '<S15>/Constant'

  { 810000.0, 0.0, 0.0, 0.0, 810000.0, 0.0, 0.0, 0.0, 810000.0 },

  // Expression: 1
  //  Referenced by: '<S18>/Constant'

  1.0,

  // Expression: 1/4
  //  Referenced by: '<S18>/Gain1'

  0.25,

  // Expression: 1/2
  //  Referenced by: '<S18>/Gain'

  0.5,

  // Computed Parameter: nextAngularState_Y0
  //  Referenced by: '<S3>/nextAngularState'

  0.0,

  // Computed Parameter: nextAngularCov_Y0
  //  Referenced by: '<S3>/nextAngularCov'

  0.0,

  // Expression: 1/frequencyPredictionAttitude
  //  Referenced by: '<S3>/Constant'

  0.02,

  // Expression: angQ
  //  Referenced by: '<S46>/Constant'

  { 0.0001, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0001, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0001, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0001, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0001, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0001 },

  // Expression: eye(3,3)
  //  Referenced by: '<S48>/Constant'

  { 1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0 },

  // Expression: -1
  //  Referenced by: '<S48>/Gain'

  -1.0,

  // Expression: eye(3,3)
  //  Referenced by: '<S48>/Constant2'

  { 1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0 },

  // Expression: zeros(3,3)
  //  Referenced by: '<S48>/Constant3'

  { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 },

  // Expression: zeros(3,3)
  //  Referenced by: '<S50>/Constant3'

  { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 },

  // Expression: eye(3,3)
  //  Referenced by: '<S50>/Constant'

  { 1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0 },

  // Expression: eye(3,3)
  //  Referenced by: '<S50>/Constant2'

  { 1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0 },

  // Expression: -1
  //  Referenced by: '<S50>/Gain'

  -1.0,

  // Expression: eye(3,3)
  //  Referenced by: '<S49>/Constant'

  { 1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0 },

  // Expression: -1
  //  Referenced by: '<S49>/Gain'

  -1.0,

  // Expression: 0
  //  Referenced by: '<S51>/Constant'

  0.0,

  // Expression: -1
  //  Referenced by: '<S51>/Gain1'

  -1.0,

  // Expression: -1
  //  Referenced by: '<S51>/Gain2'

  -1.0,

  // Expression: -1
  //  Referenced by: '<S51>/Gain'

  -1.0,

  // Expression: -1
  //  Referenced by: '<S49>/Gain1'

  -1.0,

  // Expression: zeros(3,3)
  //  Referenced by: '<S49>/Constant3'

  { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 },

  // Expression: eye(4,4)
  //  Referenced by: '<S47>/Constant'

  { 1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0,
    1.0 },

  // Expression: 0
  //  Referenced by: '<S55>/Constant'

  0.0,

  // Expression: [-0.001907000000000; -0.011470000000000; 0.002193000000000]
  //  Referenced by: '<S47>/gyro biases'

  { -0.001907, -0.01147, 0.002193 },

  // Expression: -1
  //  Referenced by: '<S55>/Gain1'

  -1.0,

  // Expression: -1
  //  Referenced by: '<S55>/Gain2'

  -1.0,

  // Expression: -1
  //  Referenced by: '<S55>/Gain'

  -1.0,

  // Expression: -1
  //  Referenced by: '<S53>/Gain'

  -1.0,

  // Expression: eye(6,6)
  //  Referenced by: '<S70>/Constant'

  { 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 1.0 },

  // Expression: [0;0]
  //  Referenced by: '<S66>/Constant'

  { 0.0, 0.0 },

  // Expression: tempGradient
  //  Referenced by: '<S66>/Gain4'

  0.0065,

  // Expression: localGravity/isaR
  //  Referenced by: '<S66>/g R'

  0.034142352448891709,

  // Expression: [0;0;0]
  //  Referenced by: '<S66>/Constant1'

  { 0.0, 0.0, 0.0 },

  // Expression: sigmaBaro
  //  Referenced by: '<S68>/Constant'

  31.0211,

  // Expression: -1
  //  Referenced by: '<S62>/Gain'

  -1.0,

  // Expression: tempGradient
  //  Referenced by: '<S69>/HeightTemperatureGradient'

  0.0065,

  // Expression: localGravity
  //  Referenced by: '<S69>/gravity'

  9.8006602590058929,

  // Expression: isaR
  //  Referenced by: '<S69>/R air'

  287.05287,

  // Expression: initialLong
  //  Referenced by: '<S74>/Constant1'

  -8.288044,

  // Expression: initialLat
  //  Referenced by: '<S74>/Constant2'

  39.389280000000007,

  // Expression: 1/gpsA
  //  Referenced by: '<S74>/Gain2'

  8.9982312154404223E-6,

  // Expression: pi/180
  //  Referenced by: '<S79>/Gain1'

  0.017453292519943295,

  // Expression: eye(6,6)
  //  Referenced by: '<S83>/Constant'

  { 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 1.0 },

  // Expression: 1/gpsA
  //  Referenced by: '<S80>/Constant'

  8.9982312154404223E-6,

  // Expression: 0
  //  Referenced by: '<S80>/Constant1'

  0.0,

  // Expression: initialLat
  //  Referenced by: '<S80>/Constant2'

  39.389280000000007,

  // Expression: 1/gpsA
  //  Referenced by: '<S80>/Gain'

  8.9982312154404223E-6,

  // Expression: pi/180
  //  Referenced by: '<S84>/Gain1'

  0.017453292519943295,

  // Expression: gpsA
  //  Referenced by: '<S80>/Constant3'

  111132.95225,

  // Expression: gpsB
  //  Referenced by: '<S80>/Gain2'

  111412.87733,

  // Expression: 1
  //  Referenced by: '<S80>/Constant4'

  1.0,

  // Expression: gpsB
  //  Referenced by: '<S80>/Gain1'

  111412.87733,

  // Expression: 1000
  //  Referenced by: '<S80>/Gain3'

  1000.0,

  // Expression: [0 0 0 0; 0 0 0 0]
  //  Referenced by: '<S80>/Constant5'

  { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 },

  // Expression: [0 0 0 1 0 0; 0 0 0 0 1 0]
  //  Referenced by: '<S80>/Constant6'

  { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 1.0, 0.0, 0.0 },

  // Expression: sigmaGPS
  //  Referenced by: '<S82>/Constant'

  { 0.0447, 0.0, 0.0, 0.0, 0.0, 0.0447, 0.0, 0.0, 0.0, 0.0, 0.1, 0.0, 0.0, 0.0,
    0.0, 0.1 },

  // Expression: gpsB
  //  Referenced by: '<S74>/Gain1'

  111412.87733,

  // Expression: [1000; 1000; 1; 1]
  //  Referenced by: '<S74>/Gain'

  { 1000.0, 1000.0, 1.0, 1.0 },

  // Expression: eye(6,6)
  //  Referenced by: '<S95>/Constant'

  { 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 1.0 },

  // Expression: [0;0]
  //  Referenced by: '<S92>/Constant'

  { 0.0, 0.0 },

  // Expression: 1
  //  Referenced by: '<S92>/Constant5'

  1.0,

  // Expression: tempGradient
  //  Referenced by: '<S92>/lambda'

  0.0065,

  // Expression: z0
  //  Referenced by: '<S92>/Constant9'

  160.0,

  // Expression: localGravity
  //  Referenced by: '<S92>/Constant3'

  9.8006602590058929,

  // Expression: isaR
  //  Referenced by: '<S92>/R'

  287.05287,

  // Expression: -1
  //  Referenced by: '<S92>/Gain'

  -1.0,

  // Expression: -1
  //  Referenced by: '<S92>/Add Constant'

  -1.0,

  // Expression: [0;0;0]
  //  Referenced by: '<S92>/Constant1'

  { 0.0, 0.0, 0.0 },

  // Expression: 2
  //  Referenced by: '<S97>/Gain'

  2.0,

  // Expression: 2
  //  Referenced by: '<S97>/Gain1'

  2.0,

  // Expression: -1
  //  Referenced by: '<S92>/Gain3'

  -1.0,

  // Expression: gamma
  //  Referenced by: '<S92>/gamma'

  1.4,

  // Expression: -1
  //  Referenced by: '<S92>/Add Constant1'

  -1.0,

  // Expression: 1/2
  //  Referenced by: '<S92>/Gain1'

  0.5,

  // Expression: +1
  //  Referenced by: '<S92>/Add Constant3'

  1.0,

  // Expression: -1
  //  Referenced by: '<S92>/Add Constant2'

  -1.0,

  // Expression: [0;0]
  //  Referenced by: '<S92>/Constant7'

  { 0.0, 0.0 },

  // Expression: -1
  //  Referenced by: '<S92>/Gain2'

  -1.0,

  // Expression: [0;0;0]
  //  Referenced by: '<S92>/Constant8'

  { 0.0, 0.0, 0.0 },

  // Expression: 2
  //  Referenced by: '<S92>/Constant6'

  2.0,

  // Expression: [zeros(3, 3),eye(3)]
  //  Referenced by: '<S92>/dstates'

  { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0, 0.0,
    0.0, 0.0, 1.0 },

  // Expression: 1
  //  Referenced by: '<S92>/Bias'

  1.0,

  // Expression: [sigmaPitotS^2, 0; 0, sigmaPitotD^2]
  //  Referenced by: '<S94>/Constant'

  { 1256.0857456899998, 0.0, 0.0, 24856.013432410004 },

  // Expression: -1
  //  Referenced by: '<S96>/Add Constant'

  -1.0,

  // Expression: 1/2
  //  Referenced by: '<S96>/Gain'

  0.5,

  // Expression: +1
  //  Referenced by: '<S96>/Add Constant1'

  1.0,

  // Expression: -1
  //  Referenced by: '<S96>/Add Constant2'

  -1.0,

  // Computed Parameter: nextLinearState_Y0
  //  Referenced by: '<S5>/nextLinearState'

  0.0,

  // Computed Parameter: nextLinearCov_Y0
  //  Referenced by: '<S5>/nextLinearCov'

  0.0,

  // Expression: 1/frequencyPredictionLinear
  //  Referenced by: '<S5>/Constant'

  0.02,

  // Expression: linQ
  //  Referenced by: '<S106>/Constant'

  { 0.1, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.1, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.1,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.1, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.1, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.1 },

  // Expression: [0 0 0 1 0 0; 0 0 0 0 1 0; 0 0 0 0 0 1; 0 0 0 0 0 0; 0 0 0 0 0 0; 0 0 0 0 0 0]
  //  Referenced by: '<S108>/Constant'

  { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 1.0, 0.0, 0.0, 0.0 },

  // Expression: 2
  //  Referenced by: '<S114>/Gain'

  2.0,

  // Expression: 2
  //  Referenced by: '<S117>/Gain'

  2.0,

  // Expression: 2
  //  Referenced by: '<S112>/Gain'

  2.0,

  // Expression: 2
  //  Referenced by: '<S118>/Gain'

  2.0,

  // Expression: 2
  //  Referenced by: '<S113>/Gain'

  2.0,

  // Expression: 2
  //  Referenced by: '<S116>/Gain'

  2.0,

  // Expression: [0;0; localGravity]
  //  Referenced by: '<S109>/Constant'

  { 0.0, 0.0, 9.8006602590058929 },

  // Expression: 1
  //  Referenced by: '<S1>/Zero5'

  1.0,

  // Expression: 0
  //  Referenced by: '<S1>/Zero2'

  0.0,

  // Expression: 0
  //  Referenced by: '<S44>/Constant'

  0.0,

  // Expression: 0
  //  Referenced by: '<S45>/Constant'

  0.0,

  // Expression: 0
  //  Referenced by: '<S72>/Constant'

  0.0,

  // Expression: 0
  //  Referenced by: '<S73>/Constant'

  0.0,

  // Expression: 0
  //  Referenced by: '<S86>/Constant'

  0.0,

  // Expression: 0
  //  Referenced by: '<S87>/Constant'

  0.0,

  // Expression: 0
  //  Referenced by: '<S99>/Constant'

  0.0,

  // Expression: 0
  //  Referenced by: '<S100>/Constant'

  0.0,

  // Expression: 0
  //  Referenced by: '<S125>/Constant'

  0.0,

  // Expression: 0
  //  Referenced by: '<S126>/Constant'

  0.0,

  // Expression: 0
  //  Referenced by: '<S128>/Constant'

  0.0,

  // Expression: 0
  //  Referenced by: '<S129>/Constant'

  0.0,

  // Expression: 1/frequencyMagnetometer
  //  Referenced by: '<S10>/Constant4'

  1.0E+6,

  // Expression: 1
  //  Referenced by: '<S10>/Constant'

  1.0,

  // Expression: 0
  //  Referenced by: '<S10>/Constant1'

  0.0,

  // Expression: 1
  //  Referenced by: '<S10>/Constant2'

  1.0,

  // Expression: 0
  //  Referenced by: '<S10>/Constant3'

  0.0,

  // Expression: flagMagnetometer
  //  Referenced by: '<S10>/Gain'

  0.0,

  // Expression: 0
  //  Referenced by: '<S10>/Data Store Memory'

  0.0,

  // Expression: 0
  //  Referenced by: '<S1>/Unit Delay'

  0.0,

  // Expression: angP0
  //  Referenced by: '<S1>/Unit Delay1'

  { 0.0001, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0001, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0001, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0001, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0001, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0001 },

  // Expression: 1
  //  Referenced by: '<S6>/Constant'

  1.0,

  // Expression: 0
  //  Referenced by: '<S6>/Constant1'

  0.0,

  // Expression: 1
  //  Referenced by: '<S6>/Constant2'

  1.0,

  // Expression: 1/frequencyPredictionAttitude
  //  Referenced by: '<S6>/Constant4'

  0.02,

  // Expression: 0
  //  Referenced by: '<S6>/Constant3'

  0.0,

  // Computed Parameter: Merge2_InitialOutput
  //  Referenced by: '<S2>/Merge2'

  0.0,

  // Computed Parameter: Merge3_InitialOutput
  //  Referenced by: '<S2>/Merge3'

  0.0,

  // Expression: 1/frequencyBarometer
  //  Referenced by: '<S64>/Constant4'

  0.02,

  // Expression: 1
  //  Referenced by: '<S64>/Constant'

  1.0,

  // Expression: 0
  //  Referenced by: '<S64>/Constant1'

  0.0,

  // Expression: 1
  //  Referenced by: '<S64>/Constant2'

  1.0,

  // Expression: 0
  //  Referenced by: '<S64>/Constant3'

  0.0,

  // Expression: flagBaro
  //  Referenced by: '<S64>/Gain'

  1.0,

  // Expression: 0
  //  Referenced by: '<S64>/Data Store Memory'

  0.0,

  // Expression: 0
  //  Referenced by: '<S1>/Unit Delay2'

  0.0,

  // Expression: linP0
  //  Referenced by: '<S1>/Unit Delay5'

  { 0.1, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.1, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.1,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.1, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.1, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.1 },

  // Expression: 1
  //  Referenced by: '<S7>/Constant'

  1.0,

  // Expression: 0
  //  Referenced by: '<S7>/Constant1'

  0.0,

  // Expression: 1
  //  Referenced by: '<S7>/Constant2'

  1.0,

  // Expression: 1/frequencyPredictionLinear
  //  Referenced by: '<S7>/Constant4'

  0.02,

  // Expression: 0
  //  Referenced by: '<S7>/Constant3'

  0.0,

  // Expression: 1
  //  Referenced by: '<S76>/Constant'

  1.0,

  // Expression: 0
  //  Referenced by: '<S76>/Constant1'

  0.0,

  // Expression: 1
  //  Referenced by: '<S76>/Constant2'

  1.0,

  // Expression: 1/frequencyGPS
  //  Referenced by: '<S76>/Constant4'

  0.1,

  // Expression: 0
  //  Referenced by: '<S76>/Constant3'

  0.0,

  // Expression: flagGPS
  //  Referenced by: '<S76>/Gain'

  1.0,

  // Computed Parameter: Merge_InitialOutput
  //  Referenced by: '<S60>/Merge'

  0.0,

  // Computed Parameter: Merge1_InitialOutput
  //  Referenced by: '<S60>/Merge1'

  0.0,

  // Computed Parameter: Merge_InitialOutput_o
  //  Referenced by: '<S59>/Merge'

  0.0,

  // Computed Parameter: Merge1_InitialOutput_n
  //  Referenced by: '<S59>/Merge1'

  0.0,

  // Expression: 0
  //  Referenced by: '<S76>/Data Store Memory'

  0.0,

  // Expression: 1/frequencyPitot
  //  Referenced by: '<S90>/Constant4'

  0.04,

  // Expression: 1
  //  Referenced by: '<S90>/Constant'

  1.0,

  // Expression: 0
  //  Referenced by: '<S90>/Constant1'

  0.0,

  // Expression: 1
  //  Referenced by: '<S90>/Constant2'

  1.0,

  // Expression: 0
  //  Referenced by: '<S90>/Constant3'

  0.0,

  // Expression: 0
  //  Referenced by: '<S90>/Constant5'

  0.0,

  // Expression: T0
  //  Referenced by: '<S105>/Sea Level  Temperature'

  288.15,

  // Expression: -1
  //  Referenced by: '<S101>/Gain'

  -1.0,

  // Expression: z0
  //  Referenced by: '<S101>/Bias'

  160.0,

  // Expression: h_trop
  //  Referenced by: '<S105>/Limit  altitude  to troposhere'

  11000.0,

  // Expression: h0
  //  Referenced by: '<S105>/Limit  altitude  to troposhere'

  0.0,

  // Expression: L
  //  Referenced by: '<S105>/Lapse Rate'

  0.0065,

  // Expression: gamma*R
  //  Referenced by: '<S105>/gamma*R'

  401.87433999999996,

  // Expression: flagPitot
  //  Referenced by: '<S90>/Gain'

  1.0,

  // Expression: 0
  //  Referenced by: '<S90>/Data Store Memory'

  0.0,

  // Computed Parameter: Merge_InitialOutput_a
  //  Referenced by: '<S61>/Merge'

  0.0,

  // Computed Parameter: Merge1_InitialOutput_k
  //  Referenced by: '<S61>/Merge1'

  0.0,

  // Expression: 0
  //  Referenced by: '<S6>/Data Store Memory'

  0.0,

  // Expression: 0
  //  Referenced by: '<S7>/Data Store Memory'

  0.0,

  // Computed Parameter: Switch_Threshold
  //  Referenced by: '<S19>/Switch'

  0.0F,

  // Computed Parameter: Gain1_Gain_ed
  //  Referenced by: '<S53>/Gain1'

  -1.0F,

  // Computed Parameter: Constant_Value_dx
  //  Referenced by: '<S53>/Constant'

  0.0F,

  // Computed Parameter: Gain_Gain_nq
  //  Referenced by: '<S47>/Gain'

  0.5F,

  // Computed Parameter: Gain3_Gain_l
  //  Referenced by: '<S66>/Gain3'

  0.0341423526F,

  // Computed Parameter: Constant1_Value_a
  //  Referenced by: '<S108>/Constant1'

  { 1.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 1.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F,
    0.0F, 1.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 1.0F, 0.0F, 0.0F, 0.0F, 0.0F,
    0.0F, 0.0F, 1.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 1.0F },

  // Computed Parameter: Constant_Value_i5
  //  Referenced by: '<S1>/Constant'

  { 0.0F, 0.0F, 0.0F },

  // Computed Parameter: RateTransition_InitialCondition
  //  Referenced by: '<S10>/Rate Transition'

  0.0F,

  // Computed Parameter: RateTransition1_InitialConditio
  //  Referenced by: '<S64>/Rate Transition1'

  0.0F,

  // Computed Parameter: RateTransition1_InitialCondit_a
  //  Referenced by: '<S90>/Rate Transition1'

  0.0F,

  // Computed Parameter: RateTransition_InitialConditi_p
  //  Referenced by: '<S6>/Rate Transition'

  0.0F,

  // Computed Parameter: RateTransition_InitialConditi_h
  //  Referenced by: '<S7>/Rate Transition'

  0.0F,

  // Computed Parameter: Zero1_Value
  //  Referenced by: '<S1>/Zero1'

  false,

  // Computed Parameter: Zero3_Value
  //  Referenced by: '<S1>/Zero3'

  3U,

  // Computed Parameter: Zero4_Value
  //  Referenced by: '<S1>/Zero4'

  2U,

  // Computed Parameter: UnitDelay3_InitialCondition
  //  Referenced by: '<S1>/Unit Delay3'

  0U,

  // Computed Parameter: UnitDelay4_InitialCondition
  //  Referenced by: '<S1>/Unit Delay4'

  0U,

  // Computed Parameter: Bias1_Bias
  //  Referenced by: '<S1>/Bias1'

  1U,

  // Computed Parameter: Bias2_Bias
  //  Referenced by: '<S1>/Bias2'

  1U,

  // Computed Parameter: Saturation1_UpperSat
  //  Referenced by: '<S1>/Saturation1'

  1U,

  // Computed Parameter: Saturation1_LowerSat
  //  Referenced by: '<S1>/Saturation1'

  0U,

  // Computed Parameter: Saturation2_UpperSat
  //  Referenced by: '<S1>/Saturation2'

  1U,

  // Computed Parameter: Saturation2_LowerSat
  //  Referenced by: '<S1>/Saturation2'

  0U
};

//
// File trailer for generated code.
//
// [EOF]
//
