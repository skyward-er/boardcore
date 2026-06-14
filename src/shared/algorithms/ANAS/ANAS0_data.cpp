//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// File: ANAS0_data.cpp
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
#include "ANAS0.h"

// Block parameters (default storage)
ANAS0::P_ANAS0_T ANAS0::ANAS0_P{
  // Mask Parameter: Difference_ICPrevInput
  //  Referenced by: '<S89>/UD'

  (0ULL),

  // Mask Parameter: Difference_ICPrevInput_j
  //  Referenced by: '<S219>/UD'

  (0ULL),

  // Mask Parameter: Difference_ICPrevInput_m
  //  Referenced by: '<S120>/UD'

  99713.7F,

  // Mask Parameter: GPSAccelerationLimit_const
  //  Referenced by: '<S217>/Constant'

  35.0F,

  // Mask Parameter: PitotMinMach_const
  //  Referenced by: '<S196>/Constant'

  0.35F,

  // Mask Parameter: g02accg02_lowlimit
  //  Referenced by: '<S49>/Lower Limit'

  9.60623455F,

  // Mask Parameter: g02accg02_uplimit
  //  Referenced by: '<S49>/Upper Limit'

  10.0062351F,

  // Computed Parameter: Memory_InitialCondition
  //  Referenced by: '<S11>/Memory'

  (0ULL),

  // Computed Parameter: Memory_InitialCondition_n
  //  Referenced by: '<S51>/Memory'

  (0ULL),

  // Computed Parameter: Memory_InitialCondition_k
  //  Referenced by: '<S203>/Memory'

  (0ULL),

  // Computed Parameter: Memory3_InitialCondition
  //  Referenced by: '<S103>/Memory3'

  (0ULL),

  // Computed Parameter: Memory2_InitialCondition
  //  Referenced by: '<S103>/Memory2'

  (0ULL),

  // Computed Parameter: Memory1_InitialCondition
  //  Referenced by: '<S103>/Memory1'

  (0ULL),

  // Computed Parameter: Constant_Value
  //  Referenced by: '<S20>/Constant'

  0.0F,

  // Computed Parameter: IdentityMatrix_IDMatrixData
  //  Referenced by: '<S22>/Identity Matrix'

  { 1.0F, 0.0F, 0.0F, 0.0F, 1.0F, 0.0F, 0.0F, 0.0F, 1.0F },

  // Computed Parameter: AccSigma_Value
  //  Referenced by: '<S10>/AccSigma'

  { 0.1F, 0.0F, 0.0F, 0.0F, 0.1F, 0.0F, 0.0F, 0.0F, 0.1F },

  // Computed Parameter: Constant_Value_k
  //  Referenced by: '<S33>/Constant'

  0.0F,

  // Computed Parameter: Gain_Gain
  //  Referenced by: '<S37>/Gain'

  2.0F,

  // Computed Parameter: Gain_Gain_o
  //  Referenced by: '<S40>/Gain'

  2.0F,

  // Computed Parameter: Gain_Gain_i
  //  Referenced by: '<S35>/Gain'

  2.0F,

  // Computed Parameter: Gain_Gain_b
  //  Referenced by: '<S41>/Gain'

  2.0F,

  // Computed Parameter: Gain_Gain_h
  //  Referenced by: '<S36>/Gain'

  2.0F,

  // Computed Parameter: Gain_Gain_g
  //  Referenced by: '<S39>/Gain'

  2.0F,

  // Computed Parameter: LocalGravity_Value
  //  Referenced by: '<S16>/Local Gravity'

  { 0.0F, 0.0F, -9.80623436F },

  // Computed Parameter: Gain1_Gain
  //  Referenced by: '<S33>/Gain1'

  -1.0F,

  // Computed Parameter: Gain2_Gain
  //  Referenced by: '<S33>/Gain2'

  -1.0F,

  // Computed Parameter: Gain_Gain_j
  //  Referenced by: '<S33>/Gain'

  -1.0F,

  // Computed Parameter: NASCondLim_Value
  //  Referenced by: '<S14>/NASCondLim'

  1.0E-15F,

  // Computed Parameter: Constant_Value_b
  //  Referenced by: '<S17>/Constant'

  { 1.0F, 0.0F, 0.0F, 0.0F, 1.0F, 0.0F, 0.0F, 0.0F, 1.0F },

  // Computed Parameter: Gain1_Gain_o
  //  Referenced by: '<S15>/Gain1'

  0.5F,

  // Computed Parameter: Gain_Gain_h5
  //  Referenced by: '<S15>/Gain'

  -0.25F,

  // Computed Parameter: Bias_Bias
  //  Referenced by: '<S15>/Bias'

  1.0F,

  // Computed Parameter: Constant_Value_c
  //  Referenced by: '<S60>/Constant'

  0.0F,

  // Computed Parameter: IdentityMatrix_IDMatrixData_a
  //  Referenced by: '<S62>/Identity Matrix'

  { 1.0F, 0.0F, 0.0F, 0.0F, 1.0F, 0.0F, 0.0F, 0.0F, 1.0F },

  // Computed Parameter: sxMatrix_Value
  //  Referenced by: '<S74>/sxMatrix'

  { 1.0F, 0.0F, 0.0F, 0.0F, -1.0F, 0.0F, 0.0F, 0.0F, 1.0F },

  // Computed Parameter: dxMatrix_Value
  //  Referenced by: '<S74>/dxMatrix'

  { 1.0F, 0.0F, 0.0F, 0.0F, 1.0F, 0.0F, 0.0F, 0.0F, 1.0F },

  // Computed Parameter: Constant_Value_l
  //  Referenced by: '<S57>/Constant'

  { 1.0F, 0.0F, 0.0F, 0.0F, 1.0F, 0.0F, 0.0F, 0.0F, 1.0F },

  // Computed Parameter: Constant_Value_p
  //  Referenced by: '<S73>/Constant'

  0.0F,

  // Computed Parameter: Gain_Gain_hq
  //  Referenced by: '<S78>/Gain'

  2.0F,

  // Computed Parameter: Gain_Gain_gr
  //  Referenced by: '<S81>/Gain'

  2.0F,

  // Computed Parameter: Gain_Gain_hf
  //  Referenced by: '<S76>/Gain'

  2.0F,

  // Computed Parameter: Gain_Gain_m
  //  Referenced by: '<S82>/Gain'

  2.0F,

  // Computed Parameter: Gain_Gain_d
  //  Referenced by: '<S77>/Gain'

  2.0F,

  // Computed Parameter: Gain_Gain_h1
  //  Referenced by: '<S80>/Gain'

  2.0F,

  // Computed Parameter: InitMagnField_Value
  //  Referenced by: '<S56>/InitMagnField'

  { 22547.6F, 1424.0F, 42151.4F },

  // Computed Parameter: Gain1_Gain_m
  //  Referenced by: '<S73>/Gain1'

  -1.0F,

  // Computed Parameter: Gain2_Gain_g
  //  Referenced by: '<S73>/Gain2'

  -1.0F,

  // Computed Parameter: Gain_Gain_gl
  //  Referenced by: '<S73>/Gain'

  -1.0F,

  // Computed Parameter: MagSigma_Value
  //  Referenced by: '<S50>/MagSigma'

  { 900.0F, 0.0F, 0.0F, 0.0F, 900.0F, 0.0F, 0.0F, 0.0F, 900.0F },

  // Computed Parameter: NASCondLim_Value_n
  //  Referenced by: '<S54>/NASCondLim'

  1.0E-15F,

  // Computed Parameter: fromGGausstonTTesla_Gain
  //  Referenced by: '<S56>/from G (Gauss) to nT (Tesla)'

  100000.0F,

  // Computed Parameter: Gain1_Gain_j
  //  Referenced by: '<S55>/Gain1'

  0.5F,

  // Computed Parameter: Gain_Gain_p
  //  Referenced by: '<S55>/Gain'

  -0.25F,

  // Computed Parameter: Bias_Bias_d
  //  Referenced by: '<S55>/Bias'

  1.0F,

  // Computed Parameter: Merge_InitialOutput
  //  Referenced by: '<S9>/Merge'

  0.0F,

  // Computed Parameter: Merge_InitialOutput_d
  //  Referenced by: '<S8>/Merge'

  0.0F,

  // Computed Parameter: Gain1_Gain_h
  //  Referenced by: '<S3>/Gain1'

  1.0E-6F,

  // Computed Parameter: Constant_Value_cv
  //  Referenced by: '<S3>/Constant'

  0.01F,

  // Computed Parameter: Constant1_Value
  //  Referenced by: '<S3>/Constant1'

  { 1.0F, 0.0F, 0.0F, 0.0F, 0.0F, 1.0F, 0.0F, 0.0F, 0.0F, 0.0F, 1.0F, 0.0F, 0.0F,
    0.0F, 0.0F, 1.0F },

  // Computed Parameter: Constant_Value_j
  //  Referenced by: '<S92>/Constant'

  0.0F,

  // Computed Parameter: Gain1_Gain_oh
  //  Referenced by: '<S92>/Gain1'

  -1.0F,

  // Computed Parameter: Gain2_Gain_d
  //  Referenced by: '<S92>/Gain2'

  -1.0F,

  // Computed Parameter: Gain_Gain_e
  //  Referenced by: '<S92>/Gain'

  -1.0F,

  // Computed Parameter: Gain_Gain_eg
  //  Referenced by: '<S90>/Gain'

  -1.0F,

  // Computed Parameter: Gain1_Gain_k
  //  Referenced by: '<S90>/Gain1'

  -1.0F,

  // Computed Parameter: Constant_Value_h
  //  Referenced by: '<S90>/Constant'

  0.0F,

  // Computed Parameter: Gain_Gain_ow
  //  Referenced by: '<S3>/Gain'

  0.5F,

  // Computed Parameter: Constant2_Value
  //  Referenced by: '<S3>/Constant2'

  { 1.0F, 0.0F, 0.0F, 0.0F, 1.0F, 0.0F, 0.0F, 0.0F, 1.0F },

  // Computed Parameter: AngularQ1_3_Value
  //  Referenced by: '<S3>/AngularQ1_3'

  { 0.0001F, 0.0F, 0.0F, 0.0F, 0.0001F, 0.0F, 0.0F, 0.0F, 0.0001F },

  // Computed Parameter: Constant_Value_e
  //  Referenced by: '<S113>/Constant'

  0.0F,

  // Computed Parameter: IdentityMatrix_IDMatrixData_i
  //  Referenced by: '<S115>/Identity Matrix'

  { 1.0F, 0.0F, 0.0F, 1.0F },

  // Computed Parameter: Constant_Value_n
  //  Referenced by: '<S109>/Constant'

  { 1.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 1.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F,
    0.0F, 1.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 1.0F, 0.0F, 0.0F, 0.0F, 0.0F,
    0.0F, 0.0F, 1.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 1.0F },

  // Computed Parameter: Constant3_Value
  //  Referenced by: '<S106>/Constant3'

  { 0.0F, 0.0F },

  // Computed Parameter: Baroa_Value
  //  Referenced by: '<S106>/Baroa'

  0.0065F,

  // Computed Parameter: BarogR_Value
  //  Referenced by: '<S106>/Barog//R'

  0.0341617726F,

  // Computed Parameter: Constant2_Value_l
  //  Referenced by: '<S106>/Constant2'

  1.0F,

  // Computed Parameter: Constant1_Value_f
  //  Referenced by: '<S106>/Constant1'

  -1.0F,

  // Computed Parameter: Constant9_Value
  //  Referenced by: '<S106>/Constant9'

  { 0.0F, 0.0F, 0.0F },

  // Computed Parameter: Constant5_Value
  //  Referenced by: '<S106>/Constant5'

  { 0.0F, 0.0F },

  // Computed Parameter: Gain1_Gain_a
  //  Referenced by: '<S106>/Gain1'

  -1.0F,

  // Computed Parameter: RAir1_Value
  //  Referenced by: '<S110>/RAir1'

  0.0065F,

  // Computed Parameter: LocalGravity_Value_j
  //  Referenced by: '<S110>/LocalGravity'

  9.80623436F,

  // Computed Parameter: RAir_Value
  //  Referenced by: '<S110>/RAir'

  287.052856F,

  // Computed Parameter: Gain2_Gain_j
  //  Referenced by: '<S106>/Gain2'

  -1.0F,

  // Computed Parameter: Constant6_Value
  //  Referenced by: '<S106>/Constant6'

  { 0.0F, 0.0F },

  // Computed Parameter: BaroVaroSigma_Value
  //  Referenced by: '<S98>/BaroVaroSigma'

  { 31.0211F, 0.0F, 0.0F, 35.0F },

  // Computed Parameter: NASCondLim_Value_a
  //  Referenced by: '<S107>/NASCondLim'

  1.0E-15F,

  // Computed Parameter: DiscreteFilter_NumCoef
  //  Referenced by: '<S116>/Discrete Filter'

  { 0.0F, 1.0F },

  // Computed Parameter: DiscreteFilter_DenCoef
  //  Referenced by: '<S116>/Discrete Filter'

  { 20.0F, -19.0F },

  // Computed Parameter: DiscreteFilter_InitialStates
  //  Referenced by: '<S116>/Discrete Filter'

  0.0F,

  // Computed Parameter: Gain_Gain_c
  //  Referenced by: '<S108>/Gain'

  -1.0F,

  // Computed Parameter: RAir1_Value_c
  //  Referenced by: '<S117>/RAir1'

  0.0065F,

  // Computed Parameter: LocalGravity_Value_h
  //  Referenced by: '<S117>/LocalGravity'

  9.80623436F,

  // Computed Parameter: RAir_Value_o
  //  Referenced by: '<S117>/RAir'

  287.052856F,

  // Computed Parameter: Baroa_Value_g
  //  Referenced by: '<S119>/Baroa'

  0.0065F,

  // Computed Parameter: BarogR_Value_i
  //  Referenced by: '<S119>/Barog//R'

  0.0341617726F,

  // Computed Parameter: Constant2_Value_lk
  //  Referenced by: '<S122>/Constant2'

  1.0F,

  // Computed Parameter: Constant1_Value_p
  //  Referenced by: '<S122>/Constant1'

  -1.0F,

  // Computed Parameter: frequencyPrediction_Value
  //  Referenced by: '<S116>/frequencyPrediction'

  100.0F,

  // Computed Parameter: Constant_Value_ez
  //  Referenced by: '<S134>/Constant'

  0.0F,

  // Computed Parameter: IdentityMatrix_IDMatrixData_n
  //  Referenced by: '<S136>/Identity Matrix'

  { 1.0F, 0.0F, 0.0F, 1.0F },

  // Computed Parameter: Constant_Value_k5
  //  Referenced by: '<S127>/Constant'

  { 1.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 1.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F,
    0.0F, 1.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 1.0F, 0.0F, 0.0F, 0.0F, 0.0F,
    0.0F, 0.0F, 1.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 1.0F },

  // Computed Parameter: Constant1_Value_h
  //  Referenced by: '<S124>/Constant1'

  { 0.0F, 0.0F },

  // Computed Parameter: Constant8_Value
  //  Referenced by: '<S124>/Constant8'

  0.0065F,

  // Computed Parameter: BarogR_Value_b
  //  Referenced by: '<S124>/Barog//R'

  0.0341617726F,

  // Computed Parameter: Constant2_Value_n
  //  Referenced by: '<S130>/Constant2'

  1.0F,

  // Computed Parameter: Constant1_Value_j
  //  Referenced by: '<S130>/Constant1'

  -1.0F,

  // Computed Parameter: Constant11_Value
  //  Referenced by: '<S129>/Constant11'

  0.4F,

  // Computed Parameter: Constant10_Value
  //  Referenced by: '<S129>/Constant10'

  287.052856F,

  // Computed Parameter: Constant6_Value_f
  //  Referenced by: '<S129>/Constant6'

  1.4F,

  // Computed Parameter: Gain2_Gain_h
  //  Referenced by: '<S129>/Gain2'

  0.5F,

  // Computed Parameter: Bias2_Bias
  //  Referenced by: '<S129>/Bias2'

  1.0F,

  // Computed Parameter: IsaGamma_Value
  //  Referenced by: '<S124>/IsaGamma'

  1.4F,

  // Computed Parameter: Bias1_Bias
  //  Referenced by: '<S124>/Bias1'

  -1.0F,

  // Computed Parameter: Bias4_Bias
  //  Referenced by: '<S124>/Bias4'

  -1.0F,

  // Computed Parameter: Gain1_Gain_b
  //  Referenced by: '<S124>/Gain1'

  -1.0F,

  // Computed Parameter: RAir1_Value_cc
  //  Referenced by: '<S128>/RAir1'

  0.0065F,

  // Computed Parameter: LocalGravity_Value_p
  //  Referenced by: '<S128>/LocalGravity'

  9.80623436F,

  // Computed Parameter: RAir_Value_p
  //  Referenced by: '<S128>/RAir'

  287.052856F,

  // Computed Parameter: Bias2_Bias_l
  //  Referenced by: '<S124>/Bias2'

  -1.0F,

  // Computed Parameter: Baro05R_Gain
  //  Referenced by: '<S124>/Baro0.5//R'

  0.00174183946F,

  // Computed Parameter: Bias3_Bias
  //  Referenced by: '<S124>/Bias3'

  -1.0F,

  // Computed Parameter: Baro1R_Gain
  //  Referenced by: '<S124>/Baro1//R'

  0.00348367891F,

  // Computed Parameter: Constant3_Value_a
  //  Referenced by: '<S124>/Constant3'

  { 0.0F, 0.0F },

  // Computed Parameter: Constant9_Value_o
  //  Referenced by: '<S124>/Constant9'

  { 0.0F, 0.0F, 0.0F },

  // Computed Parameter: PitotDBaroSigma_Value
  //  Referenced by: '<S99>/PitotDBaroSigma'

  { 157.657898F, 0.0F, 0.0F, 31.0211F },

  // Computed Parameter: NASCondLim_Value_g
  //  Referenced by: '<S125>/NASCondLim'

  1.0E-15F,

  // Computed Parameter: Gain_Gain_l
  //  Referenced by: '<S137>/Gain'

  -1.0F,

  // Computed Parameter: RAir1_Value_m
  //  Referenced by: '<S139>/RAir1'

  0.0065F,

  // Computed Parameter: LocalGravity_Value_e
  //  Referenced by: '<S139>/LocalGravity'

  9.80623436F,

  // Computed Parameter: RAir_Value_a
  //  Referenced by: '<S139>/RAir'

  287.052856F,

  // Computed Parameter: Constant_Value_pb
  //  Referenced by: '<S137>/Constant'

  1.0F,

  // Computed Parameter: IsaGamma1_Value
  //  Referenced by: '<S140>/IsaGamma-1'

  0.4F,

  // Computed Parameter: AirR_Value
  //  Referenced by: '<S140>/AirR'

  287.052856F,

  // Computed Parameter: IsaGamma_Value_i
  //  Referenced by: '<S140>/IsaGamma'

  1.4F,

  // Computed Parameter: Baroa_Value_a
  //  Referenced by: '<S137>/Baroa'

  0.0065F,

  // Computed Parameter: Gain2_Gain_k
  //  Referenced by: '<S140>/Gain2'

  0.5F,

  // Computed Parameter: Bias2_Bias_b
  //  Referenced by: '<S140>/Bias2'

  1.0F,

  // Computed Parameter: IsaGamma_Value_iz
  //  Referenced by: '<S137>/IsaGamma'

  1.4F,

  // Computed Parameter: Bias_Bias_p
  //  Referenced by: '<S137>/Bias'

  -1.0F,

  // Computed Parameter: Constant_Value_j3
  //  Referenced by: '<S154>/Constant'

  0.0F,

  // Computed Parameter: IdentityMatrix_IDMatrixData_p
  //  Referenced by: '<S156>/Identity Matrix'

  { 1.0F, 0.0F, 0.0F, 1.0F },

  // Computed Parameter: Constant_Value_i
  //  Referenced by: '<S147>/Constant'

  { 1.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 1.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F,
    0.0F, 1.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 1.0F, 0.0F, 0.0F, 0.0F, 0.0F,
    0.0F, 0.0F, 1.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 1.0F },

  // Computed Parameter: Constant1_Value_k
  //  Referenced by: '<S144>/Constant1'

  { 0.0F, 0.0F },

  // Computed Parameter: Constant8_Value_c
  //  Referenced by: '<S144>/Constant8'

  0.0065F,

  // Computed Parameter: Constant4_Value
  //  Referenced by: '<S144>/Constant4'

  9.80623436F,

  // Computed Parameter: Constant7_Value
  //  Referenced by: '<S144>/Constant7'

  287.052856F,

  // Computed Parameter: Constant2_Value_d
  //  Referenced by: '<S149>/Constant2'

  1.0F,

  // Computed Parameter: Constant1_Value_n
  //  Referenced by: '<S149>/Constant1'

  -1.0F,

  // Computed Parameter: Constant11_Value_k
  //  Referenced by: '<S148>/Constant11'

  0.4F,

  // Computed Parameter: Constant10_Value_i
  //  Referenced by: '<S148>/Constant10'

  287.052856F,

  // Computed Parameter: Constant6_Value_j
  //  Referenced by: '<S148>/Constant6'

  1.4F,

  // Computed Parameter: Gain2_Gain_n
  //  Referenced by: '<S148>/Gain2'

  0.5F,

  // Computed Parameter: Bias2_Bias_f
  //  Referenced by: '<S148>/Bias2'

  1.0F,

  // Computed Parameter: Constant2_Value_f
  //  Referenced by: '<S144>/Constant2'

  1.4F,

  // Computed Parameter: Bias1_Bias_c
  //  Referenced by: '<S144>/Bias1'

  -1.0F,

  // Computed Parameter: Bias4_Bias_f
  //  Referenced by: '<S144>/Bias4'

  -1.0F,

  // Computed Parameter: Gain1_Gain_n
  //  Referenced by: '<S144>/Gain1'

  -1.0F,

  // Computed Parameter: RAir1_Value_d
  //  Referenced by: '<S150>/RAir1'

  0.0065F,

  // Computed Parameter: LocalGravity_Value_o
  //  Referenced by: '<S150>/LocalGravity'

  9.80623436F,

  // Computed Parameter: RAir_Value_k
  //  Referenced by: '<S150>/RAir'

  287.052856F,

  // Computed Parameter: Bias2_Bias_j
  //  Referenced by: '<S144>/Bias2'

  -1.0F,

  // Computed Parameter: Gain_Gain_c1
  //  Referenced by: '<S144>/Gain'

  0.00174183946F,

  // Computed Parameter: Bias3_Bias_f
  //  Referenced by: '<S144>/Bias3'

  -1.0F,

  // Computed Parameter: Gain2_Gain_f
  //  Referenced by: '<S144>/Gain2'

  0.00348367891F,

  // Computed Parameter: Constant3_Value_l
  //  Referenced by: '<S144>/Constant3'

  { 0.0F, 0.0F },

  // Computed Parameter: Constant9_Value_m
  //  Referenced by: '<S144>/Constant9'

  { 0.0F, 0.0F, 0.0F },

  // Computed Parameter: PitotDSSigma_Value
  //  Referenced by: '<S100>/PitotDSSigma'

  { 157.657898F, 0.0F, 0.0F, 35.4413F },

  // Computed Parameter: NASCondLim_Value_i
  //  Referenced by: '<S145>/NASCondLim'

  1.0E-15F,

  // Computed Parameter: Gain_Gain_f
  //  Referenced by: '<S157>/Gain'

  -1.0F,

  // Computed Parameter: RAir1_Value_b
  //  Referenced by: '<S159>/RAir1'

  0.0065F,

  // Computed Parameter: LocalGravity_Value_d
  //  Referenced by: '<S159>/LocalGravity'

  9.80623436F,

  // Computed Parameter: RAir_Value_j
  //  Referenced by: '<S159>/RAir'

  287.052856F,

  // Computed Parameter: Constant_Value_nu
  //  Referenced by: '<S157>/Constant'

  1.0F,

  // Computed Parameter: IsaGamma1_Value_e
  //  Referenced by: '<S160>/IsaGamma-1'

  0.4F,

  // Computed Parameter: AirR_Value_m
  //  Referenced by: '<S160>/AirR'

  287.052856F,

  // Computed Parameter: IsaGamma_Value_m
  //  Referenced by: '<S160>/IsaGamma'

  1.4F,

  // Computed Parameter: Baroa_Value_p
  //  Referenced by: '<S157>/Baroa'

  0.0065F,

  // Computed Parameter: Gain2_Gain_e
  //  Referenced by: '<S160>/Gain2'

  0.5F,

  // Computed Parameter: Bias2_Bias_lw
  //  Referenced by: '<S160>/Bias2'

  1.0F,

  // Computed Parameter: IsaGamma_Value_j
  //  Referenced by: '<S157>/IsaGamma'

  1.4F,

  // Computed Parameter: Bias_Bias_h
  //  Referenced by: '<S157>/Bias'

  -1.0F,

  // Computed Parameter: Constant_Value_is
  //  Referenced by: '<S170>/Constant'

  0.0F,

  // Computed Parameter: IdentityMatrix_IDMatrixData_p4
  //  Referenced by: '<S172>/Identity Matrix'

  1.0F,

  // Computed Parameter: Constant_Value_px
  //  Referenced by: '<S167>/Constant'

  { 1.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 1.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F,
    0.0F, 1.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 1.0F, 0.0F, 0.0F, 0.0F, 0.0F,
    0.0F, 0.0F, 1.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 1.0F },

  // Computed Parameter: Constant3_Value_g
  //  Referenced by: '<S164>/Constant3'

  { 0.0F, 0.0F },

  // Computed Parameter: Baroa_Value_h
  //  Referenced by: '<S164>/Baroa'

  0.0065F,

  // Computed Parameter: BarogR_Value_k
  //  Referenced by: '<S164>/Barog//R'

  0.0341617726F,

  // Computed Parameter: Constant2_Value_l1
  //  Referenced by: '<S164>/Constant2'

  1.0F,

  // Computed Parameter: Constant1_Value_l
  //  Referenced by: '<S164>/Constant1'

  -1.0F,

  // Computed Parameter: Constant9_Value_b
  //  Referenced by: '<S164>/Constant9'

  { 0.0F, 0.0F, 0.0F },

  // Computed Parameter: BaroSigma_Value
  //  Referenced by: '<S101>/BaroSigma'

  31.0211F,

  // Computed Parameter: NASCondLim_Value_i1
  //  Referenced by: '<S165>/NASCondLim'

  1.0E-15F,

  // Computed Parameter: Gain_Gain_e5
  //  Referenced by: '<S166>/Gain'

  -1.0F,

  // Computed Parameter: RAir1_Value_ba
  //  Referenced by: '<S173>/RAir1'

  0.0065F,

  // Computed Parameter: LocalGravity_Value_dq
  //  Referenced by: '<S173>/LocalGravity'

  9.80623436F,

  // Computed Parameter: RAir_Value_oc
  //  Referenced by: '<S173>/RAir'

  287.052856F,

  // Computed Parameter: Constant_Value_m
  //  Referenced by: '<S183>/Constant'

  0.0F,

  // Computed Parameter: IdentityMatrix_IDMatrixData_iu
  //  Referenced by: '<S185>/Identity Matrix'

  1.0F,

  // Computed Parameter: Constant_Value_o
  //  Referenced by: '<S180>/Constant'

  { 1.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 1.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F,
    0.0F, 1.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 1.0F, 0.0F, 0.0F, 0.0F, 0.0F,
    0.0F, 0.0F, 1.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 1.0F },

  // Computed Parameter: Constant3_Value_gt
  //  Referenced by: '<S177>/Constant3'

  { 0.0F, 0.0F },

  // Computed Parameter: Baroa_Value_n
  //  Referenced by: '<S177>/Baroa'

  0.0065F,

  // Computed Parameter: BarogR_Value_h
  //  Referenced by: '<S177>/Barog//R'

  0.0341617726F,

  // Computed Parameter: Constant2_Value_o
  //  Referenced by: '<S177>/Constant2'

  1.0F,

  // Computed Parameter: Constant1_Value_c
  //  Referenced by: '<S177>/Constant1'

  -1.0F,

  // Computed Parameter: Constant9_Value_g
  //  Referenced by: '<S177>/Constant9'

  { 0.0F, 0.0F, 0.0F },

  // Computed Parameter: PitotSigma_Value
  //  Referenced by: '<S102>/PitotSigma'

  35.4413F,

  // Computed Parameter: NASCondLim_Value_ii
  //  Referenced by: '<S178>/NASCondLim'

  1.0E-15F,

  // Computed Parameter: Gain_Gain_n
  //  Referenced by: '<S179>/Gain'

  -1.0F,

  // Computed Parameter: RAir1_Value_g
  //  Referenced by: '<S186>/RAir1'

  0.0065F,

  // Computed Parameter: LocalGravity_Value_g
  //  Referenced by: '<S186>/LocalGravity'

  9.80623436F,

  // Computed Parameter: RAir_Value_c
  //  Referenced by: '<S186>/RAir'

  287.052856F,

  // Computed Parameter: Gain_Gain_n3
  //  Referenced by: '<S212>/Gain'

  0.0F,

  // Computed Parameter: Constant_Value_p2
  //  Referenced by: '<S213>/Constant'

  0.0F,

  // Computed Parameter: IdentityMatrix_IDMatrixData_m
  //  Referenced by: '<S215>/Identity Matrix'

  { 1.0F, 0.0F, 0.0F, 0.0F, 0.0F, 1.0F, 0.0F, 0.0F, 0.0F, 0.0F, 1.0F, 0.0F, 0.0F,
    0.0F, 0.0F, 1.0F },

  // Computed Parameter: Constant_Value_a
  //  Referenced by: '<S209>/Constant'

  { 1.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 1.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F,
    0.0F, 1.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 1.0F, 0.0F, 0.0F, 0.0F, 0.0F,
    0.0F, 0.0F, 1.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 1.0F },

  // Computed Parameter: GPS1a_Value
  //  Referenced by: '<S206>/GPS1//a'

  8.99823135E-6F,

  // Computed Parameter: Constant1_Value_i
  //  Referenced by: '<S206>/Constant1'

  0.0F,

  // Computed Parameter: GPS1a1_Gain
  //  Referenced by: '<S206>/GPS1//a 1'

  8.99823135E-6F,

  // Computed Parameter: GPSlat0_Bias
  //  Referenced by: '<S206>/GPSlat0'

  45.501091F,

  // Computed Parameter: Gain1_Gain_ke
  //  Referenced by: '<S210>/Gain1'

  0.0174532924F,

  // Computed Parameter: GPSb1_Gain
  //  Referenced by: '<S206>/GPSb1'

  8.07647352E-11F,

  // Computed Parameter: GPSb_Gain
  //  Referenced by: '<S206>/GPSb'

  111412.875F,

  // Computed Parameter: Constant2_Value_g
  //  Referenced by: '<S206>/Constant2'

  1.0F,

  // Computed Parameter: Constant5_Value_e
  //  Referenced by: '<S206>/Constant5'

  { 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F },

  // Computed Parameter: Constant6_Value_k
  //  Referenced by: '<S206>/Constant6'

  { 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 1.0F, 0.0F, 0.0F, 1.0F, 0.0F, 0.0F },

  // Computed Parameter: GPSSigma_Value
  //  Referenced by: '<S202>/GPSSigma'

  { 0.0447F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0447F, 0.0F, 0.0F, 0.0F, 0.0F, 0.1F, 0.0F,
    0.0F, 0.0F, 0.0F, 0.1F },

  // Computed Parameter: NASCondLim_Value_d
  //  Referenced by: '<S207>/NASCondLim'

  1.0E-15F,

  // Computed Parameter: GPS1a_Gain
  //  Referenced by: '<S208>/GPS1//a'

  8.99823135E-6F,

  // Computed Parameter: GPSlat0_Bias_g
  //  Referenced by: '<S208>/GPSlat0'

  45.501091F,

  // Computed Parameter: Gain1_Gain_af
  //  Referenced by: '<S216>/Gain1'

  0.0174532924F,

  // Computed Parameter: GPS1b_Gain
  //  Referenced by: '<S208>/GPS1//b'

  8.97562313E-6F,

  // Computed Parameter: GPSlon0_Bias
  //  Referenced by: '<S208>/GPSlon0'

  9.15637875F,

  // Computed Parameter: AirR_Value_i
  //  Referenced by: '<S200>/AirR'

  287.052856F,

  // Computed Parameter: IsaGamma_Value_b
  //  Referenced by: '<S200>/IsaGamma'

  1.4F,

  // Computed Parameter: Baroa_Value_f
  //  Referenced by: '<S198>/Baroa'

  0.0065F,

  // Computed Parameter: Merge_InitialOutput_f
  //  Referenced by: '<S96>/Merge'

  0.0F,

  // Computed Parameter: Merge1_InitialOutput
  //  Referenced by: '<S96>/Merge1'

  0.0F,

  // Computed Parameter: Gain1_Gain_e
  //  Referenced by: '<S5>/Gain1'

  1.0E-6F,

  // Computed Parameter: Constant_Value_p5
  //  Referenced by: '<S5>/Constant'

  0.01F,

  // Computed Parameter: Constant1_Value_hg
  //  Referenced by: '<S5>/Constant1'

  { 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F,
    0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 1.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 1.0F,
    0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 1.0F, 0.0F, 0.0F, 0.0F },

  // Computed Parameter: Bias2_Bias_m
  //  Referenced by: '<S5>/Bias2'

  { 1.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 1.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F,
    0.0F, 1.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 1.0F, 0.0F, 0.0F, 0.0F, 0.0F,
    0.0F, 0.0F, 1.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 1.0F },

  // Computed Parameter: LinearQ_Bias
  //  Referenced by: '<S5>/Linear Q'

  { 0.1F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.1F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F,
    0.0F, 0.1F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.1F, 0.0F, 0.0F, 0.0F, 0.0F,
    0.0F, 0.0F, 0.1F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.1F },

  // Computed Parameter: Gain_Gain_ie
  //  Referenced by: '<S225>/Gain'

  2.0F,

  // Computed Parameter: Gain_Gain_ob
  //  Referenced by: '<S228>/Gain'

  2.0F,

  // Computed Parameter: Gain_Gain_hm
  //  Referenced by: '<S223>/Gain'

  2.0F,

  // Computed Parameter: Gain_Gain_bt
  //  Referenced by: '<S229>/Gain'

  2.0F,

  // Computed Parameter: Gain_Gain_p3
  //  Referenced by: '<S224>/Gain'

  2.0F,

  // Computed Parameter: Gain_Gain_a
  //  Referenced by: '<S227>/Gain'

  2.0F,

  // Computed Parameter: LocalGravity_Value_jy
  //  Referenced by: '<S220>/Local Gravity'

  { 0.0F, 0.0F, 9.80623436F },

  // Computed Parameter: AccPropagationFlag_Gain
  //  Referenced by: '<S5>/AccPropagationFlag'

  1.0F,

  // Computed Parameter: Zero_Value
  //  Referenced by: '<S6>/Zero'

  { 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F,
    0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F,
    0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F,
    0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F,
    0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F,
    0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F,
    0.0F, 0.0F, 0.0F },

  // Computed Parameter: UnitDelay6_InitialCondition
  //  Referenced by: '<S7>/Unit Delay6'

  0.0F,

  // Computed Parameter: UnitDelay7_InitialCondition
  //  Referenced by: '<S7>/Unit Delay7'

  0.0F,

  // Computed Parameter: UnitDelay4_InitialCondition
  //  Referenced by: '<S7>/Unit Delay4'

  0.0F,

  // Computed Parameter: UnitDelay8_InitialCondition
  //  Referenced by: '<S7>/Unit Delay8'

  0.0F,

  // Computed Parameter: UnitDelay_InitialCondition
  //  Referenced by: '<S7>/Unit Delay'

  0.0F,

  // Computed Parameter: UnitDelay1_InitialCondition
  //  Referenced by: '<S7>/Unit Delay1'

  0.0F,

  // Computed Parameter: UnitDelay2_InitialCondition
  //  Referenced by: '<S7>/Unit Delay2'

  0.0F,

  // Computed Parameter: UnitDelay3_InitialCondition
  //  Referenced by: '<S7>/Unit Delay3'

  0.0F,

  // Expression: magLeft
  //  Referenced by: '<S74>/Constant'

  true,

  // Expression: flagAccCorrectionANAS
  //  Referenced by: '<S11>/AccCorrFlag'

  true,

  // Expression: flagMagnetometerANAS
  //  Referenced by: '<S51>/MagCorrFlag'

  true,

  // Expression: flagGPSANAS
  //  Referenced by: '<S203>/GPSCorrectionFlag'

  true,

  // Expression: flagPitotANAS
  //  Referenced by: '<S103>/PitotFlag'

  false,

  // Expression: flagBaroANAS
  //  Referenced by: '<S103>/BaroFlag'

  true,

  // Expression: flagUsePitotStatic
  //  Referenced by: '<S103>/StaticPitotFlag1'

  false,

  // Expression: flagVaroANAS
  //  Referenced by: '<S103>/VaroFlag'

  false,

  // Expression: flagUsePitotStatic
  //  Referenced by: '<S103>/StaticPitotFlag'

  false,

  // Computed Parameter: UnitDelay3_InitialCondition_c
  //  Referenced by: '<S3>/Unit Delay3'

  0U,

  // Computed Parameter: Switch_Threshold
  //  Referenced by: '<S3>/Switch'

  0U,

  // Computed Parameter: Bias1_Bias_k
  //  Referenced by: '<S3>/Bias1'

  1U,

  // Computed Parameter: Saturation2_UpperSat
  //  Referenced by: '<S3>/Saturation2'

  1U,

  // Computed Parameter: Saturation2_LowerSat
  //  Referenced by: '<S3>/Saturation2'

  0U,

  // Computed Parameter: Zero_Value_c
  //  Referenced by: '<S193>/Zero'

  3U,

  // Computed Parameter: Zero_Value_e
  //  Referenced by: '<S190>/Zero'

  2U,

  // Computed Parameter: Zero_Value_i
  //  Referenced by: '<S191>/Zero'

  0U,

  // Computed Parameter: Zero_Value_o
  //  Referenced by: '<S195>/Zero'

  4U,

  // Computed Parameter: Zero_Value_d
  //  Referenced by: '<S192>/Zero'

  1U,

  // Computed Parameter: Zero_Value_dv
  //  Referenced by: '<S194>/Zero'

  5U,

  // Computed Parameter: Merge_InitialOutput_k
  //  Referenced by: '<S103>/Merge'

  0U,

  // Computed Parameter: UnitDelay3_InitialCondition_l
  //  Referenced by: '<S5>/Unit Delay3'

  0U,

  // Computed Parameter: Bias1_Bias_o
  //  Referenced by: '<S5>/Bias1'

  1U,

  // Computed Parameter: Saturation2_UpperSat_c
  //  Referenced by: '<S5>/Saturation2'

  1U,

  // Computed Parameter: Saturation2_LowerSat_n
  //  Referenced by: '<S5>/Saturation2'

  0U,

  // Start of '<S178>/No correction'
  {
    // Computed Parameter: Gain_Gain
    //  Referenced by: '<S182>/Gain'

    0.0F
  }
  ,

  // End of '<S178>/No correction'

  // Start of '<S165>/No correction'
  {
    // Computed Parameter: Gain_Gain
    //  Referenced by: '<S169>/Gain'

    0.0F
  }
  ,

  // End of '<S165>/No correction'

  // Start of '<S145>/No correction'
  {
    // Computed Parameter: Gain_Gain
    //  Referenced by: '<S153>/Gain'

    0.0F
  }
  ,

  // End of '<S145>/No correction'

  // Start of '<S125>/No correction'
  {
    // Computed Parameter: Gain_Gain
    //  Referenced by: '<S133>/Gain'

    0.0F
  }
  ,

  // End of '<S125>/No correction'

  // Start of '<S107>/No correction'
  {
    // Computed Parameter: Gain_Gain
    //  Referenced by: '<S112>/Gain'

    0.0F
  }
  ,

  // End of '<S107>/No correction'

  // Start of '<S54>/No correction'
  {
    // Computed Parameter: Gain_Gain
    //  Referenced by: '<S59>/Gain'

    0.0F
  }
  ,

  // End of '<S54>/No correction'

  // Start of '<S14>/No correction'
  {
    // Computed Parameter: Gain_Gain
    //  Referenced by: '<S19>/Gain'

    0.0F
  }
  // End of '<S14>/No correction'
};

//
// File trailer for generated code.
//
// [EOF]
//
