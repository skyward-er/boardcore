//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// File: ANAS0.cpp
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
#include "ANAS0_private.h"
#include <stdint.h>
#include <cmath>
#include <cstring>
#include <stdbool.h>

static void rate_scheduler(ANAS0::RT_MODEL_ANAS0_T *const ANAS0_M);

//
//         This function updates active task flag for each subrate.
//         The function is called at model base rate, hence the
//         generated code self-manages all its subrates.
//
static void rate_scheduler(ANAS0::RT_MODEL_ANAS0_T *const ANAS0_M)
{
  // Compute which subrates run during the next base time step.  Subrates
  //  are an integer multiple of the base rate counter.  Therefore, the subtask
  //  counter is reset when it reaches its limit (zero means run).

  (ANAS0_M->Timing.TaskCounters.TID[1])++;
  if ((ANAS0_M->Timing.TaskCounters.TID[1]) > 1) {// Sample time: [0.02s, 0.0s]
    ANAS0_M->Timing.TaskCounters.TID[1] = 0;
  }

  (ANAS0_M->Timing.TaskCounters.TID[2])++;
  if ((ANAS0_M->Timing.TaskCounters.TID[2]) > 3) {// Sample time: [0.04s, 0.0s]
    ANAS0_M->Timing.TaskCounters.TID[2] = 0;
  }

  (ANAS0_M->Timing.TaskCounters.TID[3])++;
  if ((ANAS0_M->Timing.TaskCounters.TID[3]) > 9) {// Sample time: [0.1s, 0.0s]
    ANAS0_M->Timing.TaskCounters.TID[3] = 0;
  }
}

void rt_mrdividef3x3(const float u0[9], const float u1[9], float y[9])
{
  float A[9];
  float a21;
  float maxval;
  float y_tmp_0;
  float y_tmp_2;
  float y_tmp_3;
  float y_tmp_4;
  int32_t r1;
  int32_t r2;
  int32_t r3;
  int32_t rtemp;
  int32_t y_tmp;
  int32_t y_tmp_1;
  for (r1 = 0; r1 < 9; r1++) {
    A[r1] = u1[r1];
  }

  r1 = 0;
  r2 = 1;
  r3 = 2;
  maxval = std::abs(u1[0]);
  a21 = std::abs(u1[1]);
  if (a21 > maxval) {
    maxval = a21;
    r1 = 1;
    r2 = 0;
  }

  if (std::abs(u1[2]) > maxval) {
    r1 = 2;
    r2 = 1;
    r3 = 0;
  }

  A[r2] = u1[r2] / u1[r1];
  A[r3] /= A[r1];
  A[r2 + 3] -= A[r1 + 3] * A[r2];
  A[r3 + 3] -= A[r1 + 3] * A[r3];
  A[r2 + 6] -= A[r1 + 6] * A[r2];
  A[r3 + 6] -= A[r1 + 6] * A[r3];
  if (std::abs(A[r3 + 3]) > std::abs(A[r2 + 3])) {
    rtemp = r2 + 1;
    r2 = r3;
    r3 = rtemp - 1;
  }

  A[r3 + 3] /= A[r2 + 3];
  A[r3 + 6] -= A[r3 + 3] * A[r2 + 6];
  y[3 * r1] = u0[0] / A[r1];
  maxval = A[r1 + 3];
  y[3 * r2] = u0[3] - y[3 * r1] * maxval;
  a21 = A[r1 + 6];
  y[3 * r3] = u0[6] - y[3 * r1] * a21;
  y_tmp_0 = A[r2 + 3];
  y[3 * r2] /= y_tmp_0;
  y_tmp_2 = A[r2 + 6];
  y[3 * r3] -= y[3 * r2] * y_tmp_2;
  y_tmp_3 = A[r3 + 6];
  y[3 * r3] /= y_tmp_3;
  y_tmp_4 = A[r3 + 3];
  y[3 * r2] -= y[3 * r3] * y_tmp_4;
  y[3 * r1] -= y[3 * r3] * A[r3];
  y[3 * r1] -= y[3 * r2] * A[r2];
  rtemp = 3 * r1 + 1;
  y[rtemp] = u0[1] / A[r1];
  y_tmp = 3 * r2 + 1;
  y[y_tmp] = u0[4] - y[rtemp] * maxval;
  y_tmp_1 = 3 * r3 + 1;
  y[y_tmp_1] = u0[7] - y[rtemp] * a21;
  y[y_tmp] /= y_tmp_0;
  y[y_tmp_1] -= y[y_tmp] * y_tmp_2;
  y[y_tmp_1] /= y_tmp_3;
  y[y_tmp] -= y[y_tmp_1] * y_tmp_4;
  y[rtemp] -= y[y_tmp_1] * A[r3];
  y[rtemp] -= y[y_tmp] * A[r2];
  rtemp = 3 * r1 + 2;
  y[rtemp] = u0[2] / A[r1];
  y_tmp = 3 * r2 + 2;
  y[y_tmp] = u0[5] - y[rtemp] * maxval;
  y_tmp_1 = 3 * r3 + 2;
  y[y_tmp_1] = u0[8] - y[rtemp] * a21;
  y[y_tmp] /= y_tmp_0;
  y[y_tmp_1] -= y[y_tmp] * y_tmp_2;
  y[y_tmp_1] /= y_tmp_3;
  y[y_tmp] -= y[y_tmp_1] * y_tmp_4;
  y[rtemp] -= y[y_tmp_1] * A[r3];
  y[rtemp] -= y[y_tmp] * A[r2];
}

//
// Output and update for action system:
//    '<S14>/Correction'
//    '<S54>/Correction'
//
void ANAS0::ANAS0_Correction(float rty_S[9], const float rtu_PH_prev[9], float
  rty_K[9])
{
  // Product: '<S18>/Matrix Divide' incorporates:
  //   SignalConversion generated from: '<S18>/Sprev'

  rt_mrdividef3x3(rtu_PH_prev, rty_S, rty_K);
}

//
// Output and update for action system:
//    '<S14>/No correction'
//    '<S54>/No correction'
//
void ANAS0::ANAS0_Nocorrection(const float rtu_PH_prev[9], float rty_K[9],
  P_Nocorrection_ANAS0_T *localP)
{
  // Gain: '<S19>/Gain'
  for (int32_t i{0}; i < 9; i++) {
    rty_K[i] = localP->Gain_Gain * rtu_PH_prev[i];
  }

  // End of Gain: '<S19>/Gain'
}

void rt_mrdivide_U1f6x2_U2f2x2_Yf6x2(const float u0[12], const float u1[4],
  float y[12])
{
  float a21;
  float a22;
  float a22_tmp;
  int32_t r1;
  int32_t r2;
  if (std::abs(u1[1]) > std::abs(u1[0])) {
    r1 = 1;
    r2 = 0;
  } else {
    r1 = 0;
    r2 = 1;
  }

  a21 = u1[r2] / u1[r1];
  a22_tmp = u1[r1 + 2];
  a22 = u1[r2 + 2] - a22_tmp * a21;
  for (int32_t k{0}; k < 6; k++) {
    int32_t y_tmp;
    int32_t y_tmp_0;
    y_tmp = 6 * r1 + k;
    y[y_tmp] = u0[k] / u1[r1];
    y_tmp_0 = 6 * r2 + k;
    y[y_tmp_0] = (u0[k + 6] - y[y_tmp] * a22_tmp) / a22;
    y[y_tmp] -= y[y_tmp_0] * a21;
  }
}

//
// Output and update for action system:
//    '<S107>/Correction'
//    '<S125>/Correction'
//    '<S145>/Correction'
//
void ANAS0::ANAS0_Correction_i(float rty_S[4], const float rtu_PH_prev[12],
  float rty_K[12])
{
  // Product: '<S111>/Matrix Divide' incorporates:
  //   SignalConversion generated from: '<S111>/Sprev'

  rt_mrdivide_U1f6x2_U2f2x2_Yf6x2(rtu_PH_prev, rty_S, rty_K);
}

//
// Output and update for action system:
//    '<S107>/No correction'
//    '<S125>/No correction'
//    '<S145>/No correction'
//
void ANAS0::ANAS0_Nocorrection_o(const float rtu_PH_prev[12], float rty_K[12],
  P_Nocorrection_ANAS0_f_T *localP)
{
  // Gain: '<S112>/Gain'
  for (int32_t i{0}; i < 12; i++) {
    rty_K[i] = localP->Gain_Gain * rtu_PH_prev[i];
  }

  // End of Gain: '<S112>/Gain'
}

//
// Output and update for action system:
//    '<S165>/Correction'
//    '<S178>/Correction'
//
void ANAS0::ANAS0_Correction_m(float rtu_Sprev, const float rtu_PH_prev[6],
  float rty_K[6], float *rty_S)
{
  // Product: '<S168>/Matrix Divide' incorporates:
  //   SignalConversion generated from: '<S168>/Sprev'

  for (int32_t i{0}; i < 6; i++) {
    rty_K[i] = rtu_PH_prev[i] / rtu_Sprev;
  }

  // End of Product: '<S168>/Matrix Divide'

  // SignalConversion generated from: '<S168>/S'
  *rty_S = rtu_Sprev;
}

//
// Output and update for action system:
//    '<S165>/No correction'
//    '<S178>/No correction'
//
void ANAS0::ANAS0_Nocorrection_d(float rtu_R, const float rtu_PH_prev[6], float
  rty_K[6], float *rty_S, P_Nocorrection_ANAS0_i_T *localP)
{
  // Gain: '<S169>/Gain'
  for (int32_t i{0}; i < 6; i++) {
    rty_K[i] = localP->Gain_Gain * rtu_PH_prev[i];
  }

  // End of Gain: '<S169>/Gain'

  // SignalConversion generated from: '<S169>/R'
  *rty_S = rtu_R;
}

void LUf_boolfloatint32_t(float outU[], float outP[], int32_t N, bool outS[])
{
  // S-Function (sdsplu2): '<S22>/LU Factorization'
  // initialize status output
  outS[0U] = false;

  // initialize row-pivot indices
  for (int32_t k{0}; k < N; k++) {
    outP[k] = static_cast<float>(k + 1);
  }

  for (int32_t k{0}; k < N; k++) {
    float mTmp1;
    int32_t idx1;
    int32_t idx1_tmp;
    int32_t mTmp1_tmp;
    int32_t p;
    int32_t r;
    int32_t tmp;
    p = k;

    // Scan the lower triangular part of this column only.
    // Record row of largest value
    idx1_tmp = k * N;
    mTmp1_tmp = idx1_tmp + k;
    mTmp1 = outU[mTmp1_tmp];
    if (mTmp1 < 0.0F) {
      mTmp1 = -mTmp1;
    }

    for (r = k + 1; r < N; r++) {
      float mTmp2;
      mTmp2 = outU[idx1_tmp + r];
      if (mTmp2 < 0.0F) {
        mTmp2 = -mTmp2;
      }

      if (mTmp2 > mTmp1) {
        p = r;
        mTmp1 = mTmp2;
      }
    }

    // swap rows if required
    if (p != k) {
      for (int32_t c{0}; c < N; c++) {
        idx1 = c * N;
        r = idx1 + p;
        mTmp1 = outU[r];
        tmp = idx1 + k;
        outU[r] = outU[tmp];
        outU[tmp] = mTmp1;
      }

      // swap pivot row indices
      mTmp1 = outP[p];
      outP[p] = outP[k];
      outP[k] = mTmp1;
    }

    if (outU[mTmp1_tmp] == 0.0F) {
      outS[0U] = true;
    } else {
      for (r = k + 1; r < N; r++) {
        tmp = idx1_tmp + r;
        outU[tmp] /= outU[mTmp1_tmp];
      }

      for (int32_t c{k + 1}; c < N; c++) {
        idx1 = c * N;
        for (r = k + 1; r < N; r++) {
          tmp = idx1 + r;
          outU[tmp] -= outU[idx1_tmp + r] * outU[idx1 + k];
        }
      }
    }
  }

  // End of S-Function (sdsplu2): '<S22>/LU Factorization'
}

void rt_mrdivide_U1f6x4_U2f4x4_Yf6x4(float u0[24], const float u1[16])
{
  float x[16];
  float smax;
  int32_t c;
  int32_t jA;
  int32_t jBcol;
  int32_t jj;
  int32_t kBcol;
  int8_t ipiv[4];
  std::memcpy(&x[0], &u1[0], sizeof(float) << 4U);
  ipiv[0] = 1;
  ipiv[1] = 2;
  ipiv[2] = 3;
  ipiv[3] = 4;
  for (int32_t j{0}; j < 3; j++) {
    int32_t iy;
    c = j * 5 + 2;
    jj = j * 5;
    jBcol = 4 - j;
    iy = 1;
    smax = std::abs(x[jj]);
    for (jA = 2; jA <= jBcol; jA++) {
      float s;
      s = std::abs(x[(c + jA) - 3]);
      if (s > smax) {
        iy = jA;
        smax = s;
      }
    }

    if (x[(c + iy) - 3] != 0.0F) {
      if (iy - 1 != 0) {
        jA = j + iy;
        ipiv[j] = static_cast<int8_t>(jA);
        smax = x[j];
        x[j] = x[jA - 1];
        x[jA - 1] = smax;
        smax = x[j + 4];
        x[j + 4] = x[jA + 3];
        x[jA + 3] = smax;
        smax = x[j + 8];
        x[j + 8] = x[jA + 7];
        x[jA + 7] = smax;
        smax = x[j + 12];
        x[j + 12] = x[jA + 11];
        x[jA + 11] = smax;
      }

      iy = c - j;
      for (int32_t ix{c}; ix <= iy + 2; ix++) {
        x[ix - 1] /= x[jj];
      }
    }

    jBcol = 2 - j;
    jA = jj;
    jj += 4;
    for (int32_t ix{0}; ix <= jBcol; ix++) {
      smax = x[(ix << 2) + jj];
      if (smax != 0.0F) {
        iy = jA + 6;
        kBcol = (jA - j) + 8;
        for (int32_t ijA{iy}; ijA <= kBcol; ijA++) {
          x[ijA - 1] += x[((c + ijA) - jA) - 7] * -smax;
        }
      }

      jA += 4;
    }
  }

  for (int32_t j{0}; j < 4; j++) {
    jBcol = 6 * j - 1;
    jj = (j << 2) - 1;
    for (jA = 0; jA < j; jA++) {
      kBcol = 6 * jA - 1;
      smax = x[(jA + jj) + 1];
      if (smax != 0.0F) {
        for (int32_t ix{0}; ix < 6; ix++) {
          c = (ix + jBcol) + 1;
          u0[c] -= u0[(ix + kBcol) + 1] * smax;
        }
      }
    }

    smax = 1.0F / x[(j + jj) + 1];
    for (int32_t ix{0}; ix < 6; ix++) {
      c = (ix + jBcol) + 1;
      u0[c] *= smax;
    }
  }

  for (int32_t j{3}; j >= 0; j--) {
    jBcol = 6 * j - 1;
    jj = (j << 2) - 1;
    for (jA = j + 2; jA < 5; jA++) {
      kBcol = (jA - 1) * 6 - 1;
      smax = x[jA + jj];
      if (smax != 0.0F) {
        for (int32_t ix{0}; ix < 6; ix++) {
          c = (ix + jBcol) + 1;
          u0[c] -= u0[(ix + kBcol) + 1] * smax;
        }
      }
    }
  }

  for (int32_t j{2}; j >= 0; j--) {
    jj = ipiv[j];
    if (j + 1 != jj) {
      for (int32_t ix{0}; ix < 6; ix++) {
        jA = 6 * j + ix;
        smax = u0[jA];
        c = (jj - 1) * 6 + ix;
        u0[jA] = u0[c];
        u0[c] = smax;
      }
    }
  }
}

// Model step function
void ANAS0::step()
{
  float rtb_Assignment1[81];
  float rtb_LinearQ[36];
  float rtb_LinearQ_0[36];
  float rtb_Merge_p[36];
  float rtb_Selector[36];
  float rtb_Transpose_aq[36];
  float rtb_Transpose_e[36];
  float rtb_Merge_gj[24];
  float rtb_S_tmp[24];
  float rtb_Transpose1[24];
  float rtb_BackwardSubstitution[16];
  float rtb_LUFactorization_o1[16];
  float rtb_S[16];
  float rtb_Square_f[16];
  float rtb_MatrixConcatenate[12];
  float rtb_S_ce_tmp[12];
  float rtb_Transpose1_j[12];
  float rtb_BackwardSubstitution_e[9];
  float rtb_ExtractDiagonal[9];
  float rtb_LUFactorization_o1_c[9];
  float rtb_MatrixMultiply2[9];
  float rtb_MatrixMultiply_i[9];
  float rtb_S_jt[9];
  float rtb_Selector2[9];
  float rtb_Sum2[9];
  float rtb_Transpose_g[9];
  float rtb_VectorConcatenate_i[9];
  float rtb_MatrixMultiply_f[6];
  float rtb_Switch2[6];
  float rtb_Transpose_k[6];
  float rtb_VectorConcatenate[6];
  float rtb_VectorConcatenate_o[6];
  float rtb_BackwardSubstitution_f1[4];
  float rtb_BackwardSubstitution_j[4];
  float rtb_BackwardSubstitution_n[4];
  float rtb_LUFactorization_o1_if[4];
  float rtb_LUFactorization_o1_iq[4];
  float rtb_LUFactorization_o1_l[4];
  float rtb_LUFactorization_o2[4];
  float rtb_Switch3[4];
  float rtb_res_GPS[4];
  float rtb_LUFactorization_o2_m[3];
  float rtb_LUFactorization_o2_o[3];
  float rtb_Transpose_c[3];
  float rtb_LUFactorization_o2_b[2];
  float rtb_LUFactorization_o2_l[2];
  float rtb_LUFactorization_o2_n[2];
  float rtb_Add;
  float rtb_Bias3;
  float rtb_Divide1_p;
  float rtb_Divide5_a;
  float rtb_ExtractDiagonal_tmp;
  float rtb_ExtractDiagonal_tmp_0;
  float rtb_ExtractDiagonal_tmp_1;
  float rtb_ExtractDiagonal_tmp_2;
  float rtb_ExtractDiagonal_tmp_3;
  float rtb_ExtractDiagonal_tmp_4;
  float rtb_ExtractDiagonal_tmp_5;
  float rtb_ExtractDiagonal_tmp_6;
  float rtb_LUFactorization_o2_j;
  float rtb_LUFactorization_o2_j5;
  float rtb_Matrix1Norm1;
  float rtb_Matrix1Norm1_a;
  float rtb_Matrix1Norm1_ax;
  float rtb_Matrix1Norm1_h;
  float rtb_Matrix1Norm1_k;
  float rtb_Matrix1Norm1_p;
  float rtb_Matrix1Norm2;
  float rtb_Matrix1Norm2_c;
  float rtb_Matrix1Norm2_dg;
  float rtb_Matrix1Norm2_j;
  float rtb_Matrix1Norm2_k;
  float rtb_Matrix1Norm2_l;
  float rtb_Matrix1Norm2_m;
  float rtb_Matrix1Norm2_om;
  float rtb_Power;
  float rtb_Power2;
  float rtb_Product1_j3;
  float rtb_Product2_k0;
  float rtb_Product3_jl;
  int32_t i;
  int32_t i_0;
  int32_t i_1;
  int32_t k;
  int32_t rtb_LinearQ_tmp;
  int32_t rtb_S_tmp_0;
  uint8_t u0;
  bool rtb_AND;
  bool rtb_AND_b;
  bool rtb_AND_i;
  bool rtb_LUFactorization_o3;
  bool rtb_LUFactorization_o3_f_tmp;
  bool rtb_LUFactorization_o3_f_tmp_0;
  bool rtb_LUFactorization_o3_f_tmp_1;
  bool rtb_LUFactorization_o3_f_tmp_2;
  bool rtb_LUFactorization_o3_f_tmp_3;
  bool rtb_LUFactorization_o3_i;
  bool rtb_LUFactorization_o3_j;
  bool rtb_do_barometer_correcion;
  bool rtb_main_s_w_pitot_d;
  bool rtb_variometer;
  bool tmp;

  // Outputs for Atomic SubSystem: '<Root>/ANAS'
  // Switch: '<S7>/Switch3' incorporates:
  //   Inport: '<Root>/ANAS Reference'
  //   RelationalOperator: '<S7>/Relational Operator3'
  //   UnitDelay: '<S7>/Unit Delay6'
  //   UnitDelay: '<S7>/Unit Delay7'

  if (ANAS0_DW.UnitDelay7_DSTATE[0] ==
      ANAS0_U.ANASReference_f.InitialQuaternion[0]) {
    rtb_Switch3[0] = ANAS0_DW.UnitDelay6_DSTATE[0];
  } else {
    rtb_Switch3[0] = ANAS0_U.ANASReference_f.InitialQuaternion[0];
  }

  if (ANAS0_DW.UnitDelay7_DSTATE[1] ==
      ANAS0_U.ANASReference_f.InitialQuaternion[1]) {
    rtb_Switch3[1] = ANAS0_DW.UnitDelay6_DSTATE[1];
  } else {
    rtb_Switch3[1] = ANAS0_U.ANASReference_f.InitialQuaternion[1];
  }

  if (ANAS0_DW.UnitDelay7_DSTATE[2] ==
      ANAS0_U.ANASReference_f.InitialQuaternion[2]) {
    rtb_Switch3[2] = ANAS0_DW.UnitDelay6_DSTATE[2];
  } else {
    rtb_Switch3[2] = ANAS0_U.ANASReference_f.InitialQuaternion[2];
  }

  if (ANAS0_DW.UnitDelay7_DSTATE[3] ==
      ANAS0_U.ANASReference_f.InitialQuaternion[3]) {
    rtb_Switch3[3] = ANAS0_DW.UnitDelay6_DSTATE[3];
  } else {
    rtb_Switch3[3] = ANAS0_U.ANASReference_f.InitialQuaternion[3];
  }

  // End of Switch: '<S7>/Switch3'

  // SignalConversion generated from: '<S7>/Vector Concatenate' incorporates:
  //   Inport: '<Root>/ANAS Reference'

  rtb_VectorConcatenate[0] = ANAS0_U.ANASReference_f.InitialPosition[0];

  // SignalConversion generated from: '<S7>/Vector Concatenate' incorporates:
  //   Inport: '<Root>/ANAS Reference'

  rtb_VectorConcatenate[3] = ANAS0_U.ANASReference_f.InitialVelocity[0];

  // SignalConversion generated from: '<S7>/Vector Concatenate' incorporates:
  //   Inport: '<Root>/ANAS Reference'

  rtb_VectorConcatenate[1] = ANAS0_U.ANASReference_f.InitialPosition[1];

  // SignalConversion generated from: '<S7>/Vector Concatenate' incorporates:
  //   Inport: '<Root>/ANAS Reference'

  rtb_VectorConcatenate[4] = ANAS0_U.ANASReference_f.InitialVelocity[1];

  // SignalConversion generated from: '<S7>/Vector Concatenate' incorporates:
  //   Inport: '<Root>/ANAS Reference'

  rtb_VectorConcatenate[2] = ANAS0_U.ANASReference_f.InitialPosition[2];

  // SignalConversion generated from: '<S7>/Vector Concatenate' incorporates:
  //   Inport: '<Root>/ANAS Reference'

  rtb_VectorConcatenate[5] = ANAS0_U.ANASReference_f.InitialVelocity[2];
  for (i = 0; i < 6; i++) {
    // RelationalOperator: '<S7>/Relational Operator2'
    rtb_Divide5_a = rtb_VectorConcatenate[i];

    // Switch: '<S7>/Switch2' incorporates:
    //   RelationalOperator: '<S7>/Relational Operator2'
    //   UnitDelay: '<S7>/Unit Delay4'
    //   UnitDelay: '<S7>/Unit Delay8'

    if (ANAS0_DW.UnitDelay8_DSTATE[i] == rtb_Divide5_a) {
      rtb_Switch2[i] = ANAS0_DW.UnitDelay4_DSTATE[i];
    } else {
      rtb_Switch2[i] = rtb_Divide5_a;
    }

    // End of Switch: '<S7>/Switch2'

    // Selector: '<S7>/Selector' incorporates:
    //   Inport: '<Root>/ANAS Reference'

    for (i_0 = 0; i_0 < 6; i_0++) {
      rtb_Selector[i_0 + 6 * i] = ANAS0_U.ANASReference_f.InitialCovariance[9 *
        i + i_0];
    }

    // End of Selector: '<S7>/Selector'
  }

  // Outputs for Atomic SubSystem: '<S1>/Linear States Prediction'
  // Switch: '<S5>/Switch' incorporates:
  //   Constant: '<S5>/Constant'
  //   DataTypeConversion: '<S5>/Cast To Single'
  //   Gain: '<S5>/Gain1'
  //   Inport: '<Root>/ANAS In'
  //   Sum: '<S219>/Diff'
  //   UnitDelay: '<S219>/UD'
  //   UnitDelay: '<S5>/Unit Delay3'
  //
  //  Block description for '<S219>/Diff':
  //
  //   Add in CPU
  //
  //  Block description for '<S219>/UD':
  //
  //   Store in Global RAM

  if (ANAS0_DW.UnitDelay3_DSTATE_h != 0) {
    rtb_Bias3 = static_cast<float>(ANAS0_U.ANASIn_c.AccTimestamp -
      ANAS0_DW.UD_DSTATE) * ANAS0_P.Gain1_Gain_e;
  } else {
    rtb_Bias3 = ANAS0_P.Constant_Value_p5;
  }

  // End of Switch: '<S5>/Switch'
  for (i = 0; i < 36; i++) {
    // Bias: '<S5>/Bias2' incorporates:
    //   Constant: '<S5>/Constant1'
    //   Math: '<S176>/Transpose'
    //   Product: '<S5>/Product2'

    rtb_Transpose_e[i] = rtb_Bias3 * ANAS0_P.Constant1_Value_hg[i] +
      ANAS0_P.Bias2_Bias_m[i];

    // RelationalOperator: '<S7>/Relational Operator' incorporates:
    //   Selector: '<S7>/Selector'
    //   UnitDelay: '<S7>/Unit Delay1'

    rtb_Divide5_a = rtb_Selector[i];

    // Switch: '<S7>/Switch' incorporates:
    //   RelationalOperator: '<S7>/Relational Operator'
    //   Selector: '<S7>/Selector'
    //   UnitDelay: '<S7>/Unit Delay1'

    if (ANAS0_DW.UnitDelay1_DSTATE[i] == rtb_Divide5_a) {
      // Product: '<S5>/Matrix Multiply' incorporates:
      //   UnitDelay: '<S7>/Unit Delay'

      rtb_LinearQ[i] = ANAS0_DW.UnitDelay_DSTATE[i];
    } else {
      // Product: '<S5>/Matrix Multiply'
      rtb_LinearQ[i] = rtb_Divide5_a;
    }

    // End of Switch: '<S7>/Switch'
  }

  // Product: '<S5>/Matrix Multiply' incorporates:
  //   Math: '<S176>/Transpose'
  //   Math: '<S5>/Transpose'

  for (i_0 = 0; i_0 < 6; i_0++) {
    k = 0;
    for (i = 0; i < 6; i++) {
      rtb_Power2 = 0.0F;
      rtb_LinearQ_tmp = 0;
      for (rtb_S_tmp_0 = 0; rtb_S_tmp_0 < 6; rtb_S_tmp_0++) {
        rtb_Power2 += rtb_LinearQ[rtb_LinearQ_tmp + i_0] *
          rtb_Transpose_e[rtb_LinearQ_tmp + i];
        rtb_LinearQ_tmp += 6;
      }

      rtb_LinearQ_0[k + i_0] = rtb_Power2;
      k += 6;
    }
  }

  // Bias: '<S5>/Linear Q' incorporates:
  //   Math: '<S176>/Transpose'
  //   Product: '<S5>/Matrix Multiply'

  for (i_0 = 0; i_0 < 6; i_0++) {
    k = 0;
    for (i = 0; i < 6; i++) {
      rtb_Power2 = 0.0F;
      rtb_LinearQ_tmp = 0;
      for (rtb_S_tmp_0 = 0; rtb_S_tmp_0 < 6; rtb_S_tmp_0++) {
        rtb_Power2 += rtb_Transpose_e[rtb_LinearQ_tmp + i_0] *
          rtb_LinearQ_0[rtb_S_tmp_0 + k];
        rtb_LinearQ_tmp += 6;
      }

      rtb_LinearQ_tmp = k + i_0;
      rtb_LinearQ[rtb_LinearQ_tmp] = ANAS0_P.LinearQ_Bias[rtb_LinearQ_tmp] +
        rtb_Power2;
      k += 6;
    }
  }

  // End of Bias: '<S5>/Linear Q'

  // Sqrt: '<S233>/sqrt' incorporates:
  //   Product: '<S234>/Product'
  //   Product: '<S234>/Product1'
  //   Product: '<S234>/Product2'
  //   Product: '<S234>/Product3'
  //   Sum: '<S234>/Sum'

  rtb_Power2 = std::sqrt(((rtb_Switch3[0] * rtb_Switch3[0] + rtb_Switch3[1] *
    rtb_Switch3[1]) + rtb_Switch3[2] * rtb_Switch3[2]) + rtb_Switch3[3] *
    rtb_Switch3[3]);

  // Product: '<S232>/Product'
  rtb_Add = rtb_Switch3[0] / rtb_Power2;

  // Product: '<S232>/Product1'
  rtb_Divide1_p = rtb_Switch3[1] / rtb_Power2;

  // Product: '<S232>/Product2'
  rtb_Divide5_a = rtb_Switch3[2] / rtb_Power2;

  // Product: '<S232>/Product3'
  rtb_Power2 = rtb_Switch3[3] / rtb_Power2;

  // Product: '<S222>/Product3' incorporates:
  //   Product: '<S226>/Product3'

  rtb_ExtractDiagonal_tmp_1 = rtb_Add * rtb_Add;

  // Product: '<S222>/Product2' incorporates:
  //   Product: '<S226>/Product2'

  rtb_ExtractDiagonal_tmp_2 = rtb_Divide1_p * rtb_Divide1_p;

  // Product: '<S222>/Product1' incorporates:
  //   Product: '<S226>/Product1'
  //   Product: '<S230>/Product1'

  rtb_ExtractDiagonal_tmp_3 = rtb_Divide5_a * rtb_Divide5_a;

  // Product: '<S222>/Product' incorporates:
  //   Product: '<S226>/Product'
  //   Product: '<S230>/Product'

  rtb_ExtractDiagonal_tmp_4 = rtb_Power2 * rtb_Power2;

  // Sum: '<S222>/Sum' incorporates:
  //   Product: '<S222>/Product'
  //   Product: '<S222>/Product1'
  //   Product: '<S222>/Product2'
  //   Product: '<S222>/Product3'

  rtb_ExtractDiagonal[0] = ((rtb_ExtractDiagonal_tmp_1 +
    rtb_ExtractDiagonal_tmp_2) - rtb_ExtractDiagonal_tmp_3) -
    rtb_ExtractDiagonal_tmp_4;

  // Product: '<S225>/Product3' incorporates:
  //   Product: '<S223>/Product3'

  rtb_ExtractDiagonal_tmp = rtb_Power2 * rtb_Add;

  // Product: '<S225>/Product2' incorporates:
  //   Product: '<S223>/Product2'

  rtb_ExtractDiagonal_tmp_0 = rtb_Divide1_p * rtb_Divide5_a;

  // Gain: '<S225>/Gain' incorporates:
  //   Product: '<S225>/Product2'
  //   Product: '<S225>/Product3'
  //   Sum: '<S225>/Sum'

  rtb_ExtractDiagonal[1] = (rtb_ExtractDiagonal_tmp_0 - rtb_ExtractDiagonal_tmp)
    * ANAS0_P.Gain_Gain_ie;

  // Product: '<S228>/Product2' incorporates:
  //   Product: '<S224>/Product2'

  rtb_ExtractDiagonal_tmp_5 = rtb_Divide1_p * rtb_Power2;

  // Product: '<S228>/Product1' incorporates:
  //   Product: '<S224>/Product1'

  rtb_ExtractDiagonal_tmp_6 = rtb_Add * rtb_Divide5_a;

  // Gain: '<S228>/Gain' incorporates:
  //   Product: '<S228>/Product1'
  //   Product: '<S228>/Product2'
  //   Sum: '<S228>/Sum'

  rtb_ExtractDiagonal[2] = (rtb_ExtractDiagonal_tmp_6 +
    rtb_ExtractDiagonal_tmp_5) * ANAS0_P.Gain_Gain_ob;

  // Gain: '<S223>/Gain' incorporates:
  //   Sum: '<S223>/Sum'

  rtb_ExtractDiagonal[3] = (rtb_ExtractDiagonal_tmp + rtb_ExtractDiagonal_tmp_0)
    * ANAS0_P.Gain_Gain_hm;

  // Sum: '<S226>/Sum' incorporates:
  //   Sum: '<S230>/Sum'

  rtb_ExtractDiagonal_tmp_1 -= rtb_ExtractDiagonal_tmp_2;
  rtb_ExtractDiagonal[4] = (rtb_ExtractDiagonal_tmp_1 +
    rtb_ExtractDiagonal_tmp_3) - rtb_ExtractDiagonal_tmp_4;

  // Product: '<S229>/Product1' incorporates:
  //   Product: '<S227>/Product1'

  rtb_ExtractDiagonal_tmp_2 = rtb_Add * rtb_Divide1_p;

  // Product: '<S229>/Product2' incorporates:
  //   Product: '<S227>/Product2'

  rtb_ExtractDiagonal_tmp = rtb_Divide5_a * rtb_Power2;

  // Gain: '<S229>/Gain' incorporates:
  //   Product: '<S229>/Product1'
  //   Product: '<S229>/Product2'
  //   Sum: '<S229>/Sum'

  rtb_ExtractDiagonal[5] = (rtb_ExtractDiagonal_tmp - rtb_ExtractDiagonal_tmp_2)
    * ANAS0_P.Gain_Gain_bt;

  // Gain: '<S224>/Gain' incorporates:
  //   Sum: '<S224>/Sum'

  rtb_ExtractDiagonal[6] = (rtb_ExtractDiagonal_tmp_5 -
    rtb_ExtractDiagonal_tmp_6) * ANAS0_P.Gain_Gain_p3;

  // Gain: '<S227>/Gain' incorporates:
  //   Sum: '<S227>/Sum'

  rtb_ExtractDiagonal[7] = (rtb_ExtractDiagonal_tmp_2 + rtb_ExtractDiagonal_tmp)
    * ANAS0_P.Gain_Gain_a;

  // Sum: '<S230>/Sum'
  rtb_ExtractDiagonal[8] = (rtb_ExtractDiagonal_tmp_1 -
    rtb_ExtractDiagonal_tmp_3) + rtb_ExtractDiagonal_tmp_4;

  // Product: '<S220>/Matrix Multiply' incorporates:
  //   Constant: '<S220>/Local Gravity'
  //   Gain: '<S5>/AccPropagationFlag'
  //   Inport: '<Root>/ANAS In'
  //   Math: '<S220>/Transpose'
  //   Product: '<S5>/Product'
  //   Product: '<S5>/Product1'
  //   S-Function (sdspdiag2): '<S6>/Extract Diagonal'
  //   Sum: '<S220>/Sum'
  //   Sum: '<S5>/Add'
  //   Sum: '<S5>/Add1'

  rtb_Divide5_a = ANAS0_U.ANASIn_c.AccMeasure[1];
  rtb_Power2 = ANAS0_U.ANASIn_c.AccMeasure[0];
  rtb_Product1_j3 = ANAS0_U.ANASIn_c.AccMeasure[2];
  i = 0;
  for (i_0 = 0; i_0 < 3; i_0++) {
    rtb_Product2_k0 = rtb_Switch2[i_0 + 3];
    rtb_VectorConcatenate_o[i_0 + 3] = (((rtb_ExtractDiagonal[i + 1] *
      rtb_Divide5_a + rtb_ExtractDiagonal[i] * rtb_Power2) +
      rtb_ExtractDiagonal[i + 2] * rtb_Product1_j3) +
      ANAS0_P.LocalGravity_Value_jy[i_0]) * ANAS0_P.AccPropagationFlag_Gain *
      rtb_Bias3 + rtb_Product2_k0;
    rtb_VectorConcatenate_o[i_0] = rtb_Product2_k0 * rtb_Bias3 + rtb_Switch2[i_0];
    i += 3;
  }

  // End of Product: '<S220>/Matrix Multiply'

  // Update for UnitDelay: '<S219>/UD' incorporates:
  //   Inport: '<Root>/ANAS In'
  //
  //  Block description for '<S219>/UD':
  //
  //   Store in Global RAM

  ANAS0_DW.UD_DSTATE = ANAS0_U.ANASIn_c.AccTimestamp;

  // Bias: '<S5>/Bias1' incorporates:
  //   UnitDelay: '<S5>/Unit Delay3'

  u0 = static_cast<uint8_t>(ANAS0_DW.UnitDelay3_DSTATE_h + ANAS0_P.Bias1_Bias_o);

  // Saturate: '<S5>/Saturation2'
  if (u0 > ANAS0_P.Saturation2_UpperSat_c) {
    // Update for UnitDelay: '<S5>/Unit Delay3'
    ANAS0_DW.UnitDelay3_DSTATE_h = ANAS0_P.Saturation2_UpperSat_c;
  } else if (u0 < ANAS0_P.Saturation2_LowerSat_n) {
    // Update for UnitDelay: '<S5>/Unit Delay3'
    ANAS0_DW.UnitDelay3_DSTATE_h = ANAS0_P.Saturation2_LowerSat_n;
  } else {
    // Update for UnitDelay: '<S5>/Unit Delay3'
    ANAS0_DW.UnitDelay3_DSTATE_h = u0;
  }

  // End of Saturate: '<S5>/Saturation2'
  // End of Outputs for SubSystem: '<S1>/Linear States Prediction'

  // Selector: '<S7>/Selector2' incorporates:
  //   Inport: '<Root>/ANAS Reference'

  i_0 = 0;
  k = 0;
  for (i = 0; i < 3; i++) {
    rtb_Selector2[i_0] = ANAS0_U.ANASReference_f.InitialCovariance[k + 60];
    rtb_Selector2[i_0 + 1] = ANAS0_U.ANASReference_f.InitialCovariance[k + 61];
    rtb_Selector2[i_0 + 2] = ANAS0_U.ANASReference_f.InitialCovariance[k + 62];
    i_0 += 3;
    k += 9;
  }

  // End of Selector: '<S7>/Selector2'

  // Outputs for Atomic SubSystem: '<S1>/Angular States Prediction'
  // Switch: '<S3>/Switch' incorporates:
  //   Constant: '<S3>/Constant'
  //   DataTypeConversion: '<S3>/Cast To Single'
  //   Gain: '<S3>/Gain1'
  //   Inport: '<Root>/ANAS In'
  //   Sum: '<S89>/Diff'
  //   UnitDelay: '<S3>/Unit Delay3'
  //   UnitDelay: '<S89>/UD'
  //
  //  Block description for '<S89>/Diff':
  //
  //   Add in CPU
  //
  //  Block description for '<S89>/UD':
  //
  //   Store in Global RAM

  if (ANAS0_DW.UnitDelay3_DSTATE_o > ANAS0_P.Switch_Threshold) {
    rtb_Bias3 = static_cast<float>(ANAS0_U.ANASIn_c.GyroTimestamp -
      ANAS0_DW.UD_DSTATE_a) * ANAS0_P.Gain1_Gain_h;
  } else {
    rtb_Bias3 = ANAS0_P.Constant_Value_cv;
  }

  // End of Switch: '<S3>/Switch'

  // SignalConversion generated from: '<S93>/Vector Concatenate' incorporates:
  //   Constant: '<S92>/Constant'

  rtb_ExtractDiagonal[0] = ANAS0_P.Constant_Value_j;

  // Gain: '<S92>/Gain1' incorporates:
  //   Inport: '<Root>/ANAS In'
  //   Reshape: '<S90>/Reshape'

  rtb_ExtractDiagonal[1] = ANAS0_P.Gain1_Gain_oh * ANAS0_U.ANASIn_c.GyroMeasure
    [2];

  // SignalConversion generated from: '<S93>/Vector Concatenate' incorporates:
  //   Inport: '<Root>/ANAS In'
  //   Reshape: '<S90>/Reshape'

  rtb_ExtractDiagonal[2] = ANAS0_U.ANASIn_c.GyroMeasure[1];

  // SignalConversion generated from: '<S93>/Vector Concatenate' incorporates:
  //   Inport: '<Root>/ANAS In'
  //   Reshape: '<S90>/Reshape'

  rtb_ExtractDiagonal[3] = ANAS0_U.ANASIn_c.GyroMeasure[2];

  // SignalConversion generated from: '<S93>/Vector Concatenate' incorporates:
  //   Constant: '<S92>/Constant'

  rtb_ExtractDiagonal[4] = ANAS0_P.Constant_Value_j;

  // Gain: '<S92>/Gain2' incorporates:
  //   Inport: '<Root>/ANAS In'
  //   Reshape: '<S90>/Reshape'

  rtb_ExtractDiagonal[5] = ANAS0_P.Gain2_Gain_d * ANAS0_U.ANASIn_c.GyroMeasure[0];

  // Gain: '<S92>/Gain' incorporates:
  //   Inport: '<Root>/ANAS In'
  //   Reshape: '<S90>/Reshape'

  rtb_ExtractDiagonal[6] = ANAS0_P.Gain_Gain_e * ANAS0_U.ANASIn_c.GyroMeasure[1];

  // SignalConversion generated from: '<S93>/Vector Concatenate' incorporates:
  //   Inport: '<Root>/ANAS In'
  //   Reshape: '<S90>/Reshape'

  rtb_ExtractDiagonal[7] = ANAS0_U.ANASIn_c.GyroMeasure[0];

  // SignalConversion generated from: '<S93>/Vector Concatenate' incorporates:
  //   Constant: '<S92>/Constant'

  rtb_ExtractDiagonal[8] = ANAS0_P.Constant_Value_j;

  // Math: '<S92>/Transpose' incorporates:
  //   Product: '<S13>/Matrix Multiply2'
  //   S-Function (sdspdiag2): '<S6>/Extract Diagonal'

  i_0 = 0;
  for (k = 0; k < 3; k++) {
    rtb_MatrixMultiply2[i_0] = rtb_ExtractDiagonal[k];
    rtb_MatrixMultiply2[i_0 + 1] = rtb_ExtractDiagonal[k + 3];
    rtb_MatrixMultiply2[i_0 + 2] = rtb_ExtractDiagonal[k + 6];
    i_0 += 3;
  }

  // End of Math: '<S92>/Transpose'

  // Gain: '<S90>/Gain' incorporates:
  //   Product: '<S13>/Matrix Multiply2'

  for (i = 0; i < 9; i++) {
    rtb_MatrixConcatenate[i] = ANAS0_P.Gain_Gain_eg * rtb_MatrixMultiply2[i];
  }

  // End of Gain: '<S90>/Gain'

  // SignalConversion generated from: '<S90>/Matrix Concatenate' incorporates:
  //   Inport: '<Root>/ANAS In'
  //   Reshape: '<S90>/Reshape'

  rtb_MatrixConcatenate[9] = ANAS0_U.ANASIn_c.GyroMeasure[0];

  // Gain: '<S90>/Gain1' incorporates:
  //   Inport: '<Root>/ANAS In'
  //   Reshape: '<S90>/Reshape'

  rtb_res_GPS[0] = ANAS0_P.Gain1_Gain_k * ANAS0_U.ANASIn_c.GyroMeasure[0];

  // SignalConversion generated from: '<S90>/Matrix Concatenate' incorporates:
  //   Inport: '<Root>/ANAS In'
  //   Reshape: '<S90>/Reshape'

  rtb_MatrixConcatenate[10] = ANAS0_U.ANASIn_c.GyroMeasure[1];

  // Gain: '<S90>/Gain1' incorporates:
  //   Inport: '<Root>/ANAS In'
  //   Reshape: '<S90>/Reshape'

  rtb_res_GPS[1] = ANAS0_P.Gain1_Gain_k * ANAS0_U.ANASIn_c.GyroMeasure[1];

  // SignalConversion generated from: '<S90>/Matrix Concatenate' incorporates:
  //   Inport: '<Root>/ANAS In'
  //   Reshape: '<S90>/Reshape'

  rtb_MatrixConcatenate[11] = ANAS0_U.ANASIn_c.GyroMeasure[2];

  // Gain: '<S90>/Gain1' incorporates:
  //   Inport: '<Root>/ANAS In'
  //   Reshape: '<S90>/Reshape'

  rtb_res_GPS[2] = ANAS0_P.Gain1_Gain_k * ANAS0_U.ANASIn_c.GyroMeasure[2];

  // Constant: '<S90>/Constant'
  rtb_res_GPS[3] = ANAS0_P.Constant_Value_h;

  // Concatenate: '<S90>/Matrix Concatenate2' incorporates:
  //   Concatenate: '<S90>/Matrix Concatenate'
  //   Math: '<S202>/Square'
  //   Sum: '<S208>/Add'

  i_0 = 0;
  k = 0;
  for (i = 0; i < 4; i++) {
    rtb_Square_f[i_0] = rtb_MatrixConcatenate[k];
    rtb_Square_f[i_0 + 1] = rtb_MatrixConcatenate[k + 1];
    rtb_Square_f[i_0 + 2] = rtb_MatrixConcatenate[k + 2];
    rtb_Square_f[i_0 + 3] = rtb_res_GPS[i];
    i_0 += 4;
    k += 3;
  }

  // End of Concatenate: '<S90>/Matrix Concatenate2'
  for (i_0 = 0; i_0 < 9; i_0++) {
    // Sum: '<S3>/Add' incorporates:
    //   Constant: '<S3>/Constant2'
    //   Product: '<S13>/Matrix Multiply2'
    //   Product: '<S3>/Product2'

    rtb_MatrixMultiply2[i_0] = rtb_Bias3 * rtb_MatrixMultiply2[i_0] +
      ANAS0_P.Constant2_Value[i_0];

    // RelationalOperator: '<S7>/Relational Operator1' incorporates:
    //   Selector: '<S7>/Selector2'

    rtb_Divide5_a = rtb_Selector2[i_0];

    // Switch: '<S7>/Switch1' incorporates:
    //   RelationalOperator: '<S7>/Relational Operator1'
    //   Selector: '<S7>/Selector2'
    //   UnitDelay: '<S7>/Unit Delay2'

    if (rtb_Divide5_a != ANAS0_DW.UnitDelay2_DSTATE[i_0]) {
      // Product: '<S3>/Matrix Multiply'
      rtb_Sum2[i_0] = rtb_Divide5_a;
    } else {
      // Product: '<S3>/Matrix Multiply' incorporates:
      //   UnitDelay: '<S7>/Unit Delay3'

      rtb_Sum2[i_0] = ANAS0_DW.UnitDelay3_DSTATE[i_0];
    }

    // End of Switch: '<S7>/Switch1'
  }

  // Product: '<S3>/Matrix Multiply' incorporates:
  //   Math: '<S3>/Transpose'
  //   Product: '<S13>/Matrix Multiply2'

  for (i_0 = 0; i_0 < 3; i_0++) {
    rtb_Divide5_a = rtb_Sum2[i_0 + 3];
    rtb_Power2 = rtb_Sum2[i_0];
    rtb_Product1_j3 = rtb_Sum2[i_0 + 6];
    k = 0;
    for (i = 0; i < 3; i++) {
      rtb_S_jt[k + i_0] = (rtb_MatrixMultiply2[i + 3] * rtb_Divide5_a +
                           rtb_Power2 * rtb_MatrixMultiply2[i]) +
        rtb_MatrixMultiply2[i + 6] * rtb_Product1_j3;
      k += 3;
    }
  }

  for (i = 0; i < 3; i++) {
    // Sum: '<S3>/Sum2' incorporates:
    //   Constant: '<S3>/AngularQ1_3'
    //   Product: '<S13>/Matrix Multiply2'
    //   Product: '<S3>/Matrix Multiply'

    rtb_Divide1_p = rtb_MatrixMultiply2[i + 3];
    rtb_Add = rtb_MatrixMultiply2[i];
    rtb_Power = rtb_MatrixMultiply2[i + 6];
    i_0 = 0;
    for (k = 0; k < 3; k++) {
      rtb_LinearQ_tmp = i_0 + i;
      rtb_Sum2[rtb_LinearQ_tmp] = ((rtb_S_jt[i_0 + 1] * rtb_Divide1_p +
        rtb_S_jt[i_0] * rtb_Add) + rtb_S_jt[i_0 + 2] * rtb_Power) +
        ANAS0_P.AngularQ1_3_Value[rtb_LinearQ_tmp];
      i_0 += 3;
    }

    // End of Sum: '<S3>/Sum2'

    // SignalConversion generated from: '<S3>/Product1' incorporates:
    //   Product: '<S3>/Matrix Multiply'

    rtb_res_GPS[i] = rtb_Switch3[i + 1];
  }

  // SignalConversion generated from: '<S3>/Product1'
  rtb_res_GPS[3] = rtb_Switch3[0];

  // Sum: '<S3>/Sum' incorporates:
  //   Constant: '<S3>/Constant1'
  //   Gain: '<S3>/Gain'
  //   Math: '<S202>/Square'
  //   Product: '<S3>/Product'

  for (i_0 = 0; i_0 < 16; i_0++) {
    rtb_S[i_0] = ANAS0_P.Gain_Gain_ow * rtb_Square_f[i_0] * rtb_Bias3 +
      ANAS0_P.Constant1_Value[i_0];
  }

  // End of Sum: '<S3>/Sum'

  // Product: '<S3>/Product1'
  rtb_Divide5_a = rtb_res_GPS[1];
  rtb_Power2 = rtb_res_GPS[0];
  rtb_Product1_j3 = rtb_res_GPS[2];
  rtb_Product2_k0 = rtb_res_GPS[3];
  for (i_0 = 0; i_0 < 4; i_0++) {
    rtb_Switch3[i_0] = ((rtb_S[i_0 + 4] * rtb_Divide5_a + rtb_S[i_0] *
                         rtb_Power2) + rtb_S[i_0 + 8] * rtb_Product1_j3) +
      rtb_S[i_0 + 12] * rtb_Product2_k0;
  }

  // End of Product: '<S3>/Product1'

  // Sqrt: '<S94>/sqrt' incorporates:
  //   Product: '<S95>/Product'
  //   Product: '<S95>/Product1'
  //   Product: '<S95>/Product2'
  //   Product: '<S95>/Product3'
  //   Sum: '<S95>/Sum'

  rtb_Bias3 = std::sqrt(((rtb_Switch3[3] * rtb_Switch3[3] + rtb_Switch3[0] *
    rtb_Switch3[0]) + rtb_Switch3[1] * rtb_Switch3[1]) + rtb_Switch3[2] *
                        rtb_Switch3[2]);

  // Product: '<S91>/Product'
  rtb_Power2 = rtb_Switch3[3] / rtb_Bias3;

  // Product: '<S91>/Product1'
  rtb_Product1_j3 = rtb_Switch3[0] / rtb_Bias3;

  // Product: '<S91>/Product2'
  rtb_Product2_k0 = rtb_Switch3[1] / rtb_Bias3;

  // Product: '<S91>/Product3'
  rtb_Product3_jl = rtb_Switch3[2] / rtb_Bias3;

  // Update for UnitDelay: '<S89>/UD' incorporates:
  //   Inport: '<Root>/ANAS In'
  //
  //  Block description for '<S89>/UD':
  //
  //   Store in Global RAM

  ANAS0_DW.UD_DSTATE_a = ANAS0_U.ANASIn_c.GyroTimestamp;

  // Bias: '<S3>/Bias1' incorporates:
  //   UnitDelay: '<S3>/Unit Delay3'

  u0 = static_cast<uint8_t>(ANAS0_DW.UnitDelay3_DSTATE_o + ANAS0_P.Bias1_Bias_k);

  // Saturate: '<S3>/Saturation2'
  if (u0 > ANAS0_P.Saturation2_UpperSat) {
    // Update for UnitDelay: '<S3>/Unit Delay3'
    ANAS0_DW.UnitDelay3_DSTATE_o = ANAS0_P.Saturation2_UpperSat;
  } else if (u0 < ANAS0_P.Saturation2_LowerSat) {
    // Update for UnitDelay: '<S3>/Unit Delay3'
    ANAS0_DW.UnitDelay3_DSTATE_o = ANAS0_P.Saturation2_LowerSat;
  } else {
    // Update for UnitDelay: '<S3>/Unit Delay3'
    ANAS0_DW.UnitDelay3_DSTATE_o = u0;
  }

  // End of Saturate: '<S3>/Saturation2'
  // End of Outputs for SubSystem: '<S1>/Angular States Prediction'

  // Outputs for Atomic SubSystem: '<S1>/Linear States Corrections'
  // Outputs for Atomic SubSystem: '<S1>/Angular States Corrections'
  // Sqrt: '<S48>/Sqrt' incorporates:
  //   DotProduct: '<S48>/Dot Product'
  //   Inport: '<Root>/ANAS In'
  //   Math: '<S48>/Transpose'
  //   Sqrt: '<S218>/Sqrt'

  rtb_Divide5_a = std::sqrt((ANAS0_U.ANASIn_c.AccMeasure[0] *
    ANAS0_U.ANASIn_c.AccMeasure[0] + ANAS0_U.ANASIn_c.AccMeasure[1] *
    ANAS0_U.ANASIn_c.AccMeasure[1]) + ANAS0_U.ANASIn_c.AccMeasure[2] *
    ANAS0_U.ANASIn_c.AccMeasure[2]);

  // End of Outputs for SubSystem: '<S1>/Linear States Corrections'

  // RateTransition: '<S11>/Rate Transition' incorporates:
  //   RateTransition: '<S51>/Rate Transition'

  rtb_variometer = ((&ANAS0_M)->Timing.TaskCounters.TID[1] == 0);
  if (rtb_variometer) {
    // RateTransition: '<S11>/Rate Transition' incorporates:
    //   Inport: '<Root>/ANAS In'

    ANAS0_DW.RateTransition_i = ANAS0_U.ANASIn_c.AccTimestamp;
  }

  // End of RateTransition: '<S11>/Rate Transition'

  // Logic: '<S11>/NOT' incorporates:
  //   Inport: '<Root>/ANAS In'

  rtb_LUFactorization_o3_i = !ANAS0_U.ANASIn_c.FlyingState;

  // Logic: '<S11>/AND' incorporates:
  //   Constant: '<S11>/AccCorrFlag'
  //   Constant: '<S49>/Lower Limit'
  //   Constant: '<S49>/Upper Limit'
  //   Logic: '<S49>/AND'
  //   Memory: '<S11>/Memory'
  //   RateTransition: '<S11>/Rate Transition'
  //   RelationalOperator: '<S11>/Relational Operator'
  //   RelationalOperator: '<S49>/Lower Test'
  //   RelationalOperator: '<S49>/Upper Test'
  //   Sqrt: '<S48>/Sqrt'

  rtb_AND_b = (ANAS0_P.AccCorrFlag_Value && (ANAS0_DW.RateTransition_i >
    ANAS0_DW.Memory_PreviousInput_k) && ((ANAS0_P.g02accg02_lowlimit <=
    rtb_Divide5_a) && (rtb_Divide5_a <= ANAS0_P.g02accg02_uplimit)) &&
               rtb_LUFactorization_o3_i);

  // RateTransition: '<S51>/Rate Transition'
  if (rtb_variometer) {
    // RateTransition: '<S51>/Rate Transition' incorporates:
    //   Inport: '<Root>/ANAS In'

    ANAS0_DW.RateTransition_o = ANAS0_U.ANASIn_c.MagTimestamp;
  }

  // Logic: '<S51>/AND' incorporates:
  //   Constant: '<S51>/MagCorrFlag'
  //   Memory: '<S51>/Memory'
  //   RateTransition: '<S51>/Rate Transition'
  //   RelationalOperator: '<S51>/Relational Operator'

  rtb_AND_i = (ANAS0_P.MagCorrFlag_Value && (ANAS0_DW.RateTransition_o >
    ANAS0_DW.Memory_PreviousInput_kq));

  // Outputs for Enabled SubSystem: '<S9>/Active Correction Step Magnetometer' incorporates:
  //   EnablePort: '<S50>/Enable'

  if (rtb_AND_i) {
    // SignalConversion generated from: '<S88>/Vector Concatenate' incorporates:
    //   Constant: '<S73>/Constant'

    rtb_VectorConcatenate_i[0] = ANAS0_P.Constant_Value_p;

    // Sqrt: '<S86>/sqrt' incorporates:
    //   Product: '<S87>/Product'
    //   Product: '<S87>/Product1'
    //   Product: '<S87>/Product2'
    //   Product: '<S87>/Product3'
    //   Sum: '<S87>/Sum'

    rtb_Bias3 = std::sqrt(((rtb_Power2 * rtb_Power2 + rtb_Product1_j3 *
      rtb_Product1_j3) + rtb_Product2_k0 * rtb_Product2_k0) + rtb_Product3_jl *
                          rtb_Product3_jl);

    // Product: '<S85>/Product'
    rtb_Power = rtb_Power2 / rtb_Bias3;

    // Product: '<S85>/Product1'
    rtb_Add = rtb_Product1_j3 / rtb_Bias3;

    // Product: '<S85>/Product2'
    rtb_Divide1_p = rtb_Product2_k0 / rtb_Bias3;

    // Product: '<S85>/Product3'
    rtb_Bias3 = rtb_Product3_jl / rtb_Bias3;

    // Product: '<S75>/Product3' incorporates:
    //   Product: '<S79>/Product3'

    rtb_ExtractDiagonal_tmp_1 = rtb_Power * rtb_Power;

    // Product: '<S75>/Product2' incorporates:
    //   Product: '<S79>/Product2'

    rtb_ExtractDiagonal_tmp_2 = rtb_Add * rtb_Add;

    // Product: '<S75>/Product1' incorporates:
    //   Product: '<S79>/Product1'
    //   Product: '<S83>/Product1'

    rtb_ExtractDiagonal_tmp_3 = rtb_Divide1_p * rtb_Divide1_p;

    // Product: '<S75>/Product' incorporates:
    //   Product: '<S79>/Product'
    //   Product: '<S83>/Product'

    rtb_ExtractDiagonal_tmp_4 = rtb_Bias3 * rtb_Bias3;

    // Sum: '<S75>/Sum' incorporates:
    //   Product: '<S75>/Product'
    //   Product: '<S75>/Product1'
    //   Product: '<S75>/Product2'
    //   Product: '<S75>/Product3'

    rtb_ExtractDiagonal[0] = ((rtb_ExtractDiagonal_tmp_1 +
      rtb_ExtractDiagonal_tmp_2) - rtb_ExtractDiagonal_tmp_3) -
      rtb_ExtractDiagonal_tmp_4;

    // Product: '<S78>/Product3' incorporates:
    //   Product: '<S76>/Product3'

    rtb_ExtractDiagonal_tmp = rtb_Bias3 * rtb_Power;

    // Product: '<S78>/Product2' incorporates:
    //   Product: '<S76>/Product2'

    rtb_ExtractDiagonal_tmp_0 = rtb_Add * rtb_Divide1_p;

    // Gain: '<S78>/Gain' incorporates:
    //   Product: '<S78>/Product2'
    //   Product: '<S78>/Product3'
    //   Sum: '<S78>/Sum'

    rtb_ExtractDiagonal[1] = (rtb_ExtractDiagonal_tmp_0 -
      rtb_ExtractDiagonal_tmp) * ANAS0_P.Gain_Gain_hq;

    // Product: '<S81>/Product2' incorporates:
    //   Product: '<S77>/Product2'

    rtb_ExtractDiagonal_tmp_5 = rtb_Add * rtb_Bias3;

    // Product: '<S81>/Product1' incorporates:
    //   Product: '<S77>/Product1'

    rtb_ExtractDiagonal_tmp_6 = rtb_Power * rtb_Divide1_p;

    // Gain: '<S81>/Gain' incorporates:
    //   Product: '<S81>/Product1'
    //   Product: '<S81>/Product2'
    //   Sum: '<S81>/Sum'

    rtb_ExtractDiagonal[2] = (rtb_ExtractDiagonal_tmp_6 +
      rtb_ExtractDiagonal_tmp_5) * ANAS0_P.Gain_Gain_gr;

    // Gain: '<S76>/Gain' incorporates:
    //   Sum: '<S76>/Sum'

    rtb_ExtractDiagonal[3] = (rtb_ExtractDiagonal_tmp +
      rtb_ExtractDiagonal_tmp_0) * ANAS0_P.Gain_Gain_hf;

    // Sum: '<S79>/Sum' incorporates:
    //   Sum: '<S83>/Sum'

    rtb_ExtractDiagonal_tmp_1 -= rtb_ExtractDiagonal_tmp_2;
    rtb_ExtractDiagonal[4] = (rtb_ExtractDiagonal_tmp_1 +
      rtb_ExtractDiagonal_tmp_3) - rtb_ExtractDiagonal_tmp_4;

    // Product: '<S82>/Product1' incorporates:
    //   Product: '<S80>/Product1'

    rtb_ExtractDiagonal_tmp_2 = rtb_Power * rtb_Add;

    // Product: '<S82>/Product2' incorporates:
    //   Product: '<S80>/Product2'

    rtb_ExtractDiagonal_tmp = rtb_Divide1_p * rtb_Bias3;

    // Gain: '<S82>/Gain' incorporates:
    //   Product: '<S82>/Product1'
    //   Product: '<S82>/Product2'
    //   Sum: '<S82>/Sum'

    rtb_ExtractDiagonal[5] = (rtb_ExtractDiagonal_tmp -
      rtb_ExtractDiagonal_tmp_2) * ANAS0_P.Gain_Gain_m;

    // Gain: '<S77>/Gain' incorporates:
    //   Sum: '<S77>/Sum'

    rtb_ExtractDiagonal[6] = (rtb_ExtractDiagonal_tmp_5 -
      rtb_ExtractDiagonal_tmp_6) * ANAS0_P.Gain_Gain_d;

    // Gain: '<S80>/Gain' incorporates:
    //   Sum: '<S80>/Sum'

    rtb_ExtractDiagonal[7] = (rtb_ExtractDiagonal_tmp_2 +
      rtb_ExtractDiagonal_tmp) * ANAS0_P.Gain_Gain_h1;

    // Sum: '<S83>/Sum'
    rtb_ExtractDiagonal[8] = (rtb_ExtractDiagonal_tmp_1 -
      rtb_ExtractDiagonal_tmp_3) + rtb_ExtractDiagonal_tmp_4;

    // Product: '<S56>/Matrix Multiply1' incorporates:
    //   Constant: '<S56>/InitMagnField'

    rtb_Bias3 = ANAS0_P.InitMagnField_Value[1];
    rtb_Divide1_p = ANAS0_P.InitMagnField_Value[0];
    rtb_Add = ANAS0_P.InitMagnField_Value[2];
    for (i_0 = 0; i_0 < 3; i_0++) {
      // Math: '<S65>/Transpose' incorporates:
      //   S-Function (sdspdiag2): '<S6>/Extract Diagonal'

      rtb_Transpose_c[i_0] = (rtb_ExtractDiagonal[i_0 + 3] * rtb_Bias3 +
        rtb_ExtractDiagonal[i_0] * rtb_Divide1_p) + rtb_ExtractDiagonal[i_0 + 6]
        * rtb_Add;
    }

    // End of Product: '<S56>/Matrix Multiply1'

    // Gain: '<S73>/Gain1'
    rtb_VectorConcatenate_i[1] = ANAS0_P.Gain1_Gain_m * rtb_Transpose_c[2];

    // SignalConversion generated from: '<S88>/Vector Concatenate'
    rtb_VectorConcatenate_i[2] = rtb_Transpose_c[1];

    // SignalConversion generated from: '<S88>/Vector Concatenate'
    rtb_VectorConcatenate_i[3] = rtb_Transpose_c[2];

    // SignalConversion generated from: '<S88>/Vector Concatenate' incorporates:
    //   Constant: '<S73>/Constant'

    rtb_VectorConcatenate_i[4] = ANAS0_P.Constant_Value_p;

    // Gain: '<S73>/Gain2'
    rtb_VectorConcatenate_i[5] = ANAS0_P.Gain2_Gain_g * rtb_Transpose_c[0];

    // Gain: '<S73>/Gain'
    rtb_VectorConcatenate_i[6] = ANAS0_P.Gain_Gain_gl * rtb_Transpose_c[1];

    // SignalConversion generated from: '<S88>/Vector Concatenate'
    rtb_VectorConcatenate_i[7] = rtb_Transpose_c[0];

    // SignalConversion generated from: '<S88>/Vector Concatenate' incorporates:
    //   Constant: '<S73>/Constant'

    rtb_VectorConcatenate_i[8] = ANAS0_P.Constant_Value_p;

    // Math: '<S73>/Transpose' incorporates:
    //   Concatenate: '<S43>/Vector Concatenate'
    //   Math: '<S13>/Transpose'

    i_0 = 0;
    for (k = 0; k < 3; k++) {
      rtb_Transpose_g[i_0] = rtb_VectorConcatenate_i[k];
      rtb_Transpose_g[i_0 + 1] = rtb_VectorConcatenate_i[k + 3];
      rtb_Transpose_g[i_0 + 2] = rtb_VectorConcatenate_i[k + 6];
      i_0 += 3;
    }

    // End of Math: '<S73>/Transpose'

    // Math: '<S54>/Transpose' incorporates:
    //   Math: '<S13>/Transpose'
    //   Product: '<S13>/Matrix Multiply'

    i_0 = 0;
    for (k = 0; k < 3; k++) {
      rtb_MatrixMultiply_i[i_0] = rtb_Transpose_g[k];
      rtb_MatrixMultiply_i[i_0 + 1] = rtb_Transpose_g[k + 3];
      rtb_MatrixMultiply_i[i_0 + 2] = rtb_Transpose_g[k + 6];
      i_0 += 3;
    }

    // End of Math: '<S54>/Transpose'

    // Math: '<S50>/Square' incorporates:
    //   Constant: '<S50>/MagSigma'
    //   Sum: '<S17>/Subtract'

    for (i = 0; i < 9; i++) {
      rtb_Bias3 = ANAS0_P.MagSigma_Value[i];
      rtb_MatrixMultiply2[i] = rtb_Bias3 * rtb_Bias3;
    }

    // End of Math: '<S50>/Square'

    // Product: '<S54>/Matrix Multiply' incorporates:
    //   Product: '<S13>/Matrix Multiply'
    //   Sum: '<S3>/Sum2'

    i_0 = 0;
    for (k = 0; k < 3; k++) {
      rtb_Divide1_p = rtb_MatrixMultiply_i[i_0 + 1];
      rtb_Add = rtb_MatrixMultiply_i[i_0];
      rtb_Power = rtb_MatrixMultiply_i[i_0 + 2];
      for (i = 0; i < 3; i++) {
        rtb_LUFactorization_o1_c[i + i_0] = (rtb_Sum2[i + 3] * rtb_Divide1_p +
          rtb_Add * rtb_Sum2[i]) + rtb_Sum2[i + 6] * rtb_Power;
      }

      i_0 += 3;
    }

    for (i_0 = 0; i_0 < 3; i_0++) {
      // Sum: '<S54>/Add' incorporates:
      //   Math: '<S13>/Transpose'
      //   Product: '<S54>/Matrix Multiply'

      rtb_Bias3 = rtb_Transpose_g[i_0 + 3];
      rtb_Divide1_p = rtb_Transpose_g[i_0];
      rtb_Add = rtb_Transpose_g[i_0 + 6];

      // Sum: '<S54>/Add' incorporates:
      //   Product: '<S54>/Matrix Multiply'
      //   Sum: '<S17>/Subtract'

      k = 0;
      for (i = 0; i < 3; i++) {
        rtb_LinearQ_tmp = k + i_0;
        rtb_S_jt[rtb_LinearQ_tmp] = ((rtb_LUFactorization_o1_c[k + 1] *
          rtb_Bias3 + rtb_LUFactorization_o1_c[k] * rtb_Divide1_p) +
          rtb_LUFactorization_o1_c[k + 2] * rtb_Add) +
          rtb_MatrixMultiply2[rtb_LinearQ_tmp];
        k += 3;
      }
    }

    // S-Function (sdsplu2): '<S62>/LU Factorization' incorporates:
    //   Sum: '<S54>/Add'

    for (k = 0; k < 9; k++) {
      rtb_LUFactorization_o1_c[k] = rtb_S_jt[k];
    }

    LUf_boolfloatint32_t(&rtb_LUFactorization_o1_c[0],
                         &rtb_LUFactorization_o2_m[0], 3,
                         &rtb_LUFactorization_o3_i);

    // End of S-Function (sdsplu2): '<S62>/LU Factorization'

    // Switch: '<S60>/Switch' incorporates:
    //   Constant: '<S60>/Constant'
    //   Math: '<S60>/Math Function'
    //   Product: '<S60>/Product'
    //
    //  About '<S60>/Math Function':
    //   Operator: reciprocal

    if (rtb_LUFactorization_o3_i) {
      rtb_Bias3 = ANAS0_P.Constant_Value_c;
    } else {
      // S-Function (sdspm1norm2): '<S60>/Matrix  1-Norm2'

      // DSP System Toolbox Matrix 1-Norm (sdspm1norm2) - '<S60>/Matrix  1-Norm2' 
      {
        const float *uPtr{ &rtb_S_jt[0] };

        float m1norm{ 0.0 };

        int jdx;
        for (jdx=3; jdx-- > 0; ) {
          float sumabsAj{ 0.0 };

          int idxFlt;
          for (idxFlt=3; idxFlt-- > 0; ) {
            float temp{ *uPtr++ };

            sumabsAj += fabsf(temp);
          }

          m1norm = MAX(m1norm, sumabsAj);
        }

        rtb_Matrix1Norm2_j = m1norm;
      }

      // S-Function (sdspperm2): '<S62>/Permute Matrix' incorporates:
      //   IdentityMatrix: '<S62>/Identity Matrix'
      //   S-Function (sdspfbsub2): '<S62>/Backward Substitution'

      rtb_Bias3 = rtb_LUFactorization_o2_m[0];
      rtb_Divide1_p = rtb_LUFactorization_o2_m[1];
      rtb_Add = rtb_LUFactorization_o2_m[2];
      for (k = 0; k < 3; k++) {
        i_0 = static_cast<int32_t>(std::floor(rtb_Bias3)) - 1;
        if (i_0 < 0) {
          i_0 = 0;
        } else if (i_0 >= 3) {
          i_0 = 2;
        }

        rtb_BackwardSubstitution_e[3 * k] = ANAS0_DW.IdentityMatrix_j[3 * k +
          i_0];
        i_0 = static_cast<int32_t>(std::floor(rtb_Divide1_p)) - 1;
        if (i_0 < 0) {
          i_0 = 0;
        } else if (i_0 >= 3) {
          i_0 = 2;
        }

        rtb_BackwardSubstitution_e[3 * k + 1] = ANAS0_DW.IdentityMatrix_j[3 * k
          + i_0];
        i_0 = static_cast<int32_t>(std::floor(rtb_Add)) - 1;
        if (i_0 < 0) {
          i_0 = 0;
        } else if (i_0 >= 3) {
          i_0 = 2;
        }

        rtb_BackwardSubstitution_e[3 * k + 2] = ANAS0_DW.IdentityMatrix_j[3 * k
          + i_0];
      }

      // End of S-Function (sdspperm2): '<S62>/Permute Matrix'

      // S-Function (sdspfbsub2): '<S62>/Forward Substitution' incorporates:
      //   S-Function (sdspfbsub2): '<S62>/Backward Substitution'
      //   S-Function (sdsplu2): '<S62>/LU Factorization'

      for (i = 0; i < 3; i++) {
        i_0 = i * 3;
        rtb_BackwardSubstitution_e[i_0 + 1] -= rtb_LUFactorization_o1_c[1] *
          rtb_BackwardSubstitution_e[i_0];
        rtb_LinearQ_tmp = 2;
        rtb_Add = rtb_BackwardSubstitution_e[i_0 + 2];
        for (k = 0; k < 2; k++) {
          rtb_Add -= rtb_BackwardSubstitution_e[i_0 + k] *
            rtb_LUFactorization_o1_c[rtb_LinearQ_tmp];
          rtb_LinearQ_tmp += 3;
        }

        rtb_BackwardSubstitution_e[i_0 + 2] = rtb_Add;
      }

      // End of S-Function (sdspfbsub2): '<S62>/Forward Substitution'

      // S-Function (sdspfbsub2): '<S62>/Backward Substitution' incorporates:
      //   S-Function (sdsplu2): '<S62>/LU Factorization'

      for (i = 0; i < 3; i++) {
        i_0 = i * 3;
        rtb_Bias3 = rtb_BackwardSubstitution_e[i_0 + 2] /
          rtb_LUFactorization_o1_c[8];

        // S-Function (sdspfbsub2): '<S62>/Backward Substitution' incorporates:
        //   S-Function (sdsplu2): '<S62>/LU Factorization'

        rtb_BackwardSubstitution_e[i_0 + 2] = rtb_Bias3;
        rtb_BackwardSubstitution_e[i_0 + 1] = (rtb_BackwardSubstitution_e[i_0 +
          1] - rtb_Bias3 * rtb_LUFactorization_o1_c[7]) /
          rtb_LUFactorization_o1_c[4];
        rtb_LinearQ_tmp = 6;
        rtb_Add = rtb_BackwardSubstitution_e[i_0];
        for (k = 2; k > 0; k--) {
          rtb_Add -= rtb_BackwardSubstitution_e[i_0 + k] *
            rtb_LUFactorization_o1_c[rtb_LinearQ_tmp];
          rtb_LinearQ_tmp -= 3;
        }

        rtb_BackwardSubstitution_e[i_0] = rtb_Add /
          rtb_LUFactorization_o1_c[rtb_LinearQ_tmp];
      }

      // End of S-Function (sdspfbsub2): '<S62>/Backward Substitution'

      // S-Function (sdspm1norm2): '<S60>/Matrix  1-Norm1'

      // DSP System Toolbox Matrix 1-Norm (sdspm1norm2) - '<S60>/Matrix  1-Norm1' 
      {
        const float *uPtr{ &rtb_BackwardSubstitution_e[0] };

        float m1norm{ 0.0 };

        int jdx;
        for (jdx=3; jdx-- > 0; ) {
          float sumabsAj{ 0.0 };

          int idxFlt;
          for (idxFlt=3; idxFlt-- > 0; ) {
            float temp{ *uPtr++ };

            sumabsAj += fabsf(temp);
          }

          m1norm = MAX(m1norm, sumabsAj);
        }

        rtb_Matrix1Norm1_k = m1norm;
      }

      rtb_Bias3 = 1.0F / (rtb_Matrix1Norm1_k * rtb_Matrix1Norm2_j);
    }

    // End of Switch: '<S60>/Switch'

    // Product: '<S54>/Matrix Multiply1' incorporates:
    //   Product: '<S13>/Matrix Multiply'
    //   Sum: '<S3>/Sum2'

    i_0 = 0;
    for (k = 0; k < 3; k++) {
      rtb_Divide1_p = rtb_MatrixMultiply_i[i_0 + 1];
      rtb_Add = rtb_MatrixMultiply_i[i_0];
      rtb_Power = rtb_MatrixMultiply_i[i_0 + 2];
      for (i = 0; i < 3; i++) {
        rtb_LUFactorization_o1_c[i + i_0] = (rtb_Sum2[i + 3] * rtb_Divide1_p +
          rtb_Add * rtb_Sum2[i]) + rtb_Sum2[i + 6] * rtb_Power;
      }

      i_0 += 3;
    }

    // End of Product: '<S54>/Matrix Multiply1'

    // If: '<S54>/If' incorporates:
    //   Constant: '<S54>/NASCondLim'
    //   RelationalOperator: '<S54>/GreaterThanOrEqual'

    if (rtb_Bias3 >= ANAS0_P.NASCondLim_Value_n) {
      for (i = 0; i < 9; i++) {
        // Outputs for IfAction SubSystem: '<S54>/Correction' incorporates:
        //   ActionPort: '<S58>/Action Port'

        rtb_MatrixMultiply_i[i] = rtb_S_jt[i];

        // End of Outputs for SubSystem: '<S54>/Correction'
      }

      // Outputs for IfAction SubSystem: '<S54>/Correction' incorporates:
      //   ActionPort: '<S58>/Action Port'

      ANAS0_Correction(rtb_MatrixMultiply_i, rtb_LUFactorization_o1_c,
                       rtb_BackwardSubstitution_e);

      // End of Outputs for SubSystem: '<S54>/Correction'
    } else {
      for (i = 0; i < 9; i++) {
        // Outputs for IfAction SubSystem: '<S54>/No correction' incorporates:
        //   ActionPort: '<S59>/Action Port'

        rtb_MatrixMultiply_i[i] = rtb_S_jt[i];

        // End of Outputs for SubSystem: '<S54>/No correction'
      }

      // Outputs for IfAction SubSystem: '<S54>/No correction' incorporates:
      //   ActionPort: '<S59>/Action Port'

      ANAS0_Nocorrection(rtb_LUFactorization_o1_c, rtb_BackwardSubstitution_e,
                         &ANAS0_P.Nocorrection_k);

      // End of Outputs for SubSystem: '<S54>/No correction'
    }

    // End of If: '<S54>/If'

    // Sum: '<S57>/Subtract' incorporates:
    //   Constant: '<S57>/Constant'
    //   Math: '<S13>/Transpose'
    //   Product: '<S13>/Matrix Multiply'
    //   Product: '<S57>/Matrix Multiply'

    for (i_0 = 0; i_0 < 3; i_0++) {
      // Product: '<S57>/Matrix Multiply' incorporates:
      //   Merge: '<S54>/Merge'

      rtb_Bias3 = rtb_BackwardSubstitution_e[i_0 + 3];
      rtb_Matrix1Norm2_j = rtb_BackwardSubstitution_e[i_0];
      rtb_Matrix1Norm1_k = rtb_BackwardSubstitution_e[i_0 + 6];
      k = 0;
      for (i = 0; i < 3; i++) {
        rtb_LinearQ_tmp = k + i_0;
        rtb_MatrixMultiply_i[rtb_LinearQ_tmp] =
          ANAS0_P.Constant_Value_l[rtb_LinearQ_tmp] - ((rtb_Transpose_g[k + 1] *
          rtb_Bias3 + rtb_Transpose_g[k] * rtb_Matrix1Norm2_j) +
          rtb_Transpose_g[k + 2] * rtb_Matrix1Norm1_k);
        k += 3;
      }
    }

    // End of Sum: '<S57>/Subtract'
    for (i_0 = 0; i_0 < 3; i_0++) {
      // Product: '<S53>/Matrix Multiply2' incorporates:
      //   Product: '<S53>/Matrix Multiply'
      //   Sum: '<S17>/Subtract'

      rtb_Divide1_p = rtb_MatrixMultiply2[i_0 + 3];
      rtb_Add = rtb_MatrixMultiply2[i_0];
      rtb_Power = rtb_MatrixMultiply2[i_0 + 6];

      // Product: '<S53>/Matrix Multiply' incorporates:
      //   Math: '<S53>/Transpose'
      //   Math: '<S53>/Transpose1'
      //   Merge: '<S54>/Merge'
      //   Product: '<S13>/Matrix Multiply'
      //   Product: '<S53>/Matrix Multiply2'
      //   Sum: '<S3>/Sum2'

      rtb_Matrix1Norm2_j = rtb_Sum2[i_0 + 3];
      rtb_Matrix1Norm1_k = rtb_Sum2[i_0];
      rtb_Bias3 = rtb_Sum2[i_0 + 6];
      k = 0;
      for (i = 0; i < 3; i++) {
        rtb_LinearQ_tmp = k + i_0;
        rtb_S_jt[rtb_LinearQ_tmp] = (rtb_BackwardSubstitution_e[i + 3] *
          rtb_Divide1_p + rtb_Add * rtb_BackwardSubstitution_e[i]) +
          rtb_BackwardSubstitution_e[i + 6] * rtb_Power;
        rtb_LUFactorization_o1_c[rtb_LinearQ_tmp] = (rtb_MatrixMultiply_i[i + 3]
          * rtb_Matrix1Norm2_j + rtb_Matrix1Norm1_k * rtb_MatrixMultiply_i[i]) +
          rtb_MatrixMultiply_i[i + 6] * rtb_Bias3;
        k += 3;
      }
    }

    // Product: '<S53>/Matrix Multiply' incorporates:
    //   Merge: '<S54>/Merge'
    //   Product: '<S13>/Matrix Multiply'
    //   Product: '<S53>/Matrix Multiply2'

    i_0 = 0;
    for (k = 0; k < 3; k++) {
      rtb_Divide1_p = rtb_S_jt[i_0 + 1];
      rtb_Add = rtb_S_jt[i_0];
      rtb_Power = rtb_S_jt[i_0 + 2];
      rtb_Matrix1Norm2_j = rtb_LUFactorization_o1_c[i_0 + 1];
      rtb_Matrix1Norm1_k = rtb_LUFactorization_o1_c[i_0];
      rtb_Bias3 = rtb_LUFactorization_o1_c[i_0 + 2];
      for (i = 0; i < 3; i++) {
        rtb_LinearQ_tmp = i + i_0;
        rtb_Sum2[rtb_LinearQ_tmp] = (rtb_BackwardSubstitution_e[i + 3] *
          rtb_Divide1_p + rtb_Add * rtb_BackwardSubstitution_e[i]) +
          rtb_BackwardSubstitution_e[i + 6] * rtb_Power;
        rtb_MatrixMultiply2[rtb_LinearQ_tmp] = (rtb_MatrixMultiply_i[i + 3] *
          rtb_Matrix1Norm2_j + rtb_Matrix1Norm1_k * rtb_MatrixMultiply_i[i]) +
          rtb_MatrixMultiply_i[i + 6] * rtb_Bias3;
      }

      i_0 += 3;
    }

    for (i_0 = 0; i_0 < 9; i_0++) {
      // Merge: '<S9>/Merge1' incorporates:
      //   Sum: '<S53>/Sum1'

      ANAS0_DW.Merge1_e[i_0] = rtb_MatrixMultiply2[i_0] + rtb_Sum2[i_0];
    }

    if ((&ANAS0_M)->Timing.TaskCounters.TID[1] == 0) {
      // Switch: '<S74>/Switch' incorporates:
      //   Constant: '<S74>/Constant'
      //   Constant: '<S74>/dxMatrix'
      //   Constant: '<S74>/sxMatrix'

      for (i_0 = 0; i_0 < 9; i_0++) {
        if (ANAS0_P.Constant_Value_at) {
          rtb_Sum2[i_0] = ANAS0_P.sxMatrix_Value[i_0];
        } else {
          rtb_Sum2[i_0] = ANAS0_P.dxMatrix_Value[i_0];
        }
      }

      // End of Switch: '<S74>/Switch'

      // Product: '<S56>/Matrix Multiply' incorporates:
      //   Inport: '<Root>/ANAS In'

      rtb_Matrix1Norm2_j = ANAS0_U.ANASIn_c.MagMeasure[1];
      rtb_Matrix1Norm1_k = ANAS0_U.ANASIn_c.MagMeasure[0];
      rtb_Bias3 = ANAS0_U.ANASIn_c.MagMeasure[2];
      for (i_0 = 0; i_0 < 3; i_0++) {
        // Gain: '<S56>/from G (Gauss) to nT (Tesla)'
        ANAS0_DW.fromGGausstonTTesla[i_0] = ((rtb_Sum2[i_0 + 3] *
          rtb_Matrix1Norm2_j + rtb_Sum2[i_0] * rtb_Matrix1Norm1_k) +
          rtb_Sum2[i_0 + 6] * rtb_Bias3) * ANAS0_P.fromGGausstonTTesla_Gain;
      }

      // End of Product: '<S56>/Matrix Multiply'
    }

    // Sum: '<S56>/Add1' incorporates:
    //   Math: '<S65>/Transpose'

    rtb_Matrix1Norm2_j = ANAS0_DW.fromGGausstonTTesla[0] - rtb_Transpose_c[0];
    rtb_Matrix1Norm1_k = ANAS0_DW.fromGGausstonTTesla[1] - rtb_Transpose_c[1];
    rtb_Add = ANAS0_DW.fromGGausstonTTesla[2] - rtb_Transpose_c[2];

    // DotProduct: '<S65>/Dot Product'
    rtb_Bias3 = 0.0F;
    for (i = 0; i < 3; i++) {
      // Product: '<S55>/Matrix Multiply' incorporates:
      //   Math: '<S25>/Transpose'
      //   Merge: '<S54>/Merge'

      rtb_Divide1_p = (rtb_BackwardSubstitution_e[i + 3] * rtb_Matrix1Norm1_k +
                       rtb_BackwardSubstitution_e[i] * rtb_Matrix1Norm2_j) +
        rtb_BackwardSubstitution_e[i + 6] * rtb_Add;

      // Gain: '<S55>/Gain1' incorporates:
      //   Math: '<S25>/Transpose'
      //   Product: '<S55>/Matrix Multiply'

      rtb_res_GPS[i + 1] = ANAS0_P.Gain1_Gain_j * rtb_Divide1_p;

      // DotProduct: '<S65>/Dot Product' incorporates:
      //   Math: '<S25>/Transpose'
      //   Product: '<S55>/Matrix Multiply'

      rtb_Bias3 += rtb_Divide1_p * rtb_Divide1_p;
    }

    // Sqrt: '<S65>/Sqrt' incorporates:
    //   DotProduct: '<S65>/Dot Product'

    rtb_Bias3 = std::sqrt(rtb_Bias3);

    // Sqrt: '<S55>/Sqrt' incorporates:
    //   Bias: '<S55>/Bias'
    //   Gain: '<S55>/Gain'
    //   Math: '<S55>/Square'

    rtb_res_GPS[0] = std::sqrt(rtb_Bias3 * rtb_Bias3 * ANAS0_P.Gain_Gain_p +
      ANAS0_P.Bias_Bias_d);

    // Sqrt: '<S70>/sqrt' incorporates:
    //   Product: '<S71>/Product'
    //   Product: '<S71>/Product1'
    //   Product: '<S71>/Product2'
    //   Product: '<S71>/Product3'
    //   Sum: '<S71>/Sum'

    rtb_Bias3 = std::sqrt(((rtb_res_GPS[0] * rtb_res_GPS[0] + rtb_res_GPS[1] *
      rtb_res_GPS[1]) + rtb_res_GPS[2] * rtb_res_GPS[2]) + rtb_res_GPS[3] *
                          rtb_res_GPS[3]);

    // Product: '<S64>/Product'
    rtb_Power = rtb_res_GPS[0] / rtb_Bias3;

    // Product: '<S64>/Product1'
    rtb_Add = rtb_res_GPS[1] / rtb_Bias3;

    // Product: '<S64>/Product2'
    rtb_Divide1_p = rtb_res_GPS[2] / rtb_Bias3;

    // Product: '<S64>/Product3'
    rtb_Bias3 = rtb_res_GPS[3] / rtb_Bias3;

    // Sum: '<S66>/Sum' incorporates:
    //   Merge: '<S9>/Merge'
    //   Product: '<S66>/Product'
    //   Product: '<S66>/Product1'
    //   Product: '<S66>/Product2'
    //   Product: '<S66>/Product3'

    ANAS0_DW.Merge_l[0] = ((rtb_Power2 * rtb_Power - rtb_Product1_j3 * rtb_Add)
      - rtb_Product2_k0 * rtb_Divide1_p) - rtb_Product3_jl * rtb_Bias3;

    // Sum: '<S67>/Sum' incorporates:
    //   Merge: '<S9>/Merge'
    //   Product: '<S67>/Product'
    //   Product: '<S67>/Product1'
    //   Product: '<S67>/Product2'
    //   Product: '<S67>/Product3'

    ANAS0_DW.Merge_l[1] = ((rtb_Power2 * rtb_Add + rtb_Product1_j3 * rtb_Power)
      + rtb_Product2_k0 * rtb_Bias3) - rtb_Product3_jl * rtb_Divide1_p;

    // Sum: '<S68>/Sum' incorporates:
    //   Merge: '<S9>/Merge'
    //   Product: '<S68>/Product'
    //   Product: '<S68>/Product1'
    //   Product: '<S68>/Product2'
    //   Product: '<S68>/Product3'

    ANAS0_DW.Merge_l[2] = ((rtb_Power2 * rtb_Divide1_p - rtb_Product1_j3 *
      rtb_Bias3) + rtb_Product2_k0 * rtb_Power) + rtb_Product3_jl * rtb_Add;

    // Sum: '<S69>/Sum' incorporates:
    //   Merge: '<S9>/Merge'
    //   Product: '<S69>/Product'
    //   Product: '<S69>/Product1'
    //   Product: '<S69>/Product2'
    //   Product: '<S69>/Product3'

    ANAS0_DW.Merge_l[3] = ((rtb_Power2 * rtb_Bias3 + rtb_Product1_j3 *
      rtb_Divide1_p) - rtb_Product2_k0 * rtb_Add) + rtb_Product3_jl * rtb_Power;
  } else {
    // Outputs for Enabled SubSystem: '<S9>/No Correction Step' incorporates:
    //   EnablePort: '<S52>/Enable'

    // Merge: '<S9>/Merge' incorporates:
    //   SignalConversion generated from: '<S52>/nextAngularState'

    ANAS0_DW.Merge_l[0] = rtb_Power2;
    ANAS0_DW.Merge_l[1] = rtb_Product1_j3;
    ANAS0_DW.Merge_l[2] = rtb_Product2_k0;
    ANAS0_DW.Merge_l[3] = rtb_Product3_jl;
    for (i = 0; i < 9; i++) {
      // Merge: '<S9>/Merge1' incorporates:
      //   SignalConversion generated from: '<S52>/nextAngularCov'
      //   Sum: '<S3>/Sum2'

      ANAS0_DW.Merge1_e[i] = rtb_Sum2[i];
    }

    // End of Outputs for SubSystem: '<S9>/No Correction Step'
  }

  // End of Outputs for SubSystem: '<S9>/Active Correction Step Magnetometer'

  // Outputs for Enabled SubSystem: '<S8>/Active Correction Step Accelerometer' incorporates:
  //   EnablePort: '<S10>/Enable'

  // Outputs for Enabled SubSystem: '<S8>/No Correction Step' incorporates:
  //   EnablePort: '<S12>/Enable'

  if (rtb_AND_b) {
    // Math: '<S10>/Square' incorporates:
    //   Constant: '<S10>/AccSigma'
    //   Sum: '<S17>/Subtract'

    for (i = 0; i < 9; i++) {
      rtb_Matrix1Norm2_j = ANAS0_P.AccSigma_Value[i];
      rtb_MatrixMultiply2[i] = rtb_Matrix1Norm2_j * rtb_Matrix1Norm2_j;
    }

    // End of Math: '<S10>/Square'

    // SignalConversion generated from: '<S47>/Vector Concatenate' incorporates:
    //   Constant: '<S33>/Constant'

    rtb_ExtractDiagonal[0] = ANAS0_P.Constant_Value_k;

    // Sqrt: '<S45>/sqrt' incorporates:
    //   Product: '<S46>/Product'
    //   Product: '<S46>/Product1'
    //   Product: '<S46>/Product2'
    //   Product: '<S46>/Product3'
    //   Sum: '<S46>/Sum'

    rtb_Bias3 = std::sqrt(((ANAS0_DW.Merge_l[0] * ANAS0_DW.Merge_l[0] +
      ANAS0_DW.Merge_l[1] * ANAS0_DW.Merge_l[1]) + ANAS0_DW.Merge_l[2] *
      ANAS0_DW.Merge_l[2]) + ANAS0_DW.Merge_l[3] * ANAS0_DW.Merge_l[3]);

    // Product: '<S44>/Product'
    rtb_Power = ANAS0_DW.Merge_l[0] / rtb_Bias3;

    // Product: '<S44>/Product1'
    rtb_Add = ANAS0_DW.Merge_l[1] / rtb_Bias3;

    // Product: '<S44>/Product2'
    rtb_Divide1_p = ANAS0_DW.Merge_l[2] / rtb_Bias3;

    // Product: '<S44>/Product3'
    rtb_Bias3 = ANAS0_DW.Merge_l[3] / rtb_Bias3;

    // Product: '<S34>/Product3' incorporates:
    //   Product: '<S38>/Product3'

    rtb_Matrix1Norm2_j = rtb_Power * rtb_Power;

    // Product: '<S34>/Product2' incorporates:
    //   Product: '<S38>/Product2'

    rtb_Matrix1Norm1_k = rtb_Add * rtb_Add;

    // Product: '<S34>/Product1' incorporates:
    //   Product: '<S38>/Product1'
    //   Product: '<S42>/Product1'

    rtb_Power2 = rtb_Divide1_p * rtb_Divide1_p;

    // Product: '<S34>/Product' incorporates:
    //   Product: '<S38>/Product'
    //   Product: '<S42>/Product'

    rtb_Product1_j3 = rtb_Bias3 * rtb_Bias3;

    // Sum: '<S34>/Sum' incorporates:
    //   Product: '<S34>/Product'
    //   Product: '<S34>/Product1'
    //   Product: '<S34>/Product2'
    //   Product: '<S34>/Product3'

    rtb_VectorConcatenate_i[0] = ((rtb_Matrix1Norm2_j + rtb_Matrix1Norm1_k) -
      rtb_Power2) - rtb_Product1_j3;

    // Product: '<S37>/Product3' incorporates:
    //   Product: '<S35>/Product3'

    rtb_Product2_k0 = rtb_Bias3 * rtb_Power;

    // Product: '<S37>/Product2' incorporates:
    //   Product: '<S35>/Product2'

    rtb_Product3_jl = rtb_Add * rtb_Divide1_p;

    // Gain: '<S37>/Gain' incorporates:
    //   Product: '<S37>/Product2'
    //   Product: '<S37>/Product3'
    //   Sum: '<S37>/Sum'

    rtb_VectorConcatenate_i[1] = (rtb_Product3_jl - rtb_Product2_k0) *
      ANAS0_P.Gain_Gain;

    // Product: '<S40>/Product2' incorporates:
    //   Product: '<S36>/Product2'

    rtb_ExtractDiagonal_tmp_1 = rtb_Add * rtb_Bias3;

    // Product: '<S40>/Product1' incorporates:
    //   Product: '<S36>/Product1'

    rtb_ExtractDiagonal_tmp_2 = rtb_Power * rtb_Divide1_p;

    // Gain: '<S40>/Gain' incorporates:
    //   Product: '<S40>/Product1'
    //   Product: '<S40>/Product2'
    //   Sum: '<S40>/Sum'

    rtb_VectorConcatenate_i[2] = (rtb_ExtractDiagonal_tmp_2 +
      rtb_ExtractDiagonal_tmp_1) * ANAS0_P.Gain_Gain_o;

    // Gain: '<S35>/Gain' incorporates:
    //   Sum: '<S35>/Sum'

    rtb_VectorConcatenate_i[3] = (rtb_Product2_k0 + rtb_Product3_jl) *
      ANAS0_P.Gain_Gain_i;

    // Sum: '<S38>/Sum' incorporates:
    //   Sum: '<S42>/Sum'

    rtb_Matrix1Norm2_j -= rtb_Matrix1Norm1_k;
    rtb_VectorConcatenate_i[4] = (rtb_Matrix1Norm2_j + rtb_Power2) -
      rtb_Product1_j3;

    // Product: '<S41>/Product1' incorporates:
    //   Product: '<S39>/Product1'

    rtb_Matrix1Norm1_k = rtb_Power * rtb_Add;

    // Product: '<S41>/Product2' incorporates:
    //   Product: '<S39>/Product2'

    rtb_Product2_k0 = rtb_Divide1_p * rtb_Bias3;

    // Gain: '<S41>/Gain' incorporates:
    //   Product: '<S41>/Product1'
    //   Product: '<S41>/Product2'
    //   Sum: '<S41>/Sum'

    rtb_VectorConcatenate_i[5] = (rtb_Product2_k0 - rtb_Matrix1Norm1_k) *
      ANAS0_P.Gain_Gain_b;

    // Gain: '<S36>/Gain' incorporates:
    //   Sum: '<S36>/Sum'

    rtb_VectorConcatenate_i[6] = (rtb_ExtractDiagonal_tmp_1 -
      rtb_ExtractDiagonal_tmp_2) * ANAS0_P.Gain_Gain_h;

    // Gain: '<S39>/Gain' incorporates:
    //   Sum: '<S39>/Sum'

    rtb_VectorConcatenate_i[7] = (rtb_Matrix1Norm1_k + rtb_Product2_k0) *
      ANAS0_P.Gain_Gain_g;

    // Sum: '<S42>/Sum'
    rtb_VectorConcatenate_i[8] = (rtb_Matrix1Norm2_j - rtb_Power2) +
      rtb_Product1_j3;

    // Product: '<S16>/Matrix Multiply1' incorporates:
    //   Constant: '<S16>/Local Gravity'

    rtb_Matrix1Norm2_j = ANAS0_P.LocalGravity_Value[1];
    rtb_Matrix1Norm1_k = ANAS0_P.LocalGravity_Value[0];
    rtb_Power2 = ANAS0_P.LocalGravity_Value[2];
    for (i_0 = 0; i_0 < 3; i_0++) {
      // Math: '<S25>/Transpose' incorporates:
      //   Concatenate: '<S43>/Vector Concatenate'

      rtb_LUFactorization_o2_m[i_0] = (rtb_VectorConcatenate_i[i_0 + 3] *
        rtb_Matrix1Norm2_j + rtb_VectorConcatenate_i[i_0] * rtb_Matrix1Norm1_k)
        + rtb_VectorConcatenate_i[i_0 + 6] * rtb_Power2;
    }

    // End of Product: '<S16>/Matrix Multiply1'

    // Gain: '<S33>/Gain1'
    rtb_ExtractDiagonal[1] = ANAS0_P.Gain1_Gain * rtb_LUFactorization_o2_m[2];

    // SignalConversion generated from: '<S47>/Vector Concatenate'
    rtb_ExtractDiagonal[2] = rtb_LUFactorization_o2_m[1];

    // SignalConversion generated from: '<S47>/Vector Concatenate'
    rtb_ExtractDiagonal[3] = rtb_LUFactorization_o2_m[2];

    // SignalConversion generated from: '<S47>/Vector Concatenate' incorporates:
    //   Constant: '<S33>/Constant'

    rtb_ExtractDiagonal[4] = ANAS0_P.Constant_Value_k;

    // Gain: '<S33>/Gain2'
    rtb_ExtractDiagonal[5] = ANAS0_P.Gain2_Gain * rtb_LUFactorization_o2_m[0];

    // Gain: '<S33>/Gain'
    rtb_ExtractDiagonal[6] = ANAS0_P.Gain_Gain_j * rtb_LUFactorization_o2_m[1];

    // SignalConversion generated from: '<S47>/Vector Concatenate'
    rtb_ExtractDiagonal[7] = rtb_LUFactorization_o2_m[0];

    // SignalConversion generated from: '<S47>/Vector Concatenate' incorporates:
    //   Constant: '<S33>/Constant'

    rtb_ExtractDiagonal[8] = ANAS0_P.Constant_Value_k;

    // Math: '<S33>/Transpose' incorporates:
    //   Math: '<S13>/Transpose'
    //   S-Function (sdspdiag2): '<S6>/Extract Diagonal'

    i_0 = 0;
    for (k = 0; k < 3; k++) {
      rtb_Transpose_g[i_0] = rtb_ExtractDiagonal[k];
      rtb_Transpose_g[i_0 + 1] = rtb_ExtractDiagonal[k + 3];
      rtb_Transpose_g[i_0 + 2] = rtb_ExtractDiagonal[k + 6];
      i_0 += 3;
    }

    // End of Math: '<S33>/Transpose'
    for (i_0 = 0; i_0 < 3; i_0++) {
      // Product: '<S14>/Matrix Multiply' incorporates:
      //   Math: '<S13>/Transpose'
      //   Math: '<S14>/Transpose'
      //   Merge: '<S9>/Merge1'
      //   Product: '<S14>/Matrix Multiply1'

      rtb_Matrix1Norm2_j = ANAS0_DW.Merge1_e[i_0 + 3];
      rtb_Matrix1Norm1_k = ANAS0_DW.Merge1_e[i_0];
      rtb_Power2 = ANAS0_DW.Merge1_e[i_0 + 6];
      k = 0;
      for (i = 0; i < 3; i++) {
        rtb_Sum2[k + i_0] = (rtb_Transpose_g[i + 3] * rtb_Matrix1Norm2_j +
                             rtb_Matrix1Norm1_k * rtb_Transpose_g[i]) +
          rtb_Transpose_g[i + 6] * rtb_Power2;
        k += 3;
      }
    }

    for (i_0 = 0; i_0 < 3; i_0++) {
      // Sum: '<S14>/Add' incorporates:
      //   Math: '<S13>/Transpose'
      //   Product: '<S14>/Matrix Multiply'

      rtb_Bias3 = rtb_Transpose_g[i_0 + 3];
      rtb_Divide1_p = rtb_Transpose_g[i_0];
      rtb_Add = rtb_Transpose_g[i_0 + 6];

      // Sum: '<S14>/Add' incorporates:
      //   Product: '<S14>/Matrix Multiply'
      //   Sum: '<S17>/Subtract'

      k = 0;
      for (i = 0; i < 3; i++) {
        rtb_LinearQ_tmp = k + i_0;
        rtb_VectorConcatenate_i[rtb_LinearQ_tmp] = ((rtb_Sum2[k + 1] * rtb_Bias3
          + rtb_Sum2[k] * rtb_Divide1_p) + rtb_Sum2[k + 2] * rtb_Add) +
          rtb_MatrixMultiply2[rtb_LinearQ_tmp];
        k += 3;
      }
    }

    // S-Function (sdsplu2): '<S22>/LU Factorization' incorporates:
    //   Sum: '<S14>/Add'

    for (k = 0; k < 9; k++) {
      rtb_ExtractDiagonal[k] = rtb_VectorConcatenate_i[k];
    }

    LUf_boolfloatint32_t(&rtb_ExtractDiagonal[0], &rtb_LUFactorization_o2_o[0],
                         3, &rtb_LUFactorization_o3_i);

    // End of S-Function (sdsplu2): '<S22>/LU Factorization'

    // Switch: '<S20>/Switch' incorporates:
    //   Constant: '<S20>/Constant'
    //   Math: '<S20>/Math Function'
    //   Product: '<S20>/Product'
    //
    //  About '<S20>/Math Function':
    //   Operator: reciprocal

    if (rtb_LUFactorization_o3_i) {
      rtb_Bias3 = ANAS0_P.Constant_Value;
    } else {
      // S-Function (sdspm1norm2): '<S20>/Matrix  1-Norm2'

      // DSP System Toolbox Matrix 1-Norm (sdspm1norm2) - '<S20>/Matrix  1-Norm2' 
      {
        const float *uPtr{ &rtb_VectorConcatenate_i[0] };

        float m1norm{ 0.0 };

        int jdx;
        for (jdx=3; jdx-- > 0; ) {
          float sumabsAj{ 0.0 };

          int idxFlt;
          for (idxFlt=3; idxFlt-- > 0; ) {
            float temp{ *uPtr++ };

            sumabsAj += fabsf(temp);
          }

          m1norm = MAX(m1norm, sumabsAj);
        }

        rtb_Matrix1Norm2_om = m1norm;
      }

      // S-Function (sdspperm2): '<S22>/Permute Matrix' incorporates:
      //   IdentityMatrix: '<S22>/Identity Matrix'
      //   S-Function (sdspfbsub2): '<S22>/Backward Substitution'

      rtb_Matrix1Norm2_j = rtb_LUFactorization_o2_o[0];
      rtb_Matrix1Norm1_k = rtb_LUFactorization_o2_o[1];
      rtb_Power2 = rtb_LUFactorization_o2_o[2];
      for (k = 0; k < 3; k++) {
        i_0 = static_cast<int32_t>(std::floor(rtb_Matrix1Norm2_j)) - 1;
        if (i_0 < 0) {
          i_0 = 0;
        } else if (i_0 >= 3) {
          i_0 = 2;
        }

        rtb_BackwardSubstitution_e[3 * k] = ANAS0_DW.IdentityMatrix_h[3 * k +
          i_0];
        i_0 = static_cast<int32_t>(std::floor(rtb_Matrix1Norm1_k)) - 1;
        if (i_0 < 0) {
          i_0 = 0;
        } else if (i_0 >= 3) {
          i_0 = 2;
        }

        rtb_BackwardSubstitution_e[3 * k + 1] = ANAS0_DW.IdentityMatrix_h[3 * k
          + i_0];
        i_0 = static_cast<int32_t>(std::floor(rtb_Power2)) - 1;
        if (i_0 < 0) {
          i_0 = 0;
        } else if (i_0 >= 3) {
          i_0 = 2;
        }

        rtb_BackwardSubstitution_e[3 * k + 2] = ANAS0_DW.IdentityMatrix_h[3 * k
          + i_0];
      }

      // End of S-Function (sdspperm2): '<S22>/Permute Matrix'

      // S-Function (sdspfbsub2): '<S22>/Forward Substitution' incorporates:
      //   S-Function (sdspfbsub2): '<S22>/Backward Substitution'
      //   S-Function (sdsplu2): '<S22>/LU Factorization'

      for (i = 0; i < 3; i++) {
        i_0 = i * 3;
        rtb_BackwardSubstitution_e[i_0 + 1] -= rtb_ExtractDiagonal[1] *
          rtb_BackwardSubstitution_e[i_0];
        rtb_LinearQ_tmp = 2;
        rtb_Add = rtb_BackwardSubstitution_e[i_0 + 2];
        for (k = 0; k < 2; k++) {
          rtb_Add -= rtb_BackwardSubstitution_e[i_0 + k] *
            rtb_ExtractDiagonal[rtb_LinearQ_tmp];
          rtb_LinearQ_tmp += 3;
        }

        rtb_BackwardSubstitution_e[i_0 + 2] = rtb_Add;
      }

      // End of S-Function (sdspfbsub2): '<S22>/Forward Substitution'

      // S-Function (sdspfbsub2): '<S22>/Backward Substitution' incorporates:
      //   S-Function (sdsplu2): '<S22>/LU Factorization'

      for (i = 0; i < 3; i++) {
        i_0 = i * 3;
        rtb_Matrix1Norm2_j = rtb_BackwardSubstitution_e[i_0 + 2] /
          rtb_ExtractDiagonal[8];

        // S-Function (sdspfbsub2): '<S22>/Backward Substitution' incorporates:
        //   S-Function (sdsplu2): '<S22>/LU Factorization'

        rtb_BackwardSubstitution_e[i_0 + 2] = rtb_Matrix1Norm2_j;
        rtb_BackwardSubstitution_e[i_0 + 1] = (rtb_BackwardSubstitution_e[i_0 +
          1] - rtb_Matrix1Norm2_j * rtb_ExtractDiagonal[7]) /
          rtb_ExtractDiagonal[4];
        rtb_LinearQ_tmp = 6;
        rtb_Add = rtb_BackwardSubstitution_e[i_0];
        for (k = 2; k > 0; k--) {
          rtb_Add -= rtb_BackwardSubstitution_e[i_0 + k] *
            rtb_ExtractDiagonal[rtb_LinearQ_tmp];
          rtb_LinearQ_tmp -= 3;
        }

        rtb_BackwardSubstitution_e[i_0] = rtb_Add /
          rtb_ExtractDiagonal[rtb_LinearQ_tmp];
      }

      // End of S-Function (sdspfbsub2): '<S22>/Backward Substitution'

      // S-Function (sdspm1norm2): '<S20>/Matrix  1-Norm1'

      // DSP System Toolbox Matrix 1-Norm (sdspm1norm2) - '<S20>/Matrix  1-Norm1' 
      {
        const float *uPtr{ &rtb_BackwardSubstitution_e[0] };

        float m1norm{ 0.0 };

        int jdx;
        for (jdx=3; jdx-- > 0; ) {
          float sumabsAj{ 0.0 };

          int idxFlt;
          for (idxFlt=3; idxFlt-- > 0; ) {
            float temp{ *uPtr++ };

            sumabsAj += fabsf(temp);
          }

          m1norm = MAX(m1norm, sumabsAj);
        }

        rtb_Matrix1Norm1_p = m1norm;
      }

      rtb_Bias3 = 1.0F / (rtb_Matrix1Norm1_p * rtb_Matrix1Norm2_om);
    }

    // End of Switch: '<S20>/Switch'

    // If: '<S14>/If' incorporates:
    //   Constant: '<S14>/NASCondLim'
    //   RelationalOperator: '<S14>/GreaterThanOrEqual'

    if (rtb_Bias3 >= ANAS0_P.NASCondLim_Value) {
      for (i = 0; i < 9; i++) {
        // Outputs for IfAction SubSystem: '<S14>/Correction' incorporates:
        //   ActionPort: '<S18>/Action Port'

        rtb_BackwardSubstitution_e[i] = rtb_VectorConcatenate_i[i];

        // End of Outputs for SubSystem: '<S14>/Correction'
      }

      // Outputs for IfAction SubSystem: '<S14>/Correction' incorporates:
      //   ActionPort: '<S18>/Action Port'

      ANAS0_Correction(rtb_BackwardSubstitution_e, rtb_Sum2, rtb_ExtractDiagonal);

      // End of Outputs for SubSystem: '<S14>/Correction'
    } else {
      for (i = 0; i < 9; i++) {
        // Outputs for IfAction SubSystem: '<S14>/No correction' incorporates:
        //   ActionPort: '<S19>/Action Port'

        rtb_BackwardSubstitution_e[i] = rtb_VectorConcatenate_i[i];

        // End of Outputs for SubSystem: '<S14>/No correction'
      }

      // Outputs for IfAction SubSystem: '<S14>/No correction' incorporates:
      //   ActionPort: '<S19>/Action Port'

      ANAS0_Nocorrection(rtb_Sum2, rtb_ExtractDiagonal, &ANAS0_P.Nocorrection);

      // End of Outputs for SubSystem: '<S14>/No correction'
    }

    // End of If: '<S14>/If'
    for (i_0 = 0; i_0 < 3; i_0++) {
      // Product: '<S13>/Matrix Multiply2' incorporates:
      //   Sum: '<S17>/Subtract'

      rtb_Divide1_p = rtb_MatrixMultiply2[i_0 + 3];
      rtb_Add = rtb_MatrixMultiply2[i_0];
      rtb_Power = rtb_MatrixMultiply2[i_0 + 6];

      // Sum: '<S17>/Subtract' incorporates:
      //   Merge: '<S14>/Merge'
      //   Product: '<S17>/Matrix Multiply'

      rtb_Matrix1Norm2_om = rtb_ExtractDiagonal[i_0 + 3];
      rtb_Matrix1Norm1_p = rtb_ExtractDiagonal[i_0];
      rtb_Matrix1Norm2_j = rtb_ExtractDiagonal[i_0 + 6];
      for (k = 0; k < 3; k++) {
        // Product: '<S13>/Matrix Multiply2' incorporates:
        //   Math: '<S13>/Transpose1'
        //   Merge: '<S14>/Merge'

        rtb_BackwardSubstitution_e[i_0 + 3 * k] = (rtb_ExtractDiagonal[k + 3] *
          rtb_Divide1_p + rtb_Add * rtb_ExtractDiagonal[k]) +
          rtb_ExtractDiagonal[k + 6] * rtb_Power;

        // Sum: '<S17>/Subtract' incorporates:
        //   Constant: '<S17>/Constant'
        //   Math: '<S13>/Transpose'
        //   Product: '<S17>/Matrix Multiply'

        rtb_LinearQ_tmp = 3 * k + i_0;
        rtb_MatrixMultiply2[rtb_LinearQ_tmp] =
          ANAS0_P.Constant_Value_b[rtb_LinearQ_tmp] - ((rtb_Transpose_g[3 * k +
          1] * rtb_Matrix1Norm2_om + rtb_Transpose_g[3 * k] * rtb_Matrix1Norm1_p)
          + rtb_Transpose_g[3 * k + 2] * rtb_Matrix1Norm2_j);
      }
    }

    // Product: '<S13>/Matrix Multiply' incorporates:
    //   Math: '<S13>/Transpose'
    //   Merge: '<S14>/Merge'
    //   Merge: '<S9>/Merge1'
    //   Product: '<S13>/Matrix Multiply2'
    //   Sum: '<S17>/Subtract'

    for (i_0 = 0; i_0 < 3; i_0++) {
      rtb_Matrix1Norm2_j = ANAS0_DW.Merge1_e[i_0 + 3];
      rtb_Matrix1Norm1_k = ANAS0_DW.Merge1_e[i_0];
      rtb_Power2 = ANAS0_DW.Merge1_e[i_0 + 6];
      k = 0;
      for (i = 0; i < 3; i++) {
        rtb_Sum2[k + i_0] = (rtb_MatrixMultiply2[i + 3] * rtb_Matrix1Norm2_j +
                             rtb_Matrix1Norm1_k * rtb_MatrixMultiply2[i]) +
          rtb_MatrixMultiply2[i + 6] * rtb_Power2;
        k += 3;
      }
    }

    i_0 = 0;
    for (k = 0; k < 3; k++) {
      rtb_Matrix1Norm2_j = rtb_BackwardSubstitution_e[i_0 + 1];
      rtb_Matrix1Norm2_om = rtb_BackwardSubstitution_e[i_0];
      rtb_Matrix1Norm1_p = rtb_BackwardSubstitution_e[i_0 + 2];
      rtb_Power2 = rtb_Sum2[i_0 + 1];
      rtb_Matrix1Norm1_k = rtb_Sum2[i_0];
      rtb_Product1_j3 = rtb_Sum2[i_0 + 2];
      for (i = 0; i < 3; i++) {
        rtb_LinearQ_tmp = i + i_0;
        rtb_VectorConcatenate_i[rtb_LinearQ_tmp] = (rtb_ExtractDiagonal[i + 3] *
          rtb_Matrix1Norm2_j + rtb_Matrix1Norm2_om * rtb_ExtractDiagonal[i]) +
          rtb_ExtractDiagonal[i + 6] * rtb_Matrix1Norm1_p;
        rtb_S_jt[rtb_LinearQ_tmp] = (rtb_MatrixMultiply2[i + 3] * rtb_Power2 +
          rtb_Matrix1Norm1_k * rtb_MatrixMultiply2[i]) + rtb_MatrixMultiply2[i +
          6] * rtb_Product1_j3;
      }

      i_0 += 3;
    }

    // End of Product: '<S13>/Matrix Multiply'

    // Sum: '<S13>/Sum1' incorporates:
    //   Merge: '<S8>/Merge1'

    for (i_0 = 0; i_0 < 9; i_0++) {
      rtb_Transpose_g[i_0] = rtb_S_jt[i_0] + rtb_VectorConcatenate_i[i_0];
    }

    // End of Sum: '<S13>/Sum1'

    // Sum: '<S16>/Add1' incorporates:
    //   Inport: '<Root>/ANAS In'
    //   Math: '<S25>/Transpose'

    rtb_Matrix1Norm2_j = ANAS0_U.ANASIn_c.AccMeasure[0] -
      rtb_LUFactorization_o2_m[0];
    rtb_Matrix1Norm1_k = ANAS0_U.ANASIn_c.AccMeasure[1] -
      rtb_LUFactorization_o2_m[1];
    rtb_Add = ANAS0_U.ANASIn_c.AccMeasure[2] - rtb_LUFactorization_o2_m[2];

    // DotProduct: '<S25>/Dot Product'
    rtb_Matrix1Norm2_om = 0.0F;
    for (i = 0; i < 3; i++) {
      // Product: '<S15>/Matrix Multiply' incorporates:
      //   Merge: '<S14>/Merge'

      rtb_Matrix1Norm1_p = (rtb_ExtractDiagonal[i + 3] * rtb_Matrix1Norm1_k +
                            rtb_ExtractDiagonal[i] * rtb_Matrix1Norm2_j) +
        rtb_ExtractDiagonal[i + 6] * rtb_Add;

      // Gain: '<S15>/Gain1' incorporates:
      //   Product: '<S15>/Matrix Multiply'

      rtb_res_GPS[i + 1] = ANAS0_P.Gain1_Gain_o * rtb_Matrix1Norm1_p;

      // DotProduct: '<S25>/Dot Product' incorporates:
      //   Product: '<S15>/Matrix Multiply'

      rtb_Matrix1Norm2_om += rtb_Matrix1Norm1_p * rtb_Matrix1Norm1_p;
    }

    // Sqrt: '<S25>/Sqrt' incorporates:
    //   DotProduct: '<S25>/Dot Product'

    rtb_Bias3 = std::sqrt(rtb_Matrix1Norm2_om);

    // Sqrt: '<S15>/Sqrt' incorporates:
    //   Bias: '<S15>/Bias'
    //   Gain: '<S15>/Gain'
    //   Math: '<S15>/Square'

    rtb_res_GPS[0] = std::sqrt(rtb_Bias3 * rtb_Bias3 * ANAS0_P.Gain_Gain_h5 +
      ANAS0_P.Bias_Bias);

    // Sqrt: '<S30>/sqrt' incorporates:
    //   Product: '<S31>/Product'
    //   Product: '<S31>/Product1'
    //   Product: '<S31>/Product2'
    //   Product: '<S31>/Product3'
    //   Sum: '<S31>/Sum'

    rtb_Bias3 = std::sqrt(((rtb_res_GPS[0] * rtb_res_GPS[0] + rtb_res_GPS[1] *
      rtb_res_GPS[1]) + rtb_res_GPS[2] * rtb_res_GPS[2]) + rtb_res_GPS[3] *
                          rtb_res_GPS[3]);

    // Product: '<S24>/Product'
    rtb_Power = rtb_res_GPS[0] / rtb_Bias3;

    // Product: '<S24>/Product1'
    rtb_Divide1_p = rtb_res_GPS[1] / rtb_Bias3;

    // Product: '<S24>/Product2'
    rtb_Power2 = rtb_res_GPS[2] / rtb_Bias3;

    // Product: '<S24>/Product3'
    rtb_Bias3 = rtb_res_GPS[3] / rtb_Bias3;

    // Sum: '<S26>/Sum' incorporates:
    //   Merge: '<S8>/Merge'
    //   Product: '<S26>/Product'
    //   Product: '<S26>/Product1'
    //   Product: '<S26>/Product2'
    //   Product: '<S26>/Product3'

    ANAS0_DW.Merge_h[0] = ((ANAS0_DW.Merge_l[0] * rtb_Power - ANAS0_DW.Merge_l[1]
      * rtb_Divide1_p) - ANAS0_DW.Merge_l[2] * rtb_Power2) - ANAS0_DW.Merge_l[3]
      * rtb_Bias3;

    // Sum: '<S27>/Sum' incorporates:
    //   Merge: '<S8>/Merge'
    //   Product: '<S27>/Product'
    //   Product: '<S27>/Product1'
    //   Product: '<S27>/Product2'
    //   Product: '<S27>/Product3'

    ANAS0_DW.Merge_h[1] = ((ANAS0_DW.Merge_l[0] * rtb_Divide1_p +
      ANAS0_DW.Merge_l[1] * rtb_Power) + ANAS0_DW.Merge_l[2] * rtb_Bias3) -
      ANAS0_DW.Merge_l[3] * rtb_Power2;

    // Sum: '<S28>/Sum' incorporates:
    //   Merge: '<S8>/Merge'
    //   Product: '<S28>/Product'
    //   Product: '<S28>/Product1'
    //   Product: '<S28>/Product2'
    //   Product: '<S28>/Product3'

    ANAS0_DW.Merge_h[2] = ((ANAS0_DW.Merge_l[0] * rtb_Power2 - ANAS0_DW.Merge_l
      [1] * rtb_Bias3) + ANAS0_DW.Merge_l[2] * rtb_Power) + ANAS0_DW.Merge_l[3] *
      rtb_Divide1_p;

    // Sum: '<S29>/Sum' incorporates:
    //   Merge: '<S8>/Merge'
    //   Product: '<S29>/Product'
    //   Product: '<S29>/Product1'
    //   Product: '<S29>/Product2'
    //   Product: '<S29>/Product3'

    ANAS0_DW.Merge_h[3] = ((ANAS0_DW.Merge_l[0] * rtb_Bias3 + ANAS0_DW.Merge_l[1]
      * rtb_Power2) - ANAS0_DW.Merge_l[2] * rtb_Divide1_p) + ANAS0_DW.Merge_l[3]
      * rtb_Power;
  } else {
    // Merge: '<S8>/Merge' incorporates:
    //   SignalConversion generated from: '<S12>/nextAngularState'

    ANAS0_DW.Merge_h[0] = ANAS0_DW.Merge_l[0];
    ANAS0_DW.Merge_h[1] = ANAS0_DW.Merge_l[1];
    ANAS0_DW.Merge_h[2] = ANAS0_DW.Merge_l[2];
    ANAS0_DW.Merge_h[3] = ANAS0_DW.Merge_l[3];

    // SignalConversion generated from: '<S12>/nextAngularCov' incorporates:
    //   Merge: '<S8>/Merge1'
    //   Merge: '<S9>/Merge1'

    for (i = 0; i < 9; i++) {
      rtb_Transpose_g[i] = ANAS0_DW.Merge1_e[i];
    }
  }

  // End of Outputs for SubSystem: '<S8>/No Correction Step'
  // End of Outputs for SubSystem: '<S8>/Active Correction Step Accelerometer'

  // Update for Memory: '<S11>/Memory' incorporates:
  //   Inport: '<Root>/ANAS In'

  ANAS0_DW.Memory_PreviousInput_k = ANAS0_U.ANASIn_c.AccTimestamp;

  // Update for Memory: '<S51>/Memory' incorporates:
  //   Inport: '<Root>/ANAS In'

  ANAS0_DW.Memory_PreviousInput_kq = ANAS0_U.ANASIn_c.MagTimestamp;

  // End of Outputs for SubSystem: '<S1>/Angular States Corrections'

  // Outputs for Atomic SubSystem: '<S1>/Linear States Corrections'
  // RateTransition: '<S203>/Rate Transition'
  if ((&ANAS0_M)->Timing.TaskCounters.TID[3] == 0) {
    // RateTransition: '<S203>/Rate Transition' incorporates:
    //   Inport: '<Root>/ANAS In'

    ANAS0_DW.RateTransition = ANAS0_U.ANASIn_c.GPSTimestamp;
  }

  // End of RateTransition: '<S203>/Rate Transition'

  // Logic: '<S203>/AND' incorporates:
  //   Constant: '<S203>/GPSCorrectionFlag'
  //   Constant: '<S217>/Constant'
  //   Inport: '<Root>/ANAS In'
  //   Memory: '<S203>/Memory'
  //   RateTransition: '<S203>/Rate Transition'
  //   RelationalOperator: '<S203>/Relational Operator'
  //   RelationalOperator: '<S217>/Compare'

  rtb_AND = (ANAS0_P.GPSCorrectionFlag_Value && (ANAS0_DW.RateTransition >
              ANAS0_DW.Memory_PreviousInput) && (rtb_Divide5_a <=
              ANAS0_P.GPSAccelerationLimit_const) && ANAS0_U.ANASIn_c.GPSFix);

  // Outputs for Enabled SubSystem: '<S97>/Active Correction Step GPS' incorporates:
  //   EnablePort: '<S202>/Enable'

  // Outputs for Enabled SubSystem: '<S97>/No Correction Step' incorporates:
  //   EnablePort: '<S204>/Enable'

  if (rtb_AND) {
    // Gain: '<S210>/Gain1' incorporates:
    //   Bias: '<S206>/GPSlat0'
    //   Gain: '<S206>/GPS1//a 1'

    rtb_Add = (ANAS0_P.GPS1a1_Gain * rtb_VectorConcatenate_o[0] +
               ANAS0_P.GPSlat0_Bias) * ANAS0_P.Gain1_Gain_ke;

    // Product: '<S206>/Divide1' incorporates:
    //   Trigonometry: '<S206>/Sin'

    rtb_Divide1_p = rtb_VectorConcatenate_o[1] * std::sin(rtb_Add);

    // Trigonometry: '<S206>/Cos'
    rtb_Add = std::cos(rtb_Add);

    // Concatenate: '<S206>/Matrix Concatenate' incorporates:
    //   Constant: '<S206>/Constant1'
    //   Constant: '<S206>/Constant2'
    //   Constant: '<S206>/GPS1//a'
    //   Gain: '<S206>/GPSb'
    //   Gain: '<S206>/GPSb1'
    //   Math: '<S206>/Square'
    //   Product: '<S206>/Divide'
    //   Product: '<S206>/Divide2'

    rtb_Transpose1_j[0] = ANAS0_P.GPS1a_Value;
    rtb_Transpose1_j[1] = rtb_Divide1_p / (rtb_Add * rtb_Add) *
      ANAS0_P.GPSb1_Gain;
    rtb_Transpose1_j[2] = ANAS0_P.Constant1_Value_i;
    rtb_Transpose1_j[3] = 1.0F / (ANAS0_P.GPSb_Gain * rtb_Add) *
      ANAS0_P.Constant2_Value_g;

    // Constant: '<S206>/Constant5'
    for (i = 0; i < 8; i++) {
      // Constant: '<S206>/Constant5'
      rtb_Transpose1_j[i + 4] = ANAS0_P.Constant5_Value_e[i];
    }

    // End of Constant: '<S206>/Constant5'

    // Concatenate: '<S206>/Matrix Concatenate2' incorporates:
    //   Constant: '<S206>/Constant6'
    //   Math: '<S143>/Transpose1'
    //   Math: '<S205>/Transpose1'

    i_0 = 0;
    k = 0;
    for (i = 0; i < 6; i++) {
      rtb_Transpose1[i_0] = rtb_Transpose1_j[k];
      rtb_Transpose1[i_0 + 2] = ANAS0_P.Constant6_Value_k[k];
      rtb_Transpose1[i_0 + 1] = rtb_Transpose1_j[k + 1];
      rtb_Transpose1[i_0 + 3] = ANAS0_P.Constant6_Value_k[k + 1];
      i_0 += 4;
      k += 2;
    }

    // End of Concatenate: '<S206>/Matrix Concatenate2'

    // Math: '<S202>/Square' incorporates:
    //   Constant: '<S202>/GPSSigma'

    for (i = 0; i < 16; i++) {
      rtb_Matrix1Norm2_om = ANAS0_P.GPSSigma_Value[i];
      rtb_Square_f[i] = rtb_Matrix1Norm2_om * rtb_Matrix1Norm2_om;
    }

    // End of Math: '<S202>/Square'
    for (i_0 = 0; i_0 < 6; i_0++) {
      // Product: '<S207>/Matrix Multiply' incorporates:
      //   Bias: '<S5>/Linear Q'
      //   Math: '<S205>/Transpose1'
      //   Math: '<S207>/Transpose'
      //   Product: '<S207>/Matrix Multiply1'

      k = 0;
      for (i = 0; i < 4; i++) {
        rtb_Matrix1Norm2_om = 0.0F;
        rtb_LinearQ_tmp = 0;
        rtb_S_tmp_0 = 0;
        for (i_1 = 0; i_1 < 6; i_1++) {
          rtb_Matrix1Norm2_om += rtb_LinearQ[rtb_LinearQ_tmp + i_0] *
            rtb_Transpose1[rtb_S_tmp_0 + i];
          rtb_LinearQ_tmp += 6;
          rtb_S_tmp_0 += 4;
        }

        rtb_S_tmp[k + i_0] = rtb_Matrix1Norm2_om;
        k += 6;
      }
    }

    // Sum: '<S207>/Add' incorporates:
    //   Math: '<S202>/Square'
    //   Math: '<S205>/Transpose1'
    //   Product: '<S207>/Matrix Multiply'

    for (i_0 = 0; i_0 < 4; i_0++) {
      k = 0;
      i = 0;
      for (rtb_LinearQ_tmp = 0; rtb_LinearQ_tmp < 4; rtb_LinearQ_tmp++) {
        rtb_Power2 = 0.0F;
        rtb_S_tmp_0 = 0;
        for (i_1 = 0; i_1 < 6; i_1++) {
          rtb_Power2 += rtb_Transpose1[rtb_S_tmp_0 + i_0] * rtb_S_tmp[i_1 + i];
          rtb_S_tmp_0 += 4;
        }

        rtb_S_tmp_0 = k + i_0;
        rtb_S[rtb_S_tmp_0] = rtb_Square_f[rtb_S_tmp_0] + rtb_Power2;
        k += 4;
        i += 6;
      }
    }

    // End of Sum: '<S207>/Add'

    // S-Function (sdsplu2): '<S215>/LU Factorization' incorporates:
    //   Sum: '<S207>/Add'

    std::memcpy(&rtb_LUFactorization_o1[0], &rtb_S[0], sizeof(float) << 4U);
    LUf_boolfloatint32_t(&rtb_LUFactorization_o1[0], &rtb_LUFactorization_o2[0],
                         4, &rtb_LUFactorization_o3_j);

    // Switch: '<S213>/Switch' incorporates:
    //   Constant: '<S213>/Constant'
    //   Math: '<S213>/Math Function'
    //   Product: '<S213>/Product'
    //
    //  About '<S213>/Math Function':
    //   Operator: reciprocal

    if (rtb_LUFactorization_o3_j) {
      rtb_Add = ANAS0_P.Constant_Value_p2;
    } else {
      // S-Function (sdspm1norm2): '<S213>/Matrix  1-Norm2'

      // DSP System Toolbox Matrix 1-Norm (sdspm1norm2) - '<S213>/Matrix  1-Norm2' 
      {
        const float *uPtr{ &rtb_S[0] };

        float m1norm{ 0.0 };

        int jdx;
        for (jdx=4; jdx-- > 0; ) {
          float sumabsAj{ 0.0 };

          int idxFlt;
          for (idxFlt=4; idxFlt-- > 0; ) {
            float temp{ *uPtr++ };

            sumabsAj += fabsf(temp);
          }

          m1norm = MAX(m1norm, sumabsAj);
        }

        rtb_Matrix1Norm2 = m1norm;
      }

      // S-Function (sdspperm2): '<S215>/Permute Matrix' incorporates:
      //   IdentityMatrix: '<S215>/Identity Matrix'
      //   S-Function (sdspfbsub2): '<S215>/Backward Substitution'

      rtb_Matrix1Norm2_om = rtb_LUFactorization_o2[0];
      rtb_Matrix1Norm1_p = rtb_LUFactorization_o2[1];
      rtb_Divide5_a = rtb_LUFactorization_o2[2];
      rtb_Matrix1Norm2_j = rtb_LUFactorization_o2[3];
      for (k = 0; k < 4; k++) {
        i_0 = static_cast<int32_t>(std::floor(rtb_Matrix1Norm2_om)) - 1;
        if (i_0 < 0) {
          i_0 = 0;
        } else if (i_0 >= 4) {
          i_0 = 3;
        }

        i = k << 2;
        rtb_BackwardSubstitution[i] = ANAS0_DW.IdentityMatrix[i + i_0];
        i_0 = static_cast<int32_t>(std::floor(rtb_Matrix1Norm1_p)) - 1;
        if (i_0 < 0) {
          i_0 = 0;
        } else if (i_0 >= 4) {
          i_0 = 3;
        }

        rtb_BackwardSubstitution[i + 1] = ANAS0_DW.IdentityMatrix[i + i_0];
        i_0 = static_cast<int32_t>(std::floor(rtb_Divide5_a)) - 1;
        if (i_0 < 0) {
          i_0 = 0;
        } else if (i_0 >= 4) {
          i_0 = 3;
        }

        rtb_BackwardSubstitution[i + 2] = ANAS0_DW.IdentityMatrix[i + i_0];
        i_0 = static_cast<int32_t>(std::floor(rtb_Matrix1Norm2_j)) - 1;
        if (i_0 < 0) {
          i_0 = 0;
        } else if (i_0 >= 4) {
          i_0 = 3;
        }

        rtb_BackwardSubstitution[i + 3] = ANAS0_DW.IdentityMatrix[i + i_0];
      }

      // End of S-Function (sdspperm2): '<S215>/Permute Matrix'

      // S-Function (sdspfbsub2): '<S215>/Forward Substitution' incorporates:
      //   S-Function (sdspfbsub2): '<S215>/Backward Substitution'
      //   S-Function (sdsplu2): '<S215>/LU Factorization'

      for (i = 0; i < 4; i++) {
        i_0 = i << 2;
        rtb_BackwardSubstitution[i_0 + 1] -= rtb_LUFactorization_o1[1] *
          rtb_BackwardSubstitution[i_0];
        rtb_LinearQ_tmp = 2;
        rtb_Add = rtb_BackwardSubstitution[i_0 + 2];
        for (k = 0; k < 2; k++) {
          rtb_Add -= rtb_BackwardSubstitution[i_0 + k] *
            rtb_LUFactorization_o1[rtb_LinearQ_tmp];
          rtb_LinearQ_tmp += 4;
        }

        rtb_BackwardSubstitution[i_0 + 2] = rtb_Add;
        rtb_LinearQ_tmp = 3;
        rtb_Add = rtb_BackwardSubstitution[i_0 + 3];
        for (k = 0; k < 3; k++) {
          rtb_Add -= rtb_BackwardSubstitution[i_0 + k] *
            rtb_LUFactorization_o1[rtb_LinearQ_tmp];
          rtb_LinearQ_tmp += 4;
        }

        rtb_BackwardSubstitution[i_0 + 3] = rtb_Add;
      }

      // End of S-Function (sdspfbsub2): '<S215>/Forward Substitution'

      // S-Function (sdspfbsub2): '<S215>/Backward Substitution' incorporates:
      //   S-Function (sdsplu2): '<S215>/LU Factorization'

      for (i = 0; i < 4; i++) {
        i_0 = i << 2;
        rtb_Matrix1Norm2_om = rtb_BackwardSubstitution[i_0 + 3] /
          rtb_LUFactorization_o1[15];

        // S-Function (sdspfbsub2): '<S215>/Backward Substitution' incorporates:
        //   S-Function (sdsplu2): '<S215>/LU Factorization'

        rtb_BackwardSubstitution[i_0 + 3] = rtb_Matrix1Norm2_om;
        rtb_BackwardSubstitution[i_0 + 2] = (rtb_BackwardSubstitution[i_0 + 2] -
          rtb_Matrix1Norm2_om * rtb_LUFactorization_o1[14]) /
          rtb_LUFactorization_o1[10];
        rtb_LinearQ_tmp = 13;
        rtb_Add = rtb_BackwardSubstitution[i_0 + 1];
        for (k = 3; k > 1; k--) {
          rtb_Add -= rtb_BackwardSubstitution[i_0 + k] *
            rtb_LUFactorization_o1[rtb_LinearQ_tmp];
          rtb_LinearQ_tmp -= 4;
        }

        rtb_BackwardSubstitution[i_0 + 1] = rtb_Add /
          rtb_LUFactorization_o1[rtb_LinearQ_tmp];
        rtb_LinearQ_tmp = 12;
        rtb_Add = rtb_BackwardSubstitution[i_0];
        for (k = 3; k > 0; k--) {
          rtb_Add -= rtb_BackwardSubstitution[i_0 + k] *
            rtb_LUFactorization_o1[rtb_LinearQ_tmp];
          rtb_LinearQ_tmp -= 4;
        }

        rtb_BackwardSubstitution[i_0] = rtb_Add /
          rtb_LUFactorization_o1[rtb_LinearQ_tmp];
      }

      // End of S-Function (sdspfbsub2): '<S215>/Backward Substitution'

      // S-Function (sdspm1norm2): '<S213>/Matrix  1-Norm1'

      // DSP System Toolbox Matrix 1-Norm (sdspm1norm2) - '<S213>/Matrix  1-Norm1' 
      {
        const float *uPtr{ &rtb_BackwardSubstitution[0] };

        float m1norm{ 0.0 };

        int jdx;
        for (jdx=4; jdx-- > 0; ) {
          float sumabsAj{ 0.0 };

          int idxFlt;
          for (idxFlt=4; idxFlt-- > 0; ) {
            float temp{ *uPtr++ };

            sumabsAj += fabsf(temp);
          }

          m1norm = MAX(m1norm, sumabsAj);
        }

        rtb_Matrix1Norm1 = m1norm;
      }

      rtb_Add = 1.0F / (rtb_Matrix1Norm1 * rtb_Matrix1Norm2);
    }

    // End of Switch: '<S213>/Switch'

    // If: '<S207>/If' incorporates:
    //   Constant: '<S207>/NASCondLim'
    //   Merge: '<S207>/Merge'
    //   Product: '<S207>/Matrix Multiply1'
    //   Product: '<S211>/Matrix Divide'
    //   RelationalOperator: '<S207>/GreaterThanOrEqual'

    if (rtb_Add >= ANAS0_P.NASCondLim_Value_d) {
      // Outputs for IfAction SubSystem: '<S207>/Correction' incorporates:
      //   ActionPort: '<S211>/Action Port'

      std::memcpy(&rtb_Merge_gj[0], &rtb_S_tmp[0], 24U * sizeof(float));

      // Product: '<S211>/Matrix Divide' incorporates:
      //   Merge: '<S207>/Merge'
      //   Product: '<S207>/Matrix Multiply1'
      //   SignalConversion generated from: '<S211>/Sprev'
      //   Sum: '<S207>/Add'

      rt_mrdivide_U1f6x4_U2f4x4_Yf6x4(rtb_Merge_gj, rtb_S);

      // End of Outputs for SubSystem: '<S207>/Correction'
    } else {
      // Outputs for IfAction SubSystem: '<S207>/No correction' incorporates:
      //   ActionPort: '<S212>/Action Port'

      // Gain: '<S212>/Gain' incorporates:
      //   Merge: '<S207>/Merge'
      //   Product: '<S207>/Matrix Multiply1'

      for (i_0 = 0; i_0 < 24; i_0++) {
        rtb_Merge_gj[i_0] = ANAS0_P.Gain_Gain_n3 * rtb_S_tmp[i_0];
      }

      // End of Gain: '<S212>/Gain'
      // End of Outputs for SubSystem: '<S207>/No correction'
    }

    // End of If: '<S207>/If'

    // Sum: '<S209>/Subtract' incorporates:
    //   Constant: '<S209>/Constant'
    //   Math: '<S205>/Transpose1'
    //   Product: '<S176>/Matrix Multiply2'
    //   Product: '<S209>/Matrix Multiply'

    for (i_0 = 0; i_0 < 6; i_0++) {
      // Product: '<S209>/Matrix Multiply' incorporates:
      //   Merge: '<S207>/Merge'

      rtb_Matrix1Norm2 = rtb_Merge_gj[i_0 + 6];
      rtb_Matrix1Norm1 = rtb_Merge_gj[i_0];
      rtb_Matrix1Norm2_om = rtb_Merge_gj[i_0 + 12];
      rtb_Matrix1Norm1_p = rtb_Merge_gj[i_0 + 18];
      k = 0;
      i = 0;
      for (rtb_LinearQ_tmp = 0; rtb_LinearQ_tmp < 6; rtb_LinearQ_tmp++) {
        rtb_S_tmp_0 = k + i_0;
        rtb_Transpose_e[rtb_S_tmp_0] = ANAS0_P.Constant_Value_a[rtb_S_tmp_0] -
          (((rtb_Transpose1[i + 1] * rtb_Matrix1Norm2 + rtb_Transpose1[i] *
             rtb_Matrix1Norm1) + rtb_Transpose1[i + 2] * rtb_Matrix1Norm2_om) +
           rtb_Transpose1[i + 3] * rtb_Matrix1Norm1_p);
        k += 6;
        i += 4;
      }
    }

    // End of Sum: '<S209>/Subtract'

    // Product: '<S205>/Matrix Multiply' incorporates:
    //   Bias: '<S5>/Linear Q'
    //   Math: '<S205>/Transpose'
    //   Product: '<S176>/Matrix Multiply2'

    for (i_0 = 0; i_0 < 6; i_0++) {
      k = 0;
      for (i = 0; i < 6; i++) {
        rtb_Matrix1Norm1 = 0.0F;
        rtb_LinearQ_tmp = 0;
        for (rtb_S_tmp_0 = 0; rtb_S_tmp_0 < 6; rtb_S_tmp_0++) {
          rtb_Matrix1Norm1 += rtb_LinearQ[rtb_LinearQ_tmp + i_0] *
            rtb_Transpose_e[rtb_LinearQ_tmp + i];
          rtb_LinearQ_tmp += 6;
        }

        rtb_LinearQ_0[k + i_0] = rtb_Matrix1Norm1;
        k += 6;
      }
    }

    // Product: '<S205>/Matrix Multiply2' incorporates:
    //   Math: '<S202>/Square'
    //   Math: '<S205>/Transpose1'
    //   Merge: '<S207>/Merge'

    for (i_0 = 0; i_0 < 4; i_0++) {
      rtb_Matrix1Norm2 = rtb_Square_f[i_0 + 4];
      rtb_Matrix1Norm1 = rtb_Square_f[i_0];
      rtb_Matrix1Norm2_om = rtb_Square_f[i_0 + 8];
      rtb_Matrix1Norm1_p = rtb_Square_f[i_0 + 12];
      k = 0;
      for (i = 0; i < 6; i++) {
        rtb_Transpose1[k + i_0] = ((rtb_Merge_gj[i + 6] * rtb_Matrix1Norm2 +
          rtb_Matrix1Norm1 * rtb_Merge_gj[i]) + rtb_Merge_gj[i + 12] *
          rtb_Matrix1Norm2_om) + rtb_Merge_gj[i + 18] * rtb_Matrix1Norm1_p;
        k += 4;
      }
    }

    // Product: '<S205>/Matrix Multiply' incorporates:
    //   Merge: '<S207>/Merge'
    //   Product: '<S176>/Matrix Multiply2'
    //   Product: '<S205>/Matrix Multiply2'

    i_0 = 0;
    k = 0;
    for (i = 0; i < 6; i++) {
      for (rtb_LinearQ_tmp = 0; rtb_LinearQ_tmp < 6; rtb_LinearQ_tmp++) {
        rtb_Matrix1Norm1 = 0.0F;
        rtb_S_tmp_0 = 0;
        for (i_1 = 0; i_1 < 6; i_1++) {
          rtb_Matrix1Norm1 += rtb_Transpose_e[rtb_S_tmp_0 + rtb_LinearQ_tmp] *
            rtb_LinearQ_0[i_1 + i_0];
          rtb_S_tmp_0 += 6;
        }

        rtb_S_tmp_0 = rtb_LinearQ_tmp + i_0;
        rtb_Transpose_aq[rtb_S_tmp_0] = rtb_Matrix1Norm1;
        rtb_Merge_p[rtb_S_tmp_0] = ((rtb_Transpose1[k + 1] *
          rtb_Merge_gj[rtb_LinearQ_tmp + 6] + rtb_Transpose1[k] *
          rtb_Merge_gj[rtb_LinearQ_tmp]) + rtb_Transpose1[k + 2] *
          rtb_Merge_gj[rtb_LinearQ_tmp + 12]) + rtb_Transpose1[k + 3] *
          rtb_Merge_gj[rtb_LinearQ_tmp + 18];
      }

      i_0 += 6;
      k += 4;
    }

    // Sum: '<S205>/Sum1' incorporates:
    //   Merge: '<S97>/Merge1'

    for (i_0 = 0; i_0 < 36; i_0++) {
      rtb_LinearQ[i_0] = rtb_Transpose_aq[i_0] + rtb_Merge_p[i_0];
    }

    // End of Sum: '<S205>/Sum1'

    // Bias: '<S208>/GPSlat0' incorporates:
    //   Gain: '<S208>/GPS1//a'

    rtb_Add = ANAS0_P.GPS1a_Gain * rtb_VectorConcatenate_o[0] +
      ANAS0_P.GPSlat0_Bias_g;

    // Sum: '<S208>/Add' incorporates:
    //   Bias: '<S208>/GPSlon0'
    //   Gain: '<S208>/GPS1//b'
    //   Gain: '<S216>/Gain1'
    //   Inport: '<Root>/ANAS In'
    //   Product: '<S208>/Divide'
    //   SignalConversion generated from: '<S208>/Vector Concatenate'
    //   Trigonometry: '<S208>/Cos'

    rtb_Matrix1Norm2_j = ANAS0_U.ANASIn_c.GPSMeasure[0] - rtb_Add;
    rtb_Matrix1Norm1_k = ANAS0_U.ANASIn_c.GPSMeasure[1] - (1.0F / std::cos
      (ANAS0_P.Gain1_Gain_af * rtb_Add) * (ANAS0_P.GPS1b_Gain *
      rtb_VectorConcatenate_o[1]) + ANAS0_P.GPSlon0_Bias);
    rtb_Add = ANAS0_U.ANASIn_c.GPSMeasure[2] - rtb_VectorConcatenate_o[3];
    rtb_Matrix1Norm2 = ANAS0_U.ANASIn_c.GPSMeasure[3] - rtb_VectorConcatenate_o
      [4];
    for (i_0 = 0; i_0 < 6; i_0++) {
      // Sum: '<S202>/Sum' incorporates:
      //   Concatenate: '<S5>/Vector Concatenate'
      //   Merge: '<S207>/Merge'
      //   Merge: '<S97>/Merge'
      //   Product: '<S202>/Matrix Multiply'

      rtb_VectorConcatenate_o[i_0] += ((rtb_Merge_gj[i_0 + 6] *
        rtb_Matrix1Norm1_k + rtb_Merge_gj[i_0] * rtb_Matrix1Norm2_j) +
        rtb_Merge_gj[i_0 + 12] * rtb_Add) + rtb_Merge_gj[i_0 + 18] *
        rtb_Matrix1Norm2;
    }
  }

  // End of Outputs for SubSystem: '<S97>/No Correction Step'
  // End of Outputs for SubSystem: '<S97>/Active Correction Step GPS'

  // Outputs for Enabled SubSystem: '<S96>/Active Correction Step Pitot Static & Differential' incorporates:
  //   EnablePort: '<S100>/Enable'

  // Outputs for Enabled SubSystem: '<S96>/Active Correction Step Main Static & Pitot' incorporates:
  //   EnablePort: '<S99>/Enable'

  // Sqrt: '<S201>/Sqrt' incorporates:
  //   DotProduct: '<S201>/Dot Product'
  //   Math: '<S201>/Transpose'
  //   Sqrt: '<S131>/Sqrt'
  //   Sqrt: '<S141>/Sqrt'
  //   Sqrt: '<S151>/Sqrt'
  //   Sqrt: '<S161>/Sqrt'
  //   Sqrt: '<S197>/Sqrt'

  rtb_Matrix1Norm2_j = std::sqrt((rtb_VectorConcatenate_o[3] *
    rtb_VectorConcatenate_o[3] + rtb_VectorConcatenate_o[4] *
    rtb_VectorConcatenate_o[4]) + rtb_VectorConcatenate_o[5] *
    rtb_VectorConcatenate_o[5]);

  // End of Outputs for SubSystem: '<S96>/Active Correction Step Main Static & Pitot' 
  // End of Outputs for SubSystem: '<S96>/Active Correction Step Pitot Static & Differential' 

  // RateTransition: '<S103>/Rate Transition' incorporates:
  //   RateTransition: '<S103>/Rate Transition2'

  rtb_variometer = ((&ANAS0_M)->Timing.TaskCounters.TID[2] == 0);
  if (rtb_variometer) {
    // RateTransition: '<S103>/Rate Transition' incorporates:
    //   Inport: '<Root>/ANAS In'

    ANAS0_DW.RateTransition_c = ANAS0_U.ANASIn_c.PitotTimestamp;
  }

  // End of RateTransition: '<S103>/Rate Transition'

  // Outputs for Enabled SubSystem: '<S96>/Active Correction Step Pitot Static & Differential' incorporates:
  //   EnablePort: '<S100>/Enable'

  // Outputs for Enabled SubSystem: '<S96>/Active Correction Step Main Static & Pitot' incorporates:
  //   EnablePort: '<S99>/Enable'

  // Math: '<S200>/Square' incorporates:
  //   Math: '<S129>/Square'
  //   Math: '<S140>/Square'
  //   Math: '<S148>/Square'
  //   Math: '<S160>/Square'
  //   Sqrt: '<S201>/Sqrt'

  rtb_Matrix1Norm2 = rtb_Matrix1Norm2_j * rtb_Matrix1Norm2_j;

  // End of Outputs for SubSystem: '<S96>/Active Correction Step Main Static & Pitot' 
  // End of Outputs for SubSystem: '<S96>/Active Correction Step Pitot Static & Differential' 

  // Logic: '<S103>/AND6' incorporates:
  //   Constant: '<S103>/PitotFlag'
  //   Constant: '<S196>/Constant'
  //   Constant: '<S198>/Baroa'
  //   Constant: '<S200>/AirR'
  //   Constant: '<S200>/IsaGamma'
  //   Inport: '<Root>/ANAS Reference'
  //   Math: '<S200>/Square'
  //   Memory: '<S103>/Memory3'
  //   Product: '<S189>/Divide'
  //   Product: '<S198>/Divide2'
  //   Product: '<S200>/Divide6'
  //   RateTransition: '<S103>/Rate Transition'
  //   RelationalOperator: '<S103>/Relational Operator3'
  //   RelationalOperator: '<S196>/Compare'
  //   Sqrt: '<S200>/Sqrt'
  //   Sum: '<S198>/Add'

  rtb_LUFactorization_o3 = ((rtb_Matrix1Norm2_j / std::sqrt(1.0F /
    ANAS0_P.AirR_Value_i / ANAS0_P.IsaGamma_Value_b / (ANAS0_P.Baroa_Value_f *
    rtb_VectorConcatenate_o[2] + ANAS0_U.ANASReference_f.GroundTemperature) *
    rtb_Matrix1Norm2) >= ANAS0_P.PitotMinMach_const) &&
    (ANAS0_DW.RateTransition_c > ANAS0_DW.Memory3_PreviousInput) &&
    ANAS0_P.PitotFlag_Value);

  // RateTransition: '<S103>/Rate Transition1'
  if ((&ANAS0_M)->Timing.TaskCounters.TID[1] == 0) {
    // RateTransition: '<S103>/Rate Transition1' incorporates:
    //   Inport: '<Root>/ANAS In'

    ANAS0_DW.RateTransition1 = ANAS0_U.ANASIn_c.BaroTimestamp;
  }

  // End of RateTransition: '<S103>/Rate Transition1'

  // Logic: '<S103>/AND7' incorporates:
  //   Constant: '<S103>/BaroFlag'
  //   Logic: '<S103>/NOT2'
  //   Memory: '<S103>/Memory2'
  //   RateTransition: '<S103>/Rate Transition1'
  //   RelationalOperator: '<S103>/Relational Operator2'

  rtb_do_barometer_correcion = ((!rtb_LUFactorization_o3) &&
    (ANAS0_DW.RateTransition1 > ANAS0_DW.Memory2_PreviousInput) &&
    ANAS0_P.BaroFlag_Value);

  // Logic: '<S103>/AND11' incorporates:
  //   Constant: '<S103>/StaticPitotFlag1'
  //   Logic: '<S103>/NOT4'

  rtb_LUFactorization_o3_j = ((!ANAS0_P.StaticPitotFlag1_Value) &&
    rtb_do_barometer_correcion);

  // RateTransition: '<S103>/Rate Transition2'
  if (rtb_variometer) {
    // RateTransition: '<S103>/Rate Transition2' incorporates:
    //   Inport: '<Root>/ANAS In'

    ANAS0_DW.RateTransition2 = ANAS0_U.ANASIn_c.BaroTimestamp;
  }

  // Logic: '<S103>/AND12' incorporates:
  //   Constant: '<S103>/VaroFlag'
  //   Memory: '<S103>/Memory1'
  //   RateTransition: '<S103>/Rate Transition2'
  //   RelationalOperator: '<S103>/Relational Operator1'

  rtb_variometer = (ANAS0_P.VaroFlag_Value && rtb_LUFactorization_o3_j &&
                    (ANAS0_DW.RateTransition2 > ANAS0_DW.Memory1_PreviousInput));

  // Outputs for Enabled SubSystem: '<S96>/Active Correction Step Baro & Variometer ' incorporates:
  //   EnablePort: '<S98>/Enable'

  if (rtb_variometer) {
    // Reshape: '<S106>/Reshape' incorporates:
    //   Constant: '<S106>/Constant3'

    rtb_MatrixMultiply_f[0] = ANAS0_P.Constant3_Value[0];
    rtb_MatrixMultiply_f[1] = ANAS0_P.Constant3_Value[1];

    // Product: '<S106>/Divide' incorporates:
    //   Constant: '<S106>/Baroa'
    //   Product: '<S106>/Divide3'

    rtb_Add = rtb_VectorConcatenate_o[2] * ANAS0_P.Baroa_Value;

    // Sum: '<S106>/Add2' incorporates:
    //   Inport: '<Root>/ANAS Reference'
    //   Product: '<S106>/Divide'

    rtb_Bias3 = rtb_Add + ANAS0_U.ANASReference_f.GroundTemperature;

    // Product: '<S106>/Product' incorporates:
    //   Constant: '<S106>/Baroa'
    //   Constant: '<S106>/Barog//R'
    //   Constant: '<S106>/Constant1'
    //   Constant: '<S106>/Constant2'
    //   Inport: '<Root>/ANAS Reference'
    //   Math: '<S106>/Power'
    //   Product: '<S106>/Divide1'
    //   Product: '<S106>/Divide4'
    //   Product: '<S106>/Divide5'
    //   Sum: '<S106>/Add'

    rtb_Power = std::pow(ANAS0_P.Constant2_Value_l - 1.0F / rtb_Bias3 * rtb_Add,
                         1.0F / ANAS0_P.Baroa_Value * ANAS0_P.Constant1_Value_f *
                         ANAS0_P.BarogR_Value) * (1.0F / rtb_Bias3) *
      ANAS0_P.BarogR_Value * ANAS0_U.ANASReference_f.GroundPressure;

    // SignalConversion generated from: '<S106>/Matrix Concatenate4'
    rtb_MatrixMultiply_f[2] = rtb_Power;

    // Reshape: '<S106>/Reshape1' incorporates:
    //   Constant: '<S106>/Constant9'

    rtb_MatrixMultiply_f[3] = ANAS0_P.Constant9_Value[0];
    rtb_MatrixMultiply_f[4] = ANAS0_P.Constant9_Value[1];
    rtb_MatrixMultiply_f[5] = ANAS0_P.Constant9_Value[2];

    // Product: '<S106>/Divide8' incorporates:
    //   Constant: '<S106>/Baroa'
    //   Constant: '<S106>/Barog//R'
    //   Constant: '<S110>/LocalGravity'
    //   Constant: '<S110>/RAir'
    //   Constant: '<S110>/RAir1'
    //   Gain: '<S106>/Gain1'
    //   Gain: '<S106>/Gain2'
    //   Inport: '<Root>/ANAS Reference'
    //   Math: '<S106>/Square'
    //   Math: '<S110>/Power'
    //   Product: '<S106>/Divide6'
    //   Product: '<S106>/Divide7'
    //   Product: '<S110>/Divide'
    //   Product: '<S110>/Divide1'
    //   Product: '<S110>/Product'
    //   Product: '<S110>/Product2'
    //   Product: '<S110>/Product3'
    //   Sum: '<S106>/Add1'
    //   Sum: '<S110>/Subtract'

    rtb_Transpose_k[2] = (std::pow((ANAS0_U.ANASReference_f.GroundTemperature -
      ANAS0_P.Gain1_Gain_a * rtb_VectorConcatenate_o[2] * ANAS0_P.RAir1_Value) /
      ANAS0_U.ANASReference_f.GroundTemperature, ANAS0_P.LocalGravity_Value_j /
      (ANAS0_P.RAir1_Value * ANAS0_P.RAir_Value)) *
                          ANAS0_U.ANASReference_f.GroundPressure *
                          (ANAS0_P.Gain2_Gain_j * ANAS0_P.Baroa_Value) *
                          ANAS0_P.BarogR_Value / (rtb_Bias3 * rtb_Bias3) +
                          rtb_Power * ANAS0_P.BarogR_Value / rtb_Bias3) *
      rtb_VectorConcatenate_o[5];

    // Reshape: '<S106>/Reshape2' incorporates:
    //   Constant: '<S106>/Constant5'

    rtb_Transpose_k[0] = ANAS0_P.Constant5_Value[0];

    // Reshape: '<S106>/Reshape3' incorporates:
    //   Constant: '<S106>/Constant6'

    rtb_Transpose_k[3] = ANAS0_P.Constant6_Value[0];

    // Reshape: '<S106>/Reshape2' incorporates:
    //   Constant: '<S106>/Constant5'

    rtb_Transpose_k[1] = ANAS0_P.Constant5_Value[1];

    // Reshape: '<S106>/Reshape3' incorporates:
    //   Constant: '<S106>/Constant6'

    rtb_Transpose_k[4] = ANAS0_P.Constant6_Value[1];

    // SignalConversion generated from: '<S106>/Matrix Concatenate1'
    rtb_Transpose_k[5] = rtb_Power;

    // Concatenate: '<S106>/Matrix Concatenate2' incorporates:
    //   Math: '<S143>/Transpose1'
    //   Math: '<S178>/Transpose'
    //   Product: '<S102>/Matrix Multiply'

    i_0 = 0;
    for (k = 0; k < 6; k++) {
      rtb_Transpose1_j[i_0] = rtb_MatrixMultiply_f[k];
      rtb_Transpose1_j[i_0 + 1] = rtb_Transpose_k[k];
      i_0 += 2;
    }

    // End of Concatenate: '<S106>/Matrix Concatenate2'

    // Math: '<S107>/Transpose' incorporates:
    //   Math: '<S143>/Transpose1'
    //   Merge: '<S107>/Merge'

    i_0 = 0;
    for (k = 0; k < 2; k++) {
      i = 0;
      for (rtb_LinearQ_tmp = 0; rtb_LinearQ_tmp < 6; rtb_LinearQ_tmp++) {
        rtb_MatrixConcatenate[rtb_LinearQ_tmp + i_0] = rtb_Transpose1_j[i + k];
        i += 2;
      }

      i_0 += 6;
    }

    // End of Math: '<S107>/Transpose'

    // Math: '<S98>/Square' incorporates:
    //   Constant: '<S98>/BaroVaroSigma'
    //   Product: '<S145>/Matrix Multiply'

    rtb_res_GPS[0] = ANAS0_P.BaroVaroSigma_Value[0] *
      ANAS0_P.BaroVaroSigma_Value[0];
    rtb_res_GPS[1] = ANAS0_P.BaroVaroSigma_Value[1] *
      ANAS0_P.BaroVaroSigma_Value[1];
    rtb_res_GPS[2] = ANAS0_P.BaroVaroSigma_Value[2] *
      ANAS0_P.BaroVaroSigma_Value[2];
    rtb_res_GPS[3] = ANAS0_P.BaroVaroSigma_Value[3] *
      ANAS0_P.BaroVaroSigma_Value[3];

    // Product: '<S107>/Matrix Multiply' incorporates:
    //   Merge: '<S107>/Merge'
    //   Merge: '<S97>/Merge1'
    //   Product: '<S107>/Matrix Multiply1'

    i_0 = 0;
    for (k = 0; k < 2; k++) {
      for (i = 0; i < 6; i++) {
        rtb_Matrix1Norm1 = 0.0F;
        rtb_LinearQ_tmp = 0;
        for (rtb_S_tmp_0 = 0; rtb_S_tmp_0 < 6; rtb_S_tmp_0++) {
          rtb_Matrix1Norm1 += rtb_LinearQ[rtb_LinearQ_tmp + i] *
            rtb_MatrixConcatenate[rtb_S_tmp_0 + i_0];
          rtb_LinearQ_tmp += 6;
        }

        rtb_S_ce_tmp[i + i_0] = rtb_Matrix1Norm1;
      }

      i_0 += 6;
    }

    for (i_0 = 0; i_0 < 2; i_0++) {
      // Sum: '<S107>/Add' incorporates:
      //   Math: '<S143>/Transpose1'
      //   Product: '<S107>/Matrix Multiply'
      //   Product: '<S145>/Matrix Multiply'

      k = 0;
      i = 0;
      for (rtb_LinearQ_tmp = 0; rtb_LinearQ_tmp < 2; rtb_LinearQ_tmp++) {
        rtb_Power2 = 0.0F;
        rtb_S_tmp_0 = 0;
        for (i_1 = 0; i_1 < 6; i_1++) {
          rtb_Power2 += rtb_Transpose1_j[rtb_S_tmp_0 + i_0] * rtb_S_ce_tmp[i_1 +
            i];
          rtb_S_tmp_0 += 2;
        }

        rtb_S_tmp_0 = k + i_0;
        rtb_LUFactorization_o2[rtb_S_tmp_0] = rtb_res_GPS[rtb_S_tmp_0] +
          rtb_Power2;
        k += 2;
        i += 6;
      }
    }

    // S-Function (sdsplu2): '<S115>/LU Factorization' incorporates:
    //   Sum: '<S107>/Add'

    rtb_LUFactorization_o1_if[0] = rtb_LUFactorization_o2[0];
    rtb_LUFactorization_o1_if[1] = rtb_LUFactorization_o2[1];
    rtb_LUFactorization_o1_if[2] = rtb_LUFactorization_o2[2];
    rtb_LUFactorization_o1_if[3] = rtb_LUFactorization_o2[3];
    LUf_boolfloatint32_t(&rtb_LUFactorization_o1_if[0],
                         &rtb_LUFactorization_o2_l[0], 2,
                         &rtb_LUFactorization_o3_i);

    // Switch: '<S113>/Switch' incorporates:
    //   Constant: '<S113>/Constant'
    //   Math: '<S113>/Math Function'
    //   Product: '<S113>/Product'
    //
    //  About '<S113>/Math Function':
    //   Operator: reciprocal

    if (rtb_LUFactorization_o3_i) {
      rtb_Bias3 = ANAS0_P.Constant_Value_e;
    } else {
      // S-Function (sdspm1norm2): '<S113>/Matrix  1-Norm2'

      // DSP System Toolbox Matrix 1-Norm (sdspm1norm2) - '<S113>/Matrix  1-Norm2' 
      {
        const float *uPtr{ &rtb_LUFactorization_o2[0] };

        float m1norm{ 0.0 };

        int jdx;
        for (jdx=2; jdx-- > 0; ) {
          float sumabsAj{ 0.0 };

          int idxFlt;
          for (idxFlt=2; idxFlt-- > 0; ) {
            float temp{ *uPtr++ };

            sumabsAj += fabsf(temp);
          }

          m1norm = MAX(m1norm, sumabsAj);
        }

        rtb_Matrix1Norm2_m = m1norm;
      }

      // S-Function (sdspperm2): '<S115>/Permute Matrix' incorporates:
      //   IdentityMatrix: '<S115>/Identity Matrix'
      //   S-Function (sdspfbsub2): '<S115>/Backward Substitution'

      k = static_cast<int32_t>(std::floor(rtb_LUFactorization_o2_l[0])) - 1;
      i_0 = k;
      if (k < 0) {
        i_0 = 0;
      } else if (k >= 2) {
        i_0 = 1;
      }

      rtb_BackwardSubstitution_f1[0] = ANAS0_DW.IdentityMatrix_eg[i_0];
      i = static_cast<int32_t>(std::floor(rtb_LUFactorization_o2_l[1])) - 1;
      i_0 = i;
      if (i < 0) {
        i_0 = 0;
      } else if (i >= 2) {
        i_0 = 1;
      }

      rtb_BackwardSubstitution_f1[1] = ANAS0_DW.IdentityMatrix_eg[i_0];
      i_0 = k;
      if (k < 0) {
        i_0 = 0;
      } else if (k >= 2) {
        i_0 = 1;
      }

      rtb_BackwardSubstitution_f1[2] = ANAS0_DW.IdentityMatrix_eg[i_0 + 2];
      i_0 = i;
      if (i < 0) {
        i_0 = 0;
      } else if (i >= 2) {
        i_0 = 1;
      }

      rtb_BackwardSubstitution_f1[3] = ANAS0_DW.IdentityMatrix_eg[i_0 + 2];

      // End of S-Function (sdspperm2): '<S115>/Permute Matrix'

      // S-Function (sdspfbsub2): '<S115>/Forward Substitution' incorporates:
      //   S-Function (sdspfbsub2): '<S115>/Backward Substitution'
      //   S-Function (sdsplu2): '<S115>/LU Factorization'

      rtb_BackwardSubstitution_f1[1] -= rtb_BackwardSubstitution_f1[0] *
        rtb_LUFactorization_o1_if[1];
      rtb_BackwardSubstitution_f1[3] -= rtb_LUFactorization_o1_if[1] *
        rtb_BackwardSubstitution_f1[2];

      // S-Function (sdspfbsub2): '<S115>/Backward Substitution' incorporates:
      //   S-Function (sdsplu2): '<S115>/LU Factorization'

      rtb_BackwardSubstitution_f1[1] /= rtb_LUFactorization_o1_if[3];
      rtb_BackwardSubstitution_f1[0] = (rtb_BackwardSubstitution_f1[0] -
        rtb_BackwardSubstitution_f1[1] * rtb_LUFactorization_o1_if[2]) /
        rtb_LUFactorization_o1_if[0];
      rtb_BackwardSubstitution_f1[3] /= rtb_LUFactorization_o1_if[3];
      rtb_BackwardSubstitution_f1[2] = (rtb_BackwardSubstitution_f1[2] -
        rtb_LUFactorization_o1_if[2] * rtb_BackwardSubstitution_f1[3]) /
        rtb_LUFactorization_o1_if[0];

      // S-Function (sdspm1norm2): '<S113>/Matrix  1-Norm1'

      // DSP System Toolbox Matrix 1-Norm (sdspm1norm2) - '<S113>/Matrix  1-Norm1' 
      {
        const float *uPtr{ &rtb_BackwardSubstitution_f1[0] };

        float m1norm{ 0.0 };

        int jdx;
        for (jdx=2; jdx-- > 0; ) {
          float sumabsAj{ 0.0 };

          int idxFlt;
          for (idxFlt=2; idxFlt-- > 0; ) {
            float temp{ *uPtr++ };

            sumabsAj += fabsf(temp);
          }

          m1norm = MAX(m1norm, sumabsAj);
        }

        rtb_Matrix1Norm1_a = m1norm;
      }

      rtb_Bias3 = 1.0F / (rtb_Matrix1Norm1_a * rtb_Matrix1Norm2_m);
    }

    // End of Switch: '<S113>/Switch'

    // If: '<S107>/If' incorporates:
    //   Constant: '<S107>/NASCondLim'
    //   RelationalOperator: '<S107>/GreaterThanOrEqual'

    if (rtb_Bias3 >= ANAS0_P.NASCondLim_Value_a) {
      // Outputs for IfAction SubSystem: '<S107>/Correction' incorporates:
      //   ActionPort: '<S111>/Action Port'

      rtb_LUFactorization_o1_if[0] = rtb_LUFactorization_o2[0];
      rtb_LUFactorization_o1_if[1] = rtb_LUFactorization_o2[1];
      rtb_LUFactorization_o1_if[2] = rtb_LUFactorization_o2[2];
      rtb_LUFactorization_o1_if[3] = rtb_LUFactorization_o2[3];
      ANAS0_Correction_i(rtb_LUFactorization_o1_if, rtb_S_ce_tmp,
                         rtb_MatrixConcatenate);

      // End of Outputs for SubSystem: '<S107>/Correction'
    } else {
      // Outputs for IfAction SubSystem: '<S107>/No correction' incorporates:
      //   ActionPort: '<S112>/Action Port'

      rtb_LUFactorization_o1_if[0] = rtb_LUFactorization_o2[0];
      rtb_LUFactorization_o1_if[1] = rtb_LUFactorization_o2[1];
      rtb_LUFactorization_o1_if[2] = rtb_LUFactorization_o2[2];
      rtb_LUFactorization_o1_if[3] = rtb_LUFactorization_o2[3];
      ANAS0_Nocorrection_o(rtb_S_ce_tmp, rtb_MatrixConcatenate,
                           &ANAS0_P.Nocorrection_o);

      // End of Outputs for SubSystem: '<S107>/No correction'
    }

    // End of If: '<S107>/If'

    // Sum: '<S109>/Subtract' incorporates:
    //   Constant: '<S109>/Constant'
    //   Math: '<S143>/Transpose1'
    //   Product: '<S109>/Matrix Multiply'
    //   Product: '<S176>/Matrix Multiply2'

    for (i_0 = 0; i_0 < 6; i_0++) {
      // Product: '<S109>/Matrix Multiply' incorporates:
      //   Merge: '<S107>/Merge'

      rtb_Matrix1Norm2_m = rtb_MatrixConcatenate[i_0 + 6];
      rtb_Matrix1Norm1_a = rtb_MatrixConcatenate[i_0];
      k = 0;
      i = 0;
      for (rtb_LinearQ_tmp = 0; rtb_LinearQ_tmp < 6; rtb_LinearQ_tmp++) {
        rtb_S_tmp_0 = k + i_0;
        rtb_Transpose_e[rtb_S_tmp_0] = ANAS0_P.Constant_Value_n[rtb_S_tmp_0] -
          (rtb_Transpose1_j[i + 1] * rtb_Matrix1Norm2_m + rtb_Transpose1_j[i] *
           rtb_Matrix1Norm1_a);
        k += 6;
        i += 2;
      }
    }

    // End of Sum: '<S109>/Subtract'

    // Product: '<S105>/Matrix Multiply' incorporates:
    //   Math: '<S105>/Transpose'
    //   Merge: '<S97>/Merge1'
    //   Product: '<S176>/Matrix Multiply2'

    for (i_0 = 0; i_0 < 6; i_0++) {
      k = 0;
      for (i = 0; i < 6; i++) {
        rtb_Matrix1Norm1 = 0.0F;
        rtb_LinearQ_tmp = 0;
        for (rtb_S_tmp_0 = 0; rtb_S_tmp_0 < 6; rtb_S_tmp_0++) {
          rtb_Matrix1Norm1 += rtb_LinearQ[rtb_LinearQ_tmp + i_0] *
            rtb_Transpose_e[rtb_LinearQ_tmp + i];
          rtb_LinearQ_tmp += 6;
        }

        rtb_LinearQ_0[k + i_0] = rtb_Matrix1Norm1;
        k += 6;
      }
    }

    // Product: '<S105>/Matrix Multiply2' incorporates:
    //   Math: '<S105>/Transpose1'
    //   Merge: '<S107>/Merge'
    //   Product: '<S145>/Matrix Multiply'

    for (i_0 = 0; i_0 < 2; i_0++) {
      rtb_Divide5_a = rtb_res_GPS[i_0 + 2];
      rtb_Power2 = rtb_res_GPS[i_0];
      k = 0;
      for (i = 0; i < 6; i++) {
        rtb_Transpose1_j[k + i_0] = rtb_MatrixConcatenate[i + 6] * rtb_Divide5_a
          + rtb_Power2 * rtb_MatrixConcatenate[i];
        k += 2;
      }
    }

    // Product: '<S105>/Matrix Multiply' incorporates:
    //   Merge: '<S107>/Merge'
    //   Product: '<S105>/Matrix Multiply2'
    //   Product: '<S176>/Matrix Multiply2'

    i_0 = 0;
    k = 0;
    for (i = 0; i < 6; i++) {
      for (rtb_LinearQ_tmp = 0; rtb_LinearQ_tmp < 6; rtb_LinearQ_tmp++) {
        rtb_Matrix1Norm1 = 0.0F;
        rtb_S_tmp_0 = 0;
        for (i_1 = 0; i_1 < 6; i_1++) {
          rtb_Matrix1Norm1 += rtb_Transpose_e[rtb_S_tmp_0 + rtb_LinearQ_tmp] *
            rtb_LinearQ_0[i_1 + i_0];
          rtb_S_tmp_0 += 6;
        }

        rtb_S_tmp_0 = rtb_LinearQ_tmp + i_0;
        rtb_Transpose_aq[rtb_S_tmp_0] = rtb_Matrix1Norm1;
        rtb_Merge_p[rtb_S_tmp_0] = rtb_Transpose1_j[k + 1] *
          rtb_MatrixConcatenate[rtb_LinearQ_tmp + 6] + rtb_Transpose1_j[k] *
          rtb_MatrixConcatenate[rtb_LinearQ_tmp];
      }

      i_0 += 6;
      k += 2;
    }

    for (i_0 = 0; i_0 < 36; i_0++) {
      // Merge: '<S96>/Merge1' incorporates:
      //   Sum: '<S105>/Sum1'

      ANAS0_DW.Merge1[i_0] = rtb_Transpose_aq[i_0] + rtb_Merge_p[i_0];
    }

    // Product: '<S119>/Divide' incorporates:
    //   Constant: '<S119>/Baroa'
    //   Product: '<S122>/Divide3'

    rtb_Add = rtb_VectorConcatenate_o[2] * ANAS0_P.Baroa_Value_g;

    // Sum: '<S119>/Add' incorporates:
    //   Inport: '<Root>/ANAS Reference'
    //   Product: '<S119>/Divide'

    rtb_Bias3 = rtb_Add + ANAS0_U.ANASReference_f.GroundTemperature;

    // Sum: '<S108>/Add' incorporates:
    //   Constant: '<S117>/LocalGravity'
    //   Constant: '<S117>/RAir'
    //   Constant: '<S117>/RAir1'
    //   Constant: '<S119>/Baroa'
    //   Constant: '<S119>/Barog//R'
    //   Constant: '<S122>/Constant1'
    //   Constant: '<S122>/Constant2'
    //   DiscreteFilter: '<S116>/Discrete Filter'
    //   Gain: '<S108>/Gain'
    //   Inport: '<Root>/ANAS In'
    //   Inport: '<Root>/ANAS Reference'
    //   Math: '<S117>/Power'
    //   Math: '<S122>/Power'
    //   Product: '<S117>/Divide'
    //   Product: '<S117>/Divide1'
    //   Product: '<S117>/Product'
    //   Product: '<S117>/Product2'
    //   Product: '<S117>/Product3'
    //   Product: '<S119>/Divide1'
    //   Product: '<S122>/Divide1'
    //   Product: '<S122>/Divide4'
    //   Product: '<S122>/Divide5'
    //   Product: '<S122>/Product'
    //   SignalConversion generated from: '<S108>/Vector Concatenate'
    //   Sum: '<S117>/Subtract'
    //   Sum: '<S122>/Add'

    rtb_Matrix1Norm2_m = ANAS0_U.ANASIn_c.BaroMeasure - std::pow
      ((ANAS0_U.ANASReference_f.GroundTemperature - ANAS0_P.Gain_Gain_c *
        rtb_VectorConcatenate_o[2] * ANAS0_P.RAir1_Value_c) /
       ANAS0_U.ANASReference_f.GroundTemperature, ANAS0_P.LocalGravity_Value_h /
       (ANAS0_P.RAir1_Value_c * ANAS0_P.RAir_Value_o)) *
      ANAS0_U.ANASReference_f.GroundPressure;
    rtb_Matrix1Norm1_a = ANAS0_P.DiscreteFilter_NumCoef[1] *
      ANAS0_DW.DiscreteFilter_states - std::pow(ANAS0_P.Constant2_Value_lk -
      1.0F / rtb_Bias3 * rtb_Add, 1.0F / ANAS0_P.Baroa_Value_g *
      ANAS0_P.Constant1_Value_p * ANAS0_P.BarogR_Value_i) * (1.0F / rtb_Bias3) *
      ANAS0_P.BarogR_Value_i * ANAS0_U.ANASReference_f.GroundPressure *
      rtb_VectorConcatenate_o[5];
    for (i = 0; i < 6; i++) {
      // Merge: '<S96>/Merge' incorporates:
      //   Merge: '<S107>/Merge'
      //   Merge: '<S97>/Merge'
      //   Product: '<S98>/Matrix Multiply'
      //   Sum: '<S98>/Sum'

      ANAS0_DW.Merge[i] = (rtb_MatrixConcatenate[i + 6] * rtb_Matrix1Norm1_a +
                           rtb_MatrixConcatenate[i] * rtb_Matrix1Norm2_m) +
        rtb_VectorConcatenate_o[i];
    }

    // Update for DiscreteFilter: '<S116>/Discrete Filter' incorporates:
    //   Constant: '<S116>/frequencyPrediction'
    //   Inport: '<Root>/ANAS In'
    //   Product: '<S116>/Divide'
    //   Sum: '<S120>/Diff'
    //   UnitDelay: '<S120>/UD'
    //
    //  Block description for '<S120>/Diff':
    //
    //   Add in CPU
    //
    //  Block description for '<S120>/UD':
    //
    //   Store in Global RAM

    ANAS0_DW.DiscreteFilter_states = ((ANAS0_U.ANASIn_c.BaroMeasure -
      ANAS0_DW.UD_DSTATE_h) * ANAS0_P.frequencyPrediction_Value -
      ANAS0_P.DiscreteFilter_DenCoef[1] * ANAS0_DW.DiscreteFilter_states) /
      ANAS0_P.DiscreteFilter_DenCoef[0];

    // Update for UnitDelay: '<S120>/UD' incorporates:
    //   Inport: '<Root>/ANAS In'
    //
    //  Block description for '<S120>/UD':
    //
    //   Store in Global RAM

    ANAS0_DW.UD_DSTATE_h = ANAS0_U.ANASIn_c.BaroMeasure;
  }

  // End of Outputs for SubSystem: '<S96>/Active Correction Step Baro & Variometer ' 

  // Logic: '<S103>/AND9' incorporates:
  //   Constant: '<S103>/StaticPitotFlag'
  //   Logic: '<S103>/NOT3'

  rtb_main_s_w_pitot_d = ((!ANAS0_P.StaticPitotFlag_Value) &&
    rtb_LUFactorization_o3);

  // Outputs for Enabled SubSystem: '<S96>/Active Correction Step Main Static & Pitot' incorporates:
  //   EnablePort: '<S99>/Enable'

  if (rtb_main_s_w_pitot_d) {
    // Reshape: '<S124>/Reshape3' incorporates:
    //   Constant: '<S124>/Constant1'

    rtb_MatrixMultiply_f[0] = ANAS0_P.Constant1_Value_h[0];
    rtb_MatrixMultiply_f[1] = ANAS0_P.Constant1_Value_h[1];

    // Product: '<S124>/Divide' incorporates:
    //   Constant: '<S124>/Constant8'
    //   Product: '<S130>/Divide3'

    rtb_Add = rtb_VectorConcatenate_o[2] * ANAS0_P.Constant8_Value;

    // Sum: '<S124>/Add' incorporates:
    //   Inport: '<Root>/ANAS Reference'
    //   Product: '<S124>/Divide'

    rtb_Bias3 = rtb_Add + ANAS0_U.ANASReference_f.GroundTemperature;

    // Product: '<S130>/Product' incorporates:
    //   Constant: '<S124>/Barog//R'
    //   Constant: '<S124>/Constant8'
    //   Constant: '<S130>/Constant1'
    //   Constant: '<S130>/Constant2'
    //   Inport: '<Root>/ANAS Reference'
    //   Math: '<S130>/Power'
    //   Product: '<S130>/Divide1'
    //   Product: '<S130>/Divide4'
    //   Product: '<S130>/Divide5'
    //   Sum: '<S130>/Add'

    rtb_Power = std::pow(ANAS0_P.Constant2_Value_n - 1.0F / rtb_Bias3 * rtb_Add,
                         1.0F / ANAS0_P.Constant8_Value *
                         ANAS0_P.Constant1_Value_j * ANAS0_P.BarogR_Value_b) *
      (1.0F / rtb_Bias3) * ANAS0_P.BarogR_Value_b *
      ANAS0_U.ANASReference_f.GroundPressure;

    // Bias: '<S129>/Bias2' incorporates:
    //   Constant: '<S129>/Constant10'
    //   Constant: '<S129>/Constant11'
    //   Constant: '<S129>/Constant6'
    //   Gain: '<S129>/Gain2'
    //   Product: '<S129>/Divide6'
    //   Product: '<S129>/Divide7'

    rtb_Divide1_p = 1.0F / ANAS0_P.Constant10_Value / ANAS0_P.Constant6_Value_f /
      rtb_Bias3 * rtb_Matrix1Norm2 * ANAS0_P.Constant11_Value *
      ANAS0_P.Gain2_Gain_h + ANAS0_P.Bias2_Bias;

    // Product: '<S124>/Divide3' incorporates:
    //   Bias: '<S124>/Bias1'
    //   Constant: '<S124>/IsaGamma'

    rtb_Divide5_a = 1.0F / (ANAS0_P.IsaGamma_Value + ANAS0_P.Bias1_Bias) *
      ANAS0_P.IsaGamma_Value;

    // Product: '<S128>/Product3' incorporates:
    //   Constant: '<S128>/LocalGravity'
    //   Constant: '<S128>/RAir'
    //   Constant: '<S128>/RAir1'
    //   Gain: '<S124>/Gain1'
    //   Inport: '<Root>/ANAS Reference'
    //   Math: '<S128>/Power'
    //   Product: '<S128>/Divide'
    //   Product: '<S128>/Divide1'
    //   Product: '<S128>/Product'
    //   Product: '<S128>/Product2'
    //   Sum: '<S128>/Subtract'

    rtb_Power2 = std::pow((ANAS0_U.ANASReference_f.GroundTemperature -
      ANAS0_P.Gain1_Gain_b * rtb_VectorConcatenate_o[2] * ANAS0_P.RAir1_Value_cc)
                          / ANAS0_U.ANASReference_f.GroundTemperature,
                          ANAS0_P.LocalGravity_Value_p / (ANAS0_P.RAir1_Value_cc
      * ANAS0_P.RAir_Value_p)) * ANAS0_U.ANASReference_f.GroundPressure;

    // Reshape: '<S124>/Reshape4' incorporates:
    //   Bias: '<S124>/Bias2'
    //   Bias: '<S124>/Bias4'
    //   Constant: '<S124>/Constant8'
    //   Gain: '<S124>/Baro0.5//R'
    //   Math: '<S124>/Power'
    //   Math: '<S124>/Power1'
    //   Math: '<S124>/Square'
    //   Product: '<S124>/Divide4'
    //   Product: '<S124>/Divide5'
    //   Sum: '<S124>/Add1'

    rtb_MatrixMultiply_f[2] = (std::pow(rtb_Divide1_p, rtb_Divide5_a) +
      ANAS0_P.Bias4_Bias) * rtb_Power - std::pow(rtb_Divide1_p, rtb_Divide5_a +
      ANAS0_P.Bias2_Bias_l) * rtb_Power2 * rtb_Matrix1Norm2 *
      ANAS0_P.Constant8_Value / (rtb_Bias3 * rtb_Bias3) * ANAS0_P.Baro05R_Gain;

    // Gain: '<S124>/Baro1//R' incorporates:
    //   Bias: '<S124>/Bias3'
    //   Math: '<S124>/Power2'
    //   Product: '<S124>/Divide6'

    rtb_Bias3 = std::pow(rtb_Divide1_p, rtb_Divide5_a + ANAS0_P.Bias3_Bias) *
      rtb_Power2 / rtb_Bias3 * ANAS0_P.Baro1R_Gain;

    // Reshape: '<S124>/Reshape5' incorporates:
    //   Product: '<S124>/Matrix Multiply'

    rtb_MatrixMultiply_f[3] = rtb_VectorConcatenate_o[3] * rtb_Bias3;
    rtb_MatrixMultiply_f[4] = rtb_VectorConcatenate_o[4] * rtb_Bias3;
    rtb_MatrixMultiply_f[5] = rtb_VectorConcatenate_o[5] * rtb_Bias3;

    // Reshape: '<S124>/Reshape1' incorporates:
    //   Constant: '<S124>/Constant3'

    rtb_Transpose_k[0] = ANAS0_P.Constant3_Value_a[0];
    rtb_Transpose_k[1] = ANAS0_P.Constant3_Value_a[1];

    // Reshape: '<S124>/Reshape'
    rtb_Transpose_k[2] = rtb_Power;

    // Reshape: '<S124>/Reshape2' incorporates:
    //   Constant: '<S124>/Constant9'

    rtb_Transpose_k[3] = ANAS0_P.Constant9_Value_o[0];
    rtb_Transpose_k[4] = ANAS0_P.Constant9_Value_o[1];
    rtb_Transpose_k[5] = ANAS0_P.Constant9_Value_o[2];

    // Concatenate: '<S124>/Matrix Concatenate1' incorporates:
    //   Math: '<S143>/Transpose1'
    //   Math: '<S178>/Transpose'
    //   Product: '<S102>/Matrix Multiply'

    i_0 = 0;
    for (k = 0; k < 6; k++) {
      rtb_Transpose1_j[i_0] = rtb_MatrixMultiply_f[k];
      rtb_Transpose1_j[i_0 + 1] = rtb_Transpose_k[k];
      i_0 += 2;
    }

    // End of Concatenate: '<S124>/Matrix Concatenate1'

    // Math: '<S125>/Transpose' incorporates:
    //   Math: '<S143>/Transpose1'
    //   Merge: '<S125>/Merge'

    i_0 = 0;
    for (k = 0; k < 2; k++) {
      i = 0;
      for (rtb_LinearQ_tmp = 0; rtb_LinearQ_tmp < 6; rtb_LinearQ_tmp++) {
        rtb_MatrixConcatenate[rtb_LinearQ_tmp + i_0] = rtb_Transpose1_j[i + k];
        i += 2;
      }

      i_0 += 6;
    }

    // End of Math: '<S125>/Transpose'

    // Math: '<S99>/Square' incorporates:
    //   Constant: '<S99>/PitotDBaroSigma'
    //   Product: '<S145>/Matrix Multiply'

    rtb_res_GPS[0] = ANAS0_P.PitotDBaroSigma_Value[0] *
      ANAS0_P.PitotDBaroSigma_Value[0];
    rtb_res_GPS[1] = ANAS0_P.PitotDBaroSigma_Value[1] *
      ANAS0_P.PitotDBaroSigma_Value[1];
    rtb_res_GPS[2] = ANAS0_P.PitotDBaroSigma_Value[2] *
      ANAS0_P.PitotDBaroSigma_Value[2];
    rtb_res_GPS[3] = ANAS0_P.PitotDBaroSigma_Value[3] *
      ANAS0_P.PitotDBaroSigma_Value[3];

    // Product: '<S125>/Matrix Multiply' incorporates:
    //   Merge: '<S125>/Merge'
    //   Merge: '<S97>/Merge1'
    //   Product: '<S125>/Matrix Multiply1'

    i_0 = 0;
    for (k = 0; k < 2; k++) {
      for (i = 0; i < 6; i++) {
        rtb_Matrix1Norm1 = 0.0F;
        rtb_LinearQ_tmp = 0;
        for (rtb_S_tmp_0 = 0; rtb_S_tmp_0 < 6; rtb_S_tmp_0++) {
          rtb_Matrix1Norm1 += rtb_LinearQ[rtb_LinearQ_tmp + i] *
            rtb_MatrixConcatenate[rtb_S_tmp_0 + i_0];
          rtb_LinearQ_tmp += 6;
        }

        rtb_S_ce_tmp[i + i_0] = rtb_Matrix1Norm1;
      }

      i_0 += 6;
    }

    for (i_0 = 0; i_0 < 2; i_0++) {
      // Sum: '<S125>/Add' incorporates:
      //   Math: '<S143>/Transpose1'
      //   Product: '<S125>/Matrix Multiply'
      //   Product: '<S145>/Matrix Multiply'

      k = 0;
      i = 0;
      for (rtb_LinearQ_tmp = 0; rtb_LinearQ_tmp < 2; rtb_LinearQ_tmp++) {
        rtb_Power2 = 0.0F;
        rtb_S_tmp_0 = 0;
        for (i_1 = 0; i_1 < 6; i_1++) {
          rtb_Power2 += rtb_Transpose1_j[rtb_S_tmp_0 + i_0] * rtb_S_ce_tmp[i_1 +
            i];
          rtb_S_tmp_0 += 2;
        }

        rtb_S_tmp_0 = k + i_0;
        rtb_LUFactorization_o1_if[rtb_S_tmp_0] = rtb_res_GPS[rtb_S_tmp_0] +
          rtb_Power2;
        k += 2;
        i += 6;
      }
    }

    // S-Function (sdsplu2): '<S136>/LU Factorization' incorporates:
    //   Sum: '<S125>/Add'

    rtb_LUFactorization_o1_iq[0] = rtb_LUFactorization_o1_if[0];
    rtb_LUFactorization_o1_iq[1] = rtb_LUFactorization_o1_if[1];
    rtb_LUFactorization_o1_iq[2] = rtb_LUFactorization_o1_if[2];
    rtb_LUFactorization_o1_iq[3] = rtb_LUFactorization_o1_if[3];
    LUf_boolfloatint32_t(&rtb_LUFactorization_o1_iq[0],
                         &rtb_LUFactorization_o2_b[0], 2,
                         &rtb_LUFactorization_o3_i);

    // Switch: '<S134>/Switch' incorporates:
    //   Constant: '<S134>/Constant'
    //   Math: '<S134>/Math Function'
    //   Product: '<S134>/Product'
    //
    //  About '<S134>/Math Function':
    //   Operator: reciprocal

    if (rtb_LUFactorization_o3_i) {
      rtb_Bias3 = ANAS0_P.Constant_Value_ez;
    } else {
      // S-Function (sdspm1norm2): '<S134>/Matrix  1-Norm2'

      // DSP System Toolbox Matrix 1-Norm (sdspm1norm2) - '<S134>/Matrix  1-Norm2' 
      {
        const float *uPtr{ &rtb_LUFactorization_o1_if[0] };

        float m1norm{ 0.0 };

        int jdx;
        for (jdx=2; jdx-- > 0; ) {
          float sumabsAj{ 0.0 };

          int idxFlt;
          for (idxFlt=2; idxFlt-- > 0; ) {
            float temp{ *uPtr++ };

            sumabsAj += fabsf(temp);
          }

          m1norm = MAX(m1norm, sumabsAj);
        }

        rtb_Matrix1Norm2_dg = m1norm;
      }

      // S-Function (sdspperm2): '<S136>/Permute Matrix' incorporates:
      //   IdentityMatrix: '<S136>/Identity Matrix'
      //   S-Function (sdspfbsub2): '<S136>/Backward Substitution'

      k = static_cast<int32_t>(std::floor(rtb_LUFactorization_o2_b[0])) - 1;
      i_0 = k;
      if (k < 0) {
        i_0 = 0;
      } else if (k >= 2) {
        i_0 = 1;
      }

      rtb_BackwardSubstitution_n[0] = ANAS0_DW.IdentityMatrix_g[i_0];
      i = static_cast<int32_t>(std::floor(rtb_LUFactorization_o2_b[1])) - 1;
      i_0 = i;
      if (i < 0) {
        i_0 = 0;
      } else if (i >= 2) {
        i_0 = 1;
      }

      rtb_BackwardSubstitution_n[1] = ANAS0_DW.IdentityMatrix_g[i_0];
      i_0 = k;
      if (k < 0) {
        i_0 = 0;
      } else if (k >= 2) {
        i_0 = 1;
      }

      rtb_BackwardSubstitution_n[2] = ANAS0_DW.IdentityMatrix_g[i_0 + 2];
      i_0 = i;
      if (i < 0) {
        i_0 = 0;
      } else if (i >= 2) {
        i_0 = 1;
      }

      rtb_BackwardSubstitution_n[3] = ANAS0_DW.IdentityMatrix_g[i_0 + 2];

      // End of S-Function (sdspperm2): '<S136>/Permute Matrix'

      // S-Function (sdspfbsub2): '<S136>/Forward Substitution' incorporates:
      //   S-Function (sdspfbsub2): '<S136>/Backward Substitution'
      //   S-Function (sdsplu2): '<S136>/LU Factorization'

      rtb_BackwardSubstitution_n[1] -= rtb_BackwardSubstitution_n[0] *
        rtb_LUFactorization_o1_iq[1];
      rtb_BackwardSubstitution_n[3] -= rtb_LUFactorization_o1_iq[1] *
        rtb_BackwardSubstitution_n[2];

      // S-Function (sdspfbsub2): '<S136>/Backward Substitution' incorporates:
      //   S-Function (sdsplu2): '<S136>/LU Factorization'

      rtb_BackwardSubstitution_n[1] /= rtb_LUFactorization_o1_iq[3];
      rtb_BackwardSubstitution_n[0] = (rtb_BackwardSubstitution_n[0] -
        rtb_BackwardSubstitution_n[1] * rtb_LUFactorization_o1_iq[2]) /
        rtb_LUFactorization_o1_iq[0];
      rtb_BackwardSubstitution_n[3] /= rtb_LUFactorization_o1_iq[3];
      rtb_BackwardSubstitution_n[2] = (rtb_BackwardSubstitution_n[2] -
        rtb_LUFactorization_o1_iq[2] * rtb_BackwardSubstitution_n[3]) /
        rtb_LUFactorization_o1_iq[0];

      // S-Function (sdspm1norm2): '<S134>/Matrix  1-Norm1'

      // DSP System Toolbox Matrix 1-Norm (sdspm1norm2) - '<S134>/Matrix  1-Norm1' 
      {
        const float *uPtr{ &rtb_BackwardSubstitution_n[0] };

        float m1norm{ 0.0 };

        int jdx;
        for (jdx=2; jdx-- > 0; ) {
          float sumabsAj{ 0.0 };

          int idxFlt;
          for (idxFlt=2; idxFlt-- > 0; ) {
            float temp{ *uPtr++ };

            sumabsAj += fabsf(temp);
          }

          m1norm = MAX(m1norm, sumabsAj);
        }

        rtb_Matrix1Norm1_ax = m1norm;
      }

      rtb_Bias3 = 1.0F / (rtb_Matrix1Norm1_ax * rtb_Matrix1Norm2_dg);
    }

    // End of Switch: '<S134>/Switch'

    // If: '<S125>/If' incorporates:
    //   Constant: '<S125>/NASCondLim'
    //   RelationalOperator: '<S125>/GreaterThanOrEqual'

    if (rtb_Bias3 >= ANAS0_P.NASCondLim_Value_g) {
      // Outputs for IfAction SubSystem: '<S125>/Correction' incorporates:
      //   ActionPort: '<S132>/Action Port'

      rtb_LUFactorization_o1_iq[0] = rtb_LUFactorization_o1_if[0];
      rtb_LUFactorization_o1_iq[1] = rtb_LUFactorization_o1_if[1];
      rtb_LUFactorization_o1_iq[2] = rtb_LUFactorization_o1_if[2];
      rtb_LUFactorization_o1_iq[3] = rtb_LUFactorization_o1_if[3];
      ANAS0_Correction_i(rtb_LUFactorization_o1_iq, rtb_S_ce_tmp,
                         rtb_MatrixConcatenate);

      // End of Outputs for SubSystem: '<S125>/Correction'
    } else {
      // Outputs for IfAction SubSystem: '<S125>/No correction' incorporates:
      //   ActionPort: '<S133>/Action Port'

      rtb_LUFactorization_o1_iq[0] = rtb_LUFactorization_o1_if[0];
      rtb_LUFactorization_o1_iq[1] = rtb_LUFactorization_o1_if[1];
      rtb_LUFactorization_o1_iq[2] = rtb_LUFactorization_o1_if[2];
      rtb_LUFactorization_o1_iq[3] = rtb_LUFactorization_o1_if[3];
      ANAS0_Nocorrection_o(rtb_S_ce_tmp, rtb_MatrixConcatenate,
                           &ANAS0_P.Nocorrection_g);

      // End of Outputs for SubSystem: '<S125>/No correction'
    }

    // End of If: '<S125>/If'

    // Sum: '<S127>/Subtract' incorporates:
    //   Constant: '<S127>/Constant'
    //   Math: '<S143>/Transpose1'
    //   Product: '<S127>/Matrix Multiply'
    //   Product: '<S176>/Matrix Multiply2'

    for (i_0 = 0; i_0 < 6; i_0++) {
      // Product: '<S127>/Matrix Multiply' incorporates:
      //   Merge: '<S125>/Merge'

      rtb_Matrix1Norm2_m = rtb_MatrixConcatenate[i_0 + 6];
      rtb_Matrix1Norm1_a = rtb_MatrixConcatenate[i_0];
      k = 0;
      i = 0;
      for (rtb_LinearQ_tmp = 0; rtb_LinearQ_tmp < 6; rtb_LinearQ_tmp++) {
        rtb_S_tmp_0 = k + i_0;
        rtb_Transpose_e[rtb_S_tmp_0] = ANAS0_P.Constant_Value_k5[rtb_S_tmp_0] -
          (rtb_Transpose1_j[i + 1] * rtb_Matrix1Norm2_m + rtb_Transpose1_j[i] *
           rtb_Matrix1Norm1_a);
        k += 6;
        i += 2;
      }
    }

    // End of Sum: '<S127>/Subtract'

    // Product: '<S123>/Matrix Multiply' incorporates:
    //   Math: '<S123>/Transpose'
    //   Merge: '<S97>/Merge1'
    //   Product: '<S176>/Matrix Multiply2'

    for (i_0 = 0; i_0 < 6; i_0++) {
      k = 0;
      for (i = 0; i < 6; i++) {
        rtb_Matrix1Norm1 = 0.0F;
        rtb_LinearQ_tmp = 0;
        for (rtb_S_tmp_0 = 0; rtb_S_tmp_0 < 6; rtb_S_tmp_0++) {
          rtb_Matrix1Norm1 += rtb_LinearQ[rtb_LinearQ_tmp + i_0] *
            rtb_Transpose_e[rtb_LinearQ_tmp + i];
          rtb_LinearQ_tmp += 6;
        }

        rtb_LinearQ_0[k + i_0] = rtb_Matrix1Norm1;
        k += 6;
      }
    }

    // Product: '<S123>/Matrix Multiply2' incorporates:
    //   Math: '<S123>/Transpose1'
    //   Merge: '<S125>/Merge'
    //   Product: '<S145>/Matrix Multiply'

    for (i_0 = 0; i_0 < 2; i_0++) {
      rtb_Divide5_a = rtb_res_GPS[i_0 + 2];
      rtb_Power2 = rtb_res_GPS[i_0];
      k = 0;
      for (i = 0; i < 6; i++) {
        rtb_Transpose1_j[k + i_0] = rtb_MatrixConcatenate[i + 6] * rtb_Divide5_a
          + rtb_Power2 * rtb_MatrixConcatenate[i];
        k += 2;
      }
    }

    // Product: '<S123>/Matrix Multiply' incorporates:
    //   Merge: '<S125>/Merge'
    //   Product: '<S123>/Matrix Multiply2'
    //   Product: '<S176>/Matrix Multiply2'

    i_0 = 0;
    k = 0;
    for (i = 0; i < 6; i++) {
      for (rtb_LinearQ_tmp = 0; rtb_LinearQ_tmp < 6; rtb_LinearQ_tmp++) {
        rtb_Matrix1Norm1 = 0.0F;
        rtb_S_tmp_0 = 0;
        for (i_1 = 0; i_1 < 6; i_1++) {
          rtb_Matrix1Norm1 += rtb_Transpose_e[rtb_S_tmp_0 + rtb_LinearQ_tmp] *
            rtb_LinearQ_0[i_1 + i_0];
          rtb_S_tmp_0 += 6;
        }

        rtb_S_tmp_0 = rtb_LinearQ_tmp + i_0;
        rtb_Transpose_aq[rtb_S_tmp_0] = rtb_Matrix1Norm1;
        rtb_Merge_p[rtb_S_tmp_0] = rtb_Transpose1_j[k + 1] *
          rtb_MatrixConcatenate[rtb_LinearQ_tmp + 6] + rtb_Transpose1_j[k] *
          rtb_MatrixConcatenate[rtb_LinearQ_tmp];
      }

      i_0 += 6;
      k += 2;
    }

    for (i_0 = 0; i_0 < 36; i_0++) {
      // Merge: '<S96>/Merge1' incorporates:
      //   Sum: '<S123>/Sum1'

      ANAS0_DW.Merge1[i_0] = rtb_Transpose_aq[i_0] + rtb_Merge_p[i_0];
    }

    // Product: '<S139>/Product3' incorporates:
    //   Constant: '<S139>/LocalGravity'
    //   Constant: '<S139>/RAir'
    //   Constant: '<S139>/RAir1'
    //   Gain: '<S137>/Gain'
    //   Inport: '<Root>/ANAS Reference'
    //   Math: '<S139>/Power'
    //   Product: '<S139>/Divide'
    //   Product: '<S139>/Divide1'
    //   Product: '<S139>/Product'
    //   Product: '<S139>/Product2'
    //   Sum: '<S139>/Subtract'

    rtb_Bias3 = std::pow((ANAS0_U.ANASReference_f.GroundTemperature -
                          ANAS0_P.Gain_Gain_l * rtb_VectorConcatenate_o[2] *
                          ANAS0_P.RAir1_Value_m) /
                         ANAS0_U.ANASReference_f.GroundTemperature,
                         ANAS0_P.LocalGravity_Value_e / (ANAS0_P.RAir1_Value_m *
      ANAS0_P.RAir_Value_a)) * ANAS0_U.ANASReference_f.GroundPressure;

    // Sum: '<S126>/Add2' incorporates:
    //   Bias: '<S137>/Bias'
    //   Bias: '<S140>/Bias2'
    //   Constant: '<S137>/Baroa'
    //   Constant: '<S137>/Constant'
    //   Constant: '<S137>/IsaGamma'
    //   Constant: '<S140>/AirR'
    //   Constant: '<S140>/IsaGamma'
    //   Constant: '<S140>/IsaGamma-1'
    //   Gain: '<S140>/Gain2'
    //   Inport: '<Root>/ANAS In'
    //   Inport: '<Root>/ANAS Reference'
    //   Math: '<S137>/Power'
    //   Product: '<S137>/Divide'
    //   Product: '<S137>/Divide1'
    //   Product: '<S137>/Divide2'
    //   Product: '<S140>/Divide6'
    //   Product: '<S140>/Divide7'
    //   SignalConversion generated from: '<S126>/Vector Concatenate1'
    //   Sum: '<S137>/Add'
    //   Sum: '<S137>/Add3'

    rtb_Matrix1Norm2_dg = ANAS0_U.ANASIn_c.PitotMeasure[0] - (std::pow(1.0F /
      ANAS0_P.AirR_Value / ANAS0_P.IsaGamma_Value_i / (ANAS0_P.Baroa_Value_a *
      rtb_VectorConcatenate_o[2] + ANAS0_U.ANASReference_f.GroundTemperature) *
      rtb_Matrix1Norm2 * ANAS0_P.IsaGamma1_Value * ANAS0_P.Gain2_Gain_k +
      ANAS0_P.Bias2_Bias_b, 1.0F / (ANAS0_P.IsaGamma_Value_iz +
      ANAS0_P.Bias_Bias_p) * ANAS0_P.IsaGamma_Value_iz) -
      ANAS0_P.Constant_Value_pb) * rtb_Bias3;
    rtb_Matrix1Norm1_ax = ANAS0_U.ANASIn_c.BaroMeasure - rtb_Bias3;
    for (i = 0; i < 6; i++) {
      // Merge: '<S96>/Merge' incorporates:
      //   Merge: '<S125>/Merge'
      //   Merge: '<S97>/Merge'
      //   Product: '<S99>/Matrix Multiply'
      //   Sum: '<S99>/Sum'

      ANAS0_DW.Merge[i] = (rtb_MatrixConcatenate[i + 6] * rtb_Matrix1Norm1_ax +
                           rtb_MatrixConcatenate[i] * rtb_Matrix1Norm2_dg) +
        rtb_VectorConcatenate_o[i];
    }
  }

  // End of Outputs for SubSystem: '<S96>/Active Correction Step Main Static & Pitot' 

  // Logic: '<S103>/AND8' incorporates:
  //   Constant: '<S103>/StaticPitotFlag'

  rtb_LUFactorization_o3_i = (rtb_LUFactorization_o3 &&
    ANAS0_P.StaticPitotFlag_Value);

  // Outputs for Enabled SubSystem: '<S96>/Active Correction Step Pitot Static & Differential' incorporates:
  //   EnablePort: '<S100>/Enable'

  if (rtb_LUFactorization_o3_i) {
    // Reshape: '<S144>/Reshape3' incorporates:
    //   Constant: '<S144>/Constant1'

    rtb_MatrixMultiply_f[0] = ANAS0_P.Constant1_Value_k[0];
    rtb_MatrixMultiply_f[1] = ANAS0_P.Constant1_Value_k[1];

    // Product: '<S144>/Divide' incorporates:
    //   Constant: '<S144>/Constant8'
    //   Product: '<S149>/Divide3'

    rtb_Bias3 = rtb_VectorConcatenate_o[2] * ANAS0_P.Constant8_Value_c;

    // Sum: '<S144>/Add' incorporates:
    //   Inport: '<Root>/ANAS Reference'
    //   Product: '<S144>/Divide'

    rtb_Add = rtb_Bias3 + ANAS0_U.ANASReference_f.GroundTemperature;

    // Product: '<S144>/Divide2' incorporates:
    //   Constant: '<S144>/Constant4'
    //   Constant: '<S144>/Constant7'

    rtb_Divide1_p = ANAS0_P.Constant4_Value / ANAS0_P.Constant7_Value;

    // Product: '<S149>/Product' incorporates:
    //   Constant: '<S144>/Constant8'
    //   Constant: '<S149>/Constant1'
    //   Constant: '<S149>/Constant2'
    //   Inport: '<Root>/ANAS Reference'
    //   Math: '<S149>/Power'
    //   Product: '<S149>/Divide1'
    //   Product: '<S149>/Divide4'
    //   Product: '<S149>/Divide5'
    //   Sum: '<S149>/Add'

    rtb_Divide5_a = std::pow(ANAS0_P.Constant2_Value_d - 1.0F / rtb_Add *
      rtb_Bias3, 1.0F / ANAS0_P.Constant8_Value_c * ANAS0_P.Constant1_Value_n *
      rtb_Divide1_p) * (1.0F / rtb_Add) * rtb_Divide1_p *
      ANAS0_U.ANASReference_f.GroundPressure;

    // Bias: '<S148>/Bias2' incorporates:
    //   Constant: '<S148>/Constant10'
    //   Constant: '<S148>/Constant11'
    //   Constant: '<S148>/Constant6'
    //   Gain: '<S148>/Gain2'
    //   Product: '<S148>/Divide6'
    //   Product: '<S148>/Divide7'

    rtb_Power2 = 1.0F / ANAS0_P.Constant10_Value_i / ANAS0_P.Constant6_Value_j /
      rtb_Add * rtb_Matrix1Norm2 * ANAS0_P.Constant11_Value_k *
      ANAS0_P.Gain2_Gain_n + ANAS0_P.Bias2_Bias_f;

    // Product: '<S144>/Divide3' incorporates:
    //   Bias: '<S144>/Bias1'
    //   Constant: '<S144>/Constant2'

    rtb_Bias3 = 1.0F / (ANAS0_P.Constant2_Value_f + ANAS0_P.Bias1_Bias_c) *
      ANAS0_P.Constant2_Value_f;

    // Product: '<S150>/Product3' incorporates:
    //   Constant: '<S150>/LocalGravity'
    //   Constant: '<S150>/RAir'
    //   Constant: '<S150>/RAir1'
    //   Gain: '<S144>/Gain1'
    //   Inport: '<Root>/ANAS Reference'
    //   Math: '<S150>/Power'
    //   Product: '<S150>/Divide'
    //   Product: '<S150>/Divide1'
    //   Product: '<S150>/Product'
    //   Product: '<S150>/Product2'
    //   Sum: '<S150>/Subtract'

    rtb_Power = std::pow((ANAS0_U.ANASReference_f.GroundTemperature -
                          ANAS0_P.Gain1_Gain_n * rtb_VectorConcatenate_o[2] *
                          ANAS0_P.RAir1_Value_d) /
                         ANAS0_U.ANASReference_f.GroundTemperature,
                         ANAS0_P.LocalGravity_Value_o / (ANAS0_P.RAir1_Value_d *
      ANAS0_P.RAir_Value_k)) * ANAS0_U.ANASReference_f.GroundPressure;

    // Reshape: '<S144>/Reshape4' incorporates:
    //   Bias: '<S144>/Bias2'
    //   Bias: '<S144>/Bias4'
    //   Constant: '<S144>/Constant8'
    //   Gain: '<S144>/Gain'
    //   Math: '<S144>/Power'
    //   Math: '<S144>/Power1'
    //   Math: '<S144>/Square'
    //   Product: '<S144>/Divide4'
    //   Product: '<S144>/Divide5'
    //   Sum: '<S144>/Add1'

    rtb_MatrixMultiply_f[2] = (std::pow(rtb_Power2, rtb_Bias3) +
      ANAS0_P.Bias4_Bias_f) * rtb_Divide5_a - std::pow(rtb_Power2, rtb_Bias3 +
      ANAS0_P.Bias2_Bias_j) * rtb_Power * rtb_Matrix1Norm2 *
      ANAS0_P.Constant8_Value_c / (rtb_Add * rtb_Add) * ANAS0_P.Gain_Gain_c1;

    // Gain: '<S144>/Gain2' incorporates:
    //   Bias: '<S144>/Bias3'
    //   Math: '<S144>/Power2'
    //   Product: '<S144>/Divide6'

    rtb_Bias3 = std::pow(rtb_Power2, rtb_Bias3 + ANAS0_P.Bias3_Bias_f) *
      rtb_Power / rtb_Add * ANAS0_P.Gain2_Gain_f;

    // Reshape: '<S144>/Reshape5' incorporates:
    //   Product: '<S144>/Matrix Multiply'

    rtb_MatrixMultiply_f[3] = rtb_VectorConcatenate_o[3] * rtb_Bias3;
    rtb_MatrixMultiply_f[4] = rtb_VectorConcatenate_o[4] * rtb_Bias3;
    rtb_MatrixMultiply_f[5] = rtb_VectorConcatenate_o[5] * rtb_Bias3;

    // Reshape: '<S144>/Reshape1' incorporates:
    //   Constant: '<S144>/Constant3'

    rtb_Transpose_k[0] = ANAS0_P.Constant3_Value_l[0];
    rtb_Transpose_k[1] = ANAS0_P.Constant3_Value_l[1];

    // Reshape: '<S144>/Reshape'
    rtb_Transpose_k[2] = rtb_Divide5_a;

    // Reshape: '<S144>/Reshape2' incorporates:
    //   Constant: '<S144>/Constant9'

    rtb_Transpose_k[3] = ANAS0_P.Constant9_Value_m[0];
    rtb_Transpose_k[4] = ANAS0_P.Constant9_Value_m[1];
    rtb_Transpose_k[5] = ANAS0_P.Constant9_Value_m[2];

    // Concatenate: '<S144>/Matrix Concatenate1' incorporates:
    //   Math: '<S143>/Transpose1'
    //   Math: '<S178>/Transpose'
    //   Product: '<S102>/Matrix Multiply'

    i_0 = 0;
    for (k = 0; k < 6; k++) {
      rtb_Transpose1_j[i_0] = rtb_MatrixMultiply_f[k];
      rtb_Transpose1_j[i_0 + 1] = rtb_Transpose_k[k];
      i_0 += 2;
    }

    // End of Concatenate: '<S144>/Matrix Concatenate1'

    // Math: '<S145>/Transpose' incorporates:
    //   Math: '<S143>/Transpose1'
    //   Merge: '<S145>/Merge'

    i_0 = 0;
    for (k = 0; k < 2; k++) {
      i = 0;
      for (rtb_LinearQ_tmp = 0; rtb_LinearQ_tmp < 6; rtb_LinearQ_tmp++) {
        rtb_MatrixConcatenate[rtb_LinearQ_tmp + i_0] = rtb_Transpose1_j[i + k];
        i += 2;
      }

      i_0 += 6;
    }

    // End of Math: '<S145>/Transpose'

    // Math: '<S100>/Square' incorporates:
    //   Constant: '<S100>/PitotDSSigma'

    rtb_LUFactorization_o1_iq[0] = ANAS0_P.PitotDSSigma_Value[0] *
      ANAS0_P.PitotDSSigma_Value[0];
    rtb_LUFactorization_o1_iq[1] = ANAS0_P.PitotDSSigma_Value[1] *
      ANAS0_P.PitotDSSigma_Value[1];
    rtb_LUFactorization_o1_iq[2] = ANAS0_P.PitotDSSigma_Value[2] *
      ANAS0_P.PitotDSSigma_Value[2];
    rtb_LUFactorization_o1_iq[3] = ANAS0_P.PitotDSSigma_Value[3] *
      ANAS0_P.PitotDSSigma_Value[3];

    // Product: '<S145>/Matrix Multiply' incorporates:
    //   Merge: '<S145>/Merge'
    //   Merge: '<S97>/Merge1'
    //   Product: '<S145>/Matrix Multiply1'

    i_0 = 0;
    for (k = 0; k < 2; k++) {
      for (i = 0; i < 6; i++) {
        rtb_Matrix1Norm1 = 0.0F;
        rtb_LinearQ_tmp = 0;
        for (rtb_S_tmp_0 = 0; rtb_S_tmp_0 < 6; rtb_S_tmp_0++) {
          rtb_Matrix1Norm1 += rtb_LinearQ[rtb_LinearQ_tmp + i] *
            rtb_MatrixConcatenate[rtb_S_tmp_0 + i_0];
          rtb_LinearQ_tmp += 6;
        }

        rtb_S_ce_tmp[i + i_0] = rtb_Matrix1Norm1;
      }

      i_0 += 6;
    }

    for (i_0 = 0; i_0 < 2; i_0++) {
      // Sum: '<S145>/Add' incorporates:
      //   Math: '<S100>/Square'
      //   Math: '<S143>/Transpose1'
      //   Product: '<S145>/Matrix Multiply'

      k = 0;
      i = 0;
      for (rtb_LinearQ_tmp = 0; rtb_LinearQ_tmp < 2; rtb_LinearQ_tmp++) {
        rtb_Power2 = 0.0F;
        rtb_S_tmp_0 = 0;
        for (i_1 = 0; i_1 < 6; i_1++) {
          rtb_Power2 += rtb_Transpose1_j[rtb_S_tmp_0 + i_0] * rtb_S_ce_tmp[i_1 +
            i];
          rtb_S_tmp_0 += 2;
        }

        rtb_S_tmp_0 = k + i_0;
        rtb_BackwardSubstitution_n[rtb_S_tmp_0] =
          rtb_LUFactorization_o1_iq[rtb_S_tmp_0] + rtb_Power2;
        k += 2;
        i += 6;
      }
    }

    // S-Function (sdsplu2): '<S156>/LU Factorization' incorporates:
    //   Sum: '<S145>/Add'

    rtb_LUFactorization_o1_l[0] = rtb_BackwardSubstitution_n[0];
    rtb_LUFactorization_o1_l[1] = rtb_BackwardSubstitution_n[1];
    rtb_LUFactorization_o1_l[2] = rtb_BackwardSubstitution_n[2];
    rtb_LUFactorization_o1_l[3] = rtb_BackwardSubstitution_n[3];
    LUf_boolfloatint32_t(&rtb_LUFactorization_o1_l[0],
                         &rtb_LUFactorization_o2_n[0], 2,
                         &rtb_LUFactorization_o3);

    // Switch: '<S154>/Switch' incorporates:
    //   Constant: '<S154>/Constant'
    //   Math: '<S154>/Math Function'
    //   Product: '<S154>/Product'
    //
    //  About '<S154>/Math Function':
    //   Operator: reciprocal

    if (rtb_LUFactorization_o3) {
      rtb_Bias3 = ANAS0_P.Constant_Value_j3;
    } else {
      // S-Function (sdspm1norm2): '<S154>/Matrix  1-Norm2'

      // DSP System Toolbox Matrix 1-Norm (sdspm1norm2) - '<S154>/Matrix  1-Norm2' 
      {
        const float *uPtr{ &rtb_BackwardSubstitution_n[0] };

        float m1norm{ 0.0 };

        int jdx;
        for (jdx=2; jdx-- > 0; ) {
          float sumabsAj{ 0.0 };

          int idxFlt;
          for (idxFlt=2; idxFlt-- > 0; ) {
            float temp{ *uPtr++ };

            sumabsAj += fabsf(temp);
          }

          m1norm = MAX(m1norm, sumabsAj);
        }

        rtb_Matrix1Norm2_c = m1norm;
      }

      // S-Function (sdspperm2): '<S156>/Permute Matrix' incorporates:
      //   IdentityMatrix: '<S156>/Identity Matrix'
      //   S-Function (sdspfbsub2): '<S156>/Backward Substitution'

      k = static_cast<int32_t>(std::floor(rtb_LUFactorization_o2_n[0])) - 1;
      i_0 = k;
      if (k < 0) {
        i_0 = 0;
      } else if (k >= 2) {
        i_0 = 1;
      }

      rtb_BackwardSubstitution_j[0] = ANAS0_DW.IdentityMatrix_n[i_0];
      i = static_cast<int32_t>(std::floor(rtb_LUFactorization_o2_n[1])) - 1;
      i_0 = i;
      if (i < 0) {
        i_0 = 0;
      } else if (i >= 2) {
        i_0 = 1;
      }

      rtb_BackwardSubstitution_j[1] = ANAS0_DW.IdentityMatrix_n[i_0];
      i_0 = k;
      if (k < 0) {
        i_0 = 0;
      } else if (k >= 2) {
        i_0 = 1;
      }

      rtb_BackwardSubstitution_j[2] = ANAS0_DW.IdentityMatrix_n[i_0 + 2];
      i_0 = i;
      if (i < 0) {
        i_0 = 0;
      } else if (i >= 2) {
        i_0 = 1;
      }

      rtb_BackwardSubstitution_j[3] = ANAS0_DW.IdentityMatrix_n[i_0 + 2];

      // End of S-Function (sdspperm2): '<S156>/Permute Matrix'

      // S-Function (sdspfbsub2): '<S156>/Forward Substitution' incorporates:
      //   S-Function (sdspfbsub2): '<S156>/Backward Substitution'
      //   S-Function (sdsplu2): '<S156>/LU Factorization'

      rtb_BackwardSubstitution_j[1] -= rtb_BackwardSubstitution_j[0] *
        rtb_LUFactorization_o1_l[1];
      rtb_BackwardSubstitution_j[3] -= rtb_LUFactorization_o1_l[1] *
        rtb_BackwardSubstitution_j[2];

      // S-Function (sdspfbsub2): '<S156>/Backward Substitution' incorporates:
      //   S-Function (sdsplu2): '<S156>/LU Factorization'

      rtb_BackwardSubstitution_j[1] /= rtb_LUFactorization_o1_l[3];
      rtb_BackwardSubstitution_j[0] = (rtb_BackwardSubstitution_j[0] -
        rtb_BackwardSubstitution_j[1] * rtb_LUFactorization_o1_l[2]) /
        rtb_LUFactorization_o1_l[0];
      rtb_BackwardSubstitution_j[3] /= rtb_LUFactorization_o1_l[3];
      rtb_BackwardSubstitution_j[2] = (rtb_BackwardSubstitution_j[2] -
        rtb_LUFactorization_o1_l[2] * rtb_BackwardSubstitution_j[3]) /
        rtb_LUFactorization_o1_l[0];

      // S-Function (sdspm1norm2): '<S154>/Matrix  1-Norm1'

      // DSP System Toolbox Matrix 1-Norm (sdspm1norm2) - '<S154>/Matrix  1-Norm1' 
      {
        const float *uPtr{ &rtb_BackwardSubstitution_j[0] };

        float m1norm{ 0.0 };

        int jdx;
        for (jdx=2; jdx-- > 0; ) {
          float sumabsAj{ 0.0 };

          int idxFlt;
          for (idxFlt=2; idxFlt-- > 0; ) {
            float temp{ *uPtr++ };

            sumabsAj += fabsf(temp);
          }

          m1norm = MAX(m1norm, sumabsAj);
        }

        rtb_Matrix1Norm1_h = m1norm;
      }

      rtb_Bias3 = 1.0F / (rtb_Matrix1Norm1_h * rtb_Matrix1Norm2_c);
    }

    // End of Switch: '<S154>/Switch'

    // If: '<S145>/If' incorporates:
    //   Constant: '<S145>/NASCondLim'
    //   RelationalOperator: '<S145>/GreaterThanOrEqual'

    if (rtb_Bias3 >= ANAS0_P.NASCondLim_Value_i) {
      // Outputs for IfAction SubSystem: '<S145>/Correction' incorporates:
      //   ActionPort: '<S152>/Action Port'

      rtb_LUFactorization_o1_l[0] = rtb_BackwardSubstitution_n[0];
      rtb_LUFactorization_o1_l[1] = rtb_BackwardSubstitution_n[1];
      rtb_LUFactorization_o1_l[2] = rtb_BackwardSubstitution_n[2];
      rtb_LUFactorization_o1_l[3] = rtb_BackwardSubstitution_n[3];
      ANAS0_Correction_i(rtb_LUFactorization_o1_l, rtb_S_ce_tmp,
                         rtb_MatrixConcatenate);

      // End of Outputs for SubSystem: '<S145>/Correction'
    } else {
      // Outputs for IfAction SubSystem: '<S145>/No correction' incorporates:
      //   ActionPort: '<S153>/Action Port'

      rtb_LUFactorization_o1_l[0] = rtb_BackwardSubstitution_n[0];
      rtb_LUFactorization_o1_l[1] = rtb_BackwardSubstitution_n[1];
      rtb_LUFactorization_o1_l[2] = rtb_BackwardSubstitution_n[2];
      rtb_LUFactorization_o1_l[3] = rtb_BackwardSubstitution_n[3];
      ANAS0_Nocorrection_o(rtb_S_ce_tmp, rtb_MatrixConcatenate,
                           &ANAS0_P.Nocorrection_g4);

      // End of Outputs for SubSystem: '<S145>/No correction'
    }

    // End of If: '<S145>/If'

    // Sum: '<S147>/Subtract' incorporates:
    //   Constant: '<S147>/Constant'
    //   Math: '<S143>/Transpose1'
    //   Product: '<S147>/Matrix Multiply'
    //   Product: '<S176>/Matrix Multiply2'

    for (i_0 = 0; i_0 < 6; i_0++) {
      // Product: '<S147>/Matrix Multiply' incorporates:
      //   Merge: '<S145>/Merge'

      rtb_Matrix1Norm2_m = rtb_MatrixConcatenate[i_0 + 6];
      rtb_Matrix1Norm1_a = rtb_MatrixConcatenate[i_0];
      k = 0;
      i = 0;
      for (rtb_LinearQ_tmp = 0; rtb_LinearQ_tmp < 6; rtb_LinearQ_tmp++) {
        rtb_S_tmp_0 = k + i_0;
        rtb_Transpose_e[rtb_S_tmp_0] = ANAS0_P.Constant_Value_i[rtb_S_tmp_0] -
          (rtb_Transpose1_j[i + 1] * rtb_Matrix1Norm2_m + rtb_Transpose1_j[i] *
           rtb_Matrix1Norm1_a);
        k += 6;
        i += 2;
      }
    }

    // End of Sum: '<S147>/Subtract'

    // Product: '<S143>/Matrix Multiply' incorporates:
    //   Math: '<S143>/Transpose'
    //   Merge: '<S97>/Merge1'
    //   Product: '<S176>/Matrix Multiply2'

    for (i_0 = 0; i_0 < 6; i_0++) {
      k = 0;
      for (i = 0; i < 6; i++) {
        rtb_Matrix1Norm1 = 0.0F;
        rtb_LinearQ_tmp = 0;
        for (rtb_S_tmp_0 = 0; rtb_S_tmp_0 < 6; rtb_S_tmp_0++) {
          rtb_Matrix1Norm1 += rtb_LinearQ[rtb_LinearQ_tmp + i_0] *
            rtb_Transpose_e[rtb_LinearQ_tmp + i];
          rtb_LinearQ_tmp += 6;
        }

        rtb_LinearQ_0[k + i_0] = rtb_Matrix1Norm1;
        k += 6;
      }
    }

    // Product: '<S143>/Matrix Multiply2' incorporates:
    //   Math: '<S100>/Square'
    //   Math: '<S143>/Transpose1'
    //   Merge: '<S145>/Merge'

    for (i_0 = 0; i_0 < 2; i_0++) {
      rtb_Matrix1Norm2_dg = rtb_LUFactorization_o1_iq[i_0 + 2];
      rtb_Matrix1Norm2_c = rtb_LUFactorization_o1_iq[i_0];
      k = 0;
      for (i = 0; i < 6; i++) {
        rtb_Transpose1_j[k + i_0] = rtb_MatrixConcatenate[i + 6] *
          rtb_Matrix1Norm2_dg + rtb_Matrix1Norm2_c * rtb_MatrixConcatenate[i];
        k += 2;
      }
    }

    // Product: '<S143>/Matrix Multiply' incorporates:
    //   Merge: '<S145>/Merge'
    //   Product: '<S143>/Matrix Multiply2'
    //   Product: '<S176>/Matrix Multiply2'

    i_0 = 0;
    k = 0;
    for (i = 0; i < 6; i++) {
      for (rtb_LinearQ_tmp = 0; rtb_LinearQ_tmp < 6; rtb_LinearQ_tmp++) {
        rtb_Matrix1Norm1 = 0.0F;
        rtb_S_tmp_0 = 0;
        for (i_1 = 0; i_1 < 6; i_1++) {
          rtb_Matrix1Norm1 += rtb_Transpose_e[rtb_S_tmp_0 + rtb_LinearQ_tmp] *
            rtb_LinearQ_0[i_1 + i_0];
          rtb_S_tmp_0 += 6;
        }

        rtb_S_tmp_0 = rtb_LinearQ_tmp + i_0;
        rtb_Transpose_aq[rtb_S_tmp_0] = rtb_Matrix1Norm1;
        rtb_Merge_p[rtb_S_tmp_0] = rtb_Transpose1_j[k + 1] *
          rtb_MatrixConcatenate[rtb_LinearQ_tmp + 6] + rtb_Transpose1_j[k] *
          rtb_MatrixConcatenate[rtb_LinearQ_tmp];
      }

      i_0 += 6;
      k += 2;
    }

    for (i_0 = 0; i_0 < 36; i_0++) {
      // Merge: '<S96>/Merge1' incorporates:
      //   Sum: '<S143>/Sum1'

      ANAS0_DW.Merge1[i_0] = rtb_Transpose_aq[i_0] + rtb_Merge_p[i_0];
    }

    // Product: '<S159>/Product3' incorporates:
    //   Constant: '<S159>/LocalGravity'
    //   Constant: '<S159>/RAir'
    //   Constant: '<S159>/RAir1'
    //   Gain: '<S157>/Gain'
    //   Inport: '<Root>/ANAS Reference'
    //   Math: '<S159>/Power'
    //   Product: '<S159>/Divide'
    //   Product: '<S159>/Divide1'
    //   Product: '<S159>/Product'
    //   Product: '<S159>/Product2'
    //   Sum: '<S159>/Subtract'

    rtb_Bias3 = std::pow((ANAS0_U.ANASReference_f.GroundTemperature -
                          ANAS0_P.Gain_Gain_f * rtb_VectorConcatenate_o[2] *
                          ANAS0_P.RAir1_Value_b) /
                         ANAS0_U.ANASReference_f.GroundTemperature,
                         ANAS0_P.LocalGravity_Value_d / (ANAS0_P.RAir1_Value_b *
      ANAS0_P.RAir_Value_j)) * ANAS0_U.ANASReference_f.GroundPressure;

    // Sum: '<S146>/Add2' incorporates:
    //   Bias: '<S157>/Bias'
    //   Bias: '<S160>/Bias2'
    //   Constant: '<S157>/Baroa'
    //   Constant: '<S157>/Constant'
    //   Constant: '<S157>/IsaGamma'
    //   Constant: '<S160>/AirR'
    //   Constant: '<S160>/IsaGamma'
    //   Constant: '<S160>/IsaGamma-1'
    //   Gain: '<S160>/Gain2'
    //   Inport: '<Root>/ANAS In'
    //   Inport: '<Root>/ANAS Reference'
    //   Math: '<S157>/Power'
    //   Product: '<S157>/Divide'
    //   Product: '<S157>/Divide1'
    //   Product: '<S157>/Divide2'
    //   Product: '<S160>/Divide6'
    //   Product: '<S160>/Divide7'
    //   SignalConversion generated from: '<S146>/Vector Concatenate1'
    //   Sum: '<S157>/Add'
    //   Sum: '<S157>/Add3'

    rtb_Matrix1Norm2_dg = ANAS0_U.ANASIn_c.PitotMeasure[0] - (std::pow(1.0F /
      ANAS0_P.AirR_Value_m / ANAS0_P.IsaGamma_Value_m / (ANAS0_P.Baroa_Value_p *
      rtb_VectorConcatenate_o[2] + ANAS0_U.ANASReference_f.GroundTemperature) *
      rtb_Matrix1Norm2 * ANAS0_P.IsaGamma1_Value_e * ANAS0_P.Gain2_Gain_e +
      ANAS0_P.Bias2_Bias_lw, 1.0F / (ANAS0_P.IsaGamma_Value_j +
      ANAS0_P.Bias_Bias_h) * ANAS0_P.IsaGamma_Value_j) -
      ANAS0_P.Constant_Value_nu) * rtb_Bias3;
    rtb_Matrix1Norm2_c = ANAS0_U.ANASIn_c.PitotMeasure[1] - rtb_Bias3;
    for (i = 0; i < 6; i++) {
      // Merge: '<S96>/Merge' incorporates:
      //   Merge: '<S145>/Merge'
      //   Merge: '<S97>/Merge'
      //   Product: '<S100>/Matrix Multiply'
      //   Sum: '<S100>/Sum'

      ANAS0_DW.Merge[i] = (rtb_MatrixConcatenate[i + 6] * rtb_Matrix1Norm2_c +
                           rtb_MatrixConcatenate[i] * rtb_Matrix1Norm2_dg) +
        rtb_VectorConcatenate_o[i];
    }
  }

  // End of Outputs for SubSystem: '<S96>/Active Correction Step Pitot Static & Differential' 

  // Logic: '<S103>/AND13' incorporates:
  //   Constant: '<S103>/VaroFlag'
  //   Logic: '<S103>/NOT5'

  rtb_LUFactorization_o3 = (rtb_LUFactorization_o3_j && (!ANAS0_P.VaroFlag_Value));

  // Outputs for Enabled SubSystem: '<S96>/Active Correction Step Static Main' incorporates:
  //   EnablePort: '<S101>/Enable'

  if (rtb_LUFactorization_o3) {
    // Reshape: '<S164>/Reshape' incorporates:
    //   Constant: '<S164>/Constant3'

    rtb_MatrixMultiply_f[0] = ANAS0_P.Constant3_Value_g[0];
    rtb_MatrixMultiply_f[1] = ANAS0_P.Constant3_Value_g[1];

    // Product: '<S164>/Divide' incorporates:
    //   Constant: '<S164>/Baroa'
    //   Product: '<S164>/Divide3'

    rtb_Bias3 = rtb_VectorConcatenate_o[2] * ANAS0_P.Baroa_Value_h;

    // Sum: '<S164>/Add1' incorporates:
    //   Inport: '<Root>/ANAS Reference'
    //   Product: '<S164>/Divide'

    rtb_Add = rtb_Bias3 + ANAS0_U.ANASReference_f.GroundTemperature;

    // Product: '<S164>/Product' incorporates:
    //   Constant: '<S164>/Baroa'
    //   Constant: '<S164>/Barog//R'
    //   Constant: '<S164>/Constant1'
    //   Constant: '<S164>/Constant2'
    //   Inport: '<Root>/ANAS Reference'
    //   Math: '<S164>/Power'
    //   Product: '<S164>/Divide1'
    //   Product: '<S164>/Divide4'
    //   Product: '<S164>/Divide5'
    //   Sum: '<S164>/Add'

    rtb_MatrixMultiply_f[2] = std::pow(ANAS0_P.Constant2_Value_l1 - 1.0F /
      rtb_Add * rtb_Bias3, 1.0F / ANAS0_P.Baroa_Value_h *
      ANAS0_P.Constant1_Value_l * ANAS0_P.BarogR_Value_k) * (1.0F / rtb_Add) *
      ANAS0_P.BarogR_Value_k * ANAS0_U.ANASReference_f.GroundPressure;

    // Reshape: '<S164>/Reshape1' incorporates:
    //   Constant: '<S164>/Constant9'

    rtb_MatrixMultiply_f[3] = ANAS0_P.Constant9_Value_b[0];
    rtb_MatrixMultiply_f[4] = ANAS0_P.Constant9_Value_b[1];
    rtb_MatrixMultiply_f[5] = ANAS0_P.Constant9_Value_b[2];

    // Math: '<S101>/Square' incorporates:
    //   Constant: '<S101>/BaroSigma'

    rtb_Divide1_p = ANAS0_P.BaroSigma_Value * ANAS0_P.BaroSigma_Value;

    // Product: '<S165>/Matrix Multiply' incorporates:
    //   Product: '<S102>/Matrix Multiply'

    rtb_Matrix1Norm2_dg = 0.0F;
    for (i_0 = 0; i_0 < 6; i_0++) {
      rtb_Matrix1Norm2_c = 0.0F;

      // Product: '<S165>/Matrix Multiply1' incorporates:
      //   Math: '<S165>/Transpose'
      //   Merge: '<S97>/Merge1'
      //   Product: '<S102>/Matrix Multiply'

      k = 0;
      for (i = 0; i < 6; i++) {
        rtb_Matrix1Norm2_c += rtb_LinearQ[k + i_0] * rtb_MatrixMultiply_f[i];
        k += 6;
      }

      // End of Product: '<S165>/Matrix Multiply1'
      rtb_Transpose_k[i_0] = rtb_Matrix1Norm2_c;
      rtb_Matrix1Norm2_dg += rtb_MatrixMultiply_f[i_0] * rtb_Matrix1Norm2_c;
    }

    // Sum: '<S165>/Add' incorporates:
    //   Product: '<S165>/Matrix Multiply'

    rtb_Matrix1Norm2_dg += rtb_Divide1_p;

    // S-Function (sdsplu2): '<S172>/LU Factorization'
    rtb_Matrix1Norm2_c = rtb_Matrix1Norm2_dg;
    LUf_boolfloatint32_t(&rtb_Matrix1Norm2_c, &rtb_LUFactorization_o2_j5, 1,
                         &rtb_LUFactorization_o3_j);

    // Switch: '<S170>/Switch' incorporates:
    //   Constant: '<S170>/Constant'
    //   Math: '<S170>/Math Function'
    //   Product: '<S170>/Product'
    //
    //  About '<S170>/Math Function':
    //   Operator: reciprocal

    if (rtb_LUFactorization_o3_j) {
      rtb_Add = ANAS0_P.Constant_Value_is;
    } else {
      // S-Function (sdspm1norm2): '<S170>/Matrix  1-Norm2'

      // DSP System Toolbox Matrix 1-Norm (sdspm1norm2) - '<S170>/Matrix  1-Norm2' 
      {
        const float *uPtr{ &rtb_Matrix1Norm2_dg };

        float sumabsAj{ 0.0 };

        float temp{ *uPtr++ };

        sumabsAj += fabsf(temp);
        rtb_Matrix1Norm2_k = sumabsAj;
      }

      // S-Function (sdspperm2): '<S172>/Permute Matrix'
      rtb_LUFactorization_o2_j5 = ANAS0_DW.IdentityMatrix_e;

      // S-Function (sdspfbsub2): '<S172>/Backward Substitution'
      rtb_LUFactorization_o2_j5 /= rtb_Matrix1Norm2_c;

      // S-Function (sdspm1norm2): '<S170>/Matrix  1-Norm1'

      // DSP System Toolbox Matrix 1-Norm (sdspm1norm2) - '<S170>/Matrix  1-Norm1' 
      {
        const float *uPtr{ &rtb_LUFactorization_o2_j5 };

        float sumabsAj{ 0.0 };

        float temp{ *uPtr++ };

        sumabsAj += fabsf(temp);
        rtb_LUFactorization_o2_j5 = sumabsAj;
      }

      rtb_Add = 1.0F / (rtb_LUFactorization_o2_j5 * rtb_Matrix1Norm2_k);
    }

    // End of Switch: '<S170>/Switch'

    // If: '<S165>/If' incorporates:
    //   Constant: '<S165>/NASCondLim'
    //   RelationalOperator: '<S165>/GreaterThanOrEqual'

    if (rtb_Add >= ANAS0_P.NASCondLim_Value_i1) {
      // Outputs for IfAction SubSystem: '<S165>/Correction' incorporates:
      //   ActionPort: '<S168>/Action Port'

      ANAS0_Correction_m(rtb_Matrix1Norm2_dg, rtb_Transpose_k, rtb_Switch2,
                         &rtb_Matrix1Norm2_c);

      // End of Outputs for SubSystem: '<S165>/Correction'
    } else {
      // Outputs for IfAction SubSystem: '<S165>/No correction' incorporates:
      //   ActionPort: '<S169>/Action Port'

      ANAS0_Nocorrection_d(rtb_Matrix1Norm2_dg, rtb_Transpose_k, rtb_Switch2,
                           &rtb_Matrix1Norm2_c, &ANAS0_P.Nocorrection_d);

      // End of Outputs for SubSystem: '<S165>/No correction'
    }

    // End of If: '<S165>/If'

    // Sum: '<S167>/Subtract' incorporates:
    //   Constant: '<S167>/Constant'
    //   Merge: '<S165>/Merge'
    //   Product: '<S102>/Matrix Multiply'
    //   Product: '<S167>/Matrix Multiply'
    //   Product: '<S176>/Matrix Multiply2'

    i_0 = 0;
    for (k = 0; k < 6; k++) {
      for (i = 0; i < 6; i++) {
        rtb_S_tmp_0 = i + i_0;
        rtb_Transpose_e[rtb_S_tmp_0] = ANAS0_P.Constant_Value_px[rtb_S_tmp_0] -
          rtb_Switch2[i] * rtb_MatrixMultiply_f[k];
      }

      i_0 += 6;
    }

    // End of Sum: '<S167>/Subtract'

    // Product: '<S163>/Matrix Multiply' incorporates:
    //   Math: '<S163>/Transpose'
    //   Math: '<S163>/Transpose1'
    //   Merge: '<S165>/Merge'
    //   Merge: '<S97>/Merge1'
    //   Product: '<S163>/Matrix Multiply2'
    //   Product: '<S176>/Matrix Multiply2'

    for (i_0 = 0; i_0 < 6; i_0++) {
      k = 0;
      for (i = 0; i < 6; i++) {
        rtb_Matrix1Norm1 = 0.0F;
        rtb_LinearQ_tmp = 0;
        for (rtb_S_tmp_0 = 0; rtb_S_tmp_0 < 6; rtb_S_tmp_0++) {
          rtb_Matrix1Norm1 += rtb_LinearQ[rtb_LinearQ_tmp + i_0] *
            rtb_Transpose_e[rtb_LinearQ_tmp + i];
          rtb_LinearQ_tmp += 6;
        }

        rtb_LinearQ_0[k + i_0] = rtb_Matrix1Norm1;
        k += 6;
      }
    }

    i_0 = 0;
    for (k = 0; k < 6; k++) {
      i = 0;
      for (rtb_LinearQ_tmp = 0; rtb_LinearQ_tmp < 6; rtb_LinearQ_tmp++) {
        rtb_Matrix1Norm1 = 0.0F;
        rtb_S_tmp_0 = 0;
        for (i_1 = 0; i_1 < 6; i_1++) {
          rtb_Matrix1Norm1 += rtb_Transpose_e[rtb_S_tmp_0 + k] *
            rtb_LinearQ_0[i_1 + i];
          rtb_S_tmp_0 += 6;
        }

        rtb_Transpose_aq[i + k] = rtb_Matrix1Norm1;
        rtb_Merge_p[rtb_LinearQ_tmp + i_0] = rtb_Divide1_p * rtb_Switch2[k] *
          rtb_Switch2[rtb_LinearQ_tmp];
        i += 6;
      }

      i_0 += 6;
    }

    // End of Product: '<S163>/Matrix Multiply'
    for (i_0 = 0; i_0 < 36; i_0++) {
      // Merge: '<S96>/Merge1' incorporates:
      //   Sum: '<S163>/Sum1'

      ANAS0_DW.Merge1[i_0] = rtb_Transpose_aq[i_0] + rtb_Merge_p[i_0];
    }

    // Sum: '<S166>/Add' incorporates:
    //   Constant: '<S173>/LocalGravity'
    //   Constant: '<S173>/RAir'
    //   Constant: '<S173>/RAir1'
    //   Gain: '<S166>/Gain'
    //   Inport: '<Root>/ANAS In'
    //   Inport: '<Root>/ANAS Reference'
    //   Math: '<S173>/Power'
    //   Product: '<S173>/Divide'
    //   Product: '<S173>/Divide1'
    //   Product: '<S173>/Product'
    //   Product: '<S173>/Product2'
    //   Product: '<S173>/Product3'
    //   Sum: '<S173>/Subtract'

    rtb_Add = ANAS0_U.ANASIn_c.BaroMeasure - std::pow
      ((ANAS0_U.ANASReference_f.GroundTemperature - ANAS0_P.Gain_Gain_e5 *
        rtb_VectorConcatenate_o[2] * ANAS0_P.RAir1_Value_ba) /
       ANAS0_U.ANASReference_f.GroundTemperature, ANAS0_P.LocalGravity_Value_dq /
       (ANAS0_P.RAir1_Value_ba * ANAS0_P.RAir_Value_oc)) *
      ANAS0_U.ANASReference_f.GroundPressure;
    for (i = 0; i < 6; i++) {
      // Merge: '<S96>/Merge' incorporates:
      //   Merge: '<S165>/Merge'
      //   Merge: '<S97>/Merge'
      //   Product: '<S101>/Matrix Multiply'
      //   Sum: '<S101>/Sum'

      ANAS0_DW.Merge[i] = rtb_Switch2[i] * rtb_Add + rtb_VectorConcatenate_o[i];
    }
  }

  // End of Outputs for SubSystem: '<S96>/Active Correction Step Static Main'

  // Logic: '<S103>/AND10' incorporates:
  //   Constant: '<S103>/StaticPitotFlag1'

  rtb_do_barometer_correcion = (rtb_do_barometer_correcion &&
    ANAS0_P.StaticPitotFlag1_Value);

  // Outputs for Enabled SubSystem: '<S96>/Active Correction Step Static Pitot' incorporates:
  //   EnablePort: '<S102>/Enable'

  if (rtb_do_barometer_correcion) {
    // Reshape: '<S177>/Reshape' incorporates:
    //   Constant: '<S177>/Constant3'

    rtb_MatrixMultiply_f[0] = ANAS0_P.Constant3_Value_gt[0];
    rtb_MatrixMultiply_f[1] = ANAS0_P.Constant3_Value_gt[1];

    // Product: '<S177>/Divide' incorporates:
    //   Constant: '<S177>/Baroa'
    //   Product: '<S177>/Divide3'

    rtb_Bias3 = rtb_VectorConcatenate_o[2] * ANAS0_P.Baroa_Value_n;

    // Sum: '<S177>/Add1' incorporates:
    //   Inport: '<Root>/ANAS Reference'
    //   Product: '<S177>/Divide'

    rtb_Add = rtb_Bias3 + ANAS0_U.ANASReference_f.GroundTemperature;

    // Product: '<S177>/Product' incorporates:
    //   Constant: '<S177>/Baroa'
    //   Constant: '<S177>/Barog//R'
    //   Constant: '<S177>/Constant1'
    //   Constant: '<S177>/Constant2'
    //   Inport: '<Root>/ANAS Reference'
    //   Math: '<S177>/Power'
    //   Product: '<S177>/Divide1'
    //   Product: '<S177>/Divide4'
    //   Product: '<S177>/Divide5'
    //   Sum: '<S177>/Add'

    rtb_MatrixMultiply_f[2] = std::pow(ANAS0_P.Constant2_Value_o - 1.0F /
      rtb_Add * rtb_Bias3, 1.0F / ANAS0_P.Baroa_Value_n *
      ANAS0_P.Constant1_Value_c * ANAS0_P.BarogR_Value_h) * (1.0F / rtb_Add) *
      ANAS0_P.BarogR_Value_h * ANAS0_U.ANASReference_f.GroundPressure;

    // Reshape: '<S177>/Reshape1' incorporates:
    //   Constant: '<S177>/Constant9'

    rtb_MatrixMultiply_f[3] = ANAS0_P.Constant9_Value_g[0];
    rtb_MatrixMultiply_f[4] = ANAS0_P.Constant9_Value_g[1];
    rtb_MatrixMultiply_f[5] = ANAS0_P.Constant9_Value_g[2];

    // Math: '<S102>/Square' incorporates:
    //   Constant: '<S102>/PitotSigma'

    rtb_Divide1_p = ANAS0_P.PitotSigma_Value * ANAS0_P.PitotSigma_Value;

    // Product: '<S178>/Matrix Multiply' incorporates:
    //   Product: '<S102>/Matrix Multiply'

    rtb_Matrix1Norm2_dg = 0.0F;
    for (i_0 = 0; i_0 < 6; i_0++) {
      rtb_Matrix1Norm2_c = 0.0F;

      // Product: '<S178>/Matrix Multiply1' incorporates:
      //   Math: '<S178>/Transpose'
      //   Merge: '<S97>/Merge1'
      //   Product: '<S102>/Matrix Multiply'

      k = 0;
      for (i = 0; i < 6; i++) {
        rtb_Matrix1Norm2_c += rtb_LinearQ[k + i_0] * rtb_MatrixMultiply_f[i];
        k += 6;
      }

      // End of Product: '<S178>/Matrix Multiply1'
      rtb_Transpose_k[i_0] = rtb_Matrix1Norm2_c;
      rtb_Matrix1Norm2_dg += rtb_MatrixMultiply_f[i_0] * rtb_Matrix1Norm2_c;
    }

    // Sum: '<S178>/Add' incorporates:
    //   Product: '<S178>/Matrix Multiply'

    rtb_Matrix1Norm2_c = rtb_Matrix1Norm2_dg + rtb_Divide1_p;

    // S-Function (sdsplu2): '<S185>/LU Factorization'
    rtb_Matrix1Norm2_k = rtb_Matrix1Norm2_c;
    LUf_boolfloatint32_t(&rtb_Matrix1Norm2_k, &rtb_LUFactorization_o2_j, 1,
                         &rtb_LUFactorization_o3_j);

    // Switch: '<S183>/Switch' incorporates:
    //   Constant: '<S183>/Constant'
    //   Math: '<S183>/Math Function'
    //   Product: '<S183>/Product'
    //
    //  About '<S183>/Math Function':
    //   Operator: reciprocal

    if (rtb_LUFactorization_o3_j) {
      rtb_Add = ANAS0_P.Constant_Value_m;
    } else {
      // S-Function (sdspm1norm2): '<S183>/Matrix  1-Norm2'

      // DSP System Toolbox Matrix 1-Norm (sdspm1norm2) - '<S183>/Matrix  1-Norm2' 
      {
        const float *uPtr{ &rtb_Matrix1Norm2_c };

        float sumabsAj{ 0.0 };

        float temp{ *uPtr++ };

        sumabsAj += fabsf(temp);
        rtb_Matrix1Norm2_l = sumabsAj;
      }

      // S-Function (sdspperm2): '<S185>/Permute Matrix'
      rtb_LUFactorization_o2_j = ANAS0_DW.IdentityMatrix_i;

      // S-Function (sdspfbsub2): '<S185>/Backward Substitution'
      rtb_LUFactorization_o2_j /= rtb_Matrix1Norm2_k;

      // S-Function (sdspm1norm2): '<S183>/Matrix  1-Norm1'

      // DSP System Toolbox Matrix 1-Norm (sdspm1norm2) - '<S183>/Matrix  1-Norm1' 
      {
        const float *uPtr{ &rtb_LUFactorization_o2_j };

        float sumabsAj{ 0.0 };

        float temp{ *uPtr++ };

        sumabsAj += fabsf(temp);
        rtb_LUFactorization_o2_j = sumabsAj;
      }

      rtb_Add = 1.0F / (rtb_LUFactorization_o2_j * rtb_Matrix1Norm2_l);
    }

    // End of Switch: '<S183>/Switch'

    // If: '<S178>/If' incorporates:
    //   Constant: '<S178>/NASCondLim'
    //   RelationalOperator: '<S178>/GreaterThanOrEqual'

    if (rtb_Add >= ANAS0_P.NASCondLim_Value_ii) {
      // Outputs for IfAction SubSystem: '<S178>/Correction' incorporates:
      //   ActionPort: '<S181>/Action Port'

      ANAS0_Correction_m(rtb_Matrix1Norm2_c, rtb_Transpose_k, rtb_Switch2,
                         &rtb_Matrix1Norm2_k);

      // End of Outputs for SubSystem: '<S178>/Correction'
    } else {
      // Outputs for IfAction SubSystem: '<S178>/No correction' incorporates:
      //   ActionPort: '<S182>/Action Port'

      ANAS0_Nocorrection_d(rtb_Matrix1Norm2_c, rtb_Transpose_k, rtb_Switch2,
                           &rtb_Matrix1Norm2_k, &ANAS0_P.Nocorrection_h);

      // End of Outputs for SubSystem: '<S178>/No correction'
    }

    // End of If: '<S178>/If'

    // Sum: '<S180>/Subtract' incorporates:
    //   Constant: '<S180>/Constant'
    //   Merge: '<S178>/Merge'
    //   Product: '<S102>/Matrix Multiply'
    //   Product: '<S176>/Matrix Multiply2'
    //   Product: '<S180>/Matrix Multiply'

    i_0 = 0;
    for (k = 0; k < 6; k++) {
      for (i = 0; i < 6; i++) {
        rtb_S_tmp_0 = i + i_0;
        rtb_Transpose_e[rtb_S_tmp_0] = ANAS0_P.Constant_Value_o[rtb_S_tmp_0] -
          rtb_Switch2[i] * rtb_MatrixMultiply_f[k];
      }

      i_0 += 6;
    }

    // End of Sum: '<S180>/Subtract'

    // Product: '<S176>/Matrix Multiply' incorporates:
    //   Math: '<S176>/Transpose'
    //   Math: '<S176>/Transpose1'
    //   Merge: '<S178>/Merge'
    //   Merge: '<S97>/Merge1'
    //   Product: '<S176>/Matrix Multiply2'

    for (i_0 = 0; i_0 < 6; i_0++) {
      k = 0;
      for (i = 0; i < 6; i++) {
        rtb_Matrix1Norm1 = 0.0F;
        rtb_LinearQ_tmp = 0;
        for (rtb_S_tmp_0 = 0; rtb_S_tmp_0 < 6; rtb_S_tmp_0++) {
          rtb_Matrix1Norm1 += rtb_LinearQ[rtb_LinearQ_tmp + i_0] *
            rtb_Transpose_e[rtb_LinearQ_tmp + i];
          rtb_LinearQ_tmp += 6;
        }

        rtb_LinearQ_0[k + i_0] = rtb_Matrix1Norm1;
        k += 6;
      }
    }

    i_0 = 0;
    for (k = 0; k < 6; k++) {
      i = 0;
      for (rtb_LinearQ_tmp = 0; rtb_LinearQ_tmp < 6; rtb_LinearQ_tmp++) {
        rtb_Matrix1Norm1 = 0.0F;
        rtb_S_tmp_0 = 0;
        for (i_1 = 0; i_1 < 6; i_1++) {
          rtb_Matrix1Norm1 += rtb_Transpose_e[rtb_S_tmp_0 + k] *
            rtb_LinearQ_0[i_1 + i];
          rtb_S_tmp_0 += 6;
        }

        rtb_Transpose_aq[i + k] = rtb_Matrix1Norm1;
        rtb_Merge_p[rtb_LinearQ_tmp + i_0] = rtb_Divide1_p * rtb_Switch2[k] *
          rtb_Switch2[rtb_LinearQ_tmp];
        i += 6;
      }

      i_0 += 6;
    }

    // End of Product: '<S176>/Matrix Multiply'
    for (i_0 = 0; i_0 < 36; i_0++) {
      // Merge: '<S96>/Merge1' incorporates:
      //   Sum: '<S176>/Sum1'

      ANAS0_DW.Merge1[i_0] = rtb_Transpose_aq[i_0] + rtb_Merge_p[i_0];
    }

    // Sum: '<S179>/Add' incorporates:
    //   Constant: '<S186>/LocalGravity'
    //   Constant: '<S186>/RAir'
    //   Constant: '<S186>/RAir1'
    //   Gain: '<S179>/Gain'
    //   Inport: '<Root>/ANAS In'
    //   Inport: '<Root>/ANAS Reference'
    //   Math: '<S186>/Power'
    //   Product: '<S186>/Divide'
    //   Product: '<S186>/Divide1'
    //   Product: '<S186>/Product'
    //   Product: '<S186>/Product2'
    //   Product: '<S186>/Product3'
    //   Sum: '<S186>/Subtract'

    rtb_Add = ANAS0_U.ANASIn_c.PitotMeasure[1] - std::pow
      ((ANAS0_U.ANASReference_f.GroundTemperature - ANAS0_P.Gain_Gain_n *
        rtb_VectorConcatenate_o[2] * ANAS0_P.RAir1_Value_g) /
       ANAS0_U.ANASReference_f.GroundTemperature, ANAS0_P.LocalGravity_Value_g /
       (ANAS0_P.RAir1_Value_g * ANAS0_P.RAir_Value_c)) *
      ANAS0_U.ANASReference_f.GroundPressure;
    for (i = 0; i < 6; i++) {
      // Merge: '<S96>/Merge' incorporates:
      //   Merge: '<S178>/Merge'
      //   Merge: '<S97>/Merge'
      //   Product: '<S102>/Matrix Multiply'
      //   Sum: '<S102>/Sum'

      ANAS0_DW.Merge[i] = rtb_Switch2[i] * rtb_Add + rtb_VectorConcatenate_o[i];
    }
  }

  // End of Outputs for SubSystem: '<S96>/Active Correction Step Static Pitot'

  // If: '<S103>/If' incorporates:
  //   Logic: '<S103>/OR'

  rtb_LUFactorization_o3_f_tmp = !rtb_main_s_w_pitot_d;
  rtb_LUFactorization_o3_f_tmp_0 = !rtb_do_barometer_correcion;
  rtb_LUFactorization_o3_f_tmp_1 = !rtb_LUFactorization_o3;
  rtb_LUFactorization_o3_f_tmp_2 = !rtb_variometer;
  rtb_LUFactorization_o3_f_tmp_3 = !rtb_LUFactorization_o3_i;

  // Logic: '<S103>/NOT6' incorporates:
  //   Logic: '<S103>/OR'

  rtb_LUFactorization_o3_j = (rtb_LUFactorization_o3_f_tmp_3 &&
    rtb_LUFactorization_o3_f_tmp && rtb_LUFactorization_o3_f_tmp_0 &&
    rtb_LUFactorization_o3_f_tmp_1 && rtb_LUFactorization_o3_f_tmp_2);

  // Outputs for Enabled SubSystem: '<S96>/No Correction Step' incorporates:
  //   EnablePort: '<S104>/Enable'

  if (rtb_LUFactorization_o3_j) {
    for (i = 0; i < 6; i++) {
      // Merge: '<S96>/Merge' incorporates:
      //   Merge: '<S97>/Merge'
      //   SignalConversion generated from: '<S104>/Linear States'

      ANAS0_DW.Merge[i] = rtb_VectorConcatenate_o[i];
    }

    // Merge: '<S96>/Merge1' incorporates:
    //   Merge: '<S97>/Merge1'
    //   SignalConversion generated from: '<S104>/Linear Covariance'

    std::memcpy(&ANAS0_DW.Merge1[0], &rtb_LinearQ[0], 36U * sizeof(float));
  }

  // End of Outputs for SubSystem: '<S96>/No Correction Step'

  // If: '<S103>/If'
  tmp = !rtb_LUFactorization_o3_j;
  if (rtb_LUFactorization_o3_i && rtb_LUFactorization_o3_f_tmp && tmp &&
      rtb_LUFactorization_o3_f_tmp_0 && rtb_LUFactorization_o3_f_tmp_1 &&
      rtb_LUFactorization_o3_f_tmp_2) {
    // Outputs for IfAction SubSystem: '<S103>/If Action Subsystem4' incorporates:
    //   ActionPort: '<S193>/Action Port'

    // Merge: '<S103>/Merge' incorporates:
    //   Constant: '<S193>/Zero'
    //   SignalConversion generated from: '<S193>/Pitot'

    ANAS0_DW.Merge_k = ANAS0_P.Zero_Value_c;

    // End of Outputs for SubSystem: '<S103>/If Action Subsystem4'
  } else if (rtb_main_s_w_pitot_d && rtb_LUFactorization_o3_f_tmp_3 && tmp &&
             rtb_LUFactorization_o3_f_tmp_0 && rtb_LUFactorization_o3_f_tmp_1 &&
             rtb_LUFactorization_o3_f_tmp_2) {
    // Outputs for IfAction SubSystem: '<S103>/If Action Subsystem1' incorporates:
    //   ActionPort: '<S190>/Action Port'

    // Merge: '<S103>/Merge' incorporates:
    //   Constant: '<S190>/Zero'
    //   SignalConversion generated from: '<S190>/BaroPitot'

    ANAS0_DW.Merge_k = ANAS0_P.Zero_Value_e;

    // End of Outputs for SubSystem: '<S103>/If Action Subsystem1'
  } else if (rtb_LUFactorization_o3_j && rtb_LUFactorization_o3_f_tmp_3 &&
             rtb_LUFactorization_o3_f_tmp && rtb_LUFactorization_o3_f_tmp_0 &&
             rtb_LUFactorization_o3_f_tmp_1 && rtb_LUFactorization_o3_f_tmp_2) {
    // Outputs for IfAction SubSystem: '<S103>/If Action Subsystem2' incorporates:
    //   ActionPort: '<S191>/Action Port'

    // Merge: '<S103>/Merge' incorporates:
    //   Constant: '<S191>/Zero'
    //   SignalConversion generated from: '<S191>/No correction'

    ANAS0_DW.Merge_k = ANAS0_P.Zero_Value_i;

    // End of Outputs for SubSystem: '<S103>/If Action Subsystem2'
  } else if (rtb_do_barometer_correcion && rtb_LUFactorization_o3_f_tmp_3 && tmp
             && rtb_LUFactorization_o3_f_tmp && rtb_LUFactorization_o3_f_tmp_1 &&
             rtb_LUFactorization_o3_f_tmp_2) {
    // Outputs for IfAction SubSystem: '<S103>/If Action Subsystem6' incorporates:
    //   ActionPort: '<S195>/Action Port'

    // Merge: '<S103>/Merge' incorporates:
    //   Constant: '<S195>/Zero'
    //   SignalConversion generated from: '<S195>/Pitot Static'

    ANAS0_DW.Merge_k = ANAS0_P.Zero_Value_o;

    // End of Outputs for SubSystem: '<S103>/If Action Subsystem6'
  } else {
    tmp = (rtb_LUFactorization_o3_f_tmp && rtb_LUFactorization_o3_f_tmp_3 && tmp
           && rtb_LUFactorization_o3_f_tmp_0);
    if (tmp && rtb_LUFactorization_o3 && rtb_LUFactorization_o3_f_tmp_2) {
      // Outputs for IfAction SubSystem: '<S103>/If Action Subsystem3' incorporates:
      //   ActionPort: '<S192>/Action Port'

      // Merge: '<S103>/Merge' incorporates:
      //   Constant: '<S192>/Zero'
      //   SignalConversion generated from: '<S192>/Baro'

      ANAS0_DW.Merge_k = ANAS0_P.Zero_Value_d;

      // End of Outputs for SubSystem: '<S103>/If Action Subsystem3'
    } else if (tmp && rtb_variometer && rtb_LUFactorization_o3_f_tmp_1) {
      // Outputs for IfAction SubSystem: '<S103>/If Action Subsystem5' incorporates:
      //   ActionPort: '<S194>/Action Port'

      // Merge: '<S103>/Merge' incorporates:
      //   Constant: '<S194>/Zero'
      //   SignalConversion generated from: '<S194>/Variometer'

      ANAS0_DW.Merge_k = ANAS0_P.Zero_Value_dv;

      // End of Outputs for SubSystem: '<S103>/If Action Subsystem5'
    }
  }

  // Update for Memory: '<S203>/Memory' incorporates:
  //   Inport: '<Root>/ANAS In'

  ANAS0_DW.Memory_PreviousInput = ANAS0_U.ANASIn_c.GPSTimestamp;

  // Update for Memory: '<S103>/Memory3' incorporates:
  //   Inport: '<Root>/ANAS In'

  ANAS0_DW.Memory3_PreviousInput = ANAS0_U.ANASIn_c.PitotTimestamp;

  // Update for Memory: '<S103>/Memory2' incorporates:
  //   Inport: '<Root>/ANAS In'

  ANAS0_DW.Memory2_PreviousInput = ANAS0_U.ANASIn_c.BaroTimestamp;

  // Update for Memory: '<S103>/Memory1' incorporates:
  //   Inport: '<Root>/ANAS In'

  ANAS0_DW.Memory1_PreviousInput = ANAS0_U.ANASIn_c.BaroTimestamp;

  // End of Outputs for SubSystem: '<S1>/Linear States Corrections'

  // Assignment: '<S6>/Assignment' incorporates:
  //   Assignment: '<S6>/Assignment1'
  //   Constant: '<S6>/Zero'
  //   Merge: '<S96>/Merge1'

  std::memcpy(&rtb_Assignment1[0], &ANAS0_P.Zero_Value[0], 81U * sizeof(float));
  i_0 = 0;
  k = 0;
  for (i = 0; i < 6; i++) {
    for (rtb_LinearQ_tmp = 0; rtb_LinearQ_tmp < 6; rtb_LinearQ_tmp++) {
      rtb_Assignment1[rtb_LinearQ_tmp + i_0] = ANAS0_DW.Merge1[rtb_LinearQ_tmp +
        k];
    }

    i_0 += 9;
    k += 6;
  }

  // End of Assignment: '<S6>/Assignment'

  // Assignment: '<S6>/Assignment1' incorporates:
  //   Merge: '<S8>/Merge1'

  i_0 = 0;
  k = 0;
  for (i = 0; i < 3; i++) {
    rtb_Assignment1[i_0 + 60] = rtb_Transpose_g[k];
    rtb_Assignment1[i_0 + 61] = rtb_Transpose_g[k + 1];
    rtb_Assignment1[i_0 + 62] = rtb_Transpose_g[k + 2];
    i_0 += 9;
    k += 3;
  }

  // End of Assignment: '<S6>/Assignment1'

  // BusCreator generated from: '<S6>/ANAS Logs OBSW_BusCreator' incorporates:
  //   Outport: '<Root>/ANAS Logs OBSW'

  ANAS0_Y.ANASLogsOBSW.Timestamp = 0ULL;
  ANAS0_Y.ANASLogsOBSW.Position[0] = ANAS0_DW.Merge[0];
  ANAS0_Y.ANASLogsOBSW.Velocity[0] = ANAS0_DW.Merge[3];
  ANAS0_Y.ANASLogsOBSW.Position[1] = ANAS0_DW.Merge[1];
  ANAS0_Y.ANASLogsOBSW.Velocity[1] = ANAS0_DW.Merge[4];
  ANAS0_Y.ANASLogsOBSW.Position[2] = ANAS0_DW.Merge[2];
  ANAS0_Y.ANASLogsOBSW.Velocity[2] = ANAS0_DW.Merge[5];
  ANAS0_Y.ANASLogsOBSW.Quaternion[0] = ANAS0_DW.Merge_h[0];
  ANAS0_Y.ANASLogsOBSW.Quaternion[1] = ANAS0_DW.Merge_h[1];
  ANAS0_Y.ANASLogsOBSW.Quaternion[2] = ANAS0_DW.Merge_h[2];
  ANAS0_Y.ANASLogsOBSW.Quaternion[3] = ANAS0_DW.Merge_h[3];

  // S-Function (sdspdiag2): '<S6>/Extract Diagonal' incorporates:
  //   Assignment: '<S6>/Assignment1'
  //   BusCreator generated from: '<S6>/ANAS Logs OBSW_BusCreator'
  //   Outport: '<Root>/ANAS Logs OBSW'

  i = 0;
  for (i_0 = 0; i_0 < 9; i_0++) {
    ANAS0_Y.ANASLogsOBSW.CovarianceMatrixDiagonal[i_0] = rtb_Assignment1[i];
    i += 10;
  }

  // End of S-Function (sdspdiag2): '<S6>/Extract Diagonal'

  // BusCreator generated from: '<S6>/ANAS Logs OBSW_BusCreator' incorporates:
  //   Merge: '<S103>/Merge'
  //   Outport: '<Root>/ANAS Logs OBSW'

  ANAS0_Y.ANASLogsOBSW.BaroPitotActivation = ANAS0_DW.Merge_k;
  ANAS0_Y.ANASLogsOBSW.GPSActivation = rtb_AND;
  ANAS0_Y.ANASLogsOBSW.MagActivation = rtb_AND_i;
  ANAS0_Y.ANASLogsOBSW.AccActivation = rtb_AND_b;

  // BusCreator generated from: '<S6>/ANAS Out_BusCreator' incorporates:
  //   Outport: '<Root>/ANAS Out'

  ANAS0_Y.ANASOut_h.Timestamp = 0ULL;
  ANAS0_Y.ANASOut_h.Position[0] = ANAS0_DW.Merge[0];
  ANAS0_Y.ANASOut_h.Velocity[0] = ANAS0_DW.Merge[3];
  ANAS0_Y.ANASOut_h.Position[1] = ANAS0_DW.Merge[1];
  ANAS0_Y.ANASOut_h.Velocity[1] = ANAS0_DW.Merge[4];
  ANAS0_Y.ANASOut_h.Position[2] = ANAS0_DW.Merge[2];
  ANAS0_Y.ANASOut_h.Velocity[2] = ANAS0_DW.Merge[5];
  ANAS0_Y.ANASOut_h.Quaternion[0] = ANAS0_DW.Merge_h[0];
  ANAS0_Y.ANASOut_h.Quaternion[1] = ANAS0_DW.Merge_h[1];
  ANAS0_Y.ANASOut_h.Quaternion[2] = ANAS0_DW.Merge_h[2];
  ANAS0_Y.ANASOut_h.Quaternion[3] = ANAS0_DW.Merge_h[3];

  // BusCreator generated from: '<S6>/NASDAQ Initial State_BusCreator' incorporates:
  //   Merge: '<S96>/Merge1'
  //   Outport: '<Root>/NASDAQ Initial State'

  std::memcpy(&ANAS0_Y.NASDAQInitialState.LinearCovariance[0], &ANAS0_DW.Merge1
              [0], 36U * sizeof(float));
  ANAS0_Y.NASDAQInitialState.Position[0] = ANAS0_DW.Merge[0];
  ANAS0_Y.NASDAQInitialState.Velocity[0] = ANAS0_DW.Merge[3];
  ANAS0_Y.NASDAQInitialState.Position[1] = ANAS0_DW.Merge[1];
  ANAS0_Y.NASDAQInitialState.Velocity[1] = ANAS0_DW.Merge[4];
  ANAS0_Y.NASDAQInitialState.Position[2] = ANAS0_DW.Merge[2];
  ANAS0_Y.NASDAQInitialState.Velocity[2] = ANAS0_DW.Merge[5];

  // Update for UnitDelay: '<S7>/Unit Delay6'
  ANAS0_DW.UnitDelay6_DSTATE[0] = ANAS0_DW.Merge_h[0];

  // Update for UnitDelay: '<S7>/Unit Delay7' incorporates:
  //   Inport: '<Root>/ANAS Reference'

  ANAS0_DW.UnitDelay7_DSTATE[0] = ANAS0_U.ANASReference_f.InitialQuaternion[0];

  // Update for UnitDelay: '<S7>/Unit Delay6'
  ANAS0_DW.UnitDelay6_DSTATE[1] = ANAS0_DW.Merge_h[1];

  // Update for UnitDelay: '<S7>/Unit Delay7' incorporates:
  //   Inport: '<Root>/ANAS Reference'

  ANAS0_DW.UnitDelay7_DSTATE[1] = ANAS0_U.ANASReference_f.InitialQuaternion[1];

  // Update for UnitDelay: '<S7>/Unit Delay6'
  ANAS0_DW.UnitDelay6_DSTATE[2] = ANAS0_DW.Merge_h[2];

  // Update for UnitDelay: '<S7>/Unit Delay7' incorporates:
  //   Inport: '<Root>/ANAS Reference'

  ANAS0_DW.UnitDelay7_DSTATE[2] = ANAS0_U.ANASReference_f.InitialQuaternion[2];

  // Update for UnitDelay: '<S7>/Unit Delay6'
  ANAS0_DW.UnitDelay6_DSTATE[3] = ANAS0_DW.Merge_h[3];

  // Update for UnitDelay: '<S7>/Unit Delay7' incorporates:
  //   Inport: '<Root>/ANAS Reference'

  ANAS0_DW.UnitDelay7_DSTATE[3] = ANAS0_U.ANASReference_f.InitialQuaternion[3];
  for (i = 0; i < 6; i++) {
    // Update for UnitDelay: '<S7>/Unit Delay4'
    ANAS0_DW.UnitDelay4_DSTATE[i] = ANAS0_DW.Merge[i];

    // Update for UnitDelay: '<S7>/Unit Delay8' incorporates:
    //   UnitDelay: '<S7>/Unit Delay4'

    ANAS0_DW.UnitDelay8_DSTATE[i] = rtb_VectorConcatenate[i];
  }

  // Update for UnitDelay: '<S7>/Unit Delay' incorporates:
  //   Merge: '<S96>/Merge1'

  std::memcpy(&ANAS0_DW.UnitDelay_DSTATE[0], &ANAS0_DW.Merge1[0], 36U * sizeof
              (float));

  // Update for UnitDelay: '<S7>/Unit Delay1' incorporates:
  //   Selector: '<S7>/Selector'
  //   UnitDelay: '<S7>/Unit Delay'

  std::memcpy(&ANAS0_DW.UnitDelay1_DSTATE[0], &rtb_Selector[0], 36U * sizeof
              (float));
  for (i = 0; i < 9; i++) {
    // Update for UnitDelay: '<S7>/Unit Delay2' incorporates:
    //   Selector: '<S7>/Selector2'

    ANAS0_DW.UnitDelay2_DSTATE[i] = rtb_Selector2[i];

    // Update for UnitDelay: '<S7>/Unit Delay3' incorporates:
    //   Merge: '<S8>/Merge1'
    //   UnitDelay: '<S7>/Unit Delay2'

    ANAS0_DW.UnitDelay3_DSTATE[i] = rtb_Transpose_g[i];
  }

  // End of Outputs for SubSystem: '<Root>/ANAS'
  rate_scheduler((&ANAS0_M));
}

// Model initialize function
void ANAS0::initialize()
{
  {
    int32_t i;

    // SystemInitialize for Atomic SubSystem: '<Root>/ANAS'
    // InitializeConditions for UnitDelay: '<S7>/Unit Delay6'
    ANAS0_DW.UnitDelay6_DSTATE[0] = ANAS0_P.UnitDelay6_InitialCondition;

    // InitializeConditions for UnitDelay: '<S7>/Unit Delay7'
    ANAS0_DW.UnitDelay7_DSTATE[0] = ANAS0_P.UnitDelay7_InitialCondition;

    // InitializeConditions for UnitDelay: '<S7>/Unit Delay6'
    ANAS0_DW.UnitDelay6_DSTATE[1] = ANAS0_P.UnitDelay6_InitialCondition;

    // InitializeConditions for UnitDelay: '<S7>/Unit Delay7'
    ANAS0_DW.UnitDelay7_DSTATE[1] = ANAS0_P.UnitDelay7_InitialCondition;

    // InitializeConditions for UnitDelay: '<S7>/Unit Delay6'
    ANAS0_DW.UnitDelay6_DSTATE[2] = ANAS0_P.UnitDelay6_InitialCondition;

    // InitializeConditions for UnitDelay: '<S7>/Unit Delay7'
    ANAS0_DW.UnitDelay7_DSTATE[2] = ANAS0_P.UnitDelay7_InitialCondition;

    // InitializeConditions for UnitDelay: '<S7>/Unit Delay6'
    ANAS0_DW.UnitDelay6_DSTATE[3] = ANAS0_P.UnitDelay6_InitialCondition;

    // InitializeConditions for UnitDelay: '<S7>/Unit Delay7'
    ANAS0_DW.UnitDelay7_DSTATE[3] = ANAS0_P.UnitDelay7_InitialCondition;
    for (i = 0; i < 6; i++) {
      // InitializeConditions for UnitDelay: '<S7>/Unit Delay4'
      ANAS0_DW.UnitDelay4_DSTATE[i] = ANAS0_P.UnitDelay4_InitialCondition;

      // InitializeConditions for UnitDelay: '<S7>/Unit Delay8' incorporates:
      //   UnitDelay: '<S7>/Unit Delay4'

      ANAS0_DW.UnitDelay8_DSTATE[i] = ANAS0_P.UnitDelay8_InitialCondition;
    }

    for (i = 0; i < 36; i++) {
      // InitializeConditions for UnitDelay: '<S7>/Unit Delay'
      ANAS0_DW.UnitDelay_DSTATE[i] = ANAS0_P.UnitDelay_InitialCondition;

      // InitializeConditions for UnitDelay: '<S7>/Unit Delay1' incorporates:
      //   UnitDelay: '<S7>/Unit Delay'

      ANAS0_DW.UnitDelay1_DSTATE[i] = ANAS0_P.UnitDelay1_InitialCondition;
    }

    // SystemInitialize for Atomic SubSystem: '<S1>/Linear States Prediction'
    // InitializeConditions for UnitDelay: '<S219>/UD'
    //
    //  Block description for '<S219>/UD':
    //
    //   Store in Global RAM

    ANAS0_DW.UD_DSTATE = ANAS0_P.Difference_ICPrevInput_j;

    // InitializeConditions for UnitDelay: '<S5>/Unit Delay3'
    ANAS0_DW.UnitDelay3_DSTATE_h = ANAS0_P.UnitDelay3_InitialCondition_l;

    // End of SystemInitialize for SubSystem: '<S1>/Linear States Prediction'

    // SystemInitialize for Atomic SubSystem: '<S1>/Angular States Prediction'
    // InitializeConditions for UnitDelay: '<S89>/UD'
    //
    //  Block description for '<S89>/UD':
    //
    //   Store in Global RAM

    ANAS0_DW.UD_DSTATE_a = ANAS0_P.Difference_ICPrevInput;

    // InitializeConditions for UnitDelay: '<S3>/Unit Delay3'
    ANAS0_DW.UnitDelay3_DSTATE_o = ANAS0_P.UnitDelay3_InitialCondition_c;

    // End of SystemInitialize for SubSystem: '<S1>/Angular States Prediction'

    // SystemInitialize for Atomic SubSystem: '<S1>/Angular States Corrections'
    // InitializeConditions for Memory: '<S11>/Memory'
    ANAS0_DW.Memory_PreviousInput_k = ANAS0_P.Memory_InitialCondition;

    // InitializeConditions for Memory: '<S51>/Memory'
    ANAS0_DW.Memory_PreviousInput_kq = ANAS0_P.Memory_InitialCondition_n;

    // SystemInitialize for Merge: '<S9>/Merge'
    ANAS0_DW.Merge_l[0] = ANAS0_P.Merge_InitialOutput;
    ANAS0_DW.Merge_l[1] = ANAS0_P.Merge_InitialOutput;
    ANAS0_DW.Merge_l[2] = ANAS0_P.Merge_InitialOutput;
    ANAS0_DW.Merge_l[3] = ANAS0_P.Merge_InitialOutput;

    // SystemInitialize for Enabled SubSystem: '<S8>/Active Correction Step Accelerometer' 
    // SystemInitialize for Enabled SubSystem: '<S9>/Active Correction Step Magnetometer' 
    for (i = 0; i < 9; i++) {
      // InitializeConditions for UnitDelay: '<S7>/Unit Delay2'
      ANAS0_DW.UnitDelay2_DSTATE[i] = ANAS0_P.UnitDelay2_InitialCondition;

      // InitializeConditions for UnitDelay: '<S7>/Unit Delay3' incorporates:
      //   UnitDelay: '<S7>/Unit Delay2'

      ANAS0_DW.UnitDelay3_DSTATE[i] = ANAS0_P.UnitDelay3_InitialCondition;

      // Start for IdentityMatrix: '<S62>/Identity Matrix' incorporates:
      //   UnitDelay: '<S7>/Unit Delay2'

      ANAS0_DW.IdentityMatrix_j[i] = ANAS0_P.IdentityMatrix_IDMatrixData_a[i];

      // Start for IdentityMatrix: '<S22>/Identity Matrix' incorporates:
      //   UnitDelay: '<S7>/Unit Delay2'

      ANAS0_DW.IdentityMatrix_h[i] = ANAS0_P.IdentityMatrix_IDMatrixData[i];
    }

    // End of SystemInitialize for SubSystem: '<S9>/Active Correction Step Magnetometer' 
    // End of SystemInitialize for SubSystem: '<S8>/Active Correction Step Accelerometer' 

    // SystemInitialize for Merge: '<S8>/Merge'
    ANAS0_DW.Merge_h[0] = ANAS0_P.Merge_InitialOutput_d;
    ANAS0_DW.Merge_h[1] = ANAS0_P.Merge_InitialOutput_d;
    ANAS0_DW.Merge_h[2] = ANAS0_P.Merge_InitialOutput_d;
    ANAS0_DW.Merge_h[3] = ANAS0_P.Merge_InitialOutput_d;

    // End of SystemInitialize for SubSystem: '<S1>/Angular States Corrections'

    // SystemInitialize for Atomic SubSystem: '<S1>/Linear States Corrections'
    // InitializeConditions for Memory: '<S203>/Memory'
    ANAS0_DW.Memory_PreviousInput = ANAS0_P.Memory_InitialCondition_k;

    // InitializeConditions for Memory: '<S103>/Memory3'
    ANAS0_DW.Memory3_PreviousInput = ANAS0_P.Memory3_InitialCondition;

    // InitializeConditions for Memory: '<S103>/Memory2'
    ANAS0_DW.Memory2_PreviousInput = ANAS0_P.Memory2_InitialCondition;

    // InitializeConditions for Memory: '<S103>/Memory1'
    ANAS0_DW.Memory1_PreviousInput = ANAS0_P.Memory1_InitialCondition;

    // SystemInitialize for Enabled SubSystem: '<S97>/Active Correction Step GPS' 
    // Start for IdentityMatrix: '<S215>/Identity Matrix'
    std::memcpy(&ANAS0_DW.IdentityMatrix[0],
                &ANAS0_P.IdentityMatrix_IDMatrixData_m[0], sizeof(float) << 4U);

    // End of SystemInitialize for SubSystem: '<S97>/Active Correction Step GPS' 

    // SystemInitialize for Enabled SubSystem: '<S96>/Active Correction Step Baro & Variometer ' 
    // InitializeConditions for DiscreteFilter: '<S116>/Discrete Filter'
    ANAS0_DW.DiscreteFilter_states = ANAS0_P.DiscreteFilter_InitialStates;

    // InitializeConditions for UnitDelay: '<S120>/UD'
    //
    //  Block description for '<S120>/UD':
    //
    //   Store in Global RAM

    ANAS0_DW.UD_DSTATE_h = ANAS0_P.Difference_ICPrevInput_m;

    // Start for IdentityMatrix: '<S115>/Identity Matrix'
    ANAS0_DW.IdentityMatrix_eg[0] = ANAS0_P.IdentityMatrix_IDMatrixData_i[0];

    // End of SystemInitialize for SubSystem: '<S96>/Active Correction Step Baro & Variometer ' 

    // SystemInitialize for Enabled SubSystem: '<S96>/Active Correction Step Main Static & Pitot' 
    // Start for IdentityMatrix: '<S136>/Identity Matrix'
    ANAS0_DW.IdentityMatrix_g[0] = ANAS0_P.IdentityMatrix_IDMatrixData_n[0];

    // End of SystemInitialize for SubSystem: '<S96>/Active Correction Step Main Static & Pitot' 

    // SystemInitialize for Enabled SubSystem: '<S96>/Active Correction Step Pitot Static & Differential' 
    // Start for IdentityMatrix: '<S156>/Identity Matrix'
    ANAS0_DW.IdentityMatrix_n[0] = ANAS0_P.IdentityMatrix_IDMatrixData_p[0];

    // End of SystemInitialize for SubSystem: '<S96>/Active Correction Step Pitot Static & Differential' 

    // SystemInitialize for Enabled SubSystem: '<S96>/Active Correction Step Baro & Variometer ' 
    // Start for IdentityMatrix: '<S115>/Identity Matrix'
    ANAS0_DW.IdentityMatrix_eg[1] = ANAS0_P.IdentityMatrix_IDMatrixData_i[1];

    // End of SystemInitialize for SubSystem: '<S96>/Active Correction Step Baro & Variometer ' 

    // SystemInitialize for Enabled SubSystem: '<S96>/Active Correction Step Main Static & Pitot' 
    // Start for IdentityMatrix: '<S136>/Identity Matrix'
    ANAS0_DW.IdentityMatrix_g[1] = ANAS0_P.IdentityMatrix_IDMatrixData_n[1];

    // End of SystemInitialize for SubSystem: '<S96>/Active Correction Step Main Static & Pitot' 

    // SystemInitialize for Enabled SubSystem: '<S96>/Active Correction Step Pitot Static & Differential' 
    // Start for IdentityMatrix: '<S156>/Identity Matrix'
    ANAS0_DW.IdentityMatrix_n[1] = ANAS0_P.IdentityMatrix_IDMatrixData_p[1];

    // End of SystemInitialize for SubSystem: '<S96>/Active Correction Step Pitot Static & Differential' 

    // SystemInitialize for Enabled SubSystem: '<S96>/Active Correction Step Baro & Variometer ' 
    // Start for IdentityMatrix: '<S115>/Identity Matrix'
    ANAS0_DW.IdentityMatrix_eg[2] = ANAS0_P.IdentityMatrix_IDMatrixData_i[2];

    // End of SystemInitialize for SubSystem: '<S96>/Active Correction Step Baro & Variometer ' 

    // SystemInitialize for Enabled SubSystem: '<S96>/Active Correction Step Main Static & Pitot' 
    // Start for IdentityMatrix: '<S136>/Identity Matrix'
    ANAS0_DW.IdentityMatrix_g[2] = ANAS0_P.IdentityMatrix_IDMatrixData_n[2];

    // End of SystemInitialize for SubSystem: '<S96>/Active Correction Step Main Static & Pitot' 

    // SystemInitialize for Enabled SubSystem: '<S96>/Active Correction Step Pitot Static & Differential' 
    // Start for IdentityMatrix: '<S156>/Identity Matrix'
    ANAS0_DW.IdentityMatrix_n[2] = ANAS0_P.IdentityMatrix_IDMatrixData_p[2];

    // End of SystemInitialize for SubSystem: '<S96>/Active Correction Step Pitot Static & Differential' 

    // SystemInitialize for Enabled SubSystem: '<S96>/Active Correction Step Baro & Variometer ' 
    // Start for IdentityMatrix: '<S115>/Identity Matrix'
    ANAS0_DW.IdentityMatrix_eg[3] = ANAS0_P.IdentityMatrix_IDMatrixData_i[3];

    // End of SystemInitialize for SubSystem: '<S96>/Active Correction Step Baro & Variometer ' 

    // SystemInitialize for Enabled SubSystem: '<S96>/Active Correction Step Main Static & Pitot' 
    // Start for IdentityMatrix: '<S136>/Identity Matrix'
    ANAS0_DW.IdentityMatrix_g[3] = ANAS0_P.IdentityMatrix_IDMatrixData_n[3];

    // End of SystemInitialize for SubSystem: '<S96>/Active Correction Step Main Static & Pitot' 

    // SystemInitialize for Enabled SubSystem: '<S96>/Active Correction Step Pitot Static & Differential' 
    // Start for IdentityMatrix: '<S156>/Identity Matrix'
    ANAS0_DW.IdentityMatrix_n[3] = ANAS0_P.IdentityMatrix_IDMatrixData_p[3];

    // End of SystemInitialize for SubSystem: '<S96>/Active Correction Step Pitot Static & Differential' 

    // SystemInitialize for Enabled SubSystem: '<S96>/Active Correction Step Static Main' 
    // Start for IdentityMatrix: '<S172>/Identity Matrix'
    ANAS0_DW.IdentityMatrix_e = ANAS0_P.IdentityMatrix_IDMatrixData_p4;

    // End of SystemInitialize for SubSystem: '<S96>/Active Correction Step Static Main' 

    // SystemInitialize for Enabled SubSystem: '<S96>/Active Correction Step Static Pitot' 
    // Start for IdentityMatrix: '<S185>/Identity Matrix'
    ANAS0_DW.IdentityMatrix_i = ANAS0_P.IdentityMatrix_IDMatrixData_iu;

    // End of SystemInitialize for SubSystem: '<S96>/Active Correction Step Static Pitot' 

    // SystemInitialize for Merge: '<S103>/Merge'
    ANAS0_DW.Merge_k = ANAS0_P.Merge_InitialOutput_k;
    for (i = 0; i < 6; i++) {
      // SystemInitialize for Merge: '<S96>/Merge'
      ANAS0_DW.Merge[i] = ANAS0_P.Merge_InitialOutput_f;
    }

    for (i = 0; i < 36; i++) {
      // SystemInitialize for Merge: '<S96>/Merge1'
      ANAS0_DW.Merge1[i] = ANAS0_P.Merge1_InitialOutput;
    }

    // End of SystemInitialize for SubSystem: '<S1>/Linear States Corrections'
    // End of SystemInitialize for SubSystem: '<Root>/ANAS'
  }
}

// Model terminate function
void ANAS0::terminate()
{
  // (no terminate code required)
}

// Constructor
ANAS0::ANAS0() :
  ANAS0_U(),
  ANAS0_Y(),
  ANAS0_DW(),
  ANAS0_M()
{
  // Currently there is no constructor body generated.
}

// Destructor
// Currently there is no destructor body generated.
ANAS0::~ANAS0() = default;

// Real-Time Model get method
ANAS0::RT_MODEL_ANAS0_T * ANAS0::getRTM()
{
  return (&ANAS0_M);
}

//
// File trailer for generated code.
//
// [EOF]
//
