//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// File: ANAS0.cpp
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
#include <stdbool.h>
#include <stdint.h>
#include "ANAS0_private.h"
#include <cstring>
#include <cmath>
#include <cfloat>

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

  (ANAS0_M->Timing.TaskCounters.TID[2])++;
  if ((ANAS0_M->Timing.TaskCounters.TID[2]) > 1) {// Sample time: [0.02s, 0.0s]
    ANAS0_M->Timing.TaskCounters.TID[2] = 0;
  }

  (ANAS0_M->Timing.TaskCounters.TID[3])++;
  if ((ANAS0_M->Timing.TaskCounters.TID[3]) > 9) {// Sample time: [0.1s, 0.0s]
    ANAS0_M->Timing.TaskCounters.TID[3] = 0;
  }
}

//
// Output and update for enable system:
//    '<S59>/Subsystem1'
//    '<S61>/Subsystem1'
//
void ANAS0::ANAS0_Subsystem1(bool rtu_Enable, const double rtu_nextLinearState[6],
  const double rtu_nextLinearCov[36], double rty_State[6], double
  rty_Covariance[36])
{
  double rtu_nextLinearCov_0[36];
  double rtu_nextLinearState_0[6];

  // Outputs for Enabled SubSystem: '<S59>/Subsystem1' incorporates:
  //   EnablePort: '<S63>/Enable'

  if (rtu_Enable) {
    // SignalConversion generated from: '<S63>/nextLinearState'
    for (int32_t i{0}; i < 6; i++) {
      rtu_nextLinearState_0[i] = rtu_nextLinearState[i];
    }

    for (int32_t i{0}; i < 6; i++) {
      rty_State[i] = rtu_nextLinearState_0[i];
    }

    // SignalConversion generated from: '<S63>/nextLinearCov'
    for (int32_t i{0}; i < 36; i++) {
      rtu_nextLinearCov_0[i] = rtu_nextLinearCov[i];
    }

    for (int32_t i{0}; i < 36; i++) {
      rty_Covariance[i] = rtu_nextLinearCov_0[i];
    }
  }

  // End of Outputs for SubSystem: '<S59>/Subsystem1'
}

double rt_remd(double u0, double u1)
{
  double y;
  if ((u1 != 0.0) && (u1 != std::trunc(u1))) {
    double q;
    q = std::abs(u0 / u1);
    if (std::abs(q - std::floor(q + 0.5)) <= DBL_EPSILON * q) {
      y = 0.0;
    } else {
      y = std::fmod(u0, u1);
    }
  } else {
    y = std::fmod(u0, u1);
  }

  return y;
}

void rt_mrdivide_U1d6x3_U2d3x3_Yd6x3(const double u0[18], const double u1[9],
  double y[18])
{
  double A[9];
  double a21;
  double maxval;
  int32_t r1;
  int32_t r2;
  int32_t r3;
  int32_t rtemp;
  std::memcpy(&A[0], &u1[0], 9U * sizeof(double));
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
  for (rtemp = 0; rtemp < 6; rtemp++) {
    int32_t y_tmp;
    int32_t y_tmp_0;
    int32_t y_tmp_1;
    y_tmp = 6 * r1 + rtemp;
    y[y_tmp] = u0[rtemp] / A[r1];
    y_tmp_0 = 6 * r2 + rtemp;
    y[y_tmp_0] = u0[rtemp + 6] - A[r1 + 3] * y[y_tmp];
    y_tmp_1 = 6 * r3 + rtemp;
    y[y_tmp_1] = u0[rtemp + 12] - A[r1 + 6] * y[y_tmp];
    y[y_tmp_0] /= A[r2 + 3];
    y[y_tmp_1] -= A[r2 + 6] * y[y_tmp_0];
    y[y_tmp_1] /= A[r3 + 6];
    y[y_tmp_0] -= A[r3 + 3] * y[y_tmp_1];
    y[y_tmp] -= y[y_tmp_1] * A[r3];
    y[y_tmp] -= y[y_tmp_0] * A[r2];
  }
}

void rt_mrdivide_U1d6x4_U2d4x4_Yd6x4(double u0[24], const double u1[16])
{
  double x[16];
  double smax;
  int32_t c;
  int32_t jA;
  int32_t jBcol;
  int32_t jj;
  int32_t kBcol;
  int8_t ipiv[4];
  std::memcpy(&x[0], &u1[0], sizeof(double) << 4U);
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
      double s;
      s = std::abs(x[(c + jA) - 3]);
      if (s > smax) {
        iy = jA;
        smax = s;
      }
    }

    if (x[(c + iy) - 3] != 0.0) {
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
      if (smax != 0.0) {
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
      if (smax != 0.0) {
        for (int32_t ix{0}; ix < 6; ix++) {
          c = (ix + jBcol) + 1;
          u0[c] -= u0[(ix + kBcol) + 1] * smax;
        }
      }
    }

    smax = 1.0 / x[(j + jj) + 1];
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
      if (smax != 0.0) {
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

void rt_mrdivide_U1d6x2_U2d2x2_Yd6x2(const double u0[12], const double u1[4],
  double y[12])
{
  double a21;
  double a22;
  double a22_tmp;
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

// Model step function
void ANAS0::step()
{
  // local block i/o variables
  uint8_t rtb_Saturation1;
  uint8_t rtb_Saturation2;

  {
    double rtb_MatrixMultiply2_k3[36];
    double rtb_Transpose_m[36];
    double rtb_UnitDelay1[36];
    double rtb_UnitDelay1_0[36];
    double rtb_UnitDelay5[36];
    double tmp[36];
    double rtb_MatrixDivide[24];
    double rtb_Transpose1[24];
    double rtb_MatrixConcatenate1_b[18];
    double rtb_MatrixDivide_o[18];
    double rtb_Transpose1_d[18];
    double rtb_Square_er[16];
    double rtb_Transpose1_0[16];
    double rtb_VectorConcatenate[13];
    double rtb_K[12];
    double rtb_K_tmp[12];
    double rtb_Transpose1_l[12];
    double rtb_Transpose[9];
    double rtb_Transpose_1[9];
    double rtb_VectorConcatenate_p[9];
    double rtb_Switch3[7];
    double rtb_MatrixMultiply2_b[6];
    double rtb_MatrixMultiply2_n[6];
    double rtb_Switch4[6];
    double rtb_VectorConcatenate_a[6];
    double rtb_MatrixConcatenate_a[4];
    double rtb_VectorConcatenate_d[3];
    double Gain;
    double rtb_AddConstant1;
    double rtb_Divide1;
    double rtb_Divide1_a;
    double rtb_Power;
    double rtb_Power3;
    double rtb_Square_k;
    double rtb_Switch1_l;
    double rtb_VectorConcatenate_i_tmp;
    double rtb_VectorConcatenate_i_tmp_0;
    double rtb_VectorConcatenate_i_tmp_1;
    double rtb_VectorConcatenate_i_tmp_2;
    double rtb_VectorConcatenate_i_tmp_3;
    double rtb_VectorConcatenate_i_tmp_4;
    double tmp_1;
    float rtb_CastToSingle2[36];
    float rtb_CastToSingle3[13];
    float rtb_MatrixConcatenate_n[12];
    float tmp_0[9];
    float rtb_VectorConcatenate1[7];
    float rtb_VectorConcatenate2[6];
    float rtb_MatrixConcatenate1_l[4];
    float rtb_Sqrt;
    float rtb_Switch_op_idx_0;
    float rtb_Switch_op_idx_1;
    int32_t Sum1_tmp;
    int32_t i;
    int32_t i_0;
    int32_t i_1;
    int32_t i_2;
    int32_t i_3;
    int32_t rtb_MatrixMultiply2_f_tmp;
    uint8_t rtb_UnitDelay3;
    uint8_t rtb_UnitDelay4;
    bool rtb_NOT;
    bool rtb_NOT1_h;
    bool rtb_NOT1_n;
    bool rtb_NOT_k;

    // Outputs for Atomic SubSystem: '<Root>/ANAS - Autocoding'
    // Switch: '<S10>/Switch' incorporates:
    //   Clock: '<S10>/Clock2'
    //   Constant: '<S10>/Constant3'
    //   Constant: '<S10>/Constant4'
    //   Constant: '<S43>/Constant'
    //   Constant: '<S45>/Constant'
    //   DataStoreWrite: '<S10>/Data Store Write'
    //   Math: '<S10>/Math Function1'
    //   RelationalOperator: '<S43>/Compare'
    //   RelationalOperator: '<S45>/Compare'
    //   Switch: '<S10>/OutputSwitch2'

    if (ANAS0_DW.ProbeSampleTime[0] > ANAS0_P.CompareToConstant1_const) {
      // Switch: '<S10>/OutputSwitch1' incorporates:
      //   Clock: '<S10>/Clock1'
      //   Constant: '<S10>/Constant'
      //   Constant: '<S10>/Constant1'
      //   Constant: '<S44>/Constant'
      //   Math: '<S10>/Math Function'
      //   RelationalOperator: '<S44>/Compare'

      if (rt_remd((&ANAS0_M)->Timing.t[0], ANAS0_DW.ProbeSampleTime[0]) ==
          ANAS0_P.Constant_Value_i) {
        rtb_Switch1_l = ANAS0_P.Constant_Value_bm;
      } else {
        rtb_Switch1_l = ANAS0_P.Constant1_Value_n;
      }

      // End of Switch: '<S10>/OutputSwitch1'
    } else if (rt_remd((&ANAS0_M)->Timing.t[0], ANAS0_P.Constant4_Value_c) ==
               ANAS0_P.Constant_Value_dl) {
      // Switch: '<S10>/OutputSwitch2' incorporates:
      //   Constant: '<S10>/Constant2'

      rtb_Switch1_l = ANAS0_P.Constant2_Value_b;
    } else {
      rtb_Switch1_l = ANAS0_P.Constant3_Value_p;
    }

    // End of Switch: '<S10>/Switch'

    // Gain: '<S10>/Gain'
    Gain = ANAS0_P.Gain_Gain_gl * rtb_Switch1_l;

    // RateTransition: '<S10>/Rate Transition' incorporates:
    //   RateTransition: '<S2>/Rate Transition1'
    //   RateTransition: '<S60>/Rate Transition'
    //   RateTransition: '<S64>/Rate Transition1'
    //   RateTransition: '<S76>/Rate Transition1'
    //   RateTransition: '<S7>/Rate Transition'
    //   RateTransition: '<S90>/Rate Transition1'

    rtb_NOT1_h = ((&ANAS0_M)->Timing.TaskCounters.TID[2] == 0);
    if (rtb_NOT1_h) {
      // UnitDelay: '<S1>/Unit Delay3'
      rtb_UnitDelay3 = ANAS0_DW.UnitDelay3_DSTATE;

      // Switch: '<S1>/Switch3' incorporates:
      //   Constant: '<S1>/Constant'
      //   DataTypeConversion: '<S1>/Cast To Double1'
      //   UnitDelay: '<S1>/Unit Delay'
      //   UnitDelay: '<S1>/Unit Delay3'

      if (ANAS0_DW.UnitDelay3_DSTATE != 0) {
        for (i_0 = 0; i_0 < 7; i_0++) {
          rtb_Switch3[i_0] = ANAS0_DW.UnitDelay_DSTATE[i_0];
        }
      } else {
        rtb_VectorConcatenate1[4] = ANAS0_P.Constant_Value_i5[0];
        rtb_VectorConcatenate1[5] = ANAS0_P.Constant_Value_i5[1];
        rtb_VectorConcatenate1[6] = ANAS0_P.Constant_Value_i5[2];

        // SignalConversion generated from: '<S1>/Vector Concatenate1' incorporates:
        //   Constant: '<S1>/Constant'
        //   Inport: '<Root>/ANAS Reference In'

        rtb_VectorConcatenate1[0] = ANAS0_U.ANASReferenceIn.InitialQuaternion[0];
        rtb_VectorConcatenate1[1] = ANAS0_U.ANASReferenceIn.InitialQuaternion[1];
        rtb_VectorConcatenate1[2] = ANAS0_U.ANASReferenceIn.InitialQuaternion[2];
        rtb_VectorConcatenate1[3] = ANAS0_U.ANASReferenceIn.InitialQuaternion[3];
        for (i_0 = 0; i_0 < 7; i_0++) {
          rtb_Switch3[i_0] = rtb_VectorConcatenate1[i_0];
        }
      }

      // End of Switch: '<S1>/Switch3'

      // UnitDelay: '<S1>/Unit Delay1'
      std::memcpy(&rtb_UnitDelay1[0], &ANAS0_DW.UnitDelay1_DSTATE[0], 36U *
                  sizeof(double));
    }

    // End of RateTransition: '<S10>/Rate Transition'

    // Switch: '<S6>/Switch' incorporates:
    //   Clock: '<S6>/Clock2'
    //   Constant: '<S124>/Constant'
    //   Constant: '<S126>/Constant'
    //   Constant: '<S6>/Constant4'
    //   DataStoreRead: '<S6>/Data Store Read'
    //   Math: '<S6>/Math Function1'
    //   RelationalOperator: '<S124>/Compare'
    //   RelationalOperator: '<S126>/Compare'
    //   Switch: '<S6>/OutputSwitch2'

    if (ANAS0_DW.A_g > ANAS0_P.CompareToConstant1_const_e) {
      // Switch: '<S6>/OutputSwitch1' incorporates:
      //   Clock: '<S6>/Clock1'
      //   Constant: '<S125>/Constant'
      //   Math: '<S6>/Math Function'
      //   RelationalOperator: '<S125>/Compare'

      if (rt_remd((&ANAS0_M)->Timing.t[0], ANAS0_DW.A_g) ==
          ANAS0_P.Constant_Value_e) {
        // Switch: '<S6>/Switch' incorporates:
        //   Constant: '<S6>/Constant'

        rtb_Switch1_l = ANAS0_P.Constant_Value_ez;
      } else {
        // Switch: '<S6>/Switch' incorporates:
        //   Constant: '<S6>/Constant1'

        rtb_Switch1_l = ANAS0_P.Constant1_Value_o;
      }

      // End of Switch: '<S6>/OutputSwitch1'
    } else if (rt_remd((&ANAS0_M)->Timing.t[0], ANAS0_P.Constant4_Value_h) ==
               ANAS0_P.Constant_Value_mg) {
      // Switch: '<S6>/OutputSwitch2' incorporates:
      //   Constant: '<S6>/Constant2'
      //   Switch: '<S6>/Switch'

      rtb_Switch1_l = ANAS0_P.Constant2_Value_c;
    } else {
      // Switch: '<S6>/Switch' incorporates:
      //   Constant: '<S6>/Constant3'

      rtb_Switch1_l = ANAS0_P.Constant3_Value_j4;
    }

    // End of Switch: '<S6>/Switch'

    // Outputs for Enabled SubSystem: '<S1>/Attitude predictor' incorporates:
    //   EnablePort: '<S3>/Enable'

    if ((rtb_Switch1_l > 0.0) && ((&ANAS0_M)->Timing.TaskCounters.TID[2] == 0))
    {
      for (i_0 = 0; i_0 < 9; i_0++) {
        // Gain: '<S48>/Gain' incorporates:
        //   Constant: '<S48>/Constant'
        //   Math: '<S109>/Transpose'

        rtb_Switch1_l = ANAS0_P.Gain_Gain_e * ANAS0_P.Constant_Value_n[i_0];

        // SignalConversion generated from: '<S48>/Matrix Concatenate' incorporates:
        //   Gain: '<S48>/Gain'
        //   Math: '<S109>/Transpose'

        rtb_MatrixConcatenate1_b[i_0] = rtb_Switch1_l;

        // Constant: '<S48>/Constant2' incorporates:
        //   Gain: '<S48>/Gain'

        rtb_Transpose1_d[i_0 + 9] = ANAS0_P.Constant2_Value[i_0];

        // Constant: '<S48>/Constant3' incorporates:
        //   Gain: '<S48>/Gain'

        rtb_Transpose1_d[i_0] = ANAS0_P.Constant3_Value[i_0];

        // Product: '<S48>/Product' incorporates:
        //   Constant: '<S3>/Constant'
        //   Gain: '<S48>/Gain'
        //   Math: '<S109>/Transpose'

        rtb_MatrixConcatenate1_b[i_0 + 9] = rtb_Switch1_l *
          ANAS0_P.Constant_Value_l;
      }

      // Concatenate: '<S48>/Matrix Concatenate2' incorporates:
      //   Concatenate: '<S50>/Matrix Concatenate1'
      //   Math: '<S11>/Transpose1'
      //   Product: '<S91>/Matrix Multiply2'

      i_0 = 0;
      i = 0;
      for (Sum1_tmp = 0; Sum1_tmp < 6; Sum1_tmp++) {
        rtb_MatrixMultiply2_k3[i_0] = rtb_MatrixConcatenate1_b[i];
        rtb_MatrixMultiply2_k3[i_0 + 3] = rtb_Transpose1_d[i];
        rtb_MatrixMultiply2_k3[i_0 + 1] = rtb_MatrixConcatenate1_b[i + 1];
        rtb_MatrixMultiply2_k3[i_0 + 4] = rtb_Transpose1_d[i + 1];
        rtb_MatrixMultiply2_k3[i_0 + 2] = rtb_MatrixConcatenate1_b[i + 2];
        rtb_MatrixMultiply2_k3[i_0 + 5] = rtb_Transpose1_d[i + 2];
        i_0 += 6;
        i += 3;
      }

      // End of Concatenate: '<S48>/Matrix Concatenate2'
      for (i_0 = 0; i_0 < 9; i_0++) {
        // SignalConversion generated from: '<S50>/Matrix Concatenate1' incorporates:
        //   Constant: '<S50>/Constant3'

        rtb_Switch1_l = ANAS0_P.Constant3_Value_j[i_0];
        rtb_MatrixConcatenate1_b[i_0] = rtb_Switch1_l;

        // SignalConversion generated from: '<S50>/Matrix Concatenate' incorporates:
        //   Constant: '<S50>/Constant3'

        rtb_Transpose1_d[i_0 + 9] = rtb_Switch1_l;

        // Constant: '<S50>/Constant2' incorporates:
        //   SignalConversion generated from: '<S50>/Matrix Concatenate1'

        rtb_MatrixConcatenate1_b[i_0 + 9] = ANAS0_P.Constant2_Value_i[i_0];

        // Gain: '<S50>/Gain' incorporates:
        //   Constant: '<S50>/Constant'
        //   SignalConversion generated from: '<S50>/Matrix Concatenate1'

        rtb_Transpose1_d[i_0] = ANAS0_P.Gain_Gain_l *
          ANAS0_P.Constant_Value_m[i_0];
      }

      // Concatenate: '<S50>/Matrix Concatenate2' incorporates:
      //   Concatenate: '<S50>/Matrix Concatenate1'
      //   Math: '<S11>/Transpose1'
      //   Math: '<S46>/Transpose'
      //   Math: '<S91>/Transpose'
      //   Product: '<S46>/Matrix Multiply'
      //   Product: '<S91>/Matrix Multiply2'
      //   UnitDelay: '<S1>/Unit Delay1'

      i_0 = 0;
      i = 0;
      for (Sum1_tmp = 0; Sum1_tmp < 6; Sum1_tmp++) {
        rtb_Transpose_m[i_0] = rtb_Transpose1_d[i];
        rtb_Transpose_m[i_0 + 3] = rtb_MatrixConcatenate1_b[i];
        rtb_Transpose_m[i_0 + 1] = rtb_Transpose1_d[i + 1];
        rtb_Transpose_m[i_0 + 4] = rtb_MatrixConcatenate1_b[i + 1];
        rtb_Transpose_m[i_0 + 2] = rtb_Transpose1_d[i + 2];
        rtb_Transpose_m[i_0 + 5] = rtb_MatrixConcatenate1_b[i + 2];
        i_2 = 0;
        for (rtb_MatrixMultiply2_f_tmp = 0; rtb_MatrixMultiply2_f_tmp < 6;
             rtb_MatrixMultiply2_f_tmp++) {
          rtb_Switch1_l = 0.0;
          i_3 = 0;
          for (i_1 = 0; i_1 < 6; i_1++) {
            rtb_Switch1_l += rtb_UnitDelay1[i_3 + Sum1_tmp] *
              rtb_MatrixMultiply2_k3[i_3 + rtb_MatrixMultiply2_f_tmp];
            i_3 += 6;
          }

          rtb_UnitDelay1_0[i_2 + Sum1_tmp] = rtb_Switch1_l;
          i_2 += 6;
        }

        i_0 += 6;
        i += 3;
      }

      // End of Concatenate: '<S50>/Matrix Concatenate2'
      for (i_0 = 0; i_0 < 6; i_0++) {
        // Product: '<S46>/Matrix Multiply2' incorporates:
        //   Constant: '<S46>/Constant'
        //   Math: '<S46>/Transpose1'
        //   Math: '<S91>/Transpose'
        //   Product: '<S46>/Matrix Multiply'
        //   Product: '<S91>/Matrix Multiply2'

        i = 0;
        for (Sum1_tmp = 0; Sum1_tmp < 6; Sum1_tmp++) {
          rtb_Square_k = 0.0;
          rtb_Power3 = 0.0;
          i_2 = 0;
          for (rtb_MatrixMultiply2_f_tmp = 0; rtb_MatrixMultiply2_f_tmp < 6;
               rtb_MatrixMultiply2_f_tmp++) {
            i_3 = i_2 + i_0;
            rtb_Square_k += rtb_Transpose_m[i_2 + Sum1_tmp] *
              ANAS0_P.Constant_Value_hu[i_3];
            rtb_Power3 += rtb_UnitDelay1_0[rtb_MatrixMultiply2_f_tmp + i] *
              rtb_MatrixMultiply2_k3[i_3];
            i_2 += 6;
          }

          rtb_MatrixMultiply2_f_tmp = i + i_0;
          rtb_UnitDelay1[rtb_MatrixMultiply2_f_tmp] = rtb_Power3;
          tmp[rtb_MatrixMultiply2_f_tmp] = rtb_Square_k;
          i += 6;
        }
      }

      // Product: '<S46>/Matrix Multiply2' incorporates:
      //   Math: '<S91>/Transpose'

      i_0 = 0;
      for (i = 0; i < 6; i++) {
        for (Sum1_tmp = 0; Sum1_tmp < 6; Sum1_tmp++) {
          rtb_Switch1_l = 0.0;
          i_2 = 0;
          for (rtb_MatrixMultiply2_f_tmp = 0; rtb_MatrixMultiply2_f_tmp < 6;
               rtb_MatrixMultiply2_f_tmp++) {
            rtb_Switch1_l += rtb_Transpose_m[i_2 + Sum1_tmp] *
              tmp[rtb_MatrixMultiply2_f_tmp + i_0];
            i_2 += 6;
          }

          rtb_MatrixMultiply2_k3[Sum1_tmp + i_0] = rtb_Switch1_l;
        }

        i_0 += 6;
      }

      for (i_0 = 0; i_0 < 36; i_0++) {
        // Sum: '<S46>/Sum1'
        ANAS0_DW.Sum1_a[i_0] = rtb_UnitDelay1[i_0] + rtb_MatrixMultiply2_k3[i_0];
      }

      for (i_0 = 0; i_0 < 7; i_0++) {
        // SignalConversion generated from: '<S3>/prevAngularState' incorporates:
        //   Switch: '<S1>/Switch3'

        ANAS0_DW.prevAngularState[i_0] = rtb_Switch3[i_0];
      }

      // SignalConversion generated from: '<S56>/Vector Concatenate' incorporates:
      //   Constant: '<S55>/Constant'

      rtb_VectorConcatenate_p[0] = ANAS0_P.Constant_Value_c3;

      // Sum: '<S53>/Sum' incorporates:
      //   Constant: '<S47>/gyro biases'
      //   DataTypeConversion: '<S53>/Cast To Single'
      //   Inport: '<Root>/ANAS In'
      //   Switch: '<S19>/Switch'

      rtb_Sqrt = ANAS0_U.ANASIn_j.GyroMeasure[0] - static_cast<float>
        (ANAS0_P.gyrobiases_Value[0]);
      rtb_Switch_op_idx_0 = rtb_Sqrt;

      // DataTypeConversion: '<S55>/Cast To Double' incorporates:
      //   Concatenate: '<S97>/Vector Concatenate'
      //   Switch: '<S19>/Switch'

      rtb_VectorConcatenate_d[0] = rtb_Sqrt;

      // Sum: '<S53>/Sum' incorporates:
      //   Constant: '<S47>/gyro biases'
      //   DataTypeConversion: '<S53>/Cast To Single'
      //   Inport: '<Root>/ANAS In'
      //   Switch: '<S19>/Switch'

      rtb_Sqrt = ANAS0_U.ANASIn_j.GyroMeasure[1] - static_cast<float>
        (ANAS0_P.gyrobiases_Value[1]);
      rtb_Switch_op_idx_1 = rtb_Sqrt;

      // DataTypeConversion: '<S55>/Cast To Double' incorporates:
      //   Concatenate: '<S97>/Vector Concatenate'
      //   Switch: '<S19>/Switch'

      rtb_VectorConcatenate_d[1] = rtb_Sqrt;

      // Sum: '<S53>/Sum' incorporates:
      //   Constant: '<S47>/gyro biases'
      //   DataTypeConversion: '<S53>/Cast To Single'
      //   Inport: '<Root>/ANAS In'
      //   Switch: '<S19>/Switch'

      rtb_Sqrt = ANAS0_U.ANASIn_j.GyroMeasure[2] - static_cast<float>
        (ANAS0_P.gyrobiases_Value[2]);

      // Gain: '<S55>/Gain1' incorporates:
      //   DataTypeConversion: '<S55>/Cast To Double'
      //   Switch: '<S19>/Switch'

      rtb_VectorConcatenate_p[1] = ANAS0_P.Gain1_Gain_p * rtb_Sqrt;

      // SignalConversion generated from: '<S56>/Vector Concatenate'
      rtb_VectorConcatenate_p[2] = rtb_VectorConcatenate_d[1];

      // SignalConversion generated from: '<S56>/Vector Concatenate' incorporates:
      //   DataTypeConversion: '<S55>/Cast To Double'
      //   Switch: '<S19>/Switch'

      rtb_VectorConcatenate_p[3] = rtb_Sqrt;

      // SignalConversion generated from: '<S56>/Vector Concatenate' incorporates:
      //   Constant: '<S55>/Constant'

      rtb_VectorConcatenate_p[4] = ANAS0_P.Constant_Value_c3;

      // Gain: '<S55>/Gain2'
      rtb_VectorConcatenate_p[5] = ANAS0_P.Gain2_Gain_h *
        rtb_VectorConcatenate_d[0];

      // Gain: '<S55>/Gain'
      rtb_VectorConcatenate_p[6] = ANAS0_P.Gain_Gain_f *
        rtb_VectorConcatenate_d[1];

      // SignalConversion generated from: '<S56>/Vector Concatenate'
      rtb_VectorConcatenate_p[7] = rtb_VectorConcatenate_d[0];

      // SignalConversion generated from: '<S56>/Vector Concatenate' incorporates:
      //   Constant: '<S55>/Constant'

      rtb_VectorConcatenate_p[8] = ANAS0_P.Constant_Value_c3;

      // Gain: '<S53>/Gain' incorporates:
      //   Concatenate: '<S120>/Vector Concatenate'
      //   Math: '<S55>/Transpose'

      i_0 = 0;
      for (i = 0; i < 3; i++) {
        tmp_0[i_0] = static_cast<float>(ANAS0_P.Gain_Gain_h *
          rtb_VectorConcatenate_p[i]);
        tmp_0[i_0 + 1] = static_cast<float>(rtb_VectorConcatenate_p[i + 3] *
          ANAS0_P.Gain_Gain_h);
        tmp_0[i_0 + 2] = static_cast<float>(rtb_VectorConcatenate_p[i + 6] *
          ANAS0_P.Gain_Gain_h);
        i_0 += 3;
      }

      for (i_0 = 0; i_0 < 9; i_0++) {
        rtb_MatrixConcatenate_n[i_0] = tmp_0[i_0];
      }

      // End of Gain: '<S53>/Gain'

      // SignalConversion generated from: '<S53>/Matrix Concatenate' incorporates:
      //   Switch: '<S19>/Switch'

      rtb_MatrixConcatenate_n[9] = rtb_Switch_op_idx_0;

      // Gain: '<S53>/Gain1' incorporates:
      //   SignalConversion generated from: '<S53>/Matrix Concatenate'
      //   Switch: '<S19>/Switch'

      rtb_MatrixConcatenate1_l[0] = ANAS0_P.Gain1_Gain_ed * rtb_Switch_op_idx_0;

      // SignalConversion generated from: '<S53>/Matrix Concatenate' incorporates:
      //   Switch: '<S19>/Switch'

      rtb_MatrixConcatenate_n[10] = rtb_Switch_op_idx_1;

      // Gain: '<S53>/Gain1' incorporates:
      //   SignalConversion generated from: '<S53>/Matrix Concatenate'
      //   Switch: '<S19>/Switch'

      rtb_MatrixConcatenate1_l[1] = ANAS0_P.Gain1_Gain_ed * rtb_Switch_op_idx_1;

      // SignalConversion generated from: '<S53>/Matrix Concatenate' incorporates:
      //   Sum: '<S53>/Sum'
      //   Switch: '<S19>/Switch'

      rtb_MatrixConcatenate_n[11] = rtb_Sqrt;

      // Gain: '<S53>/Gain1' incorporates:
      //   Sum: '<S53>/Sum'
      //   Switch: '<S19>/Switch'

      rtb_MatrixConcatenate1_l[2] = ANAS0_P.Gain1_Gain_ed * rtb_Sqrt;

      // Constant: '<S53>/Constant'
      rtb_MatrixConcatenate1_l[3] = ANAS0_P.Constant_Value_dx;

      // Concatenate: '<S53>/Matrix Concatenate2' incorporates:
      //   Concatenate: '<S53>/Matrix Concatenate'
      //   Concatenate: '<S53>/Matrix Concatenate1'
      //   Constant: '<S3>/Constant'
      //   Gain: '<S47>/Gain'
      //   Product: '<S47>/Product'

      i_0 = 0;
      i = 0;
      for (Sum1_tmp = 0; Sum1_tmp < 4; Sum1_tmp++) {
        rtb_Square_er[i_0] = rtb_MatrixConcatenate_n[i] * ANAS0_P.Gain_Gain_nq *
          ANAS0_P.Constant_Value_l;
        rtb_Square_er[i_0 + 1] = rtb_MatrixConcatenate_n[i + 1] *
          ANAS0_P.Gain_Gain_nq * ANAS0_P.Constant_Value_l;
        rtb_Square_er[i_0 + 2] = rtb_MatrixConcatenate_n[i + 2] *
          ANAS0_P.Gain_Gain_nq * ANAS0_P.Constant_Value_l;
        rtb_Square_er[i_0 + 3] = ANAS0_P.Gain_Gain_nq *
          rtb_MatrixConcatenate1_l[Sum1_tmp] * ANAS0_P.Constant_Value_l;
        i_0 += 4;
        i += 3;
      }

      // End of Concatenate: '<S53>/Matrix Concatenate2'

      // Sum: '<S47>/Sum' incorporates:
      //   Constant: '<S47>/Constant'

      for (i_0 = 0; i_0 < 16; i_0++) {
        rtb_Transpose1_0[i_0] = rtb_Square_er[i_0] +
          ANAS0_P.Constant_Value_k[i_0];
      }

      // End of Sum: '<S47>/Sum'

      // Product: '<S47>/Product1' incorporates:
      //   SignalConversion generated from: '<S47>/Product1'

      rtb_Divide1_a = ANAS0_DW.prevAngularState[1];
      rtb_Switch1_l = ANAS0_DW.prevAngularState[2];
      tmp_1 = ANAS0_DW.prevAngularState[3];
      rtb_Square_k = ANAS0_DW.prevAngularState[0];
      for (i_0 = 0; i_0 < 4; i_0++) {
        // Concatenate: '<S14>/Matrix Concatenate'
        rtb_MatrixConcatenate_a[i_0] = ((rtb_Transpose1_0[i_0 + 4] *
          rtb_Switch1_l + rtb_Transpose1_0[i_0] * rtb_Divide1_a) +
          rtb_Transpose1_0[i_0 + 8] * tmp_1) + rtb_Transpose1_0[i_0 + 12] *
          rtb_Square_k;
      }

      // End of Product: '<S47>/Product1'

      // Sqrt: '<S57>/sqrt' incorporates:
      //   Product: '<S58>/Product'
      //   Product: '<S58>/Product1'
      //   Product: '<S58>/Product2'
      //   Product: '<S58>/Product3'
      //   Sum: '<S58>/Sum'

      rtb_Switch1_l = std::sqrt(((rtb_MatrixConcatenate_a[3] *
        rtb_MatrixConcatenate_a[3] + rtb_MatrixConcatenate_a[0] *
        rtb_MatrixConcatenate_a[0]) + rtb_MatrixConcatenate_a[1] *
        rtb_MatrixConcatenate_a[1]) + rtb_MatrixConcatenate_a[2] *
        rtb_MatrixConcatenate_a[2]);

      // Product: '<S54>/Product'
      ANAS0_DW.Product = rtb_MatrixConcatenate_a[3] / rtb_Switch1_l;

      // Product: '<S54>/Product1'
      ANAS0_DW.Product1 = rtb_MatrixConcatenate_a[0] / rtb_Switch1_l;

      // Product: '<S54>/Product2'
      ANAS0_DW.Product2 = rtb_MatrixConcatenate_a[1] / rtb_Switch1_l;

      // Product: '<S54>/Product3'
      ANAS0_DW.Product3 = rtb_MatrixConcatenate_a[2] / rtb_Switch1_l;
    }

    // End of Outputs for SubSystem: '<S1>/Attitude predictor'

    // Outputs for Enabled SubSystem: '<S2>/Attitude corrector' incorporates:
    //   EnablePort: '<S8>/Enable'

    if ((Gain > 0.0) && ((&ANAS0_M)->Timing.TaskCounters.TID[2] == 0)) {
      // SignalConversion generated from: '<S27>/Vector Concatenate' incorporates:
      //   Constant: '<S17>/Constant'

      rtb_Transpose[0] = ANAS0_P.Constant_Value_h;

      // Sqrt: '<S41>/sqrt' incorporates:
      //   Product: '<S42>/Product'
      //   Product: '<S42>/Product1'
      //   Product: '<S42>/Product2'
      //   Product: '<S42>/Product3'
      //   Sum: '<S42>/Sum'

      rtb_Switch1_l = std::sqrt(((ANAS0_DW.Product * ANAS0_DW.Product +
        ANAS0_DW.Product1 * ANAS0_DW.Product1) + ANAS0_DW.Product2 *
        ANAS0_DW.Product2) + ANAS0_DW.Product3 * ANAS0_DW.Product3);

      // Product: '<S40>/Product'
      rtb_Divide1_a = ANAS0_DW.Product / rtb_Switch1_l;

      // Product: '<S40>/Product1'
      rtb_Power = ANAS0_DW.Product1 / rtb_Switch1_l;

      // Product: '<S40>/Product2'
      rtb_Square_k = ANAS0_DW.Product2 / rtb_Switch1_l;

      // Product: '<S40>/Product3'
      rtb_Switch1_l = ANAS0_DW.Product3 / rtb_Switch1_l;

      // Product: '<S30>/Product3' incorporates:
      //   Product: '<S34>/Product3'

      tmp_1 = rtb_Divide1_a * rtb_Divide1_a;

      // Product: '<S30>/Product2' incorporates:
      //   Product: '<S34>/Product2'

      rtb_AddConstant1 = rtb_Power * rtb_Power;

      // Product: '<S30>/Product1' incorporates:
      //   Product: '<S34>/Product1'
      //   Product: '<S38>/Product1'

      rtb_VectorConcatenate_i_tmp_1 = rtb_Square_k * rtb_Square_k;

      // Product: '<S30>/Product' incorporates:
      //   Product: '<S34>/Product'
      //   Product: '<S38>/Product'

      rtb_VectorConcatenate_i_tmp_2 = rtb_Switch1_l * rtb_Switch1_l;

      // Sum: '<S30>/Sum' incorporates:
      //   Product: '<S30>/Product'
      //   Product: '<S30>/Product1'
      //   Product: '<S30>/Product2'
      //   Product: '<S30>/Product3'

      rtb_VectorConcatenate_p[0] = ((tmp_1 + rtb_AddConstant1) -
        rtb_VectorConcatenate_i_tmp_1) - rtb_VectorConcatenate_i_tmp_2;

      // Product: '<S33>/Product3' incorporates:
      //   Product: '<S31>/Product3'

      rtb_VectorConcatenate_i_tmp = rtb_Switch1_l * rtb_Divide1_a;

      // Product: '<S33>/Product2' incorporates:
      //   Product: '<S31>/Product2'

      rtb_VectorConcatenate_i_tmp_0 = rtb_Power * rtb_Square_k;

      // Gain: '<S33>/Gain' incorporates:
      //   Product: '<S33>/Product2'
      //   Product: '<S33>/Product3'
      //   Sum: '<S33>/Sum'

      rtb_VectorConcatenate_p[1] = (rtb_VectorConcatenate_i_tmp_0 -
        rtb_VectorConcatenate_i_tmp) * ANAS0_P.Gain_Gain;

      // Product: '<S36>/Product2' incorporates:
      //   Product: '<S32>/Product2'

      rtb_VectorConcatenate_i_tmp_3 = rtb_Power * rtb_Switch1_l;

      // Product: '<S36>/Product1' incorporates:
      //   Product: '<S32>/Product1'

      rtb_VectorConcatenate_i_tmp_4 = rtb_Divide1_a * rtb_Square_k;

      // Gain: '<S36>/Gain' incorporates:
      //   Product: '<S36>/Product1'
      //   Product: '<S36>/Product2'
      //   Sum: '<S36>/Sum'

      rtb_VectorConcatenate_p[2] = (rtb_VectorConcatenate_i_tmp_4 +
        rtb_VectorConcatenate_i_tmp_3) * ANAS0_P.Gain_Gain_d;

      // Gain: '<S31>/Gain' incorporates:
      //   Sum: '<S31>/Sum'

      rtb_VectorConcatenate_p[3] = (rtb_VectorConcatenate_i_tmp +
        rtb_VectorConcatenate_i_tmp_0) * ANAS0_P.Gain_Gain_k;

      // Sum: '<S34>/Sum' incorporates:
      //   Sum: '<S38>/Sum'

      tmp_1 -= rtb_AddConstant1;
      rtb_VectorConcatenate_p[4] = (tmp_1 + rtb_VectorConcatenate_i_tmp_1) -
        rtb_VectorConcatenate_i_tmp_2;

      // Product: '<S37>/Product1' incorporates:
      //   Product: '<S35>/Product1'

      rtb_AddConstant1 = rtb_Divide1_a * rtb_Power;

      // Product: '<S37>/Product2' incorporates:
      //   Product: '<S35>/Product2'

      rtb_VectorConcatenate_i_tmp = rtb_Square_k * rtb_Switch1_l;

      // Gain: '<S37>/Gain' incorporates:
      //   Product: '<S37>/Product1'
      //   Product: '<S37>/Product2'
      //   Sum: '<S37>/Sum'

      rtb_VectorConcatenate_p[5] = (rtb_VectorConcatenate_i_tmp -
        rtb_AddConstant1) * ANAS0_P.Gain_Gain_o;

      // Gain: '<S32>/Gain' incorporates:
      //   Sum: '<S32>/Sum'

      rtb_VectorConcatenate_p[6] = (rtb_VectorConcatenate_i_tmp_3 -
        rtb_VectorConcatenate_i_tmp_4) * ANAS0_P.Gain_Gain_g;

      // Gain: '<S35>/Gain' incorporates:
      //   Sum: '<S35>/Sum'

      rtb_VectorConcatenate_p[7] = (rtb_AddConstant1 +
        rtb_VectorConcatenate_i_tmp) * ANAS0_P.Gain_Gain_m;

      // Sum: '<S38>/Sum'
      rtb_VectorConcatenate_p[8] = (tmp_1 - rtb_VectorConcatenate_i_tmp_1) +
        rtb_VectorConcatenate_i_tmp_2;

      // Product: '<S12>/Matrix Multiply' incorporates:
      //   Concatenate: '<S120>/Vector Concatenate'
      //   Constant: '<S12>/known mag direction'

      rtb_Switch1_l = ANAS0_P.knownmagdirection_Value[1];
      rtb_Power = ANAS0_P.knownmagdirection_Value[0];
      rtb_Divide1_a = ANAS0_P.knownmagdirection_Value[2];
      for (i_0 = 0; i_0 < 3; i_0++) {
        rtb_VectorConcatenate_d[i_0] = (rtb_VectorConcatenate_p[i_0 + 3] *
          rtb_Switch1_l + rtb_VectorConcatenate_p[i_0] * rtb_Power) +
          rtb_VectorConcatenate_p[i_0 + 6] * rtb_Divide1_a;
      }

      // End of Product: '<S12>/Matrix Multiply'

      // Gain: '<S17>/Gain1'
      rtb_Transpose[1] = ANAS0_P.Gain1_Gain * rtb_VectorConcatenate_d[2];

      // SignalConversion generated from: '<S27>/Vector Concatenate'
      rtb_Transpose[2] = rtb_VectorConcatenate_d[1];

      // SignalConversion generated from: '<S27>/Vector Concatenate'
      rtb_Transpose[3] = rtb_VectorConcatenate_d[2];

      // SignalConversion generated from: '<S27>/Vector Concatenate' incorporates:
      //   Constant: '<S17>/Constant'

      rtb_Transpose[4] = ANAS0_P.Constant_Value_h;

      // Gain: '<S17>/Gain2'
      rtb_Transpose[5] = ANAS0_P.Gain2_Gain * rtb_VectorConcatenate_d[0];

      // Gain: '<S17>/Gain'
      rtb_Transpose[6] = ANAS0_P.Gain_Gain_ok * rtb_VectorConcatenate_d[1];

      // SignalConversion generated from: '<S27>/Vector Concatenate'
      rtb_Transpose[7] = rtb_VectorConcatenate_d[0];

      // SignalConversion generated from: '<S27>/Vector Concatenate' incorporates:
      //   Constant: '<S17>/Constant'

      rtb_Transpose[8] = ANAS0_P.Constant_Value_h;

      // Math: '<S17>/Transpose' incorporates:
      //   Concatenate: '<S27>/Vector Concatenate'
      //   Constant: '<S12>/Constant'

      i_0 = 0;
      for (i = 0; i < 3; i++) {
        rtb_Transpose_1[i_0] = rtb_Transpose[i];
        rtb_Transpose_1[i_0 + 1] = rtb_Transpose[i + 3];
        rtb_Transpose_1[i_0 + 2] = rtb_Transpose[i + 6];
        i_0 += 3;
      }

      for (i_0 = 0; i_0 < 9; i_0++) {
        rtb_Transpose1_d[i_0] = rtb_Transpose_1[i_0];
        rtb_Transpose1_d[i_0 + 9] = ANAS0_P.Constant_Value_d[i_0];
      }

      // End of Math: '<S17>/Transpose'
      for (i_0 = 0; i_0 < 6; i_0++) {
        // Product: '<S15>/Matrix Multiply2' incorporates:
        //   Math: '<S11>/Transpose1'
        //   Math: '<S15>/Transpose1'
        //   Product: '<S15>/Matrix Multiply1'
        //   Sum: '<S46>/Sum1'

        i = 0;
        for (Sum1_tmp = 0; Sum1_tmp < 3; Sum1_tmp++) {
          rtb_Switch1_l = 0.0;
          i_2 = 0;
          rtb_MatrixMultiply2_f_tmp = 0;
          for (i_3 = 0; i_3 < 6; i_3++) {
            rtb_Switch1_l += ANAS0_DW.Sum1_a[i_2 + i_0] *
              rtb_Transpose1_d[rtb_MatrixMultiply2_f_tmp + Sum1_tmp];
            i_2 += 6;
            rtb_MatrixMultiply2_f_tmp += 3;
          }

          rtb_MatrixConcatenate1_b[i + i_0] = rtb_Switch1_l;
          i += 6;
        }
      }

      // Sum: '<S15>/Sum' incorporates:
      //   Constant: '<S15>/Constant'
      //   Math: '<S11>/Transpose1'
      //   Product: '<S15>/Matrix Multiply2'

      for (i_0 = 0; i_0 < 3; i_0++) {
        i = 0;
        Sum1_tmp = 0;
        for (i_2 = 0; i_2 < 3; i_2++) {
          rtb_Square_k = 0.0;
          rtb_MatrixMultiply2_f_tmp = 0;
          for (i_3 = 0; i_3 < 6; i_3++) {
            rtb_Square_k += rtb_Transpose1_d[rtb_MatrixMultiply2_f_tmp + i_0] *
              rtb_MatrixConcatenate1_b[i_3 + Sum1_tmp];
            rtb_MatrixMultiply2_f_tmp += 3;
          }

          rtb_MatrixMultiply2_f_tmp = i + i_0;
          rtb_Transpose[rtb_MatrixMultiply2_f_tmp] =
            ANAS0_P.Constant_Value_he[rtb_MatrixMultiply2_f_tmp] + rtb_Square_k;
          i += 3;
          Sum1_tmp += 6;
        }
      }

      // End of Sum: '<S15>/Sum'

      // Product: '<S15>/Matrix Divide' incorporates:
      //   Product: '<S15>/Matrix Multiply1'

      rt_mrdivide_U1d6x3_U2d3x3_Yd6x3(rtb_MatrixConcatenate1_b, rtb_Transpose,
        rtb_MatrixDivide_o);

      // Sum: '<S13>/Subtract' incorporates:
      //   Constant: '<S13>/Constant'
      //   Math: '<S11>/Transpose1'
      //   Product: '<S11>/Matrix Multiply2'
      //   Product: '<S13>/Matrix Multiply'

      for (i_0 = 0; i_0 < 6; i_0++) {
        // Product: '<S13>/Matrix Multiply' incorporates:
        //   Product: '<S15>/Matrix Divide'

        rtb_Switch1_l = rtb_MatrixDivide_o[i_0 + 6];
        rtb_Power = rtb_MatrixDivide_o[i_0];
        rtb_Divide1_a = rtb_MatrixDivide_o[i_0 + 12];
        i = 0;
        Sum1_tmp = 0;
        for (i_2 = 0; i_2 < 6; i_2++) {
          rtb_MatrixMultiply2_f_tmp = i + i_0;
          rtb_UnitDelay1[rtb_MatrixMultiply2_f_tmp] =
            ANAS0_P.Constant_Value_c[rtb_MatrixMultiply2_f_tmp] -
            ((rtb_Transpose1_d[Sum1_tmp + 1] * rtb_Switch1_l +
              rtb_Transpose1_d[Sum1_tmp] * rtb_Power) +
             rtb_Transpose1_d[Sum1_tmp + 2] * rtb_Divide1_a);
          i += 6;
          Sum1_tmp += 3;
        }
      }

      // End of Sum: '<S13>/Subtract'

      // Product: '<S11>/Matrix Multiply' incorporates:
      //   Math: '<S11>/Transpose'
      //   Product: '<S11>/Matrix Multiply2'
      //   Sum: '<S46>/Sum1'

      for (i_0 = 0; i_0 < 6; i_0++) {
        i = 0;
        for (Sum1_tmp = 0; Sum1_tmp < 6; Sum1_tmp++) {
          rtb_Square_k = 0.0;
          i_2 = 0;
          for (rtb_MatrixMultiply2_f_tmp = 0; rtb_MatrixMultiply2_f_tmp < 6;
               rtb_MatrixMultiply2_f_tmp++) {
            rtb_Square_k += ANAS0_DW.Sum1_a[i_2 + i_0] * rtb_UnitDelay1[i_2 +
              Sum1_tmp];
            i_2 += 6;
          }

          tmp[i + i_0] = rtb_Square_k;
          i += 6;
        }
      }

      // Product: '<S11>/Matrix Multiply2' incorporates:
      //   Constant: '<S11>/Constant'
      //   Math: '<S11>/Transpose1'
      //   Product: '<S15>/Matrix Divide'

      for (i_0 = 0; i_0 < 3; i_0++) {
        rtb_Switch1_l = ANAS0_P.Constant_Value[i_0 + 3];
        rtb_Power = ANAS0_P.Constant_Value[i_0];
        rtb_Divide1_a = ANAS0_P.Constant_Value[i_0 + 6];
        i = 0;
        for (Sum1_tmp = 0; Sum1_tmp < 6; Sum1_tmp++) {
          rtb_MatrixConcatenate1_b[i + i_0] = (rtb_MatrixDivide_o[Sum1_tmp + 6] *
            rtb_Switch1_l + rtb_Power * rtb_MatrixDivide_o[Sum1_tmp]) +
            rtb_MatrixDivide_o[Sum1_tmp + 12] * rtb_Divide1_a;
          i += 3;
        }
      }

      // Product: '<S11>/Matrix Multiply' incorporates:
      //   Product: '<S11>/Matrix Multiply2'
      //   Product: '<S15>/Matrix Divide'

      i_0 = 0;
      i = 0;
      for (Sum1_tmp = 0; Sum1_tmp < 6; Sum1_tmp++) {
        for (i_2 = 0; i_2 < 6; i_2++) {
          rtb_Switch1_l = 0.0;
          rtb_MatrixMultiply2_f_tmp = 0;
          for (i_3 = 0; i_3 < 6; i_3++) {
            rtb_Switch1_l += rtb_UnitDelay1[rtb_MatrixMultiply2_f_tmp + i_2] *
              tmp[i_3 + i_0];
            rtb_MatrixMultiply2_f_tmp += 6;
          }

          rtb_MatrixMultiply2_f_tmp = i_2 + i_0;
          rtb_UnitDelay1_0[rtb_MatrixMultiply2_f_tmp] = rtb_Switch1_l;
          rtb_MatrixMultiply2_k3[rtb_MatrixMultiply2_f_tmp] =
            (rtb_MatrixConcatenate1_b[i + 1] * rtb_MatrixDivide_o[i_2 + 6] +
             rtb_MatrixConcatenate1_b[i] * rtb_MatrixDivide_o[i_2]) +
            rtb_MatrixConcatenate1_b[i + 2] * rtb_MatrixDivide_o[i_2 + 12];
        }

        i_0 += 6;
        i += 3;
      }

      for (i_0 = 0; i_0 < 36; i_0++) {
        // Merge: '<S2>/Merge3' incorporates:
        //   Sum: '<S11>/Sum1'

        ANAS0_DW.Merge3[i_0] = rtb_UnitDelay1_0[i_0] +
          rtb_MatrixMultiply2_k3[i_0];
      }

      // Sqrt: '<S28>/Sqrt' incorporates:
      //   DotProduct: '<S28>/Dot Product'
      //   Inport: '<Root>/ANAS In'

      rtb_Sqrt = std::sqrt((ANAS0_U.ANASIn_j.MagMeasure[0] *
                            ANAS0_U.ANASIn_j.MagMeasure[0] +
                            ANAS0_U.ANASIn_j.MagMeasure[1] *
                            ANAS0_U.ANASIn_j.MagMeasure[1]) +
                           ANAS0_U.ANASIn_j.MagMeasure[2] *
                           ANAS0_U.ANASIn_j.MagMeasure[2]);

      // Switch: '<S19>/Switch' incorporates:
      //   Inport: '<Root>/ANAS In'
      //   Product: '<S19>/Divide'
      //   Sum: '<S19>/Subtract'

      if (rtb_Sqrt > ANAS0_P.Switch_Threshold) {
        rtb_Switch_op_idx_0 = ANAS0_U.ANASIn_j.MagMeasure[0] / rtb_Sqrt;
        rtb_Switch_op_idx_1 = ANAS0_U.ANASIn_j.MagMeasure[1] / rtb_Sqrt;
        rtb_Sqrt = ANAS0_U.ANASIn_j.MagMeasure[2] / rtb_Sqrt;
      } else {
        rtb_Switch_op_idx_0 = 0.0F;
        rtb_Switch_op_idx_1 = 0.0F;
        rtb_Sqrt = 0.0F;
      }

      // End of Switch: '<S19>/Switch'

      // Sum: '<S12>/Subtract'
      rtb_Switch1_l = rtb_Switch_op_idx_0 - rtb_VectorConcatenate_d[0];
      rtb_Power = rtb_Switch_op_idx_1 - rtb_VectorConcatenate_d[1];
      rtb_Divide1_a = rtb_Sqrt - rtb_VectorConcatenate_d[2];

      // Product: '<S12>/Matrix Multiply1' incorporates:
      //   Product: '<S15>/Matrix Divide'

      for (i_0 = 0; i_0 < 6; i_0++) {
        rtb_MatrixMultiply2_n[i_0] = (rtb_MatrixDivide_o[i_0 + 6] * rtb_Power +
          rtb_MatrixDivide_o[i_0] * rtb_Switch1_l) + rtb_MatrixDivide_o[i_0 + 12]
          * rtb_Divide1_a;
      }

      // End of Product: '<S12>/Matrix Multiply1'

      // Gain: '<S18>/Gain' incorporates:
      //   Concatenate: '<S97>/Vector Concatenate'
      //   DotProduct: '<S18>/Dot Product'

      rtb_VectorConcatenate_d[0] = ANAS0_P.Gain_Gain_p * rtb_MatrixMultiply2_n[0];
      rtb_VectorConcatenate_d[1] = ANAS0_P.Gain_Gain_p * rtb_MatrixMultiply2_n[1];
      rtb_VectorConcatenate_d[2] = ANAS0_P.Gain_Gain_p * rtb_MatrixMultiply2_n[2];

      // Sqrt: '<S18>/Sqrt' incorporates:
      //   Constant: '<S18>/Constant'
      //   DotProduct: '<S18>/Dot Product'
      //   Gain: '<S18>/Gain1'
      //   Sum: '<S18>/Subtract'

      rtb_Switch1_l = std::sqrt(ANAS0_P.Constant_Value_p -
        ((rtb_MatrixMultiply2_n[0] * rtb_MatrixMultiply2_n[0] +
          rtb_MatrixMultiply2_n[1] * rtb_MatrixMultiply2_n[1]) +
         rtb_MatrixMultiply2_n[2] * rtb_MatrixMultiply2_n[2]) *
        ANAS0_P.Gain1_Gain_b);

      // Sqrt: '<S25>/sqrt' incorporates:
      //   Product: '<S26>/Product'
      //   Product: '<S26>/Product1'
      //   Product: '<S26>/Product2'
      //   Product: '<S26>/Product3'
      //   Sum: '<S26>/Sum'

      rtb_Divide1_a = std::sqrt(((rtb_VectorConcatenate_d[0] *
        rtb_VectorConcatenate_d[0] + rtb_VectorConcatenate_d[1] *
        rtb_VectorConcatenate_d[1]) + rtb_VectorConcatenate_d[2] *
        rtb_VectorConcatenate_d[2]) + rtb_Switch1_l * rtb_Switch1_l);

      // Product: '<S16>/Product3'
      rtb_Switch1_l /= rtb_Divide1_a;

      // Reshape: '<S14>/Reshape' incorporates:
      //   Concatenate: '<S97>/Vector Concatenate'
      //   Product: '<S16>/Product'
      //   Product: '<S16>/Product1'
      //   Product: '<S16>/Product2'

      rtb_VectorConcatenate_d[0] /= rtb_Divide1_a;
      rtb_VectorConcatenate_d[1] /= rtb_Divide1_a;
      rtb_VectorConcatenate_d[2] /= rtb_Divide1_a;

      // Sum: '<S14>/Subtract' incorporates:
      //   Concatenate: '<S97>/Vector Concatenate'
      //   Product: '<S14>/Matrix Multiply'
      //   Product: '<S14>/Matrix Multiply1'
      //   Product: '<S21>/Element Product'
      //   Reshape: '<S14>/Reshape1'
      //   Sum: '<S21>/Sum'

      rtb_MatrixConcatenate_a[0] = (rtb_Switch1_l * ANAS0_DW.Product1 +
        ANAS0_DW.Product * rtb_VectorConcatenate_d[0]) -
        (rtb_VectorConcatenate_d[1] * ANAS0_DW.Product3 - ANAS0_DW.Product2 *
         rtb_VectorConcatenate_d[2]);

      // Sum: '<S12>/Sum' incorporates:
      //   Merge: '<S2>/Merge2'

      ANAS0_DW.Merge2[4] = rtb_MatrixMultiply2_n[3] + ANAS0_DW.prevAngularState
        [4];

      // Sum: '<S14>/Subtract' incorporates:
      //   Concatenate: '<S97>/Vector Concatenate'
      //   Product: '<S14>/Matrix Multiply'
      //   Product: '<S14>/Matrix Multiply1'
      //   Product: '<S21>/Element Product'
      //   Reshape: '<S14>/Reshape1'
      //   Sum: '<S21>/Sum'

      rtb_MatrixConcatenate_a[1] = (rtb_Switch1_l * ANAS0_DW.Product2 +
        ANAS0_DW.Product * rtb_VectorConcatenate_d[1]) - (ANAS0_DW.Product1 *
        rtb_VectorConcatenate_d[2] - rtb_VectorConcatenate_d[0] *
        ANAS0_DW.Product3);

      // Sum: '<S12>/Sum' incorporates:
      //   Merge: '<S2>/Merge2'

      ANAS0_DW.Merge2[5] = rtb_MatrixMultiply2_n[4] + ANAS0_DW.prevAngularState
        [5];

      // Sum: '<S14>/Subtract' incorporates:
      //   Concatenate: '<S97>/Vector Concatenate'
      //   Product: '<S14>/Matrix Multiply'
      //   Product: '<S14>/Matrix Multiply1'
      //   Product: '<S21>/Element Product'
      //   Reshape: '<S14>/Reshape1'
      //   Sum: '<S21>/Sum'

      rtb_MatrixConcatenate_a[2] = (rtb_Switch1_l * ANAS0_DW.Product3 +
        ANAS0_DW.Product * rtb_VectorConcatenate_d[2]) -
        (rtb_VectorConcatenate_d[0] * ANAS0_DW.Product2 - ANAS0_DW.Product1 *
         rtb_VectorConcatenate_d[1]);

      // Sum: '<S12>/Sum' incorporates:
      //   Merge: '<S2>/Merge2'

      ANAS0_DW.Merge2[6] = rtb_MatrixMultiply2_n[5] + ANAS0_DW.prevAngularState
        [6];

      // Sum: '<S14>/Subtract1' incorporates:
      //   Concatenate: '<S97>/Vector Concatenate'
      //   DotProduct: '<S14>/Dot Product'
      //   Product: '<S14>/Matrix Multiply1'
      //   Product: '<S14>/Matrix Multiply2'
      //   Reshape: '<S14>/Reshape1'

      rtb_MatrixConcatenate_a[3] = rtb_Switch1_l * ANAS0_DW.Product -
        ((rtb_VectorConcatenate_d[0] * ANAS0_DW.Product1 +
          rtb_VectorConcatenate_d[1] * ANAS0_DW.Product2) +
         rtb_VectorConcatenate_d[2] * ANAS0_DW.Product3);

      // Sqrt: '<S23>/sqrt' incorporates:
      //   Product: '<S24>/Product'
      //   Product: '<S24>/Product1'
      //   Product: '<S24>/Product2'
      //   Product: '<S24>/Product3'
      //   Sum: '<S24>/Sum'

      rtb_Switch1_l = std::sqrt(((rtb_MatrixConcatenate_a[0] *
        rtb_MatrixConcatenate_a[0] + rtb_MatrixConcatenate_a[1] *
        rtb_MatrixConcatenate_a[1]) + rtb_MatrixConcatenate_a[2] *
        rtb_MatrixConcatenate_a[2]) + rtb_MatrixConcatenate_a[3] *
        rtb_MatrixConcatenate_a[3]);

      // SignalConversion generated from: '<S12>/Vector Concatenate' incorporates:
      //   Merge: '<S2>/Merge2'
      //   Product: '<S22>/Product'
      //   Product: '<S22>/Product1'
      //   Product: '<S22>/Product2'

      ANAS0_DW.Merge2[1] = rtb_MatrixConcatenate_a[0] / rtb_Switch1_l;
      ANAS0_DW.Merge2[2] = rtb_MatrixConcatenate_a[1] / rtb_Switch1_l;
      ANAS0_DW.Merge2[3] = rtb_MatrixConcatenate_a[2] / rtb_Switch1_l;

      // Product: '<S22>/Product3' incorporates:
      //   Merge: '<S2>/Merge2'

      ANAS0_DW.Merge2[0] = rtb_MatrixConcatenate_a[3] / rtb_Switch1_l;
    }

    // End of Outputs for SubSystem: '<S2>/Attitude corrector'

    // RateTransition: '<S2>/Rate Transition1' incorporates:
    //   Logic: '<S2>/NOT1'

    if (rtb_NOT1_h) {
      // Outputs for Enabled SubSystem: '<S2>/Subsystem1' incorporates:
      //   EnablePort: '<S9>/Enable'

      if (Gain == 0.0) {
        // Merge: '<S2>/Merge2' incorporates:
        //   SignalConversion generated from: '<S9>/nextAngularState'

        ANAS0_DW.Merge2[0] = ANAS0_DW.Product;
        ANAS0_DW.Merge2[1] = ANAS0_DW.Product1;
        ANAS0_DW.Merge2[2] = ANAS0_DW.Product2;
        ANAS0_DW.Merge2[3] = ANAS0_DW.Product3;
        ANAS0_DW.Merge2[4] = ANAS0_DW.prevAngularState[4];
        ANAS0_DW.Merge2[5] = ANAS0_DW.prevAngularState[5];
        ANAS0_DW.Merge2[6] = ANAS0_DW.prevAngularState[6];

        // Merge: '<S2>/Merge3' incorporates:
        //   SignalConversion generated from: '<S9>/nextAngularCov'
        //   Sum: '<S46>/Sum1'

        std::memcpy(&ANAS0_DW.Merge3[0], &ANAS0_DW.Sum1_a[0], 36U * sizeof
                    (double));
      }

      // End of Outputs for SubSystem: '<S2>/Subsystem1'
    }

    // Switch: '<S64>/Switch' incorporates:
    //   Clock: '<S64>/Clock2'
    //   Constant: '<S64>/Constant3'
    //   Constant: '<S64>/Constant4'
    //   Constant: '<S71>/Constant'
    //   Constant: '<S73>/Constant'
    //   DataStoreWrite: '<S64>/Data Store Write'
    //   Math: '<S64>/Math Function1'
    //   RelationalOperator: '<S71>/Compare'
    //   RelationalOperator: '<S73>/Compare'
    //   Switch: '<S64>/OutputSwitch2'

    if (ANAS0_DW.ProbeSampleTime1[0] > ANAS0_P.CompareToConstant1_const_n) {
      // Switch: '<S64>/OutputSwitch1' incorporates:
      //   Clock: '<S64>/Clock1'
      //   Constant: '<S64>/Constant'
      //   Constant: '<S64>/Constant1'
      //   Constant: '<S72>/Constant'
      //   Math: '<S64>/Math Function'
      //   RelationalOperator: '<S72>/Compare'

      if (rt_remd((&ANAS0_M)->Timing.t[0], ANAS0_DW.ProbeSampleTime1[0]) ==
          ANAS0_P.Constant_Value_b2) {
        rtb_Switch1_l = ANAS0_P.Constant_Value_es;
      } else {
        rtb_Switch1_l = ANAS0_P.Constant1_Value_nf;
      }

      // End of Switch: '<S64>/OutputSwitch1'
    } else if (rt_remd((&ANAS0_M)->Timing.t[0], ANAS0_P.Constant4_Value_d) ==
               ANAS0_P.Constant_Value_bh) {
      // Switch: '<S64>/OutputSwitch2' incorporates:
      //   Constant: '<S64>/Constant2'

      rtb_Switch1_l = ANAS0_P.Constant2_Value_j;
    } else {
      rtb_Switch1_l = ANAS0_P.Constant3_Value_i;
    }

    // End of Switch: '<S64>/Switch'

    // Gain: '<S64>/Gain'
    rtb_Power = ANAS0_P.Gain_Gain_j * rtb_Switch1_l;

    // RateTransition: '<S64>/Rate Transition1'
    if (rtb_NOT1_h) {
      // UnitDelay: '<S1>/Unit Delay4'
      rtb_UnitDelay4 = ANAS0_DW.UnitDelay4_DSTATE;

      // Switch: '<S1>/Switch4' incorporates:
      //   UnitDelay: '<S1>/Unit Delay2'
      //   UnitDelay: '<S1>/Unit Delay4'

      if (ANAS0_DW.UnitDelay4_DSTATE != 0) {
        for (i_0 = 0; i_0 < 6; i_0++) {
          rtb_Switch4[i_0] = ANAS0_DW.UnitDelay2_DSTATE[i_0];
        }
      } else {
        // SignalConversion generated from: '<S1>/Vector Concatenate2' incorporates:
        //   Inport: '<Root>/ANAS Reference In'

        rtb_VectorConcatenate2[3] = ANAS0_U.ANASReferenceIn.InitialVelocity[0];

        // SignalConversion generated from: '<S1>/Vector Concatenate2' incorporates:
        //   Inport: '<Root>/ANAS Reference In'

        rtb_VectorConcatenate2[0] = ANAS0_U.ANASReferenceIn.InitialPosition[0];

        // SignalConversion generated from: '<S1>/Vector Concatenate2' incorporates:
        //   Inport: '<Root>/ANAS Reference In'

        rtb_VectorConcatenate2[4] = ANAS0_U.ANASReferenceIn.InitialVelocity[1];

        // SignalConversion generated from: '<S1>/Vector Concatenate2' incorporates:
        //   Inport: '<Root>/ANAS Reference In'

        rtb_VectorConcatenate2[1] = ANAS0_U.ANASReferenceIn.InitialPosition[1];

        // SignalConversion generated from: '<S1>/Vector Concatenate2' incorporates:
        //   Inport: '<Root>/ANAS Reference In'

        rtb_VectorConcatenate2[5] = ANAS0_U.ANASReferenceIn.InitialVelocity[2];

        // SignalConversion generated from: '<S1>/Vector Concatenate2' incorporates:
        //   Inport: '<Root>/ANAS Reference In'

        rtb_VectorConcatenate2[2] = ANAS0_U.ANASReferenceIn.InitialPosition[2];
        for (i_0 = 0; i_0 < 6; i_0++) {
          rtb_Switch4[i_0] = rtb_VectorConcatenate2[i_0];
        }
      }

      // End of Switch: '<S1>/Switch4'

      // UnitDelay: '<S1>/Unit Delay5'
      std::memcpy(&rtb_UnitDelay5[0], &ANAS0_DW.UnitDelay5_DSTATE[0], 36U *
                  sizeof(double));
    }

    // Switch: '<S7>/Switch' incorporates:
    //   Clock: '<S7>/Clock2'
    //   Constant: '<S127>/Constant'
    //   Constant: '<S129>/Constant'
    //   Constant: '<S7>/Constant4'
    //   DataStoreRead: '<S7>/Data Store Read'
    //   Math: '<S7>/Math Function1'
    //   RelationalOperator: '<S127>/Compare'
    //   RelationalOperator: '<S129>/Compare'
    //   Switch: '<S7>/OutputSwitch2'

    if (ANAS0_DW.A_gb > ANAS0_P.CompareToConstant1_const_k) {
      // Switch: '<S7>/OutputSwitch1' incorporates:
      //   Clock: '<S7>/Clock1'
      //   Constant: '<S128>/Constant'
      //   Math: '<S7>/Math Function'
      //   RelationalOperator: '<S128>/Compare'

      if (rt_remd((&ANAS0_M)->Timing.t[0], ANAS0_DW.A_gb) ==
          ANAS0_P.Constant_Value_pf) {
        // Switch: '<S7>/Switch' incorporates:
        //   Constant: '<S7>/Constant'

        rtb_Switch1_l = ANAS0_P.Constant_Value_a;
      } else {
        // Switch: '<S7>/Switch' incorporates:
        //   Constant: '<S7>/Constant1'

        rtb_Switch1_l = ANAS0_P.Constant1_Value_j;
      }

      // End of Switch: '<S7>/OutputSwitch1'
    } else if (rt_remd((&ANAS0_M)->Timing.t[0], ANAS0_P.Constant4_Value_n) ==
               ANAS0_P.Constant_Value_f0) {
      // Switch: '<S7>/OutputSwitch2' incorporates:
      //   Constant: '<S7>/Constant2'
      //   Switch: '<S7>/Switch'

      rtb_Switch1_l = ANAS0_P.Constant2_Value_d;
    } else {
      // Switch: '<S7>/Switch' incorporates:
      //   Constant: '<S7>/Constant3'

      rtb_Switch1_l = ANAS0_P.Constant3_Value_i5;
    }

    // End of Switch: '<S7>/Switch'

    // Outputs for Enabled SubSystem: '<S1>/Linear predictor' incorporates:
    //   EnablePort: '<S5>/Enable'

    if ((rtb_Switch1_l > 0.0) && ((&ANAS0_M)->Timing.TaskCounters.TID[2] == 0))
    {
      // Sum: '<S108>/Sum' incorporates:
      //   Constant: '<S108>/Constant'
      //   Constant: '<S108>/Constant1'
      //   Constant: '<S5>/Constant'
      //   Product: '<S108>/Product'
      //   Product: '<S91>/Matrix Multiply'

      for (i_0 = 0; i_0 < 36; i_0++) {
        rtb_UnitDelay1[i_0] = ANAS0_P.Constant_Value_g0[i_0] *
          ANAS0_P.Constant_Value_px + ANAS0_P.Constant1_Value_a[i_0];
      }

      // End of Sum: '<S108>/Sum'
      for (i_0 = 0; i_0 < 6; i_0++) {
        // Product: '<S106>/Matrix Multiply' incorporates:
        //   Product: '<S91>/Matrix Multiply'
        //   UnitDelay: '<S1>/Unit Delay5'

        for (i = 0; i < 6; i++) {
          rtb_Switch1_l = 0.0;
          for (Sum1_tmp = 0; Sum1_tmp < 6; Sum1_tmp++) {
            rtb_Switch1_l += rtb_UnitDelay1[6 * Sum1_tmp + i_0] *
              rtb_UnitDelay5[6 * i + Sum1_tmp];
          }

          rtb_UnitDelay1_0[i_0 + 6 * i] = rtb_Switch1_l;
        }

        // End of Product: '<S106>/Matrix Multiply'
        for (i = 0; i < 6; i++) {
          // Sum: '<S106>/Sum1' incorporates:
          //   Math: '<S106>/Transpose'
          //   Product: '<S106>/Matrix Multiply1'
          //   Product: '<S91>/Matrix Multiply'

          rtb_Square_k = 0.0;
          for (Sum1_tmp = 0; Sum1_tmp < 6; Sum1_tmp++) {
            rtb_Square_k += rtb_UnitDelay1_0[6 * Sum1_tmp + i_0] *
              rtb_UnitDelay1[6 * Sum1_tmp + i];
          }

          // Sum: '<S106>/Sum1' incorporates:
          //   Constant: '<S106>/Constant'
          //   Math: '<S106>/Transpose'
          //   Product: '<S106>/Matrix Multiply1'

          Sum1_tmp = 6 * i + i_0;

          // Sum: '<S106>/Sum1' incorporates:
          //   Constant: '<S106>/Constant'

          ANAS0_DW.Sum1[Sum1_tmp] = ANAS0_P.Constant_Value_o[Sum1_tmp] +
            rtb_Square_k;
        }
      }

      // Sqrt: '<S122>/sqrt' incorporates:
      //   Product: '<S123>/Product'
      //   Product: '<S123>/Product1'
      //   Product: '<S123>/Product2'
      //   Product: '<S123>/Product3'
      //   Sum: '<S123>/Sum'

      Gain = std::sqrt(((rtb_Switch3[0] * rtb_Switch3[0] + rtb_Switch3[1] *
                         rtb_Switch3[1]) + rtb_Switch3[2] * rtb_Switch3[2]) +
                       rtb_Switch3[3] * rtb_Switch3[3]);

      // Product: '<S121>/Product'
      rtb_Square_k = rtb_Switch3[0] / Gain;

      // Product: '<S121>/Product1'
      rtb_Power3 = rtb_Switch3[1] / Gain;

      // Product: '<S121>/Product2'
      rtb_Divide1 = rtb_Switch3[2] / Gain;

      // Product: '<S121>/Product3'
      Gain = rtb_Switch3[3] / Gain;

      // Product: '<S111>/Product3' incorporates:
      //   Product: '<S115>/Product3'

      tmp_1 = rtb_Square_k * rtb_Square_k;

      // Product: '<S111>/Product2' incorporates:
      //   Product: '<S115>/Product2'

      rtb_AddConstant1 = rtb_Power3 * rtb_Power3;

      // Product: '<S111>/Product1' incorporates:
      //   Product: '<S115>/Product1'
      //   Product: '<S119>/Product1'

      rtb_VectorConcatenate_i_tmp_1 = rtb_Divide1 * rtb_Divide1;

      // Product: '<S111>/Product' incorporates:
      //   Product: '<S115>/Product'
      //   Product: '<S119>/Product'

      rtb_VectorConcatenate_i_tmp_2 = Gain * Gain;

      // Sum: '<S111>/Sum' incorporates:
      //   Product: '<S111>/Product'
      //   Product: '<S111>/Product1'
      //   Product: '<S111>/Product2'
      //   Product: '<S111>/Product3'

      rtb_VectorConcatenate_p[0] = ((tmp_1 + rtb_AddConstant1) -
        rtb_VectorConcatenate_i_tmp_1) - rtb_VectorConcatenate_i_tmp_2;

      // Product: '<S114>/Product3' incorporates:
      //   Product: '<S112>/Product3'

      rtb_VectorConcatenate_i_tmp = Gain * rtb_Square_k;

      // Product: '<S114>/Product2' incorporates:
      //   Product: '<S112>/Product2'

      rtb_VectorConcatenate_i_tmp_0 = rtb_Power3 * rtb_Divide1;

      // Gain: '<S114>/Gain' incorporates:
      //   Product: '<S114>/Product2'
      //   Product: '<S114>/Product3'
      //   Sum: '<S114>/Sum'

      rtb_VectorConcatenate_p[1] = (rtb_VectorConcatenate_i_tmp_0 -
        rtb_VectorConcatenate_i_tmp) * ANAS0_P.Gain_Gain_im;

      // Product: '<S117>/Product2' incorporates:
      //   Product: '<S113>/Product2'

      rtb_VectorConcatenate_i_tmp_3 = rtb_Power3 * Gain;

      // Product: '<S117>/Product1' incorporates:
      //   Product: '<S113>/Product1'

      rtb_VectorConcatenate_i_tmp_4 = rtb_Square_k * rtb_Divide1;

      // Gain: '<S117>/Gain' incorporates:
      //   Product: '<S117>/Product1'
      //   Product: '<S117>/Product2'
      //   Sum: '<S117>/Sum'

      rtb_VectorConcatenate_p[2] = (rtb_VectorConcatenate_i_tmp_4 +
        rtb_VectorConcatenate_i_tmp_3) * ANAS0_P.Gain_Gain_n;

      // Gain: '<S112>/Gain' incorporates:
      //   Sum: '<S112>/Sum'

      rtb_VectorConcatenate_p[3] = (rtb_VectorConcatenate_i_tmp +
        rtb_VectorConcatenate_i_tmp_0) * ANAS0_P.Gain_Gain_no;

      // Sum: '<S115>/Sum' incorporates:
      //   Sum: '<S119>/Sum'

      tmp_1 -= rtb_AddConstant1;
      rtb_VectorConcatenate_p[4] = (tmp_1 + rtb_VectorConcatenate_i_tmp_1) -
        rtb_VectorConcatenate_i_tmp_2;

      // Product: '<S118>/Product1' incorporates:
      //   Product: '<S116>/Product1'

      rtb_AddConstant1 = rtb_Square_k * rtb_Power3;

      // Product: '<S118>/Product2' incorporates:
      //   Product: '<S116>/Product2'

      rtb_VectorConcatenate_i_tmp = rtb_Divide1 * Gain;

      // Gain: '<S118>/Gain' incorporates:
      //   Product: '<S118>/Product1'
      //   Product: '<S118>/Product2'
      //   Sum: '<S118>/Sum'

      rtb_VectorConcatenate_p[5] = (rtb_VectorConcatenate_i_tmp -
        rtb_AddConstant1) * ANAS0_P.Gain_Gain_ec;

      // Gain: '<S113>/Gain' incorporates:
      //   Sum: '<S113>/Sum'

      rtb_VectorConcatenate_p[6] = (rtb_VectorConcatenate_i_tmp_3 -
        rtb_VectorConcatenate_i_tmp_4) * ANAS0_P.Gain_Gain_ij;

      // Gain: '<S116>/Gain' incorporates:
      //   Sum: '<S116>/Sum'

      rtb_VectorConcatenate_p[7] = (rtb_AddConstant1 +
        rtb_VectorConcatenate_i_tmp) * ANAS0_P.Gain_Gain_ih;

      // Sum: '<S119>/Sum'
      rtb_VectorConcatenate_p[8] = (tmp_1 - rtb_VectorConcatenate_i_tmp_1) +
        rtb_VectorConcatenate_i_tmp_2;

      // Product: '<S109>/Matrix Multiply' incorporates:
      //   Inport: '<Root>/ANAS In'
      //   Math: '<S109>/Transpose'

      rtb_Switch_op_idx_0 = ANAS0_U.ANASIn_j.AccMeasure[1];
      rtb_Switch_op_idx_1 = ANAS0_U.ANASIn_j.AccMeasure[0];
      rtb_Sqrt = ANAS0_U.ANASIn_j.AccMeasure[2];

      // Sum: '<S107>/Sum' incorporates:
      //   Concatenate: '<S120>/Vector Concatenate'
      //   Constant: '<S109>/Constant'
      //   Constant: '<S5>/Constant'
      //   Math: '<S109>/Transpose'
      //   Product: '<S107>/Product'
      //   Product: '<S107>/Product1'
      //   Product: '<S109>/Matrix Multiply'
      //   Sum: '<S107>/Sum1'
      //   Sum: '<S109>/Sum'

      i_0 = 0;
      for (i = 0; i < 3; i++) {
        // Product: '<S107>/Product'
        Gain = rtb_Switch4[i + 3];
        ANAS0_DW.Sum[i] = ANAS0_P.Constant_Value_px * Gain + rtb_Switch4[i];
        ANAS0_DW.Sum1_j[i] = (((rtb_VectorConcatenate_p[i_0 + 1] *
          rtb_Switch_op_idx_0 + rtb_VectorConcatenate_p[i_0] *
          rtb_Switch_op_idx_1) + rtb_VectorConcatenate_p[i_0 + 2] * rtb_Sqrt) +
                              ANAS0_P.Constant_Value_dm[i]) *
          ANAS0_P.Constant_Value_px + Gain;
        i_0 += 3;
      }

      // End of Sum: '<S107>/Sum'
    }

    // End of Outputs for SubSystem: '<S1>/Linear predictor'

    // Switch: '<S76>/Switch' incorporates:
    //   Clock: '<S76>/Clock2'
    //   Constant: '<S76>/Constant3'
    //   Constant: '<S76>/Constant4'
    //   Constant: '<S85>/Constant'
    //   Constant: '<S87>/Constant'
    //   DataStoreRead: '<S76>/Data Store Read'
    //   Math: '<S76>/Math Function1'
    //   RelationalOperator: '<S85>/Compare'
    //   RelationalOperator: '<S87>/Compare'
    //   Switch: '<S76>/OutputSwitch2'

    if (ANAS0_DW.A_b > ANAS0_P.CompareToConstant1_const_p) {
      // Switch: '<S76>/OutputSwitch1' incorporates:
      //   Clock: '<S76>/Clock1'
      //   Constant: '<S76>/Constant'
      //   Constant: '<S76>/Constant1'
      //   Constant: '<S86>/Constant'
      //   Math: '<S76>/Math Function'
      //   RelationalOperator: '<S86>/Compare'

      if (rt_remd((&ANAS0_M)->Timing.t[0], ANAS0_DW.A_b) ==
          ANAS0_P.Constant_Value_fs) {
        rtb_Switch1_l = ANAS0_P.Constant_Value_gf;
      } else {
        rtb_Switch1_l = ANAS0_P.Constant1_Value_or;
      }

      // End of Switch: '<S76>/OutputSwitch1'
    } else if (rt_remd((&ANAS0_M)->Timing.t[0], ANAS0_P.Constant4_Value_b) ==
               ANAS0_P.Constant_Value_j) {
      // Switch: '<S76>/OutputSwitch2' incorporates:
      //   Constant: '<S76>/Constant2'

      rtb_Switch1_l = ANAS0_P.Constant2_Value_i0;
    } else {
      rtb_Switch1_l = ANAS0_P.Constant3_Value_d;
    }

    // End of Switch: '<S76>/Switch'

    // RateTransition: '<S60>/Rate Transition' incorporates:
    //   RateTransition: '<S76>/Rate Transition1'

    if (rtb_NOT1_h) {
      // Logic: '<S60>/NOT' incorporates:
      //   Gain: '<S76>/Gain'

      rtb_NOT = (ANAS0_P.Gain_Gain_en * rtb_Switch1_l == 0.0);

      // Logic: '<S60>/NOT1'
      rtb_NOT1_n = !rtb_NOT;

      // Outputs for Enabled SubSystem: '<S60>/State and Covariance' incorporates:
      //   EnablePort: '<S74>/Enable'

      if (rtb_NOT1_n) {
        // Sum: '<S74>/Sum1' incorporates:
        //   Constant: '<S74>/Constant2'
        //   Gain: '<S74>/Gain2'

        rtb_Switch1_l = ANAS0_P.Gain2_Gain_g * ANAS0_DW.Sum[0] +
          ANAS0_P.Constant2_Value_m;

        // Gain: '<S84>/Gain1' incorporates:
        //   Constant: '<S80>/Constant2'
        //   Gain: '<S80>/Gain'
        //   Sum: '<S80>/Sum'

        rtb_Divide1_a = (ANAS0_DW.Sum[0] + ANAS0_P.Constant2_Value_o) *
          ANAS0_P.Gain_Gain_c4 * ANAS0_P.Gain1_Gain_i;

        // Trigonometry: '<S80>/Cos' incorporates:
        //   Trigonometry: '<S80>/Cos1'

        rtb_Square_k = std::cos(rtb_Divide1_a);

        // Gain: '<S80>/Gain3' incorporates:
        //   Constant: '<S80>/Constant'
        //   Constant: '<S80>/Constant1'
        //   Constant: '<S80>/Constant3'
        //   Constant: '<S80>/Constant4'
        //   Gain: '<S80>/Gain1'
        //   Gain: '<S80>/Gain2'
        //   Math: '<S80>/Square'
        //   Product: '<S80>/Divide'
        //   Product: '<S80>/Divide1'
        //   Product: '<S80>/Product'
        //   Product: '<S80>/Product1'
        //   Trigonometry: '<S80>/Cos'
        //   Trigonometry: '<S80>/Sin'

        rtb_Transpose1_l[0] = ANAS0_P.Gain3_Gain * ANAS0_P.Constant_Value_fh;
        rtb_Transpose1_l[1] = ANAS0_DW.Sum[1] * std::sin(rtb_Divide1_a) /
          (ANAS0_P.Gain2_Gain_j * ANAS0_P.Constant3_Value_e * (rtb_Square_k *
            rtb_Square_k)) * ANAS0_P.Gain3_Gain;
        rtb_Transpose1_l[2] = ANAS0_P.Gain3_Gain * ANAS0_P.Constant1_Value_p;
        rtb_Transpose1_l[3] = ANAS0_P.Constant4_Value / (ANAS0_P.Gain1_Gain_g *
          rtb_Square_k) * ANAS0_P.Gain3_Gain;
        std::memcpy(&rtb_Transpose1_l[4], &ANAS0_P.Constant5_Value[0], sizeof
                    (double) << 3U);

        // Concatenate: '<S80>/Matrix Concatenate2' incorporates:
        //   Constant: '<S80>/Constant5'
        //   Constant: '<S80>/Constant6'
        //   Math: '<S78>/Transpose1'
        //   Math: '<S91>/Transpose1'

        i_0 = 0;
        i = 0;
        for (Sum1_tmp = 0; Sum1_tmp < 6; Sum1_tmp++) {
          rtb_Transpose1[i_0] = rtb_Transpose1_l[i];
          rtb_Transpose1[i_0 + 2] = ANAS0_P.Constant6_Value[i];
          rtb_Transpose1[i_0 + 1] = rtb_Transpose1_l[i + 1];
          rtb_Transpose1[i_0 + 3] = ANAS0_P.Constant6_Value[i + 1];
          i_0 += 4;
          i += 2;
        }

        // End of Concatenate: '<S80>/Matrix Concatenate2'

        // Math: '<S82>/Square' incorporates:
        //   Constant: '<S82>/Constant'

        for (i_0 = 0; i_0 < 16; i_0++) {
          Gain = ANAS0_P.Constant_Value_m2[i_0];
          rtb_Square_er[i_0] = Gain * Gain;
        }

        // End of Math: '<S82>/Square'
        for (i_0 = 0; i_0 < 6; i_0++) {
          // Product: '<S81>/Matrix Multiply' incorporates:
          //   Math: '<S78>/Transpose1'
          //   Math: '<S81>/Transpose1'
          //   Product: '<S81>/Matrix Multiply1'
          //   Sum: '<S106>/Sum1'

          i = 0;
          for (Sum1_tmp = 0; Sum1_tmp < 4; Sum1_tmp++) {
            Gain = 0.0;
            i_2 = 0;
            rtb_MatrixMultiply2_f_tmp = 0;
            for (i_3 = 0; i_3 < 6; i_3++) {
              Gain += ANAS0_DW.Sum1[i_2 + i_0] *
                rtb_Transpose1[rtb_MatrixMultiply2_f_tmp + Sum1_tmp];
              i_2 += 6;
              rtb_MatrixMultiply2_f_tmp += 4;
            }

            rtb_MatrixDivide[i + i_0] = Gain;
            i += 6;
          }
        }

        // Sum: '<S81>/Sum' incorporates:
        //   Math: '<S78>/Transpose1'
        //   Math: '<S82>/Square'
        //   Product: '<S81>/Matrix Multiply'

        for (i_0 = 0; i_0 < 4; i_0++) {
          i = 0;
          Sum1_tmp = 0;
          for (i_2 = 0; i_2 < 4; i_2++) {
            rtb_Square_k = 0.0;
            rtb_MatrixMultiply2_f_tmp = 0;
            for (i_3 = 0; i_3 < 6; i_3++) {
              rtb_Square_k += rtb_Transpose1[rtb_MatrixMultiply2_f_tmp + i_0] *
                rtb_MatrixDivide[i_3 + Sum1_tmp];
              rtb_MatrixMultiply2_f_tmp += 4;
            }

            rtb_MatrixMultiply2_f_tmp = i + i_0;
            rtb_Transpose1_0[rtb_MatrixMultiply2_f_tmp] =
              rtb_Square_er[rtb_MatrixMultiply2_f_tmp] + rtb_Square_k;
            i += 4;
            Sum1_tmp += 6;
          }
        }

        // End of Sum: '<S81>/Sum'

        // Product: '<S81>/Matrix Divide'
        rt_mrdivide_U1d6x4_U2d4x4_Yd6x4(rtb_MatrixDivide, rtb_Transpose1_0);

        // Sum: '<S83>/Subtract' incorporates:
        //   Constant: '<S83>/Constant'
        //   Math: '<S78>/Transpose1'
        //   Product: '<S83>/Matrix Multiply'
        //   Product: '<S91>/Matrix Multiply2'

        for (i_0 = 0; i_0 < 6; i_0++) {
          // Product: '<S83>/Matrix Multiply' incorporates:
          //   Product: '<S81>/Matrix Divide'

          Gain = rtb_MatrixDivide[i_0 + 6];
          rtb_Divide1_a = rtb_MatrixDivide[i_0];
          tmp_1 = rtb_MatrixDivide[i_0 + 12];
          rtb_Square_k = rtb_MatrixDivide[i_0 + 18];
          i = 0;
          Sum1_tmp = 0;
          for (i_2 = 0; i_2 < 6; i_2++) {
            rtb_MatrixMultiply2_f_tmp = i + i_0;
            rtb_MatrixMultiply2_k3[rtb_MatrixMultiply2_f_tmp] =
              ANAS0_P.Constant_Value_f[rtb_MatrixMultiply2_f_tmp] -
              (((rtb_Transpose1[Sum1_tmp + 1] * Gain + rtb_Transpose1[Sum1_tmp] *
                 rtb_Divide1_a) + rtb_Transpose1[Sum1_tmp + 2] * tmp_1) +
               rtb_Transpose1[Sum1_tmp + 3] * rtb_Square_k);
            i += 6;
            Sum1_tmp += 4;
          }
        }

        // End of Sum: '<S83>/Subtract'

        // Product: '<S78>/Matrix Multiply' incorporates:
        //   Math: '<S78>/Transpose'
        //   Product: '<S91>/Matrix Multiply2'
        //   Sum: '<S106>/Sum1'

        for (i_0 = 0; i_0 < 6; i_0++) {
          i = 0;
          for (Sum1_tmp = 0; Sum1_tmp < 6; Sum1_tmp++) {
            rtb_Square_k = 0.0;
            i_2 = 0;
            for (rtb_MatrixMultiply2_f_tmp = 0; rtb_MatrixMultiply2_f_tmp < 6;
                 rtb_MatrixMultiply2_f_tmp++) {
              rtb_Square_k += ANAS0_DW.Sum1[i_2 + i_0] *
                rtb_MatrixMultiply2_k3[i_2 + Sum1_tmp];
              i_2 += 6;
            }

            tmp[i + i_0] = rtb_Square_k;
            i += 6;
          }
        }

        // Product: '<S78>/Matrix Multiply2' incorporates:
        //   Math: '<S78>/Transpose1'
        //   Math: '<S82>/Square'
        //   Product: '<S81>/Matrix Divide'

        for (i_0 = 0; i_0 < 4; i_0++) {
          Gain = rtb_Square_er[i_0 + 4];
          rtb_Divide1_a = rtb_Square_er[i_0];
          tmp_1 = rtb_Square_er[i_0 + 8];
          rtb_Square_k = rtb_Square_er[i_0 + 12];
          i = 0;
          for (Sum1_tmp = 0; Sum1_tmp < 6; Sum1_tmp++) {
            rtb_Transpose1[i + i_0] = ((rtb_MatrixDivide[Sum1_tmp + 6] * Gain +
              rtb_Divide1_a * rtb_MatrixDivide[Sum1_tmp]) +
              rtb_MatrixDivide[Sum1_tmp + 12] * tmp_1) +
              rtb_MatrixDivide[Sum1_tmp + 18] * rtb_Square_k;
            i += 4;
          }
        }

        // Product: '<S78>/Matrix Multiply' incorporates:
        //   Product: '<S78>/Matrix Multiply2'
        //   Product: '<S81>/Matrix Divide'
        //   Product: '<S91>/Matrix Multiply2'

        i_0 = 0;
        i = 0;
        for (Sum1_tmp = 0; Sum1_tmp < 6; Sum1_tmp++) {
          for (i_2 = 0; i_2 < 6; i_2++) {
            rtb_Power3 = 0.0;
            rtb_MatrixMultiply2_f_tmp = 0;
            for (i_3 = 0; i_3 < 6; i_3++) {
              rtb_Power3 += rtb_MatrixMultiply2_k3[rtb_MatrixMultiply2_f_tmp +
                i_2] * tmp[i_3 + i_0];
              rtb_MatrixMultiply2_f_tmp += 6;
            }

            rtb_MatrixMultiply2_f_tmp = i_2 + i_0;
            rtb_UnitDelay1[rtb_MatrixMultiply2_f_tmp] = rtb_Power3;
            rtb_UnitDelay5[rtb_MatrixMultiply2_f_tmp] = ((rtb_Transpose1[i + 1] *
              rtb_MatrixDivide[i_2 + 6] + rtb_Transpose1[i] *
              rtb_MatrixDivide[i_2]) + rtb_Transpose1[i + 2] *
              rtb_MatrixDivide[i_2 + 12]) + rtb_Transpose1[i + 3] *
              rtb_MatrixDivide[i_2 + 18];
          }

          i_0 += 6;
          i += 4;
        }

        for (i_0 = 0; i_0 < 36; i_0++) {
          // Merge: '<S60>/Merge1' incorporates:
          //   Sum: '<S78>/Sum1'

          ANAS0_DW.Merge1[i_0] = rtb_UnitDelay1[i_0] + rtb_UnitDelay5[i_0];
        }

        // Gain: '<S74>/Gain' incorporates:
        //   Constant: '<S74>/Constant1'
        //   Gain: '<S74>/Gain1'
        //   Gain: '<S79>/Gain1'
        //   Inport: '<Root>/ANAS In'
        //   Product: '<S74>/Divide'
        //   Sum: '<S74>/Sum2'
        //   Sum: '<S74>/Sum3'
        //   Trigonometry: '<S74>/Cos'

        rtb_Divide1_a = (ANAS0_U.ANASIn_j.GPSMeasure[0] - rtb_Switch1_l) *
          ANAS0_P.Gain_Gain_on[0];
        rtb_Switch1_l = (ANAS0_U.ANASIn_j.GPSMeasure[1] - (ANAS0_DW.Sum[1] /
          (std::cos(ANAS0_P.Gain1_Gain_bu * rtb_Switch1_l) *
           ANAS0_P.Gain1_Gain_j) + ANAS0_P.Constant1_Value_e)) *
          ANAS0_P.Gain_Gain_on[1];
        tmp_1 = (ANAS0_U.ANASIn_j.GPSMeasure[2] - ANAS0_DW.Sum1_j[0]) *
          ANAS0_P.Gain_Gain_on[2];
        rtb_Square_k = (ANAS0_U.ANASIn_j.GPSMeasure[3] - ANAS0_DW.Sum1_j[1]) *
          ANAS0_P.Gain_Gain_on[3];

        // Product: '<S74>/Matrix Multiply2' incorporates:
        //   Product: '<S81>/Matrix Divide'

        for (i_0 = 0; i_0 < 6; i_0++) {
          rtb_MatrixMultiply2_b[i_0] = ((rtb_MatrixDivide[i_0 + 6] *
            rtb_Switch1_l + rtb_MatrixDivide[i_0] * rtb_Divide1_a) +
            rtb_MatrixDivide[i_0 + 12] * tmp_1) + rtb_MatrixDivide[i_0 + 18] *
            rtb_Square_k;
        }

        // End of Product: '<S74>/Matrix Multiply2'

        // Merge: '<S60>/Merge' incorporates:
        //   Sum: '<S74>/Sum4'

        ANAS0_DW.Merge[0] = ANAS0_DW.Sum[0] + rtb_MatrixMultiply2_b[0];
        ANAS0_DW.Merge[3] = ANAS0_DW.Sum1_j[0] + rtb_MatrixMultiply2_b[3];
        ANAS0_DW.Merge[1] = ANAS0_DW.Sum[1] + rtb_MatrixMultiply2_b[1];
        ANAS0_DW.Merge[4] = ANAS0_DW.Sum1_j[1] + rtb_MatrixMultiply2_b[4];
        ANAS0_DW.Merge[2] = ANAS0_DW.Sum[2] + rtb_MatrixMultiply2_b[2];
        ANAS0_DW.Merge[5] = ANAS0_DW.Sum1_j[2] + rtb_MatrixMultiply2_b[5];
      }

      // End of Outputs for SubSystem: '<S60>/State and Covariance'

      // Outputs for Enabled SubSystem: '<S60>/Subsystem1' incorporates:
      //   EnablePort: '<S75>/Enable'

      if (rtb_NOT) {
        // Merge: '<S60>/Merge' incorporates:
        //   SignalConversion generated from: '<S75>/nextLinearState'

        ANAS0_DW.Merge[0] = ANAS0_DW.Sum[0];
        ANAS0_DW.Merge[3] = ANAS0_DW.Sum1_j[0];
        ANAS0_DW.Merge[1] = ANAS0_DW.Sum[1];
        ANAS0_DW.Merge[4] = ANAS0_DW.Sum1_j[1];
        ANAS0_DW.Merge[2] = ANAS0_DW.Sum[2];
        ANAS0_DW.Merge[5] = ANAS0_DW.Sum1_j[2];

        // Merge: '<S60>/Merge1' incorporates:
        //   SignalConversion generated from: '<S75>/nextLinearCov'
        //   Sum: '<S106>/Sum1'

        std::memcpy(&ANAS0_DW.Merge1[0], &ANAS0_DW.Sum1[0], 36U * sizeof(double));
      }

      // End of Outputs for SubSystem: '<S60>/Subsystem1'

      // Logic: '<S59>/NOT' incorporates:
      //   Constant: '<S80>/Constant5'
      //   RateTransition: '<S59>/Rate Transition'

      rtb_NOT_k = (rtb_Power == 0.0);

      // Logic: '<S59>/NOT1'
      rtb_NOT = !rtb_NOT_k;

      // Outputs for Enabled SubSystem: '<S59>/State and Covariance' incorporates:
      //   EnablePort: '<S62>/Enable'

      if (rtb_NOT) {
        rtb_MatrixMultiply2_b[0] = ANAS0_P.Constant_Value_bd[0];
        rtb_MatrixMultiply2_b[1] = ANAS0_P.Constant_Value_bd[1];

        // Sum: '<S66>/Sum1' incorporates:
        //   Constant: '<S66>/Constant'
        //   Gain: '<S66>/Gain4'
        //   Inport: '<Root>/ANAS Reference In'

        rtb_Divide1_a = ANAS0_P.Gain4_Gain * ANAS0_DW.Merge[2] +
          ANAS0_U.ANASReferenceIn.GroundTemperature;

        // Product: '<S66>/Product1' incorporates:
        //   Constant: '<S66>/g R'
        //   Gain: '<S66>/Gain3'
        //   Inport: '<Root>/ANAS Reference In'
        //   Math: '<S66>/Exp'
        //   Math: '<S66>/Square'
        //   Product: '<S66>/Divide2'
        //   Product: '<S66>/Divide3'
        //   Product: '<S66>/Product'
        //   Product: '<S66>/Product4'
        //
        //  About '<S66>/Exp':
        //   Operator: exp

        rtb_MatrixMultiply2_b[2] = ANAS0_U.ANASReferenceIn.GroundPressure *
          ANAS0_U.ANASReferenceIn.GroundTemperature * ANAS0_P.Gain3_Gain_l /
          (rtb_Divide1_a * rtb_Divide1_a) * std::exp(ANAS0_P.gR_Value /
          rtb_Divide1_a * ANAS0_DW.Merge[2]);
        rtb_MatrixMultiply2_b[3] = ANAS0_P.Constant1_Value[0];
        rtb_MatrixMultiply2_b[4] = ANAS0_P.Constant1_Value[1];
        rtb_MatrixMultiply2_b[5] = ANAS0_P.Constant1_Value[2];

        // Math: '<S68>/Square' incorporates:
        //   Constant: '<S66>/Constant1'
        //   Constant: '<S68>/Constant'

        rtb_Divide1_a = ANAS0_P.Constant_Value_lw * ANAS0_P.Constant_Value_lw;

        // Product: '<S67>/Matrix Multiply' incorporates:
        //   Product: '<S88>/Matrix Multiply2'

        Gain = 0.0;
        for (i_0 = 0; i_0 < 6; i_0++) {
          rtb_Power = 0.0;

          // Product: '<S67>/Matrix Multiply1' incorporates:
          //   Math: '<S67>/Transpose'
          //   Merge: '<S60>/Merge1'
          //   Product: '<S88>/Matrix Multiply2'

          i = 0;
          for (Sum1_tmp = 0; Sum1_tmp < 6; Sum1_tmp++) {
            rtb_Power += ANAS0_DW.Merge1[i + i_0] *
              rtb_MatrixMultiply2_b[Sum1_tmp];
            i += 6;
          }

          rtb_VectorConcatenate_a[i_0] = rtb_Power;
          Gain += rtb_MatrixMultiply2_b[i_0] * rtb_Power;
        }

        // Sum: '<S67>/Sum' incorporates:
        //   Product: '<S67>/Matrix Multiply'

        rtb_Switch1_l = Gain + rtb_Divide1_a;

        // Product: '<S67>/Matrix Divide' incorporates:
        //   Concatenate: '<S92>/Vector Concatenate'
        //   Product: '<S67>/Matrix Multiply1'

        for (i_0 = 0; i_0 < 6; i_0++) {
          rtb_VectorConcatenate_a[i_0] /= rtb_Switch1_l;
        }

        // End of Product: '<S67>/Matrix Divide'

        // Sum: '<S70>/Subtract' incorporates:
        //   Concatenate: '<S92>/Vector Concatenate'
        //   Constant: '<S70>/Constant'
        //   Product: '<S70>/Matrix Multiply'
        //   Product: '<S88>/Matrix Multiply2'
        //   Product: '<S91>/Matrix Multiply2'

        i_0 = 0;
        for (i = 0; i < 6; i++) {
          for (Sum1_tmp = 0; Sum1_tmp < 6; Sum1_tmp++) {
            rtb_MatrixMultiply2_f_tmp = Sum1_tmp + i_0;
            rtb_MatrixMultiply2_k3[rtb_MatrixMultiply2_f_tmp] =
              ANAS0_P.Constant_Value_b[rtb_MatrixMultiply2_f_tmp] -
              rtb_VectorConcatenate_a[Sum1_tmp] * rtb_MatrixMultiply2_b[i];
          }

          i_0 += 6;
        }

        // End of Sum: '<S70>/Subtract'

        // Product: '<S65>/Matrix Multiply' incorporates:
        //   Concatenate: '<S92>/Vector Concatenate'
        //   Math: '<S65>/Transpose'
        //   Math: '<S65>/Transpose1'
        //   Merge: '<S60>/Merge1'
        //   Product: '<S65>/Matrix Multiply2'
        //   Product: '<S91>/Matrix Multiply2'

        for (i_0 = 0; i_0 < 6; i_0++) {
          i = 0;
          for (Sum1_tmp = 0; Sum1_tmp < 6; Sum1_tmp++) {
            rtb_Square_k = 0.0;
            i_2 = 0;
            for (rtb_MatrixMultiply2_f_tmp = 0; rtb_MatrixMultiply2_f_tmp < 6;
                 rtb_MatrixMultiply2_f_tmp++) {
              rtb_Square_k += ANAS0_DW.Merge1[i_2 + i_0] *
                rtb_MatrixMultiply2_k3[i_2 + Sum1_tmp];
              i_2 += 6;
            }

            tmp[i + i_0] = rtb_Square_k;
            i += 6;
          }
        }

        i_0 = 0;
        for (i = 0; i < 6; i++) {
          Sum1_tmp = 0;
          for (i_2 = 0; i_2 < 6; i_2++) {
            rtb_Power3 = 0.0;
            rtb_MatrixMultiply2_f_tmp = 0;
            for (i_3 = 0; i_3 < 6; i_3++) {
              rtb_Power3 += rtb_MatrixMultiply2_k3[rtb_MatrixMultiply2_f_tmp + i]
                * tmp[i_3 + Sum1_tmp];
              rtb_MatrixMultiply2_f_tmp += 6;
            }

            rtb_UnitDelay1[Sum1_tmp + i] = rtb_Power3;
            rtb_UnitDelay5[i_2 + i_0] = rtb_Divide1_a *
              rtb_VectorConcatenate_a[i] * rtb_VectorConcatenate_a[i_2];
            Sum1_tmp += 6;
          }

          i_0 += 6;
        }

        // End of Product: '<S65>/Matrix Multiply'
        for (i_0 = 0; i_0 < 36; i_0++) {
          // Merge: '<S59>/Merge' incorporates:
          //   Sum: '<S65>/Sum1'

          ANAS0_DW.Merge_l[i_0] = rtb_UnitDelay1[i_0] + rtb_UnitDelay5[i_0];
        }

        // Sum: '<S62>/Sum1' incorporates:
        //   Constant: '<S69>/HeightTemperatureGradient'
        //   Constant: '<S69>/R air'
        //   Constant: '<S69>/gravity'
        //   Gain: '<S62>/Gain'
        //   Inport: '<Root>/ANAS In'
        //   Inport: '<Root>/ANAS Reference In'
        //   Math: '<S69>/Power'
        //   Product: '<S69>/Divide'
        //   Product: '<S69>/Divide1'
        //   Product: '<S69>/Product'
        //   Product: '<S69>/Product2'
        //   Product: '<S69>/Product3'
        //   Sum: '<S69>/Subtract'

        rtb_Switch1_l = ANAS0_U.ANASIn_j.BaroMeasure - std::pow
          ((ANAS0_U.ANASReferenceIn.GroundTemperature - ANAS0_P.Gain_Gain_mn *
            ANAS0_DW.Merge[2] * ANAS0_P.HeightTemperatureGradient_Value) /
           ANAS0_U.ANASReferenceIn.GroundTemperature, ANAS0_P.gravity_Value /
           (ANAS0_P.HeightTemperatureGradient_Value * ANAS0_P.Rair_Value)) *
          ANAS0_U.ANASReferenceIn.GroundPressure;
        for (i_0 = 0; i_0 < 6; i_0++) {
          // Merge: '<S59>/Merge1' incorporates:
          //   Concatenate: '<S92>/Vector Concatenate'
          //   Merge: '<S60>/Merge'
          //   Product: '<S62>/Matrix Multiply2'
          //   Sum: '<S62>/Sum4'

          ANAS0_DW.Merge1_a[i_0] = rtb_VectorConcatenate_a[i_0] * rtb_Switch1_l
            + ANAS0_DW.Merge[i_0];
        }
      }

      // End of Outputs for SubSystem: '<S59>/State and Covariance'

      // Outputs for Enabled SubSystem: '<S59>/Subsystem1'
      ANAS0_Subsystem1(rtb_NOT_k, ANAS0_DW.Merge, ANAS0_DW.Merge1,
                       ANAS0_DW.Merge1_a, ANAS0_DW.Merge_l);

      // End of Outputs for SubSystem: '<S59>/Subsystem1'

      // Bias: '<S101>/Bias' incorporates:
      //   Constant: '<S66>/Constant'
      //   Constant: '<S66>/Constant1'
      //   Gain: '<S101>/Gain'

      Gain = ANAS0_P.Gain_Gain_e3 * ANAS0_DW.Merge1_a[2] + ANAS0_P.Bias_Bias_j;

      // Saturate: '<S105>/Limit  altitude  to troposhere'
      if (Gain > ANAS0_P.Limitaltitudetotroposhere_Upper) {
        Gain = ANAS0_P.Limitaltitudetotroposhere_Upper;
      } else if (Gain < ANAS0_P.Limitaltitudetotroposhere_Lower) {
        Gain = ANAS0_P.Limitaltitudetotroposhere_Lower;
      }

      // Logic: '<S90>/NOT' incorporates:
      //   Constant: '<S102>/Constant'
      //   Constant: '<S105>/Sea Level  Temperature'
      //   DotProduct: '<S104>/Dot Product'
      //   Gain: '<S105>/Lapse Rate'
      //   Gain: '<S105>/gamma*R'
      //   Math: '<S104>/Transpose'
      //   Product: '<S101>/Divide'
      //   RelationalOperator: '<S102>/Compare'
      //   Saturate: '<S105>/Limit  altitude  to troposhere'
      //   Sqrt: '<S104>/Sqrt'
      //   Sqrt: '<S105>/a'
      //   Sum: '<S105>/Sum1'

      ANAS0_DW.NOT = (std::sqrt((ANAS0_DW.Merge1_a[3] * ANAS0_DW.Merge1_a[3] +
        ANAS0_DW.Merge1_a[4] * ANAS0_DW.Merge1_a[4]) + ANAS0_DW.Merge1_a[5] *
        ANAS0_DW.Merge1_a[5]) / std::sqrt((ANAS0_P.SeaLevelTemperature_Value -
        ANAS0_P.LapseRate_Gain * Gain) * ANAS0_P.gammaR_Gain) >
                      ANAS0_P.CompareToConstant_const);
    }

    if ((&ANAS0_M)->Timing.TaskCounters.TID[3] == 0) {
      // DataStoreWrite: '<S76>/Data Store Write'
      ANAS0_DW.A_b = ANAS0_DW.ProbeSampleTime1_b[0];
    }

    // Switch: '<S90>/Switch1' incorporates:
    //   Constant: '<S90>/Constant5'

    if (ANAS0_DW.NOT) {
      // Switch: '<S90>/Switch' incorporates:
      //   Clock: '<S90>/Clock2'
      //   Constant: '<S100>/Constant'
      //   Constant: '<S90>/Constant3'
      //   Constant: '<S90>/Constant4'
      //   Constant: '<S98>/Constant'
      //   DataStoreWrite: '<S90>/Data Store Write'
      //   Math: '<S90>/Math Function1'
      //   RelationalOperator: '<S100>/Compare'
      //   RelationalOperator: '<S98>/Compare'
      //   Switch: '<S90>/OutputSwitch2'

      if (ANAS0_DW.ProbeSampleTime1_i[0] > ANAS0_P.CompareToConstant1_const_ky)
      {
        // Switch: '<S90>/OutputSwitch1' incorporates:
        //   Clock: '<S90>/Clock1'
        //   Constant: '<S90>/Constant'
        //   Constant: '<S90>/Constant1'
        //   Constant: '<S99>/Constant'
        //   Math: '<S90>/Math Function'
        //   RelationalOperator: '<S99>/Compare'

        if (rt_remd((&ANAS0_M)->Timing.t[0], ANAS0_DW.ProbeSampleTime1_i[0]) ==
            ANAS0_P.Constant_Value_ct) {
          rtb_Switch1_l = ANAS0_P.Constant_Value_ij;
        } else {
          rtb_Switch1_l = ANAS0_P.Constant1_Value_pt;
        }

        // End of Switch: '<S90>/OutputSwitch1'
      } else if (rt_remd((&ANAS0_M)->Timing.t[0], ANAS0_P.Constant4_Value_h1) ==
                 ANAS0_P.Constant_Value_hx) {
        // Switch: '<S90>/OutputSwitch2' incorporates:
        //   Constant: '<S90>/Constant2'

        rtb_Switch1_l = ANAS0_P.Constant2_Value_bb;
      } else {
        rtb_Switch1_l = ANAS0_P.Constant3_Value_dl;
      }

      // End of Switch: '<S90>/Switch'
    } else {
      rtb_Switch1_l = ANAS0_P.Constant5_Value_b;
    }

    // End of Switch: '<S90>/Switch1'

    // RateTransition: '<S90>/Rate Transition1' incorporates:
    //   RateTransition: '<S7>/Rate Transition'

    if (rtb_NOT1_h) {
      // Logic: '<S61>/NOT' incorporates:
      //   Gain: '<S90>/Gain'

      rtb_NOT_k = (ANAS0_P.Gain_Gain_mw * rtb_Switch1_l == 0.0);

      // Logic: '<S61>/NOT1'
      rtb_NOT1_h = !rtb_NOT_k;

      // Outputs for Enabled SubSystem: '<S61>/State and Covariance' incorporates:
      //   EnablePort: '<S88>/Enable'

      if (rtb_NOT1_h) {
        rtb_VectorConcatenate_a[0] = ANAS0_P.Constant_Value_h4[0];
        rtb_VectorConcatenate_a[1] = ANAS0_P.Constant_Value_h4[1];

        // Product: '<S92>/Divide' incorporates:
        //   Constant: '<S92>/Constant'
        //   Constant: '<S92>/Constant9'
        //   Constant: '<S92>/lambda'
        //   Inport: '<Root>/ANAS Reference In'
        //   Product: '<S92>/Product'
        //   Sum: '<S92>/Add2'

        rtb_Switch1_l = (ANAS0_DW.Merge1_a[2] - ANAS0_P.Constant9_Value) *
          ANAS0_P.lambda_Value;
        rtb_Power3 = rtb_Switch1_l / ANAS0_U.ANASReferenceIn.GroundTemperature;

        // Product: '<S92>/Divide1' incorporates:
        //   Constant: '<S92>/Constant3'
        //   Constant: '<S92>/R'
        //   Constant: '<S92>/lambda'

        rtb_Divide1 = ANAS0_P.Constant3_Value_ji / ANAS0_P.R_Value /
          ANAS0_P.lambda_Value;

        // Product: '<S92>/Divide2' incorporates:
        //   Bias: '<S92>/Add Constant'
        //   Constant: '<S92>/Constant3'
        //   Constant: '<S92>/Constant5'
        //   Constant: '<S92>/R'
        //   Gain: '<S92>/Gain'
        //   Inport: '<Root>/ANAS Reference In'
        //   Math: '<S92>/Power'
        //   Sum: '<S92>/Sum'

        rtb_VectorConcatenate_a[2] = std::pow(ANAS0_P.Constant5_Value_p -
          rtb_Power3, ANAS0_P.Gain_Gain_pa * rtb_Divide1 +
          ANAS0_P.AddConstant_Bias) * ANAS0_U.ANASReferenceIn.GroundPressure *
          ANAS0_P.Constant3_Value_ji / ANAS0_P.R_Value /
          ANAS0_U.ANASReferenceIn.GroundTemperature;

        // Sum: '<S97>/Add' incorporates:
        //   Math: '<S97>/Square'
        //   Math: '<S97>/Square1'
        //   Math: '<S97>/Square2'
        //   Math: '<S97>/Square3'

        rtb_VectorConcatenate_d[0] = ((rtb_Switch3[1] * rtb_Switch3[1] -
          rtb_Switch3[2] * rtb_Switch3[2]) - rtb_Switch3[3] * rtb_Switch3[3]) +
          rtb_Switch3[0] * rtb_Switch3[0];

        // Gain: '<S97>/Gain' incorporates:
        //   Product: '<S97>/Product'
        //   Product: '<S97>/Product1'
        //   Sum: '<S97>/Add1'

        rtb_VectorConcatenate_d[1] = (rtb_Switch3[1] * rtb_Switch3[2] +
          rtb_Switch3[0] * rtb_Switch3[3]) * ANAS0_P.Gain_Gain_f4;

        // Gain: '<S97>/Gain1' incorporates:
        //   Product: '<S97>/Product2'
        //   Product: '<S97>/Product3'
        //   Sum: '<S97>/Add2'

        rtb_VectorConcatenate_d[2] = (rtb_Switch3[1] * rtb_Switch3[3] -
          rtb_Switch3[0] * rtb_Switch3[2]) * ANAS0_P.Gain1_Gain_d;
        rtb_VectorConcatenate_a[3] = ANAS0_P.Constant1_Value_m[0];
        rtb_VectorConcatenate_a[4] = ANAS0_P.Constant1_Value_m[1];
        rtb_VectorConcatenate_a[5] = ANAS0_P.Constant1_Value_m[2];

        // Gain: '<S92>/Gain3' incorporates:
        //   Concatenate: '<S97>/Vector Concatenate'
        //   Constant: '<S92>/Constant1'
        //   Product: '<S92>/Matrix Multiply1'

        rtb_Power = ((rtb_VectorConcatenate_d[0] * ANAS0_DW.Merge1_a[3] +
                      rtb_VectorConcatenate_d[1] * ANAS0_DW.Merge1_a[4]) +
                     rtb_VectorConcatenate_d[2] * ANAS0_DW.Merge1_a[5]) *
          ANAS0_P.Gain3_Gain_e;

        // Math: '<S92>/Square' incorporates:
        //   Math: '<S96>/Square'

        tmp_1 = rtb_Power * rtb_Power;

        // Bias: '<S92>/Add Constant1' incorporates:
        //   Constant: '<S92>/gamma'

        rtb_AddConstant1 = ANAS0_P.gamma_Value + ANAS0_P.AddConstant1_Bias;

        // Sum: '<S92>/Sum1' incorporates:
        //   Inport: '<Root>/ANAS Reference In'

        Gain = rtb_Switch1_l + ANAS0_U.ANASReferenceIn.GroundTemperature;

        // Bias: '<S92>/Add Constant3' incorporates:
        //   Constant: '<S92>/R'
        //   Constant: '<S92>/gamma'
        //   Gain: '<S92>/Gain1'
        //   Math: '<S92>/Square'
        //   Product: '<S92>/Divide3'

        rtb_Square_k = ANAS0_P.Gain1_Gain_c * rtb_AddConstant1 * tmp_1 /
          ANAS0_P.gamma_Value / ANAS0_P.R_Value / Gain +
          ANAS0_P.AddConstant3_Bias;

        // Product: '<S92>/Divide7' incorporates:
        //   Constant: '<S92>/gamma'

        rtb_Divide1_a = ANAS0_P.gamma_Value / rtb_AddConstant1;

        // Math: '<S92>/Power1'
        rtb_Switch1_l = std::pow(rtb_Square_k, rtb_Divide1_a);

        // Product: '<S92>/Product1' incorporates:
        //   Concatenate: '<S92>/Vector Concatenate'

        for (i_0 = 0; i_0 < 6; i_0++) {
          rtb_MatrixMultiply2_n[i_0] = rtb_VectorConcatenate_a[i_0] *
            rtb_Switch1_l;
        }

        // End of Product: '<S92>/Product1'
        rtb_MatrixMultiply2_b[0] = ANAS0_P.Constant7_Value[0];
        rtb_MatrixMultiply2_b[1] = ANAS0_P.Constant7_Value[1];

        // Product: '<S92>/Divide6' incorporates:
        //   Constant: '<S92>/Constant7'
        //   Constant: '<S92>/lambda'
        //   Gain: '<S92>/Gain2'
        //   Math: '<S92>/Square1'

        rtb_MatrixMultiply2_b[2] = ANAS0_P.Gain2_Gain_gz * ANAS0_P.lambda_Value /
          (Gain * Gain);
        rtb_MatrixMultiply2_b[3] = ANAS0_P.Constant8_Value[0];
        rtb_MatrixMultiply2_b[4] = ANAS0_P.Constant8_Value[1];
        rtb_MatrixMultiply2_b[5] = ANAS0_P.Constant8_Value[2];

        // Product: '<S92>/Divide9' incorporates:
        //   Bias: '<S92>/Bias'
        //   Constant: '<S92>/Constant8'
        //   Inport: '<Root>/ANAS Reference In'
        //   Math: '<S92>/Power3'

        rtb_Switch1_l = std::pow(rtb_Power3 + ANAS0_P.Bias_Bias, rtb_Divide1) *
          ANAS0_U.ANASReferenceIn.GroundPressure;

        // Product: '<S92>/Divide8' incorporates:
        //   Bias: '<S92>/Add Constant2'
        //   Math: '<S92>/Power2'

        rtb_Divide1_a *= std::pow(rtb_Square_k, rtb_Divide1_a +
          ANAS0_P.AddConstant2_Bias);

        // Product: '<S92>/Divide4' incorporates:
        //   Constant: '<S92>/Constant6'

        rtb_Square_k = ANAS0_P.Constant6_Value_e * rtb_Power;

        // Product: '<S92>/Matrix Multiply' incorporates:
        //   Concatenate: '<S92>/Matrix Concatenate'
        //   Concatenate: '<S92>/Vector Concatenate'
        //   Concatenate: '<S97>/Vector Concatenate'
        //   Constant: '<S92>/Constant6'
        //   Constant: '<S92>/R'
        //   Constant: '<S92>/dstates'
        //   Constant: '<S92>/gamma'
        //   Math: '<S91>/Transpose1'
        //   Math: '<S92>/Square'
        //   Product: '<S88>/Matrix Multiply2'
        //   Product: '<S92>/Divide4'
        //   Product: '<S92>/Divide5'
        //   Product: '<S92>/Divide8'
        //   Product: '<S92>/Product1'
        //   Product: '<S92>/Product2'
        //   Sum: '<S92>/Add'
        //   Sum: '<S92>/Add1'
        //   Sum: '<S92>/Sum2'

        rtb_Power3 = rtb_VectorConcatenate_d[1];
        rtb_Divide1 = rtb_VectorConcatenate_d[0];
        rtb_VectorConcatenate_i_tmp_1 = rtb_VectorConcatenate_d[2];
        i_0 = 0;
        i = 0;
        for (Sum1_tmp = 0; Sum1_tmp < 6; Sum1_tmp++) {
          // Concatenate: '<S92>/Vector Concatenate'
          rtb_Power = rtb_VectorConcatenate_a[Sum1_tmp];
          rtb_Transpose1_l[i_0] = rtb_Power;
          rtb_Transpose1_l[i_0 + 1] = ((((ANAS0_P.dstates_Value[i + 1] *
            rtb_Power3 + ANAS0_P.dstates_Value[i] * rtb_Divide1) +
            ANAS0_P.dstates_Value[i + 2] * rtb_VectorConcatenate_i_tmp_1) *
            rtb_Square_k * rtb_AddConstant1 / ANAS0_P.Constant6_Value_e /
            ANAS0_P.gamma_Value / ANAS0_P.R_Value / Gain +
            rtb_MatrixMultiply2_b[Sum1_tmp] * rtb_AddConstant1 * tmp_1 /
            ANAS0_P.Constant6_Value_e / ANAS0_P.gamma_Value / ANAS0_P.R_Value) *
            rtb_Divide1_a * rtb_Switch1_l + rtb_MatrixMultiply2_n[Sum1_tmp]) -
            rtb_Power;
          i_0 += 2;
          i += 3;
        }

        // End of Product: '<S92>/Matrix Multiply'
        for (i_0 = 0; i_0 < 6; i_0++) {
          // Product: '<S93>/Matrix Multiply' incorporates:
          //   Math: '<S91>/Transpose1'
          //   Math: '<S93>/Transpose1'
          //   Merge: '<S59>/Merge'
          //   Product: '<S93>/Matrix Multiply1'

          i = 0;
          for (Sum1_tmp = 0; Sum1_tmp < 2; Sum1_tmp++) {
            rtb_Power = 0.0;
            i_2 = 0;
            rtb_MatrixMultiply2_f_tmp = 0;
            for (i_3 = 0; i_3 < 6; i_3++) {
              rtb_Power += ANAS0_DW.Merge_l[i_2 + i_0] *
                rtb_Transpose1_l[rtb_MatrixMultiply2_f_tmp + Sum1_tmp];
              i_2 += 6;
              rtb_MatrixMultiply2_f_tmp += 2;
            }

            rtb_K_tmp[i + i_0] = rtb_Power;
            i += 6;
          }
        }

        // Sum: '<S93>/Sum' incorporates:
        //   Constant: '<S94>/Constant'
        //   Math: '<S91>/Transpose1'
        //   Product: '<S93>/Matrix Multiply'

        for (i_0 = 0; i_0 < 2; i_0++) {
          i = 0;
          Sum1_tmp = 0;
          for (i_2 = 0; i_2 < 2; i_2++) {
            rtb_Square_k = 0.0;
            rtb_MatrixMultiply2_f_tmp = 0;
            for (i_3 = 0; i_3 < 6; i_3++) {
              rtb_Square_k += rtb_Transpose1_l[rtb_MatrixMultiply2_f_tmp + i_0] *
                rtb_K_tmp[i_3 + Sum1_tmp];
              rtb_MatrixMultiply2_f_tmp += 2;
            }

            rtb_MatrixMultiply2_f_tmp = i + i_0;
            rtb_MatrixConcatenate_a[rtb_MatrixMultiply2_f_tmp] =
              ANAS0_P.Constant_Value_fb[rtb_MatrixMultiply2_f_tmp] +
              rtb_Square_k;
            i += 2;
            Sum1_tmp += 6;
          }
        }

        // End of Sum: '<S93>/Sum'

        // Product: '<S93>/Matrix Divide'
        rt_mrdivide_U1d6x2_U2d2x2_Yd6x2(rtb_K_tmp, rtb_MatrixConcatenate_a,
          rtb_K);

        // Sum: '<S95>/Subtract' incorporates:
        //   Constant: '<S95>/Constant'
        //   Math: '<S91>/Transpose1'
        //   Product: '<S91>/Matrix Multiply2'
        //   Product: '<S95>/Matrix Multiply'

        for (i_0 = 0; i_0 < 6; i_0++) {
          // Product: '<S95>/Matrix Multiply' incorporates:
          //   Product: '<S93>/Matrix Divide'

          rtb_Power = rtb_K[i_0 + 6];
          rtb_Divide1_a = rtb_K[i_0];
          i = 0;
          Sum1_tmp = 0;
          for (i_2 = 0; i_2 < 6; i_2++) {
            rtb_MatrixMultiply2_f_tmp = i + i_0;
            rtb_MatrixMultiply2_k3[rtb_MatrixMultiply2_f_tmp] =
              ANAS0_P.Constant_Value_ki[rtb_MatrixMultiply2_f_tmp] -
              (rtb_Transpose1_l[Sum1_tmp + 1] * rtb_Power +
               rtb_Transpose1_l[Sum1_tmp] * rtb_Divide1_a);
            i += 6;
            Sum1_tmp += 2;
          }
        }

        // End of Sum: '<S95>/Subtract'

        // Product: '<S91>/Matrix Multiply' incorporates:
        //   Math: '<S91>/Transpose'
        //   Merge: '<S59>/Merge'
        //   Product: '<S91>/Matrix Multiply2'

        for (i_0 = 0; i_0 < 6; i_0++) {
          i = 0;
          for (Sum1_tmp = 0; Sum1_tmp < 6; Sum1_tmp++) {
            rtb_Square_k = 0.0;
            i_2 = 0;
            for (rtb_MatrixMultiply2_f_tmp = 0; rtb_MatrixMultiply2_f_tmp < 6;
                 rtb_MatrixMultiply2_f_tmp++) {
              rtb_Square_k += ANAS0_DW.Merge_l[i_2 + i_0] *
                rtb_MatrixMultiply2_k3[i_2 + Sum1_tmp];
              i_2 += 6;
            }

            tmp[i + i_0] = rtb_Square_k;
            i += 6;
          }
        }

        // Product: '<S91>/Matrix Multiply2' incorporates:
        //   Constant: '<S94>/Constant'
        //   Math: '<S91>/Transpose1'
        //   Product: '<S93>/Matrix Divide'

        for (i_0 = 0; i_0 < 2; i_0++) {
          rtb_Power = ANAS0_P.Constant_Value_fb[i_0 + 2];
          rtb_Divide1_a = ANAS0_P.Constant_Value_fb[i_0];
          i = 0;
          for (Sum1_tmp = 0; Sum1_tmp < 6; Sum1_tmp++) {
            rtb_Transpose1_l[i + i_0] = rtb_K[Sum1_tmp + 6] * rtb_Power +
              rtb_Divide1_a * rtb_K[Sum1_tmp];
            i += 2;
          }
        }

        // Product: '<S91>/Matrix Multiply' incorporates:
        //   Product: '<S91>/Matrix Multiply2'
        //   Product: '<S93>/Matrix Divide'

        i_0 = 0;
        i = 0;
        for (Sum1_tmp = 0; Sum1_tmp < 6; Sum1_tmp++) {
          for (i_2 = 0; i_2 < 6; i_2++) {
            rtb_Power3 = 0.0;
            rtb_MatrixMultiply2_f_tmp = 0;
            for (i_3 = 0; i_3 < 6; i_3++) {
              rtb_Power3 += rtb_MatrixMultiply2_k3[rtb_MatrixMultiply2_f_tmp +
                i_2] * tmp[i_3 + i_0];
              rtb_MatrixMultiply2_f_tmp += 6;
            }

            rtb_MatrixMultiply2_f_tmp = i_2 + i_0;
            rtb_UnitDelay1[rtb_MatrixMultiply2_f_tmp] = rtb_Power3;
            rtb_UnitDelay5[rtb_MatrixMultiply2_f_tmp] = rtb_Transpose1_l[i + 1] *
              rtb_K[i_2 + 6] + rtb_Transpose1_l[i] * rtb_K[i_2];
          }

          i_0 += 6;
          i += 2;
        }

        for (i_0 = 0; i_0 < 36; i_0++) {
          // Merge: '<S61>/Merge' incorporates:
          //   Sum: '<S91>/Sum1'

          ANAS0_DW.Merge_j[i_0] = rtb_UnitDelay1[i_0] + rtb_UnitDelay5[i_0];
        }

        // Bias: '<S96>/Add Constant' incorporates:
        //   Constant: '<S92>/gamma'

        rtb_Divide1_a = ANAS0_P.gamma_Value + ANAS0_P.AddConstant_Bias_e;

        // Sum: '<S88>/Sum1' incorporates:
        //   Bias: '<S96>/Add Constant1'
        //   Bias: '<S96>/Add Constant2'
        //   Constant: '<S92>/R'
        //   Constant: '<S92>/gamma'
        //   Gain: '<S96>/Gain'
        //   Inport: '<Root>/ANAS In'
        //   Math: '<S96>/Power'
        //   Product: '<S96>/Divide'
        //   Product: '<S96>/Divide1'
        //   Product: '<S96>/Product'
        //   Product: '<S96>/Product1'
        //   SignalConversion generated from: '<S96>/Vector Concatenate'

        rtb_Divide1_a = ANAS0_U.ANASIn_j.PitotMeasure[0] - (std::pow(tmp_1 /
          ANAS0_P.gamma_Value / ANAS0_P.R_Value / Gain * (ANAS0_P.Gain_Gain_i *
          rtb_Divide1_a) + ANAS0_P.AddConstant1_Bias_i, ANAS0_P.gamma_Value /
          rtb_Divide1_a) + ANAS0_P.AddConstant2_Bias_b) * rtb_Switch1_l;
        rtb_Switch1_l = ANAS0_U.ANASIn_j.PitotMeasure[1] - rtb_Switch1_l;
        for (i_0 = 0; i_0 < 6; i_0++) {
          // Merge: '<S61>/Merge1' incorporates:
          //   Merge: '<S59>/Merge1'
          //   Product: '<S88>/Matrix Multiply2'
          //   Product: '<S93>/Matrix Divide'
          //   Sum: '<S88>/Sum4'

          ANAS0_DW.Merge1_b[i_0] = (rtb_K[i_0 + 6] * rtb_Switch1_l + rtb_K[i_0] *
            rtb_Divide1_a) + ANAS0_DW.Merge1_a[i_0];
        }
      }

      // End of Outputs for SubSystem: '<S61>/State and Covariance'

      // Outputs for Enabled SubSystem: '<S61>/Subsystem1'
      ANAS0_Subsystem1(rtb_NOT_k, ANAS0_DW.Merge1_a, ANAS0_DW.Merge_l,
                       ANAS0_DW.Merge1_b, ANAS0_DW.Merge_j);

      // End of Outputs for SubSystem: '<S61>/Subsystem1'

      // SignalConversion generated from: '<S1>/Vector Concatenate' incorporates:
      //   Constant: '<S92>/Constant'
      //   Constant: '<S92>/Constant1'
      //   Constant: '<S92>/Constant7'
      //   Constant: '<S92>/Constant8'
      //   Merge: '<S61>/Merge1'

      for (i_0 = 0; i_0 < 6; i_0++) {
        rtb_VectorConcatenate[i_0] = ANAS0_DW.Merge1_b[i_0];
      }

      // SignalConversion generated from: '<S1>/Vector Concatenate' incorporates:
      //   Merge: '<S2>/Merge2'

      for (i_0 = 0; i_0 < 7; i_0++) {
        rtb_VectorConcatenate[i_0 + 6] = ANAS0_DW.Merge2[i_0];
      }

      // DataTypeConversion: '<S1>/Cast To Single3' incorporates:
      //   Concatenate: '<S1>/Vector Concatenate'

      for (i_0 = 0; i_0 < 13; i_0++) {
        rtb_CastToSingle3[i_0] = static_cast<float>(rtb_VectorConcatenate[i_0]);
      }

      // End of DataTypeConversion: '<S1>/Cast To Single3'

      // DataTypeConversion: '<S1>/Cast To Single2' incorporates:
      //   Merge: '<S61>/Merge'

      for (i_0 = 0; i_0 < 36; i_0++) {
        rtb_CastToSingle2[i_0] = static_cast<float>(ANAS0_DW.Merge_j[i_0]);
      }

      // End of DataTypeConversion: '<S1>/Cast To Single2'

      // S-Function (sdspdiag2): '<S1>/Extract Diagonal' incorporates:
      //   DataTypeConversion: '<S1>/Cast To Single2'

      i_0 = 0;
      for (i = 0; i < 6; i++) {
        rtb_VectorConcatenate2[i] = rtb_CastToSingle2[i_0];
        i_0 += 7;
      }

      // End of S-Function (sdspdiag2): '<S1>/Extract Diagonal'

      // Switch: '<S1>/Switch2' incorporates:
      //   Switch: '<S1>/Switch1'

      if (rtb_NOT) {
        // Switch: '<S1>/Switch'
        if (rtb_NOT1_h) {
          // Outport: '<Root>/ANAS OBSW Logs' incorporates:
          //   Constant: '<S1>/Zero3'

          ANAS0_Y.ANASOBSWLogs.BaroPitotActivation = ANAS0_P.Zero3_Value;
        } else {
          // Outport: '<Root>/ANAS OBSW Logs' incorporates:
          //   Constant: '<S1>/Zero4'

          ANAS0_Y.ANASOBSWLogs.BaroPitotActivation = ANAS0_P.Zero4_Value;
        }
      } else if (rtb_NOT1_h) {
        // Switch: '<S1>/Switch1' incorporates:
        //   Constant: '<S1>/Zero5'
        //   Outport: '<Root>/ANAS OBSW Logs'
        //   Switch: '<S1>/Switch'

        ANAS0_Y.ANASOBSWLogs.BaroPitotActivation = static_cast<uint8_t>
          (ANAS0_P.Zero5_Value);
      } else {
        // Outport: '<Root>/ANAS OBSW Logs' incorporates:
        //   Constant: '<S1>/Zero2'
        //   Switch: '<S1>/Switch'

        ANAS0_Y.ANASOBSWLogs.BaroPitotActivation = static_cast<uint8_t>
          (ANAS0_P.Zero2_Value);
      }

      // End of Switch: '<S1>/Switch2'

      // Bias: '<S1>/Bias1'
      rtb_UnitDelay4 = static_cast<uint8_t>(rtb_UnitDelay4 + ANAS0_P.Bias1_Bias);

      // Saturate: '<S1>/Saturation1'
      if (rtb_UnitDelay4 > ANAS0_P.Saturation1_UpperSat) {
        // Saturate: '<S1>/Saturation1'
        rtb_Saturation1 = ANAS0_P.Saturation1_UpperSat;
      } else if (rtb_UnitDelay4 < ANAS0_P.Saturation1_LowerSat) {
        // Saturate: '<S1>/Saturation1'
        rtb_Saturation1 = ANAS0_P.Saturation1_LowerSat;
      } else {
        // Saturate: '<S1>/Saturation1'
        rtb_Saturation1 = rtb_UnitDelay4;
      }

      // End of Saturate: '<S1>/Saturation1'

      // Bias: '<S1>/Bias2'
      rtb_UnitDelay4 = static_cast<uint8_t>(rtb_UnitDelay3 + ANAS0_P.Bias2_Bias);

      // Saturate: '<S1>/Saturation2'
      if (rtb_UnitDelay4 > ANAS0_P.Saturation2_UpperSat) {
        // Saturate: '<S1>/Saturation2'
        rtb_Saturation2 = ANAS0_P.Saturation2_UpperSat;
      } else if (rtb_UnitDelay4 < ANAS0_P.Saturation2_LowerSat) {
        // Saturate: '<S1>/Saturation2'
        rtb_Saturation2 = ANAS0_P.Saturation2_LowerSat;
      } else {
        // Saturate: '<S1>/Saturation2'
        rtb_Saturation2 = rtb_UnitDelay4;
      }

      // End of Saturate: '<S1>/Saturation2'

      // Outport: '<Root>/ANAS Out' incorporates:
      //   BusCreator generated from: '<S1>/ANAS Out_BusCreator'

      ANAS0_Y.ANASOut_p.Timestamp = 0ULL;
      ANAS0_Y.ANASOut_p.Position[0] = rtb_CastToSingle3[0];
      ANAS0_Y.ANASOut_p.Velocity[0] = rtb_CastToSingle3[3];
      ANAS0_Y.ANASOut_p.Position[1] = rtb_CastToSingle3[1];
      ANAS0_Y.ANASOut_p.Velocity[1] = rtb_CastToSingle3[4];
      ANAS0_Y.ANASOut_p.Position[2] = rtb_CastToSingle3[2];
      ANAS0_Y.ANASOut_p.Velocity[2] = rtb_CastToSingle3[5];
      ANAS0_Y.ANASOut_p.Quaternion[0] = rtb_CastToSingle3[6];
      ANAS0_Y.ANASOut_p.Quaternion[1] = rtb_CastToSingle3[7];
      ANAS0_Y.ANASOut_p.Quaternion[2] = rtb_CastToSingle3[8];
      ANAS0_Y.ANASOut_p.Quaternion[3] = rtb_CastToSingle3[9];

      // Outport: '<Root>/NASDAQ Initial State' incorporates:
      //   BusCreator generated from: '<S1>/NASDAQ Initial State_BusCreator'
      //   DataTypeConversion: '<S1>/Cast To Single2'

      std::memcpy(&ANAS0_Y.NASDAQInitialState.LinearCovariance[0],
                  &rtb_CastToSingle2[0], 36U * sizeof(float));

      // Outport: '<Root>/ANAS OBSW Logs' incorporates:
      //   BusCreator generated from: '<S1>/ANAS OBSW Logs_BusCreator'

      ANAS0_Y.ANASOBSWLogs.Timestamp = 0ULL;

      // Outport: '<Root>/NASDAQ Initial State' incorporates:
      //   BusCreator generated from: '<S1>/NASDAQ Initial State_BusCreator'

      ANAS0_Y.NASDAQInitialState.Position[0] = rtb_CastToSingle3[0];
      ANAS0_Y.NASDAQInitialState.Velocity[0] = rtb_CastToSingle3[3];

      // Outport: '<Root>/ANAS OBSW Logs' incorporates:
      //   BusCreator generated from: '<S1>/NASDAQ Initial State_BusCreator'

      ANAS0_Y.ANASOBSWLogs.Position[0] = rtb_CastToSingle3[0];
      ANAS0_Y.ANASOBSWLogs.Velocity[0] = rtb_CastToSingle3[3];

      // Outport: '<Root>/NASDAQ Initial State' incorporates:
      //   BusCreator generated from: '<S1>/NASDAQ Initial State_BusCreator'

      ANAS0_Y.NASDAQInitialState.Position[1] = rtb_CastToSingle3[1];
      ANAS0_Y.NASDAQInitialState.Velocity[1] = rtb_CastToSingle3[4];

      // Outport: '<Root>/ANAS OBSW Logs' incorporates:
      //   BusCreator generated from: '<S1>/NASDAQ Initial State_BusCreator'

      ANAS0_Y.ANASOBSWLogs.Position[1] = rtb_CastToSingle3[1];
      ANAS0_Y.ANASOBSWLogs.Velocity[1] = rtb_CastToSingle3[4];

      // Outport: '<Root>/NASDAQ Initial State' incorporates:
      //   BusCreator generated from: '<S1>/NASDAQ Initial State_BusCreator'

      ANAS0_Y.NASDAQInitialState.Position[2] = rtb_CastToSingle3[2];
      ANAS0_Y.NASDAQInitialState.Velocity[2] = rtb_CastToSingle3[5];

      // Outport: '<Root>/ANAS OBSW Logs' incorporates:
      //   BusCreator generated from: '<S1>/NASDAQ Initial State_BusCreator'
      //   Constant: '<S1>/Zero1'

      ANAS0_Y.ANASOBSWLogs.Position[2] = rtb_CastToSingle3[2];
      ANAS0_Y.ANASOBSWLogs.Velocity[2] = rtb_CastToSingle3[5];
      ANAS0_Y.ANASOBSWLogs.Quaternion[0] = rtb_CastToSingle3[6];
      ANAS0_Y.ANASOBSWLogs.Quaternion[1] = rtb_CastToSingle3[7];
      ANAS0_Y.ANASOBSWLogs.Quaternion[2] = rtb_CastToSingle3[8];
      ANAS0_Y.ANASOBSWLogs.Quaternion[3] = rtb_CastToSingle3[9];
      for (i_0 = 0; i_0 < 6; i_0++) {
        ANAS0_Y.ANASOBSWLogs.CovarianceMatrixDiagonal[i_0] =
          rtb_VectorConcatenate2[i_0];
      }

      ANAS0_Y.ANASOBSWLogs.GPSActivation = rtb_NOT1_n;
      ANAS0_Y.ANASOBSWLogs.MagActivation = ANAS0_P.Zero1_Value;
      ANAS0_Y.ANASOBSWLogs.AccActivation = ANAS0_P.Zero1_Value;
    }

    // DataStoreWrite: '<S6>/Data Store Write'
    ANAS0_DW.A_g = ANAS0_DW.ProbeSampleTime_i[0];

    // DataStoreWrite: '<S7>/Data Store Write'
    ANAS0_DW.A_gb = ANAS0_DW.ProbeSampleTime_k[0];

    // End of Outputs for SubSystem: '<Root>/ANAS - Autocoding'
  }

  {
    int32_t i;

    // Update for Atomic SubSystem: '<Root>/ANAS - Autocoding'
    // Update for RateTransition: '<S10>/Rate Transition'
    if ((&ANAS0_M)->Timing.TaskCounters.TID[2] == 0) {
      // Update for UnitDelay: '<S1>/Unit Delay' incorporates:
      //   Merge: '<S2>/Merge2'

      for (i = 0; i < 7; i++) {
        ANAS0_DW.UnitDelay_DSTATE[i] = ANAS0_DW.Merge2[i];
      }

      // End of Update for UnitDelay: '<S1>/Unit Delay'

      // Update for UnitDelay: '<S1>/Unit Delay3'
      ANAS0_DW.UnitDelay3_DSTATE = rtb_Saturation2;

      // Update for UnitDelay: '<S1>/Unit Delay1' incorporates:
      //   Merge: '<S2>/Merge3'

      std::memcpy(&ANAS0_DW.UnitDelay1_DSTATE[0], &ANAS0_DW.Merge3[0], 36U *
                  sizeof(double));

      // Update for UnitDelay: '<S1>/Unit Delay2' incorporates:
      //   Merge: '<S61>/Merge1'

      for (i = 0; i < 6; i++) {
        ANAS0_DW.UnitDelay2_DSTATE[i] = ANAS0_DW.Merge1_b[i];
      }

      // End of Update for UnitDelay: '<S1>/Unit Delay2'

      // Update for UnitDelay: '<S1>/Unit Delay4'
      ANAS0_DW.UnitDelay4_DSTATE = rtb_Saturation1;

      // Update for UnitDelay: '<S1>/Unit Delay5' incorporates:
      //   Merge: '<S61>/Merge'

      std::memcpy(&ANAS0_DW.UnitDelay5_DSTATE[0], &ANAS0_DW.Merge_j[0], 36U *
                  sizeof(double));
    }

    // End of Update for RateTransition: '<S10>/Rate Transition'
    // End of Update for SubSystem: '<Root>/ANAS - Autocoding'
  }

  // Update absolute time for base rate
  // The "clockTick0" counts the number of times the code of this task has
  //  been executed. The absolute time is the multiplication of "clockTick0"
  //  and "Timing.stepSize0". Size of "clockTick0" ensures timer will not
  //  overflow during the application lifespan selected.

  (&ANAS0_M)->Timing.t[0] =
    ((double)(++(&ANAS0_M)->Timing.clockTick0)) * (&ANAS0_M)->Timing.stepSize0;

  {
    // Update absolute timer for sample time: [0.01s, 0.0s]
    // The "clockTick1" counts the number of times the code of this task has
    //  been executed. The resolution of this integer timer is 0.01, which is the step size
    //  of the task. Size of "clockTick1" ensures timer will not overflow during the
    //  application lifespan selected.

    (&ANAS0_M)->Timing.clockTick1++;
  }

  rate_scheduler((&ANAS0_M));
}

// Model initialize function
void ANAS0::initialize()
{
  // Registration code
  {
    // Setup solver object
    rtsiSetSimTimeStepPtr(&(&ANAS0_M)->solverInfo, &(&ANAS0_M)
                          ->Timing.simTimeStep);
    rtsiSetTPtr(&(&ANAS0_M)->solverInfo, (&ANAS0_M)->getTPtrPtr());
    rtsiSetStepSizePtr(&(&ANAS0_M)->solverInfo, &(&ANAS0_M)->Timing.stepSize0);
    rtsiSetErrorStatusPtr(&(&ANAS0_M)->solverInfo, (&ANAS0_M)->getErrorStatusPtr
                          ());
    rtsiSetRTModelPtr(&(&ANAS0_M)->solverInfo, (&ANAS0_M));
  }

  rtsiSetSimTimeStep(&(&ANAS0_M)->solverInfo, MAJOR_TIME_STEP);
  rtsiSetIsMinorTimeStepWithModeChange(&(&ANAS0_M)->solverInfo, false);
  rtsiSetIsContModeFrozen(&(&ANAS0_M)->solverInfo, false);
  rtsiSetSolverName(&(&ANAS0_M)->solverInfo,"FixedStepDiscrete");
  (&ANAS0_M)->setTPtr(&(&ANAS0_M)->Timing.tArray[0]);
  (&ANAS0_M)->Timing.stepSize0 = 0.01;

  {
    int32_t i;

    // SystemInitialize for Atomic SubSystem: '<Root>/ANAS - Autocoding'
    // Start for Probe: '<S10>/Probe Sample Time'
    ANAS0_DW.ProbeSampleTime[0] = 0.01;
    ANAS0_DW.ProbeSampleTime[1] = 0.0;

    // Start for Probe: '<S64>/Probe Sample Time1'
    ANAS0_DW.ProbeSampleTime1[0] = 0.01;
    ANAS0_DW.ProbeSampleTime1[1] = 0.0;

    // Start for Probe: '<S76>/Probe Sample Time1'
    ANAS0_DW.ProbeSampleTime1_b[0] = 0.1;
    ANAS0_DW.ProbeSampleTime1_b[1] = 0.0;

    // Start for DataStoreMemory: '<S76>/Data Store Memory'
    ANAS0_DW.A_b = ANAS0_P.DataStoreMemory_InitialValue_c;

    // Start for Probe: '<S90>/Probe Sample Time1'
    ANAS0_DW.ProbeSampleTime1_i[0] = 0.01;
    ANAS0_DW.ProbeSampleTime1_i[1] = 0.0;

    // Start for Probe: '<S6>/Probe Sample Time'
    ANAS0_DW.ProbeSampleTime_i[0] = 0.01;
    ANAS0_DW.ProbeSampleTime_i[1] = 0.0;

    // Start for DataStoreMemory: '<S6>/Data Store Memory'
    ANAS0_DW.A_g = ANAS0_P.DataStoreMemory_InitialValue_n;

    // Start for Probe: '<S7>/Probe Sample Time'
    ANAS0_DW.ProbeSampleTime_k[0] = 0.01;
    ANAS0_DW.ProbeSampleTime_k[1] = 0.0;

    // Start for DataStoreMemory: '<S7>/Data Store Memory'
    ANAS0_DW.A_gb = ANAS0_P.DataStoreMemory_InitialValue_f;

    // InitializeConditions for UnitDelay: '<S1>/Unit Delay'
    for (i = 0; i < 7; i++) {
      ANAS0_DW.UnitDelay_DSTATE[i] = ANAS0_P.UnitDelay_InitialCondition;
    }

    // End of InitializeConditions for UnitDelay: '<S1>/Unit Delay'

    // InitializeConditions for UnitDelay: '<S1>/Unit Delay3'
    ANAS0_DW.UnitDelay3_DSTATE = ANAS0_P.UnitDelay3_InitialCondition;

    // InitializeConditions for UnitDelay: '<S1>/Unit Delay1'
    std::memcpy(&ANAS0_DW.UnitDelay1_DSTATE[0],
                &ANAS0_P.UnitDelay1_InitialCondition[0], 36U * sizeof(double));

    // InitializeConditions for UnitDelay: '<S1>/Unit Delay2'
    for (i = 0; i < 6; i++) {
      ANAS0_DW.UnitDelay2_DSTATE[i] = ANAS0_P.UnitDelay2_InitialCondition;
    }

    // End of InitializeConditions for UnitDelay: '<S1>/Unit Delay2'

    // InitializeConditions for UnitDelay: '<S1>/Unit Delay4'
    ANAS0_DW.UnitDelay4_DSTATE = ANAS0_P.UnitDelay4_InitialCondition;

    // SystemInitialize for Enabled SubSystem: '<S1>/Attitude predictor'
    // SystemInitialize for Product: '<S54>/Product' incorporates:
    //   Outport: '<S3>/nextAngularState'

    ANAS0_DW.Product = ANAS0_P.nextAngularState_Y0;

    // SystemInitialize for Product: '<S54>/Product1' incorporates:
    //   Outport: '<S3>/nextAngularState'

    ANAS0_DW.Product1 = ANAS0_P.nextAngularState_Y0;

    // SystemInitialize for Product: '<S54>/Product2' incorporates:
    //   Outport: '<S3>/nextAngularState'

    ANAS0_DW.Product2 = ANAS0_P.nextAngularState_Y0;

    // SystemInitialize for Product: '<S54>/Product3' incorporates:
    //   Outport: '<S3>/nextAngularState'

    ANAS0_DW.Product3 = ANAS0_P.nextAngularState_Y0;

    // SystemInitialize for Outport: '<S3>/nextAngularState' incorporates:
    //   SignalConversion generated from: '<S3>/prevAngularState'

    ANAS0_DW.prevAngularState[4] = ANAS0_P.nextAngularState_Y0;
    ANAS0_DW.prevAngularState[5] = ANAS0_P.nextAngularState_Y0;
    ANAS0_DW.prevAngularState[6] = ANAS0_P.nextAngularState_Y0;
    for (i = 0; i < 36; i++) {
      // InitializeConditions for UnitDelay: '<S1>/Unit Delay5'
      ANAS0_DW.UnitDelay5_DSTATE[i] = ANAS0_P.UnitDelay5_InitialCondition[i];

      // SystemInitialize for Sum: '<S46>/Sum1' incorporates:
      //   Outport: '<S3>/nextAngularCov'
      //   UnitDelay: '<S1>/Unit Delay5'

      ANAS0_DW.Sum1_a[i] = ANAS0_P.nextAngularCov_Y0;
    }

    // End of SystemInitialize for SubSystem: '<S1>/Attitude predictor'
    for (i = 0; i < 7; i++) {
      // SystemInitialize for Merge: '<S2>/Merge2'
      ANAS0_DW.Merge2[i] = ANAS0_P.Merge2_InitialOutput;
    }

    // SystemInitialize for Enabled SubSystem: '<S1>/Linear predictor'
    // SystemInitialize for Sum: '<S107>/Sum' incorporates:
    //   Outport: '<S5>/nextLinearState'

    ANAS0_DW.Sum[0] = ANAS0_P.nextLinearState_Y0;

    // SystemInitialize for Sum: '<S107>/Sum1' incorporates:
    //   Outport: '<S5>/nextLinearState'

    ANAS0_DW.Sum1_j[0] = ANAS0_P.nextLinearState_Y0;

    // SystemInitialize for Sum: '<S107>/Sum' incorporates:
    //   Outport: '<S5>/nextLinearState'

    ANAS0_DW.Sum[1] = ANAS0_P.nextLinearState_Y0;

    // SystemInitialize for Sum: '<S107>/Sum1' incorporates:
    //   Outport: '<S5>/nextLinearState'

    ANAS0_DW.Sum1_j[1] = ANAS0_P.nextLinearState_Y0;

    // SystemInitialize for Sum: '<S107>/Sum' incorporates:
    //   Outport: '<S5>/nextLinearState'

    ANAS0_DW.Sum[2] = ANAS0_P.nextLinearState_Y0;

    // SystemInitialize for Sum: '<S107>/Sum1' incorporates:
    //   Outport: '<S5>/nextLinearState'

    ANAS0_DW.Sum1_j[2] = ANAS0_P.nextLinearState_Y0;
    for (i = 0; i < 36; i++) {
      // SystemInitialize for Merge: '<S2>/Merge3'
      ANAS0_DW.Merge3[i] = ANAS0_P.Merge3_InitialOutput;

      // SystemInitialize for Sum: '<S106>/Sum1' incorporates:
      //   Merge: '<S2>/Merge3'
      //   Outport: '<S5>/nextLinearCov'

      ANAS0_DW.Sum1[i] = ANAS0_P.nextLinearCov_Y0;
    }

    // End of SystemInitialize for SubSystem: '<S1>/Linear predictor'
    for (i = 0; i < 6; i++) {
      // SystemInitialize for Merge: '<S60>/Merge'
      ANAS0_DW.Merge[i] = ANAS0_P.Merge_InitialOutput;
    }

    for (i = 0; i < 36; i++) {
      // SystemInitialize for Merge: '<S60>/Merge1'
      ANAS0_DW.Merge1[i] = ANAS0_P.Merge1_InitialOutput;

      // SystemInitialize for Merge: '<S59>/Merge' incorporates:
      //   Merge: '<S60>/Merge1'

      ANAS0_DW.Merge_l[i] = ANAS0_P.Merge_InitialOutput_o;
    }

    for (i = 0; i < 6; i++) {
      // SystemInitialize for Merge: '<S59>/Merge1'
      ANAS0_DW.Merge1_a[i] = ANAS0_P.Merge1_InitialOutput_n;
    }

    for (i = 0; i < 36; i++) {
      // SystemInitialize for Merge: '<S61>/Merge'
      ANAS0_DW.Merge_j[i] = ANAS0_P.Merge_InitialOutput_a;
    }

    for (i = 0; i < 6; i++) {
      // SystemInitialize for Merge: '<S61>/Merge1'
      ANAS0_DW.Merge1_b[i] = ANAS0_P.Merge1_InitialOutput_k;
    }

    // End of SystemInitialize for SubSystem: '<Root>/ANAS - Autocoding'
  }
}

// Model terminate function
void ANAS0::terminate()
{
  // (no terminate code required)
}

double** ANAS0::RT_MODEL_ANAS0_T::getTPtrPtr()
{
  return &(Timing.t);
}

const char* ANAS0::RT_MODEL_ANAS0_T::getErrorStatus() const
{
  return (errorStatus);
}

void ANAS0::RT_MODEL_ANAS0_T::setErrorStatus(const char* const aErrorStatus)
{
  (errorStatus = aErrorStatus);
}

double* ANAS0::RT_MODEL_ANAS0_T::getTPtr() const
{
  return (Timing.t);
}

void ANAS0::RT_MODEL_ANAS0_T::setTPtr(double* aTPtr)
{
  (Timing.t = aTPtr);
}

const char** ANAS0::RT_MODEL_ANAS0_T::getErrorStatusPtr()
{
  return &errorStatus;
}

bool ANAS0::RT_MODEL_ANAS0_T::isMajorTimeStep() const
{
  return ((Timing.simTimeStep) == MAJOR_TIME_STEP);
}

bool ANAS0::RT_MODEL_ANAS0_T::isMinorTimeStep() const
{
  return ((Timing.simTimeStep) == MINOR_TIME_STEP);
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
