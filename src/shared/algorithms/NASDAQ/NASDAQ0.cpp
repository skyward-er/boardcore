//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// File: NASDAQ0.cpp
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
#include "NASDAQ0.h"
#include <stdbool.h>
#include <stdint.h>
#include <cmath>
#include "NASDAQ0_private.h"
#include <cstring>

static void rate_scheduler(NASDAQ0::RT_MODEL_NASDAQ0_T *const NASDAQ0_M);

//
//         This function updates active task flag for each subrate.
//         The function is called at model base rate, hence the
//         generated code self-manages all its subrates.
//
static void rate_scheduler(NASDAQ0::RT_MODEL_NASDAQ0_T *const NASDAQ0_M)
{
  // Compute which subrates run during the next base time step.  Subrates
  //  are an integer multiple of the base rate counter.  Therefore, the subtask
  //  counter is reset when it reaches its limit (zero means run).

  (NASDAQ0_M->Timing.TaskCounters.TID[1])++;
  if ((NASDAQ0_M->Timing.TaskCounters.TID[1]) > 1) {// Sample time: [0.02s, 0.0s] 
    NASDAQ0_M->Timing.TaskCounters.TID[1] = 0;
  }

  (NASDAQ0_M->Timing.TaskCounters.TID[2])++;
  if ((NASDAQ0_M->Timing.TaskCounters.TID[2]) > 9) {// Sample time: [0.1s, 0.0s] 
    NASDAQ0_M->Timing.TaskCounters.TID[2] = 0;
  }
}

//
// Output and update for enable system:
//    '<S4>/No Correction Step'
//    '<S5>/No Correction Step'
//    '<S6>/No Correction Step'
//
void NASDAQ0::NASDAQ0_NoCorrectionStep(bool rtu_Enable, const float
  rtu_nextLinearState[6], const float rtu_nextLinearCov[36], float rty_State[6],
  float rty_Covariance[36])
{
  float rtu_nextLinearCov_0[36];
  float rtu_nextLinearState_0[6];

  // Outputs for Enabled SubSystem: '<S4>/No Correction Step' incorporates:
  //   EnablePort: '<S9>/Enable'

  if (rtu_Enable) {
    // SignalConversion generated from: '<S9>/nextLinearState'
    for (int16_t i{0}; i < 6; i++) {
      rtu_nextLinearState_0[i] = rtu_nextLinearState[i];
    }

    for (int16_t i{0}; i < 6; i++) {
      rty_State[i] = rtu_nextLinearState_0[i];
    }

    // SignalConversion generated from: '<S9>/nextLinearCov'
    for (int16_t i{0}; i < 36; i++) {
      rtu_nextLinearCov_0[i] = rtu_nextLinearCov[i];
    }

    for (int16_t i{0}; i < 36; i++) {
      rty_Covariance[i] = rtu_nextLinearCov_0[i];
    }
  }

  // End of Outputs for SubSystem: '<S4>/No Correction Step'
}

void rt_mrdivide_U1f6x4_U2f4x4_Yf6x4(float u0[24], const float u1[16])
{
  float x[16];
  float smax;
  int16_t c;
  int16_t jA;
  int16_t jBcol;
  int16_t jj;
  int16_t kBcol;
  int8_t ipiv[4];
  std::memcpy(&x[0], &u1[0], sizeof(float) << 4U);
  ipiv[0] = 1;
  ipiv[1] = 2;
  ipiv[2] = 3;
  ipiv[3] = 4;
  for (int16_t j{0}; j < 3; j++) {
    int16_t iy;
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
      for (int16_t ix{c}; ix <= iy + 2; ix++) {
        x[ix - 1] /= x[jj];
      }
    }

    jBcol = 2 - j;
    jA = jj;
    jj += 4;
    for (int16_t ix{0}; ix <= jBcol; ix++) {
      smax = x[(ix << 2) + jj];
      if (smax != 0.0F) {
        iy = jA + 6;
        kBcol = (jA - j) + 8;
        for (int16_t ijA{iy}; ijA <= kBcol; ijA++) {
          x[ijA - 1] += x[((c + ijA) - jA) - 7] * -smax;
        }
      }

      jA += 4;
    }
  }

  for (int16_t j{0}; j < 4; j++) {
    jBcol = 6 * j - 1;
    jj = (j << 2) - 1;
    for (jA = 0; jA < j; jA++) {
      kBcol = 6 * jA - 1;
      smax = x[(jA + jj) + 1];
      if (smax != 0.0F) {
        for (int16_t ix{0}; ix < 6; ix++) {
          c = (ix + jBcol) + 1;
          u0[c] -= u0[(ix + kBcol) + 1] * smax;
        }
      }
    }

    smax = 1.0F / x[(j + jj) + 1];
    for (int16_t ix{0}; ix < 6; ix++) {
      c = (ix + jBcol) + 1;
      u0[c] *= smax;
    }
  }

  for (int16_t j{3}; j >= 0; j--) {
    jBcol = 6 * j - 1;
    jj = (j << 2) - 1;
    for (jA = j + 2; jA < 5; jA++) {
      kBcol = (jA - 1) * 6 - 1;
      smax = x[jA + jj];
      if (smax != 0.0F) {
        for (int16_t ix{0}; ix < 6; ix++) {
          c = (ix + jBcol) + 1;
          u0[c] -= u0[(ix + kBcol) + 1] * smax;
        }
      }
    }
  }

  for (int16_t j{2}; j >= 0; j--) {
    jj = ipiv[j];
    if (j + 1 != jj) {
      for (int16_t ix{0}; ix < 6; ix++) {
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
void NASDAQ0::step()
{
  float rtb_Bias1[36];
  float rtb_MatrixDivide_0[36];
  float rtb_MatrixMultiply[36];
  float rtb_MatrixMultiply_0[36];
  float rtb_Merge1[36];
  float rtb_Merge_fr[36];
  float rtb_Merge_j[36];
  float rtb_MatrixDivide[24];
  float rtb_Transpose1[24];
  float rtb_Square_l[16];
  float rtb_Transpose1_0[16];
  float rtb_MatrixConcatenate1[12];
  float rtb_Merge[6];
  float rtb_Merge1_a[6];
  float rtb_Merge1_b[6];
  float rtb_NASStateInterface[6];
  float rtb_VectorConcatenate[6];
  float rtb_Gain1_hp;
  float rtb_MatrixDivide_1;
  float rtb_MatrixDivide_2;
  float rtb_Switch;
  int16_t i;
  int16_t rtb_Bias1_tmp;
  int16_t rtb_MatrixMultiply_tmp;
  int16_t rtb_Transpose1_tmp;
  bool rtb_AND;

  // Outputs for Atomic SubSystem: '<Root>/NASDAQ'
  // Memory: '<S1>/NAS State Interface'
  for (i = 0; i < 6; i++) {
    rtb_NASStateInterface[i] = NASDAQ0_DW.NASStateInterface_PreviousInput[i];
  }

  // End of Memory: '<S1>/NAS State Interface'

  // Outputs for Atomic SubSystem: '<S1>/Prediction Step'
  // Bias: '<S3>/Bias' incorporates:
  //   Constant: '<S3>/Constant1'
  //   Gain: '<S3>/Gain1'
  //   Product: '<S10>/Matrix Multiply'

  for (i = 0; i < 36; i++) {
    rtb_MatrixMultiply[i] = NASDAQ0_P.Gain1_Gain_g2 *
      NASDAQ0_P.Constant1_Value_o[i] + NASDAQ0_P.Bias_Bias_l[i];
  }

  // End of Bias: '<S3>/Bias'

  // Product: '<S3>/Matrix Multiply' incorporates:
  //   Math: '<S3>/Transpose'
  //   Memory: '<S1>/NAS Variance Interface'
  //   Product: '<S10>/Matrix Multiply'

  for (i = 0; i < 6; i++) {
    for (rtb_Transpose1_tmp = 0; rtb_Transpose1_tmp < 6; rtb_Transpose1_tmp++) {
      rtb_MatrixDivide_1 = 0.0F;
      for (rtb_Bias1_tmp = 0; rtb_Bias1_tmp < 6; rtb_Bias1_tmp++) {
        rtb_MatrixDivide_1 += NASDAQ0_DW.NASVarianceInterface_PreviousIn[6 *
          rtb_Bias1_tmp + i] * rtb_MatrixMultiply[6 * rtb_Bias1_tmp +
          rtb_Transpose1_tmp];
      }

      rtb_MatrixMultiply_0[i + 6 * rtb_Transpose1_tmp] = rtb_MatrixDivide_1;
    }
  }

  // Bias: '<S3>/Bias1' incorporates:
  //   Product: '<S10>/Matrix Multiply'
  //   Product: '<S3>/Matrix Multiply'

  for (i = 0; i < 6; i++) {
    for (rtb_Transpose1_tmp = 0; rtb_Transpose1_tmp < 6; rtb_Transpose1_tmp++) {
      rtb_MatrixDivide_1 = 0.0F;
      for (rtb_Bias1_tmp = 0; rtb_Bias1_tmp < 6; rtb_Bias1_tmp++) {
        rtb_MatrixDivide_1 += rtb_MatrixMultiply[6 * rtb_Bias1_tmp + i] *
          rtb_MatrixMultiply_0[6 * rtb_Transpose1_tmp + rtb_Bias1_tmp];
      }

      rtb_Bias1_tmp = 6 * rtb_Transpose1_tmp + i;
      rtb_Bias1[rtb_Bias1_tmp] = NASDAQ0_P.Bias1_Bias_m[rtb_Bias1_tmp] +
        rtb_MatrixDivide_1;
    }
  }

  // End of Bias: '<S3>/Bias1'

  // SignalConversion generated from: '<S3>/Vector Concatenate'
  rtb_VectorConcatenate[3] = rtb_NASStateInterface[3];

  // Sum: '<S3>/Add1' incorporates:
  //   Gain: '<S3>/Gain'
  //   SignalConversion generated from: '<S3>/Previous State'

  rtb_VectorConcatenate[0] = NASDAQ0_P.Gain_Gain_b * rtb_NASStateInterface[3] +
    rtb_NASStateInterface[0];

  // SignalConversion generated from: '<S3>/Vector Concatenate'
  rtb_VectorConcatenate[4] = rtb_NASStateInterface[4];

  // Sum: '<S3>/Add1' incorporates:
  //   Gain: '<S3>/Gain'
  //   SignalConversion generated from: '<S3>/Previous State'

  rtb_VectorConcatenate[1] = NASDAQ0_P.Gain_Gain_b * rtb_NASStateInterface[4] +
    rtb_NASStateInterface[1];

  // SignalConversion generated from: '<S3>/Vector Concatenate'
  rtb_VectorConcatenate[5] = rtb_NASStateInterface[5];

  // Sum: '<S3>/Add1' incorporates:
  //   Gain: '<S3>/Gain'
  //   SignalConversion generated from: '<S3>/Previous State'

  rtb_VectorConcatenate[2] = NASDAQ0_P.Gain_Gain_b * rtb_NASStateInterface[5] +
    rtb_NASStateInterface[2];

  // End of Outputs for SubSystem: '<S1>/Prediction Step'

  // Outputs for Atomic SubSystem: '<S1>/Correction Step'
  // Outputs for Atomic SubSystem: '<S2>/GPS Correction'
  // Logic: '<S24>/AND' incorporates:
  //   Constant: '<S24>/Constant'
  //   Inport: '<Root>/GPS'
  //   Memory: '<S24>/Memory'
  //   RelationalOperator: '<S24>/Relational Operator'

  rtb_AND = ((NASDAQ0_P.nasdaq.flags.GPS != 0.0) && (NASDAQ0_U.GPS.Timestamp >
              NASDAQ0_DW.Memory_PreviousInput));

  // Outputs for Enabled SubSystem: '<S6>/Active Correction Step' incorporates:
  //   EnablePort: '<S23>/Enable'

  if (rtb_AND) {
    // Gain: '<S31>/Gain1' incorporates:
    //   Bias: '<S27>/Bias'
    //   Gain: '<S27>/Gain'

    rtb_Gain1_hp = (NASDAQ0_P.Gain_Gain_a * rtb_VectorConcatenate[0] +
                    NASDAQ0_P.Bias_Bias) * NASDAQ0_P.Gain1_Gain;

    // Trigonometry: '<S27>/Cos' incorporates:
    //   Trigonometry: '<S27>/Cos1'

    rtb_Switch = std::cos(rtb_Gain1_hp);

    // Gain: '<S27>/Gain3' incorporates:
    //   Constant: '<S27>/Constant'
    //   Constant: '<S27>/Constant1'
    //   Constant: '<S27>/Constant3'
    //   Constant: '<S27>/Constant4'
    //   Gain: '<S27>/Gain1'
    //   Math: '<S27>/Square'
    //   Product: '<S27>/Divide'
    //   Product: '<S27>/Divide1'
    //   Product: '<S27>/Product'
    //   Product: '<S27>/Product1'
    //   Trigonometry: '<S27>/Cos'
    //   Trigonometry: '<S27>/Sin'

    rtb_MatrixConcatenate1[0] = NASDAQ0_P.Gain3_Gain_b *
      NASDAQ0_P.Constant_Value_c;
    rtb_MatrixConcatenate1[1] = rtb_VectorConcatenate[1] * std::sin(rtb_Gain1_hp)
      / (rtb_Switch * rtb_Switch * NASDAQ0_P.Constant3_Value) *
      NASDAQ0_P.Gain3_Gain_b;
    rtb_MatrixConcatenate1[2] = NASDAQ0_P.Gain3_Gain_b *
      NASDAQ0_P.Constant1_Value_e;
    rtb_MatrixConcatenate1[3] = NASDAQ0_P.Constant4_Value /
      (NASDAQ0_P.Gain1_Gain_m * rtb_Switch) * NASDAQ0_P.Gain3_Gain_b;

    // Constant: '<S27>/Constant5'
    for (i = 0; i < 8; i++) {
      // Constant: '<S27>/Constant5'
      rtb_MatrixConcatenate1[i + 4] = NASDAQ0_P.Constant5_Value[i];
    }

    // End of Constant: '<S27>/Constant5'

    // Concatenate: '<S27>/Matrix Concatenate2' incorporates:
    //   Concatenate: '<S27>/Matrix Concatenate1'
    //   Constant: '<S27>/Constant6'
    //   Math: '<S26>/Transpose1'

    for (i = 0; i < 6; i++) {
      rtb_Bias1_tmp = i << 1;
      rtb_Transpose1_tmp = i << 2;
      rtb_Transpose1[rtb_Transpose1_tmp] = rtb_MatrixConcatenate1[rtb_Bias1_tmp];
      rtb_Transpose1[rtb_Transpose1_tmp + 2] =
        NASDAQ0_P.Constant6_Value[rtb_Bias1_tmp];
      rtb_Transpose1[rtb_Transpose1_tmp + 1] =
        rtb_MatrixConcatenate1[rtb_Bias1_tmp + 1];
      rtb_Transpose1[rtb_Transpose1_tmp + 3] =
        NASDAQ0_P.Constant6_Value[rtb_Bias1_tmp + 1];
    }

    // End of Concatenate: '<S27>/Matrix Concatenate2'

    // Math: '<S23>/Square' incorporates:
    //   Constant: '<S23>/Constant'

    for (i = 0; i < 16; i++) {
      rtb_Switch = NASDAQ0_P.Constant_Value_n[i];
      rtb_Square_l[i] = rtb_Switch * rtb_Switch;
    }

    // End of Math: '<S23>/Square'
    for (i = 0; i < 6; i++) {
      for (rtb_Transpose1_tmp = 0; rtb_Transpose1_tmp < 4; rtb_Transpose1_tmp++)
      {
        // Product: '<S28>/Matrix Multiply1' incorporates:
        //   Math: '<S28>/Transpose'

        rtb_Switch = 0.0F;
        for (rtb_Bias1_tmp = 0; rtb_Bias1_tmp < 6; rtb_Bias1_tmp++) {
          // Product: '<S28>/Matrix Multiply' incorporates:
          //   Bias: '<S3>/Bias1'
          //   Math: '<S26>/Transpose1'

          rtb_Switch += rtb_Bias1[6 * rtb_Bias1_tmp + i] * rtb_Transpose1
            [(rtb_Bias1_tmp << 2) + rtb_Transpose1_tmp];
        }

        rtb_MatrixDivide[i + 6 * rtb_Transpose1_tmp] = rtb_Switch;

        // End of Product: '<S28>/Matrix Multiply1'
      }
    }

    // Sum: '<S28>/Add' incorporates:
    //   Math: '<S23>/Square'
    //   Math: '<S26>/Transpose1'
    //   Product: '<S28>/Matrix Multiply'

    for (i = 0; i < 4; i++) {
      for (rtb_Transpose1_tmp = 0; rtb_Transpose1_tmp < 4; rtb_Transpose1_tmp++)
      {
        rtb_MatrixDivide_1 = 0.0F;
        for (rtb_Bias1_tmp = 0; rtb_Bias1_tmp < 6; rtb_Bias1_tmp++) {
          rtb_MatrixDivide_1 += rtb_Transpose1[(rtb_Bias1_tmp << 2) + i] *
            rtb_MatrixDivide[6 * rtb_Transpose1_tmp + rtb_Bias1_tmp];
        }

        rtb_Bias1_tmp = (rtb_Transpose1_tmp << 2) + i;
        rtb_Transpose1_0[rtb_Bias1_tmp] = rtb_Square_l[rtb_Bias1_tmp] +
          rtb_MatrixDivide_1;
      }
    }

    // End of Sum: '<S28>/Add'

    // Product: '<S28>/Matrix Divide'
    rt_mrdivide_U1f6x4_U2f4x4_Yf6x4(rtb_MatrixDivide, rtb_Transpose1_0);

    // Sum: '<S30>/Subtract' incorporates:
    //   Constant: '<S30>/Constant'
    //   Math: '<S26>/Transpose1'
    //   Product: '<S10>/Matrix Multiply2'
    //   Product: '<S30>/Matrix Multiply'

    for (i = 0; i < 6; i++) {
      // Product: '<S30>/Matrix Multiply' incorporates:
      //   Product: '<S28>/Matrix Divide'

      rtb_Switch = rtb_MatrixDivide[i + 6];
      rtb_MatrixDivide_1 = rtb_MatrixDivide[i];
      rtb_Gain1_hp = rtb_MatrixDivide[i + 12];
      rtb_MatrixDivide_2 = rtb_MatrixDivide[i + 18];
      for (rtb_Transpose1_tmp = 0; rtb_Transpose1_tmp < 6; rtb_Transpose1_tmp++)
      {
        // Product: '<S30>/Matrix Multiply'
        rtb_Bias1_tmp = rtb_Transpose1_tmp << 2;
        rtb_MatrixMultiply_tmp = 6 * rtb_Transpose1_tmp + i;
        rtb_MatrixMultiply[rtb_MatrixMultiply_tmp] =
          NASDAQ0_P.Constant_Value_a[rtb_MatrixMultiply_tmp] -
          (((rtb_Transpose1[rtb_Bias1_tmp + 1] * rtb_Switch +
             rtb_Transpose1[rtb_Bias1_tmp] * rtb_MatrixDivide_1) +
            rtb_Transpose1[rtb_Bias1_tmp + 2] * rtb_Gain1_hp) +
           rtb_Transpose1[rtb_Bias1_tmp + 3] * rtb_MatrixDivide_2);
      }
    }

    // End of Sum: '<S30>/Subtract'

    // Product: '<S26>/Matrix Multiply' incorporates:
    //   Bias: '<S3>/Bias1'
    //   Math: '<S26>/Transpose'
    //   Product: '<S10>/Matrix Multiply2'

    for (i = 0; i < 6; i++) {
      for (rtb_Transpose1_tmp = 0; rtb_Transpose1_tmp < 6; rtb_Transpose1_tmp++)
      {
        rtb_Switch = 0.0F;
        for (rtb_Bias1_tmp = 0; rtb_Bias1_tmp < 6; rtb_Bias1_tmp++) {
          rtb_Switch += rtb_Bias1[6 * rtb_Bias1_tmp + i] * rtb_MatrixMultiply[6 *
            rtb_Bias1_tmp + rtb_Transpose1_tmp];
        }

        rtb_Merge1[i + 6 * rtb_Transpose1_tmp] = rtb_Switch;
      }
    }

    // Product: '<S26>/Matrix Multiply2' incorporates:
    //   Math: '<S23>/Square'
    //   Math: '<S26>/Transpose1'
    //   Product: '<S28>/Matrix Divide'

    for (i = 0; i < 4; i++) {
      rtb_Switch = rtb_Square_l[i + 4];
      rtb_MatrixDivide_1 = rtb_Square_l[i];
      rtb_Gain1_hp = rtb_Square_l[i + 8];
      rtb_MatrixDivide_2 = rtb_Square_l[i + 12];
      for (rtb_Transpose1_tmp = 0; rtb_Transpose1_tmp < 6; rtb_Transpose1_tmp++)
      {
        rtb_Transpose1[i + (rtb_Transpose1_tmp << 2)] =
          ((rtb_MatrixDivide[rtb_Transpose1_tmp + 6] * rtb_Switch +
            rtb_MatrixDivide_1 * rtb_MatrixDivide[rtb_Transpose1_tmp]) +
           rtb_MatrixDivide[rtb_Transpose1_tmp + 12] * rtb_Gain1_hp) +
          rtb_MatrixDivide[rtb_Transpose1_tmp + 18] * rtb_MatrixDivide_2;
      }
    }

    for (i = 0; i < 6; i++) {
      for (rtb_Transpose1_tmp = 0; rtb_Transpose1_tmp < 6; rtb_Transpose1_tmp++)
      {
        // Product: '<S26>/Matrix Multiply' incorporates:
        //   Product: '<S10>/Matrix Multiply2'
        //   Product: '<S26>/Matrix Multiply2'

        rtb_MatrixDivide_1 = 0.0F;
        for (rtb_Bias1_tmp = 0; rtb_Bias1_tmp < 6; rtb_Bias1_tmp++) {
          rtb_MatrixDivide_1 += rtb_MatrixMultiply[6 * rtb_Bias1_tmp +
            rtb_Transpose1_tmp] * rtb_Merge1[6 * i + rtb_Bias1_tmp];
        }

        rtb_Bias1_tmp = 6 * i + rtb_Transpose1_tmp;
        rtb_MatrixMultiply_0[rtb_Bias1_tmp] = rtb_MatrixDivide_1;

        // Product: '<S26>/Matrix Multiply2' incorporates:
        //   Product: '<S28>/Matrix Divide'

        rtb_MatrixMultiply_tmp = i << 2;
        rtb_MatrixDivide_0[rtb_Bias1_tmp] =
          ((rtb_Transpose1[rtb_MatrixMultiply_tmp + 1] *
            rtb_MatrixDivide[rtb_Transpose1_tmp + 6] +
            rtb_Transpose1[rtb_MatrixMultiply_tmp] *
            rtb_MatrixDivide[rtb_Transpose1_tmp]) +
           rtb_Transpose1[rtb_MatrixMultiply_tmp + 2] *
           rtb_MatrixDivide[rtb_Transpose1_tmp + 12]) +
          rtb_Transpose1[rtb_MatrixMultiply_tmp + 3] *
          rtb_MatrixDivide[rtb_Transpose1_tmp + 18];
      }
    }

    // Sum: '<S26>/Sum1' incorporates:
    //   Merge: '<S6>/Merge1'

    for (i = 0; i < 36; i++) {
      rtb_Merge1[i] = rtb_MatrixMultiply_0[i] + rtb_MatrixDivide_0[i];
    }

    // End of Sum: '<S26>/Sum1'

    // Bias: '<S29>/Bias' incorporates:
    //   Gain: '<S29>/Gain3'

    rtb_Gain1_hp = NASDAQ0_P.Gain3_Gain_g * rtb_VectorConcatenate[0] +
      NASDAQ0_P.Bias_Bias_g;

    // Gain: '<S29>/Gain' incorporates:
    //   Bias: '<S29>/Bias1'
    //   Gain: '<S29>/Gain4'
    //   Gain: '<S32>/Gain1'
    //   Inport: '<Root>/GPS'
    //   Product: '<S29>/Divide1'
    //   SignalConversion generated from: '<S29>/Vector Concatenate'
    //   Sum: '<S29>/Add1'
    //   Trigonometry: '<S29>/Cos1'

    rtb_Switch = (NASDAQ0_U.GPS.Measure[0] - rtb_Gain1_hp) *
      NASDAQ0_P.Gain_Gain_jx[0];
    rtb_MatrixDivide_1 = (NASDAQ0_U.GPS.Measure[1] - (1.0F / (std::cos
      (NASDAQ0_P.Gain1_Gain_g * rtb_Gain1_hp) * NASDAQ0_P.Gain4_Gain_o) *
      rtb_VectorConcatenate[1] + NASDAQ0_P.Bias1_Bias)) *
      NASDAQ0_P.Gain_Gain_jx[1];

    // Outputs for Atomic SubSystem: '<S1>/Prediction Step'
    rtb_Gain1_hp = (NASDAQ0_U.GPS.Measure[3] - rtb_NASStateInterface[3]) *
      NASDAQ0_P.Gain_Gain_jx[2];
    rtb_MatrixDivide_2 = (NASDAQ0_U.GPS.Measure[4] - rtb_NASStateInterface[4]) *
      NASDAQ0_P.Gain_Gain_jx[3];

    // End of Outputs for SubSystem: '<S1>/Prediction Step'
    for (i = 0; i < 6; i++) {
      // Sum: '<S23>/Add' incorporates:
      //   Product: '<S23>/Matrix Multiply2'
      //   Product: '<S28>/Matrix Divide'

      rtb_Merge[i] = (((rtb_MatrixDivide[i + 6] * rtb_MatrixDivide_1 +
                        rtb_MatrixDivide[i] * rtb_Switch) + rtb_MatrixDivide[i +
                       12] * rtb_Gain1_hp) + rtb_MatrixDivide[i + 18] *
                      rtb_MatrixDivide_2) + rtb_VectorConcatenate[i];
    }
  }

  // End of Outputs for SubSystem: '<S6>/Active Correction Step'

  // Outputs for Enabled SubSystem: '<S6>/No Correction Step'
  // Logic: '<S6>/NOT'
  NASDAQ0_NoCorrectionStep(!rtb_AND, rtb_VectorConcatenate, rtb_Bias1, rtb_Merge,
    rtb_Merge1);

  // End of Outputs for SubSystem: '<S6>/No Correction Step'

  // Update for Memory: '<S24>/Memory' incorporates:
  //   Inport: '<Root>/GPS'

  NASDAQ0_DW.Memory_PreviousInput = NASDAQ0_U.GPS.Timestamp;

  // End of Outputs for SubSystem: '<S2>/GPS Correction'

  // Outputs for Atomic SubSystem: '<S2>/Baro Correction'
  // Logic: '<S15>/AND' incorporates:
  //   Constant: '<S15>/Constant'
  //   Inport: '<Root>/Baro'
  //   Memory: '<S15>/Memory'
  //   RelationalOperator: '<S15>/Relational Operator'

  rtb_AND = (NASDAQ0_P.Constant_Value_n0 && (NASDAQ0_U.Baro.Timestamp >
              NASDAQ0_DW.Memory_PreviousInput_m));

  // Outputs for Enabled SubSystem: '<S5>/Active Correction Step' incorporates:
  //   EnablePort: '<S14>/Enable'

  if (rtb_AND) {
    // Constant: '<S18>/Constant'
    rtb_NASStateInterface[0] = NASDAQ0_P.Constant_Value_j[0];
    rtb_NASStateInterface[1] = NASDAQ0_P.Constant_Value_j[1];

    // Sum: '<S18>/Sum1' incorporates:
    //   Constant: '<S18>/StandardAirTemperature T0'
    //   Gain: '<S18>/Gain4'

    rtb_Gain1_hp = NASDAQ0_P.Gain4_Gain * rtb_Merge[2] +
      NASDAQ0_P.StandardAirTemperatureT0_Value;

    // Product: '<S18>/Product1' incorporates:
    //   Constant: '<S18>/StandardAirPressure P0'
    //   Constant: '<S18>/g R'
    //   Gain: '<S18>/Gain3'
    //   Math: '<S18>/Exp'
    //   Math: '<S18>/Square'
    //   Product: '<S18>/Divide2'
    //   Product: '<S18>/Divide3'
    //   Product: '<S18>/Product4'
    //
    //  About '<S18>/Exp':
    //   Operator: exp

    rtb_NASStateInterface[2] = NASDAQ0_P.Gain3_Gain *
      NASDAQ0_P.StandardAirPressureP0_Value / (rtb_Gain1_hp * rtb_Gain1_hp) *
      std::exp(NASDAQ0_P.gR_Value / rtb_Gain1_hp * rtb_Merge[2]);

    // Constant: '<S18>/Constant1'
    rtb_NASStateInterface[3] = NASDAQ0_P.Constant1_Value[0];
    rtb_NASStateInterface[4] = NASDAQ0_P.Constant1_Value[1];
    rtb_NASStateInterface[5] = NASDAQ0_P.Constant1_Value[2];

    // Math: '<S14>/Square' incorporates:
    //   Constant: '<S14>/Constant'

    rtb_Switch = NASDAQ0_P.Constant_Value_jb * NASDAQ0_P.Constant_Value_jb;

    // Product: '<S19>/Matrix Multiply' incorporates:
    //   Concatenate: '<S18>/Vector Concatenate'

    rtb_MatrixDivide_1 = 0.0F;
    for (i = 0; i < 6; i++) {
      rtb_Gain1_hp = 0.0F;

      // Product: '<S19>/Matrix Multiply1' incorporates:
      //   Concatenate: '<S18>/Vector Concatenate'
      //   Math: '<S19>/Transpose'
      //   Merge: '<S6>/Merge1'

      for (rtb_Transpose1_tmp = 0; rtb_Transpose1_tmp < 6; rtb_Transpose1_tmp++)
      {
        rtb_Gain1_hp += rtb_Merge1[6 * rtb_Transpose1_tmp + i] *
          rtb_NASStateInterface[rtb_Transpose1_tmp];
      }

      rtb_VectorConcatenate[i] = rtb_Gain1_hp;
      rtb_MatrixDivide_1 += rtb_NASStateInterface[i] * rtb_Gain1_hp;
    }

    // Sum: '<S19>/Add' incorporates:
    //   Product: '<S19>/Matrix Multiply'

    rtb_Gain1_hp = rtb_MatrixDivide_1 + rtb_Switch;

    // Product: '<S19>/Matrix Divide' incorporates:
    //   Product: '<S11>/Matrix Divide'
    //   Product: '<S19>/Matrix Multiply1'

    for (i = 0; i < 6; i++) {
      rtb_VectorConcatenate[i] /= rtb_Gain1_hp;
    }

    // End of Product: '<S19>/Matrix Divide'

    // Sum: '<S21>/Subtract' incorporates:
    //   Concatenate: '<S18>/Vector Concatenate'
    //   Constant: '<S21>/Constant'
    //   Product: '<S10>/Matrix Multiply2'
    //   Product: '<S11>/Matrix Divide'
    //   Product: '<S21>/Matrix Multiply'

    for (i = 0; i < 6; i++) {
      for (rtb_Transpose1_tmp = 0; rtb_Transpose1_tmp < 6; rtb_Transpose1_tmp++)
      {
        rtb_Bias1_tmp = 6 * i + rtb_Transpose1_tmp;
        rtb_MatrixMultiply[rtb_Bias1_tmp] =
          NASDAQ0_P.Constant_Value_h[rtb_Bias1_tmp] -
          rtb_VectorConcatenate[rtb_Transpose1_tmp] * rtb_NASStateInterface[i];
      }
    }

    // End of Sum: '<S21>/Subtract'

    // Product: '<S17>/Matrix Multiply' incorporates:
    //   Math: '<S17>/Transpose'
    //   Merge: '<S6>/Merge1'
    //   Product: '<S10>/Matrix Multiply2'

    for (i = 0; i < 6; i++) {
      for (rtb_Transpose1_tmp = 0; rtb_Transpose1_tmp < 6; rtb_Transpose1_tmp++)
      {
        rtb_MatrixDivide_1 = 0.0F;
        for (rtb_Bias1_tmp = 0; rtb_Bias1_tmp < 6; rtb_Bias1_tmp++) {
          rtb_MatrixDivide_1 += rtb_Merge1[6 * rtb_Bias1_tmp + i] *
            rtb_MatrixMultiply[6 * rtb_Bias1_tmp + rtb_Transpose1_tmp];
        }

        rtb_Merge_fr[i + 6 * rtb_Transpose1_tmp] = rtb_MatrixDivide_1;
      }
    }

    for (i = 0; i < 6; i++) {
      // Product: '<S17>/Matrix Multiply2' incorporates:
      //   Math: '<S17>/Transpose1'
      //   Product: '<S10>/Matrix Multiply2'
      //   Product: '<S11>/Matrix Divide'

      for (rtb_Transpose1_tmp = 0; rtb_Transpose1_tmp < 6; rtb_Transpose1_tmp++)
      {
        rtb_MatrixDivide_1 = 0.0F;
        for (rtb_Bias1_tmp = 0; rtb_Bias1_tmp < 6; rtb_Bias1_tmp++) {
          rtb_MatrixDivide_1 += rtb_MatrixMultiply[6 * rtb_Bias1_tmp + i] *
            rtb_Merge_fr[6 * rtb_Transpose1_tmp + rtb_Bias1_tmp];
        }

        rtb_MatrixMultiply_0[i + 6 * rtb_Transpose1_tmp] = rtb_MatrixDivide_1;
        rtb_Bias1[rtb_Transpose1_tmp + 6 * i] = rtb_Switch *
          rtb_VectorConcatenate[i] * rtb_VectorConcatenate[rtb_Transpose1_tmp];
      }

      // End of Product: '<S17>/Matrix Multiply2'
    }

    // End of Product: '<S17>/Matrix Multiply'

    // Sum: '<S17>/Sum1' incorporates:
    //   Merge: '<S5>/Merge'

    for (i = 0; i < 36; i++) {
      rtb_Merge_fr[i] = rtb_MatrixMultiply_0[i] + rtb_Bias1[i];
    }

    // End of Sum: '<S17>/Sum1'

    // Sum: '<S20>/Subtract' incorporates:
    //   Constant: '<S22>/HeightTemperatureGradient'
    //   Constant: '<S22>/R air'
    //   Constant: '<S22>/StandardAirPressure P0'
    //   Constant: '<S22>/StandardAirTemperature T0'
    //   Constant: '<S22>/gravity'
    //   Gain: '<S20>/Gain'
    //   Inport: '<Root>/Baro'
    //   Math: '<S22>/Power'
    //   Product: '<S22>/Divide'
    //   Product: '<S22>/Divide1'
    //   Product: '<S22>/Product'
    //   Product: '<S22>/Product2'
    //   Product: '<S22>/Product3'
    //   Sum: '<S22>/Subtract'

    rtb_Gain1_hp = NASDAQ0_U.Baro.Measure - std::pow
      ((NASDAQ0_P.StandardAirTemperatureT0_Valu_o - NASDAQ0_P.Gain_Gain_j *
        rtb_Merge[2] * NASDAQ0_P.HeightTemperatureGradient_Value) /
       NASDAQ0_P.StandardAirTemperatureT0_Valu_o, NASDAQ0_P.gravity_Value /
       (NASDAQ0_P.HeightTemperatureGradient_Value * NASDAQ0_P.Rair_Value)) *
      NASDAQ0_P.StandardAirPressureP0_Value_a;

    // Sum: '<S14>/Add' incorporates:
    //   Product: '<S11>/Matrix Divide'
    //   Product: '<S14>/Matrix Multiply2'

    for (i = 0; i < 6; i++) {
      rtb_Merge1_a[i] = rtb_VectorConcatenate[i] * rtb_Gain1_hp + rtb_Merge[i];
    }

    // End of Sum: '<S14>/Add'
  }

  // End of Outputs for SubSystem: '<S5>/Active Correction Step'

  // Outputs for Enabled SubSystem: '<S5>/No Correction Step'
  // Logic: '<S5>/NOT'
  NASDAQ0_NoCorrectionStep(!rtb_AND, rtb_Merge, rtb_Merge1, rtb_Merge1_a,
    rtb_Merge_fr);

  // End of Outputs for SubSystem: '<S5>/No Correction Step'

  // Update for Memory: '<S15>/Memory' incorporates:
  //   Inport: '<Root>/Baro'

  NASDAQ0_DW.Memory_PreviousInput_m = NASDAQ0_U.Baro.Timestamp;

  // End of Outputs for SubSystem: '<S2>/Baro Correction'

  // Outputs for Atomic SubSystem: '<S2>/ADA Correction'
  // Logic: '<S8>/AND' incorporates:
  //   Constant: '<S8>/Constant'
  //   Inport: '<Root>/ADA States'
  //   Memory: '<S8>/Memory'
  //   RelationalOperator: '<S8>/Relational Operator'

  rtb_AND = ((NASDAQ0_P.nasdaq.flags.ADA != 0.0) &&
             (NASDAQ0_U.ADAStates.timestamp > NASDAQ0_DW.Memory_PreviousInput_mp));

  // Outputs for Enabled SubSystem: '<S4>/Active Correction Step' incorporates:
  //   EnablePort: '<S7>/Enable'

  if (rtb_AND) {
    // Switch: '<S7>/Switch' incorporates:
    //   Constant: '<S7>/Constant'
    //   Constant: '<S7>/Constant1'
    //   Inport: '<Root>/ADA States'
    //   Math: '<S7>/Square'

    if (NASDAQ0_P.Constant1_Value_m) {
      rtb_Switch = NASDAQ0_U.ADAStates.verticalSpeedCovariance;
    } else {
      rtb_Switch = NASDAQ0_P.Constant_Value * NASDAQ0_P.Constant_Value;
    }

    // End of Switch: '<S7>/Switch'

    // Product: '<S11>/Matrix Multiply' incorporates:
    //   Constant: '<S7>/H Matrix'
    //   Math: '<S11>/Transpose'
    //   Merge: '<S5>/Merge'

    rtb_MatrixDivide_1 = 0.0F;
    for (i = 0; i < 6; i++) {
      rtb_Gain1_hp = 0.0F;
      for (rtb_Transpose1_tmp = 0; rtb_Transpose1_tmp < 6; rtb_Transpose1_tmp++)
      {
        rtb_Gain1_hp += rtb_Merge_fr[6 * i + rtb_Transpose1_tmp] * static_cast<
          float>(NASDAQ0_P.HMatrix_Value[rtb_Transpose1_tmp]);
      }

      rtb_MatrixDivide_1 += rtb_Gain1_hp * static_cast<float>
        (NASDAQ0_P.HMatrix_Value[i]);
    }

    // Sum: '<S11>/Add' incorporates:
    //   Product: '<S11>/Matrix Multiply'
    //   Switch: '<S7>/Switch'

    rtb_Gain1_hp = rtb_MatrixDivide_1 + rtb_Switch;

    // Product: '<S11>/Matrix Divide' incorporates:
    //   Constant: '<S7>/H Matrix'
    //   Math: '<S11>/Transpose'
    //   Merge: '<S5>/Merge'
    //   Product: '<S11>/Matrix Multiply1'

    for (i = 0; i < 6; i++) {
      rtb_MatrixDivide_1 = 0.0F;
      for (rtb_Transpose1_tmp = 0; rtb_Transpose1_tmp < 6; rtb_Transpose1_tmp++)
      {
        rtb_MatrixDivide_1 += rtb_Merge_fr[6 * rtb_Transpose1_tmp + i] *
          static_cast<float>(NASDAQ0_P.HMatrix_Value[rtb_Transpose1_tmp]);
      }

      rtb_VectorConcatenate[i] = rtb_MatrixDivide_1 / rtb_Gain1_hp;
    }

    // End of Product: '<S11>/Matrix Divide'

    // Sum: '<S13>/Subtract' incorporates:
    //   Constant: '<S13>/Constant'
    //   Constant: '<S7>/H Matrix'
    //   Product: '<S10>/Matrix Multiply2'
    //   Product: '<S11>/Matrix Divide'
    //   Product: '<S13>/Matrix Multiply'

    for (i = 0; i < 6; i++) {
      for (rtb_Transpose1_tmp = 0; rtb_Transpose1_tmp < 6; rtb_Transpose1_tmp++)
      {
        rtb_Bias1_tmp = 6 * i + rtb_Transpose1_tmp;
        rtb_MatrixMultiply[rtb_Bias1_tmp] =
          NASDAQ0_P.Constant_Value_g[rtb_Bias1_tmp] -
          rtb_VectorConcatenate[rtb_Transpose1_tmp] * static_cast<float>
          (NASDAQ0_P.HMatrix_Value[i]);
      }
    }

    // End of Sum: '<S13>/Subtract'

    // Product: '<S10>/Matrix Multiply' incorporates:
    //   Math: '<S10>/Transpose'
    //   Merge: '<S5>/Merge'
    //   Product: '<S10>/Matrix Multiply2'

    for (i = 0; i < 6; i++) {
      for (rtb_Transpose1_tmp = 0; rtb_Transpose1_tmp < 6; rtb_Transpose1_tmp++)
      {
        rtb_MatrixDivide_1 = 0.0F;
        for (rtb_Bias1_tmp = 0; rtb_Bias1_tmp < 6; rtb_Bias1_tmp++) {
          rtb_MatrixDivide_1 += rtb_Merge_fr[6 * rtb_Bias1_tmp + i] *
            rtb_MatrixMultiply[6 * rtb_Bias1_tmp + rtb_Transpose1_tmp];
        }

        rtb_Merge_j[i + 6 * rtb_Transpose1_tmp] = rtb_MatrixDivide_1;
      }
    }

    for (i = 0; i < 6; i++) {
      // Product: '<S10>/Matrix Multiply2' incorporates:
      //   Math: '<S10>/Transpose1'
      //   Product: '<S11>/Matrix Divide'
      //   Switch: '<S7>/Switch'

      for (rtb_Transpose1_tmp = 0; rtb_Transpose1_tmp < 6; rtb_Transpose1_tmp++)
      {
        rtb_MatrixDivide_1 = 0.0F;
        for (rtb_Bias1_tmp = 0; rtb_Bias1_tmp < 6; rtb_Bias1_tmp++) {
          rtb_MatrixDivide_1 += rtb_MatrixMultiply[6 * rtb_Bias1_tmp + i] *
            rtb_Merge_j[6 * rtb_Transpose1_tmp + rtb_Bias1_tmp];
        }

        rtb_MatrixMultiply_0[i + 6 * rtb_Transpose1_tmp] = rtb_MatrixDivide_1;
        rtb_Bias1[rtb_Transpose1_tmp + 6 * i] = rtb_Switch *
          rtb_VectorConcatenate[i] * rtb_VectorConcatenate[rtb_Transpose1_tmp];
      }

      // End of Product: '<S10>/Matrix Multiply2'
    }

    // End of Product: '<S10>/Matrix Multiply'

    // Sum: '<S10>/Sum1' incorporates:
    //   Merge: '<S4>/Merge'

    for (i = 0; i < 36; i++) {
      rtb_Merge_j[i] = rtb_MatrixMultiply_0[i] + rtb_Bias1[i];
    }

    // End of Sum: '<S10>/Sum1'

    // Sum: '<S12>/Subtract' incorporates:
    //   Gain: '<S4>/Gain'
    //   Inport: '<Root>/ADA States'

    rtb_Gain1_hp = NASDAQ0_P.Gain_Gain * NASDAQ0_U.ADAStates.verticalSpeed -
      rtb_Merge1_a[5];

    // Sum: '<S7>/Add' incorporates:
    //   Merge: '<S4>/Merge1'
    //   Product: '<S11>/Matrix Divide'
    //   Product: '<S7>/Matrix Multiply2'

    for (i = 0; i < 6; i++) {
      rtb_Merge1_b[i] = rtb_VectorConcatenate[i] * rtb_Gain1_hp + rtb_Merge1_a[i];
    }

    // End of Sum: '<S7>/Add'
  }

  // End of Outputs for SubSystem: '<S4>/Active Correction Step'

  // Outputs for Enabled SubSystem: '<S4>/No Correction Step'
  // Logic: '<S4>/NOT'
  NASDAQ0_NoCorrectionStep(!rtb_AND, rtb_Merge1_a, rtb_Merge_fr, rtb_Merge1_b,
    rtb_Merge_j);

  // End of Outputs for SubSystem: '<S4>/No Correction Step'

  // Update for Memory: '<S8>/Memory' incorporates:
  //   Inport: '<Root>/ADA States'

  NASDAQ0_DW.Memory_PreviousInput_mp = NASDAQ0_U.ADAStates.timestamp;

  // End of Outputs for SubSystem: '<S2>/ADA Correction'
  // End of Outputs for SubSystem: '<S1>/Correction Step'

  // S-Function (sdspdiag2): '<S1>/Extract Diagonal' incorporates:
  //   Merge: '<S4>/Merge'

  for (i = 0; i < 6; i++) {
    rtb_NASStateInterface[static_cast<int32_t>(i)] = rtb_Merge_j
      [static_cast<int32_t>(i * 7LL)];
  }

  // End of S-Function (sdspdiag2): '<S1>/Extract Diagonal'

  // Update for Memory: '<S1>/NAS Variance Interface' incorporates:
  //   Merge: '<S4>/Merge'

  std::memcpy(&NASDAQ0_DW.NASVarianceInterface_PreviousIn[0], &rtb_Merge_j[0],
              36U * sizeof(float));

  // End of Outputs for SubSystem: '<Root>/NASDAQ'

  // Outport: '<Root>/Position'
  NASDAQ0_Y.Position[0] = rtb_Merge1_b[0];

  // Outport: '<Root>/Velocity'
  NASDAQ0_Y.Velocity[0] = rtb_Merge1_b[0];

  // Outport: '<Root>/Position'
  NASDAQ0_Y.Position[1] = rtb_Merge1_b[1];

  // Outport: '<Root>/Velocity'
  NASDAQ0_Y.Velocity[1] = rtb_Merge1_b[1];

  // Outport: '<Root>/Position'
  NASDAQ0_Y.Position[2] = rtb_Merge1_b[2];

  // Outport: '<Root>/Velocity'
  NASDAQ0_Y.Velocity[2] = rtb_Merge1_b[2];

  // Outputs for Atomic SubSystem: '<Root>/NASDAQ'
  for (i = 0; i < 6; i++) {
    // Update for Memory: '<S1>/NAS State Interface'
    NASDAQ0_DW.NASStateInterface_PreviousInput[i] = rtb_Merge1_b[i];

    // Outport: '<Root>/Covariance'
    NASDAQ0_Y.Covariance[i] = rtb_NASStateInterface[i];
  }

  // End of Outputs for SubSystem: '<Root>/NASDAQ'
  rate_scheduler((&NASDAQ0_M));
}

// Model initialize function
void NASDAQ0::initialize()
{
  {
    int16_t i;

    // SystemInitialize for Atomic SubSystem: '<Root>/NASDAQ'
    // InitializeConditions for Memory: '<S1>/NAS Variance Interface'
    std::memcpy(&NASDAQ0_DW.NASVarianceInterface_PreviousIn[0],
                &NASDAQ0_P.NASVarianceInterface_InitialCon[0], 36U * sizeof
                (float));

    // InitializeConditions for Memory: '<S1>/NAS State Interface'
    for (i = 0; i < 6; i++) {
      NASDAQ0_DW.NASStateInterface_PreviousInput[i] =
        NASDAQ0_P.NASStateInterface_InitialCondit[i];
    }

    // End of InitializeConditions for Memory: '<S1>/NAS State Interface'

    // SystemInitialize for Atomic SubSystem: '<S1>/Correction Step'
    // SystemInitialize for Atomic SubSystem: '<S2>/GPS Correction'
    // InitializeConditions for Memory: '<S24>/Memory'
    NASDAQ0_DW.Memory_PreviousInput = NASDAQ0_P.Memory_InitialCondition_b;

    // End of SystemInitialize for SubSystem: '<S2>/GPS Correction'

    // SystemInitialize for Atomic SubSystem: '<S2>/Baro Correction'
    // InitializeConditions for Memory: '<S15>/Memory'
    NASDAQ0_DW.Memory_PreviousInput_m = NASDAQ0_P.Memory_InitialCondition;

    // End of SystemInitialize for SubSystem: '<S2>/Baro Correction'

    // SystemInitialize for Atomic SubSystem: '<S2>/ADA Correction'
    // InitializeConditions for Memory: '<S8>/Memory'
    NASDAQ0_DW.Memory_PreviousInput_mp = NASDAQ0_P.Memory_InitialCondition_l;

    // End of SystemInitialize for SubSystem: '<S2>/ADA Correction'
    // End of SystemInitialize for SubSystem: '<S1>/Correction Step'
    // End of SystemInitialize for SubSystem: '<Root>/NASDAQ'
  }
}

// Model terminate function
void NASDAQ0::terminate()
{
  // (no terminate code required)
}

// Constructor
NASDAQ0::NASDAQ0() :
  NASDAQ0_U(),
  NASDAQ0_Y(),
  NASDAQ0_DW(),
  NASDAQ0_M()
{
  // Currently there is no constructor body generated.
}

// Destructor
// Currently there is no destructor body generated.
NASDAQ0::~NASDAQ0() = default;

// Real-Time Model get method
NASDAQ0::RT_MODEL_NASDAQ0_T * NASDAQ0::getRTM()
{
  return (&NASDAQ0_M);
}

//
// File trailer for generated code.
//
// [EOF]
//
