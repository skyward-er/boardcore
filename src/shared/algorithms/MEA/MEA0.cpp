//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// File: MEA0.cpp
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
#include "MEA0.h"
#include <stdint.h>
#include "multiword_types.h"
#include "MEA0_private.h"

static void rate_scheduler(MEA0::RT_MODEL_MEA0_T *const MEA0_M);
bool uMultiWordGt(const uint32_t u1[], const uint32_t u2[], int32_t n)
{
  return uMultiWordCmp(u1, u2, n) > 0;
}

int32_t uMultiWordCmp(const uint32_t u1[], const uint32_t u2[], int32_t n)
{
  int32_t i;
  int32_t y;
  y = 0;
  i = n;
  while ((y == 0) && (i > 0)) {
    uint32_t u1i;
    uint32_t u2i;
    i--;
    u1i = u1[i];
    u2i = u2[i];
    if (u1i != u2i) {
      y = u1i > u2i ? 1 : -1;
    }
  }

  return y;
}

//
//         This function updates active task flag for each subrate.
//         The function is called at model base rate, hence the
//         generated code self-manages all its subrates.
//
static void rate_scheduler(MEA0::RT_MODEL_MEA0_T *const MEA0_M)
{
  // Compute which subrates run during the next base time step.  Subrates
  //  are an integer multiple of the base rate counter.  Therefore, the subtask
  //  counter is reset when it reaches its limit (zero means run).

  (MEA0_M->Timing.TaskCounters.TID[1])++;
  if ((MEA0_M->Timing.TaskCounters.TID[1]) > 1) {// Sample time: [0.02s, 0.0s]
    MEA0_M->Timing.TaskCounters.TID[1] = 0;
  }
}

// Model step function
void MEA0::step(float verticalSpeed, float mslAltitude)
{
  // Manually Added: Update internal speed and altitude
  this->lastVerticalSpeed = verticalSpeed;
  this->lastMslAltitude = mslAltitude;

  const uint64m_T *tmp_6;
  uint64m_T *tmp_7;
  float tmp[4];
  float v1[4];
  float APA;
  float Ax;
  float Constant1_a;
  float tmp_0;
  float tmp_1;
  float tmp_2;
  float tmp_3;
  float tmp_4;
  float tmp_5;
  float v1_0;
  float v1_idx_0;
  float v1_idx_1;
  int32_t i;
  int32_t v1_tmp;
  int32_t v1_tmp_0;
  if ((&MEA0_M)->Timing.TaskCounters.TID[1] == 0) {
    // Outputs for Atomic SubSystem: '<Root>/MEA - Biliquid'
    // Constant: '<S1>/Constant1'
    MEA0_B.Constant1 = MEA0_P.Constant1_Value_g;

    // Constant: '<S1>/Constant'
    MEA0_B.Constant[0] = MEA0_P.Constant_Value_o[0];

    // Constant: '<S4>/Constant1'
    Constant1_a = MEA0_P.Constant1_Value_m[0];

    // End of Outputs for SubSystem: '<Root>/MEA - Biliquid'
    MEA0_B.Constant1_a[0] = Constant1_a;

    // Outputs for Atomic SubSystem: '<Root>/MEA - Biliquid'
    // Product: '<S4>/Matrix Multiply2' incorporates:
    //   Inport: '<Root>/Main Valve Position'

    Constant1_a = MEA0_U.MainValvePosition ? Constant1_a : 0.0F;

    // Product: '<S4>/Matrix Multiply2'
    MEA0_B.Bu[0] = Constant1_a;

    // Constant: '<S1>/Constant'
    MEA0_B.Constant[1] = MEA0_P.Constant_Value_o[1];

    // Constant: '<S4>/Constant1'
    Constant1_a = MEA0_P.Constant1_Value_m[1];

    // End of Outputs for SubSystem: '<Root>/MEA - Biliquid'
    MEA0_B.Constant1_a[1] = Constant1_a;

    // Outputs for Atomic SubSystem: '<Root>/MEA - Biliquid'
    // Product: '<S4>/Matrix Multiply2' incorporates:
    //   Inport: '<Root>/Main Valve Position'

    Constant1_a = MEA0_U.MainValvePosition ? Constant1_a : 0.0F;

    // Product: '<S4>/Matrix Multiply2'
    MEA0_B.Bu[1] = Constant1_a;

    // Constant: '<S4>/Constant'
    MEA0_B.Constant_i[0] = MEA0_P.Constant_Value_i[0];
    MEA0_B.Constant_i[1] = MEA0_P.Constant_Value_i[1];
    MEA0_B.Constant_i[2] = MEA0_P.Constant_Value_i[2];
    MEA0_B.Constant_i[3] = MEA0_P.Constant_Value_i[3];

    // Memory: '<S1>/x_memory'
    MEA0_B.x_memory[0] = MEA0_DW.x_memory_PreviousInput[0];
    MEA0_B.x_memory[1] = MEA0_DW.x_memory_PreviousInput[1];

    // Product: '<S4>/Matrix Multiply' incorporates:
    //   Constant: '<S4>/Constant'
    //   Memory: '<S1>/x_memory'
    //   Product: '<S4>/Matrix Multiply1'

    Constant1_a = MEA0_B.Constant_i[0];
    tmp_1 = MEA0_B.Constant_i[1];
    tmp_0 = MEA0_B.Constant_i[2];
    tmp_2 = MEA0_B.Constant_i[3];
    tmp_3 = MEA0_B.x_memory[0];
    v1_0 = MEA0_B.x_memory[1];

    // Product: '<S4>/Matrix Multiply'
    Ax = Constant1_a * tmp_3;
    Ax += tmp_0 * v1_0;
    MEA0_B.Ax[0] = Ax;

    // Sum: '<S4>/Add1' incorporates:
    //   Product: '<S4>/Matrix Multiply'

    MEA0_B.AxBu[0] = MEA0_B.Bu[0] + Ax;

    // Product: '<S4>/Matrix Multiply'
    Ax = tmp_1 * tmp_3;
    Ax += tmp_2 * v1_0;
    MEA0_B.Ax[1] = Ax;

    // Sum: '<S4>/Add1' incorporates:
    //   Product: '<S4>/Matrix Multiply'

    MEA0_B.AxBu[1] = MEA0_B.Bu[1] + Ax;

    // Memory: '<S1>/P_memory'
    MEA0_B.P_memory[0] = MEA0_DW.P_memory_PreviousInput[0];
    MEA0_B.P_memory[1] = MEA0_DW.P_memory_PreviousInput[1];
    MEA0_B.P_memory[2] = MEA0_DW.P_memory_PreviousInput[2];
    MEA0_B.P_memory[3] = MEA0_DW.P_memory_PreviousInput[3];

    // Math: '<S4>/Transpose' incorporates:
    //   Constant: '<S4>/Constant'

    MEA0_B.Transpose[0] = MEA0_B.Constant_i[0];
    MEA0_B.Transpose[1] = MEA0_B.Constant_i[2];
    MEA0_B.Transpose[2] = MEA0_B.Constant_i[1];
    MEA0_B.Transpose[3] = MEA0_B.Constant_i[3];

    // Product: '<S4>/Matrix Multiply1' incorporates:
    //   Math: '<S4>/Transpose'
    //   Memory: '<S1>/P_memory'

    tmp[0] = MEA0_B.P_memory[0];
    v1[0] = MEA0_B.Transpose[0];
    tmp[1] = MEA0_B.P_memory[1];
    v1[1] = MEA0_B.Transpose[1];
    tmp[2] = MEA0_B.P_memory[2];
    v1[2] = MEA0_B.Transpose[2];
    tmp[3] = MEA0_B.P_memory[3];
    v1[3] = MEA0_B.Transpose[3];
    Ax = tmp[0];
    tmp_3 = tmp[2];
    v1_idx_1 = tmp[1];
    tmp_5 = tmp[3];
    for (i = 0; i < 2; i++) {
      v1_tmp = i << 1;
      APA = v1[v1_tmp];
      v1_idx_0 = APA * Ax;
      v1_tmp_0 = v1_tmp + 1;
      tmp_4 = v1[v1_tmp_0];
      v1_idx_0 += tmp_4 * tmp_3;
      v1_0 = v1_idx_0;
      v1_idx_0 = APA * v1_idx_1;
      v1_idx_0 += tmp_4 * tmp_5;

      // Product: '<S4>/Matrix Multiply1'
      APA = v1_0 * Constant1_a;
      APA += v1_idx_0 * tmp_0;
      MEA0_B.APA[v1_tmp] = APA;

      // Product: '<S4>/Matrix Multiply1'
      APA = v1_0 * tmp_1;
      APA += v1_idx_0 * tmp_2;
      MEA0_B.APA[v1_tmp_0] = APA;
    }

    // Constant: '<S4>/Constant2'
    Constant1_a = MEA0_P.Constant2_Value_j[0];

    // End of Outputs for SubSystem: '<Root>/MEA - Biliquid'
    MEA0_B.Constant2[0] = Constant1_a;

    // Outputs for Atomic SubSystem: '<Root>/MEA - Biliquid'
    // Sum: '<S4>/Add' incorporates:
    //   Constant: '<S4>/Constant2'
    //   Product: '<S4>/Matrix Multiply1'

    MEA0_B.APAQ[0] = MEA0_B.APA[0] + Constant1_a;

    // Constant: '<S4>/Constant2'
    Constant1_a = MEA0_P.Constant2_Value_j[1];

    // End of Outputs for SubSystem: '<Root>/MEA - Biliquid'
    MEA0_B.Constant2[1] = Constant1_a;

    // Outputs for Atomic SubSystem: '<Root>/MEA - Biliquid'
    // Sum: '<S4>/Add' incorporates:
    //   Constant: '<S4>/Constant2'
    //   Product: '<S4>/Matrix Multiply1'

    MEA0_B.APAQ[1] = MEA0_B.APA[1] + Constant1_a;

    // Constant: '<S4>/Constant2'
    Constant1_a = MEA0_P.Constant2_Value_j[2];

    // End of Outputs for SubSystem: '<Root>/MEA - Biliquid'
    MEA0_B.Constant2[2] = Constant1_a;

    // Outputs for Atomic SubSystem: '<Root>/MEA - Biliquid'
    // Sum: '<S4>/Add' incorporates:
    //   Constant: '<S4>/Constant2'
    //   Product: '<S4>/Matrix Multiply1'

    MEA0_B.APAQ[2] = MEA0_B.APA[2] + Constant1_a;

    // Constant: '<S4>/Constant2'
    Constant1_a = MEA0_P.Constant2_Value_j[3];

    // End of Outputs for SubSystem: '<Root>/MEA - Biliquid'
    MEA0_B.Constant2[3] = Constant1_a;

    // Outputs for Atomic SubSystem: '<Root>/MEA - Biliquid'
    // Sum: '<S4>/Add' incorporates:
    //   Constant: '<S4>/Constant2'
    //   Product: '<S4>/Matrix Multiply1'

    MEA0_B.APAQ[3] = MEA0_B.APA[3] + Constant1_a;

    // Memory: '<S1>/Memory'
    MEA0_B.Memory = MEA0_DW.Memory_PreviousInput;

    // RelationalOperator: '<S1>/Relational Operator' incorporates:
    //   Inport: '<Root>/Combustion Chamber Bus'
    //   Memory: '<S1>/Memory'

    tmp_6 = &MEA0_U.CombustionChamberBus.CCTimestamp;
    tmp_7 = &MEA0_B.Memory;

    // RelationalOperator: '<S1>/Relational Operator'
    MEA0_B.RelationalOperator = uMultiWordGt(&tmp_6->chunks[0U], &tmp_7->chunks
      [0U], 2);

    // Outputs for Enabled SubSystem: '<S1>/cc baro correction' incorporates:
    //   EnablePort: '<S3>/Enable'

    if (MEA0_B.RelationalOperator) {
      // Constant: '<S3>/Constant'
      Constant1_a = MEA0_P.Constant_Value_p[0];
      MEA0_B.Constant_o[0] = Constant1_a;

      // Math: '<S3>/Transpose' incorporates:
      //   Constant: '<S3>/Constant'

      MEA0_B.Transpose_g[0] = Constant1_a;

      // Constant: '<S3>/Constant'
      Constant1_a = MEA0_P.Constant_Value_p[1];
      MEA0_B.Constant_o[1] = Constant1_a;

      // Math: '<S3>/Transpose' incorporates:
      //   Constant: '<S3>/Constant'

      MEA0_B.Transpose_g[1] = Constant1_a;

      // Outputs for Enabled SubSystem: '<S3>/Subsystem' incorporates:
      //   EnablePort: '<S6>/Enable'

      // Product: '<S3>/Matrix Multiply' incorporates:
      //   Constant: '<S3>/Constant'
      //   Math: '<S3>/Transpose'
      //   Product: '<S6>/Matrix Multiply'
      //   Product: '<S6>/Matrix Multiply2'
      //   Sum: '<S4>/Add'

      Constant1_a = MEA0_B.APAQ[0];
      tmp_1 = MEA0_B.APAQ[1];
      tmp_0 = MEA0_B.APAQ[2];
      tmp_2 = MEA0_B.APAQ[3];

      // End of Outputs for SubSystem: '<S3>/Subsystem'
      tmp_3 = MEA0_B.Transpose_g[0];
      v1_0 = MEA0_B.Transpose_g[1];
      v1_idx_1 = Constant1_a * tmp_3;
      v1_idx_1 += tmp_0 * v1_0;
      v1_idx_0 = v1_idx_1;
      v1_idx_1 = tmp_1 * tmp_3;
      v1_idx_1 += tmp_2 * v1_0;
      Ax = MEA0_B.Constant_o[0];
      tmp_3 = Ax * v1_idx_0;
      Ax = MEA0_B.Constant_o[1];
      tmp_3 += Ax * v1_idx_1;

      // Product: '<S3>/Matrix Multiply'
      MEA0_B.CPC = tmp_3;

      // Constant: '<S3>/Constant1'
      MEA0_B.R = MEA0_P.Constant1_Value_k;

      // Sum: '<S3>/Sum'
      MEA0_B.CPCR = MEA0_B.CPC + MEA0_B.R;

      // RelationalOperator: '<S5>/Compare' incorporates:
      //   Constant: '<S5>/Constant'

      MEA0_B.Compare = (MEA0_B.CPCR != MEA0_P.CompareToConstant_const);

      // Outputs for Enabled SubSystem: '<S3>/Subsystem' incorporates:
      //   EnablePort: '<S6>/Enable'

      if (MEA0_B.Compare) {
        // Constant: '<S6>/Constant'
        v1_0 = MEA0_P.Constant_Value[0];
        MEA0_B.Constant_ow[0] = v1_0;

        // Math: '<S6>/Transpose'
        MEA0_B.Transpose_a[0] = v1_0;

        // Product: '<S6>/Matrix Multiply' incorporates:
        //   Math: '<S6>/Transpose'

        tmp_3 = v1_0;

        // Constant: '<S6>/Constant'
        v1_0 = MEA0_P.Constant_Value[1];
        MEA0_B.Constant_ow[1] = v1_0;

        // Math: '<S6>/Transpose'
        MEA0_B.Transpose_a[1] = v1_0;

        // Product: '<S6>/Matrix Multiply'
        Ax = Constant1_a * tmp_3;
        Ax += tmp_0 * v1_0;
        MEA0_B.PC[0] = Ax;

        // Product: '<S6>/Divide' incorporates:
        //   Product: '<S6>/Matrix Multiply'

        MEA0_B.K[0] = Ax / MEA0_B.CPCR;

        // Product: '<S6>/Matrix Multiply'
        Ax = tmp_1 * tmp_3;
        Ax += tmp_2 * v1_0;
        MEA0_B.PC[1] = Ax;

        // Product: '<S6>/Divide' incorporates:
        //   Product: '<S6>/Matrix Multiply'

        MEA0_B.K[1] = Ax / MEA0_B.CPCR;

        // Product: '<S6>/Matrix Multiply1' incorporates:
        //   Constant: '<S6>/Constant'
        //   Product: '<S6>/Divide'
        //   Product: '<S6>/Matrix Multiply3'
        //   Product: '<S6>/Matrix Multiply4'

        tmp_3 = MEA0_B.K[0];
        v1_idx_0 = MEA0_B.Constant_ow[0];
        v1_0 = MEA0_B.K[1];
        v1_idx_1 = MEA0_B.Constant_ow[1];

        // Product: '<S6>/Matrix Multiply1'
        MEA0_B.KC[0] = tmp_3 * v1_idx_0;
        MEA0_B.KC[1] = v1_0 * v1_idx_0;
        MEA0_B.KC[2] = tmp_3 * v1_idx_1;
        MEA0_B.KC[3] = v1_0 * v1_idx_1;

        // Sum: '<S6>/Add' incorporates:
        //   IdentityMatrix: '<S6>/IdentityMatrix'
        //   Product: '<S6>/Matrix Multiply1'

        MEA0_B.IKC[0] = MEA0_B.I[0] - MEA0_B.KC[0];
        MEA0_B.IKC[1] = MEA0_B.I[1] - MEA0_B.KC[1];
        MEA0_B.IKC[2] = MEA0_B.I[2] - MEA0_B.KC[2];
        MEA0_B.IKC[3] = MEA0_B.I[3] - MEA0_B.KC[3];

        // Product: '<S6>/Matrix Multiply3' incorporates:
        //   Sum: '<S4>/Add1'

        Ax = MEA0_B.AxBu[0];
        v1_idx_0 *= Ax;
        Ax = MEA0_B.AxBu[1];
        v1_idx_0 += v1_idx_1 * Ax;

        // Product: '<S6>/Matrix Multiply3'
        MEA0_B.Cx = v1_idx_0;

        // Sum: '<S6>/Add1' incorporates:
        //   Inport: '<Root>/Combustion Chamber Bus'

        MEA0_B.e = MEA0_U.CombustionChamberBus.CCPressure - MEA0_B.Cx;

        // Constant: '<S6>/Constant1'
        MEA0_B.R_k = MEA0_P.Constant1_Value;

        // Product: '<S6>/Matrix Multiply4'
        Ax = MEA0_B.R_k;

        // Product: '<S6>/Product' incorporates:
        //   Product: '<S6>/Divide'

        v1_idx_0 = MEA0_B.K[0];

        // Product: '<S6>/Product' incorporates:
        //   Product: '<S6>/Divide'

        v1_idx_1 = v1_idx_0 * MEA0_B.e;
        MEA0_B.Ke[0] = v1_idx_1;

        // Merge: '<S1>/Merge' incorporates:
        //   Product: '<S6>/Product'
        //   Sum: '<S4>/Add1'
        //   Sum: '<S6>/Add2'

        MEA0_B.x_est[0] = MEA0_B.AxBu[0] + v1_idx_1;

        // Math: '<S6>/Transpose2'
        MEA0_B.K_d[0] = v1_idx_0;

        // Product: '<S6>/Matrix Multiply4'
        v1_idx_0 *= Ax;

        // Product: '<S6>/Matrix Multiply4'
        MEA0_B.KRK[0] = tmp_3 * v1_idx_0;

        // Math: '<S6>/Transpose1' incorporates:
        //   Sum: '<S6>/Add'

        MEA0_B.IKC_f[0] = MEA0_B.IKC[0];

        // Product: '<S6>/Matrix Multiply4'
        MEA0_B.KRK[1] = v1_0 * v1_idx_0;

        // Math: '<S6>/Transpose1' incorporates:
        //   Sum: '<S6>/Add'

        MEA0_B.IKC_f[1] = MEA0_B.IKC[2];

        // Product: '<S6>/Product' incorporates:
        //   Product: '<S6>/Divide'

        v1_idx_0 = MEA0_B.K[1];

        // Product: '<S6>/Product' incorporates:
        //   Product: '<S6>/Divide'

        v1_idx_1 = v1_idx_0 * MEA0_B.e;
        MEA0_B.Ke[1] = v1_idx_1;

        // Merge: '<S1>/Merge' incorporates:
        //   Product: '<S6>/Product'
        //   Sum: '<S4>/Add1'
        //   Sum: '<S6>/Add2'

        MEA0_B.x_est[1] = MEA0_B.AxBu[1] + v1_idx_1;

        // Math: '<S6>/Transpose2'
        MEA0_B.K_d[1] = v1_idx_0;

        // Product: '<S6>/Matrix Multiply4'
        v1_idx_0 *= Ax;
        v1_idx_1 = v1_idx_0;

        // Product: '<S6>/Matrix Multiply4'
        MEA0_B.KRK[2] = tmp_3 * v1_idx_1;

        // Math: '<S6>/Transpose1' incorporates:
        //   Sum: '<S6>/Add'

        MEA0_B.IKC_f[2] = MEA0_B.IKC[1];

        // Product: '<S6>/Matrix Multiply4'
        MEA0_B.KRK[3] = v1_0 * v1_idx_1;

        // Math: '<S6>/Transpose1' incorporates:
        //   Sum: '<S6>/Add'

        MEA0_B.IKC_f[3] = MEA0_B.IKC[3];

        // Product: '<S6>/Matrix Multiply2' incorporates:
        //   Math: '<S6>/Transpose1'
        //   Sum: '<S6>/Add'

        tmp[0] = MEA0_B.IKC_f[0];
        tmp[1] = MEA0_B.IKC_f[1];
        tmp[2] = MEA0_B.IKC_f[2];
        tmp[3] = MEA0_B.IKC_f[3];
        for (i = 0; i < 2; i++) {
          v1_tmp = i << 1;
          Ax = tmp[v1_tmp];
          v1_idx_0 = Ax * Constant1_a;
          v1_tmp_0 = v1_tmp + 1;
          tmp_3 = tmp[v1_tmp_0];
          v1_idx_0 += tmp_3 * tmp_0;
          v1[v1_tmp] = v1_idx_0;
          v1_idx_0 = Ax * tmp_1;
          v1_idx_0 += tmp_3 * tmp_2;
          v1[v1_tmp_0] = v1_idx_0;
        }

        Constant1_a = MEA0_B.IKC[0];
        tmp_1 = MEA0_B.IKC[1];
        tmp_0 = MEA0_B.IKC[2];
        tmp_2 = MEA0_B.IKC[3];
        for (i = 0; i < 2; i++) {
          // Product: '<S6>/Matrix Multiply2'
          v1_tmp = i << 1;
          v1_idx_0 = v1[v1_tmp];

          // Product: '<S6>/Matrix Multiply2'
          Ax = v1_idx_0 * Constant1_a;

          // Product: '<S6>/Matrix Multiply2'
          v1_tmp_0 = v1_tmp + 1;
          v1_0 = v1[v1_tmp_0];

          // Product: '<S6>/Matrix Multiply2'
          Ax += v1_0 * tmp_0;

          // Product: '<S6>/Matrix Multiply2'
          MEA0_B.IKCPIKC[v1_tmp] = Ax;

          // Product: '<S6>/Matrix Multiply2'
          Ax = v1_idx_0 * tmp_1;
          Ax += v1_0 * tmp_2;

          // Product: '<S6>/Matrix Multiply2'
          MEA0_B.IKCPIKC[v1_tmp_0] = Ax;
        }

        // Merge: '<S1>/Merge1' incorporates:
        //   Product: '<S6>/Matrix Multiply2'
        //   Product: '<S6>/Matrix Multiply4'
        //   Sum: '<S6>/Add3'

        MEA0_B.x_est_c[0] = MEA0_B.KRK[0] + MEA0_B.IKCPIKC[0];
        MEA0_B.x_est_c[1] = MEA0_B.KRK[1] + MEA0_B.IKCPIKC[1];
        MEA0_B.x_est_c[2] = MEA0_B.KRK[2] + MEA0_B.IKCPIKC[2];
        MEA0_B.x_est_c[3] = MEA0_B.KRK[3] + MEA0_B.IKCPIKC[3];
      }

      // End of Outputs for SubSystem: '<S3>/Subsystem'

      // Logic: '<S3>/NOT'
      MEA0_B.NOT_i = !MEA0_B.Compare;

      // Outputs for Enabled SubSystem: '<S3>/Subsystem1' incorporates:
      //   EnablePort: '<S7>/Enable'

      if (MEA0_B.NOT_i) {
        // Merge: '<S1>/Merge1' incorporates:
        //   SignalConversion generated from: '<S7>/P_pred'
        //   Sum: '<S4>/Add'

        MEA0_B.x_est_c[0] = MEA0_B.APAQ[0];
        MEA0_B.x_est_c[1] = MEA0_B.APAQ[1];
        MEA0_B.x_est_c[2] = MEA0_B.APAQ[2];
        MEA0_B.x_est_c[3] = MEA0_B.APAQ[3];

        // Merge: '<S1>/Merge' incorporates:
        //   SignalConversion generated from: '<S7>/x_pred'
        //   Sum: '<S4>/Add1'

        MEA0_B.x_est[0] = MEA0_B.AxBu[0];
        MEA0_B.x_est[1] = MEA0_B.AxBu[1];
      }

      // End of Outputs for SubSystem: '<S3>/Subsystem1'
    }

    // End of Outputs for SubSystem: '<S1>/cc baro correction'

    // Logic: '<S1>/NOT'
    MEA0_B.NOT = !MEA0_B.RelationalOperator;

    // Outputs for Enabled SubSystem: '<S1>/No Correction Step' incorporates:
    //   EnablePort: '<S2>/Enable'

    if (MEA0_B.NOT) {
      // Merge: '<S1>/Merge' incorporates:
      //   SignalConversion generated from: '<S2>/nextLinearState'
      //   Sum: '<S4>/Add1'

      MEA0_B.x_est[0] = MEA0_B.AxBu[0];
      MEA0_B.x_est[1] = MEA0_B.AxBu[1];

      // Merge: '<S1>/Merge1' incorporates:
      //   SignalConversion generated from: '<S2>/nextLinearCov'
      //   Sum: '<S4>/Add'

      MEA0_B.x_est_c[0] = MEA0_B.APAQ[0];
      MEA0_B.x_est_c[1] = MEA0_B.APAQ[1];
      MEA0_B.x_est_c[2] = MEA0_B.APAQ[2];
      MEA0_B.x_est_c[3] = MEA0_B.APAQ[3];
    }

    // End of Outputs for SubSystem: '<S1>/No Correction Step'

    // Memory: '<S1>/mass_memory'
    MEA0_B.m_old = MEA0_DW.mass_memory_PreviousInput;

    // Product: '<S1>/Matrix Multiply' incorporates:
    //   Constant: '<S1>/Constant'
    //   Merge: '<S1>/Merge'

    Ax = MEA0_B.Constant[0];
    Constant1_a = MEA0_B.x_est[0];
    tmp_3 = Ax * Constant1_a;

    // Update for Memory: '<S1>/x_memory' incorporates:
    //   Merge: '<S1>/Merge'

    MEA0_DW.x_memory_PreviousInput[0] = Constant1_a;

    // Product: '<S1>/Matrix Multiply' incorporates:
    //   Constant: '<S1>/Constant'
    //   Merge: '<S1>/Merge'

    Ax = MEA0_B.Constant[1];
    Constant1_a = MEA0_B.x_est[1];
    tmp_3 += Ax * Constant1_a;

    // Update for Memory: '<S1>/x_memory' incorporates:
    //   Merge: '<S1>/Merge'

    MEA0_DW.x_memory_PreviousInput[1] = Constant1_a;

    // Product: '<S1>/Matrix Multiply'
    MEA0_B.MatrixMultiply = tmp_3;

    // Product: '<S1>/Product'
    MEA0_B.m_dot = MEA0_B.Constant1 * MEA0_B.MatrixMultiply;

    // Product: '<S1>/Product1' incorporates:
    //   Constant: '<S1>/Constant2'

    MEA0_B.Product1 = MEA0_P.Constant2_Value * MEA0_B.m_dot;

    // Sum: '<S1>/Subtract'
    MEA0_B.m_new = MEA0_B.m_old - MEA0_B.Product1;

    // Saturate: '<S1>/Saturation'
    Constant1_a = MEA0_B.m_new;
    tmp_1 = MEA0_P.Saturation_LowerSat;
    tmp_0 = MEA0_P.Saturation_UpperSat;
    if (Constant1_a > tmp_0) {
      // Saturate: '<S1>/Saturation'
      MEA0_B.Saturation = tmp_0;
    } else if (Constant1_a < tmp_1) {
      // Saturate: '<S1>/Saturation'
      MEA0_B.Saturation = tmp_1;
    } else {
      // Saturate: '<S1>/Saturation'
      MEA0_B.Saturation = Constant1_a;
    }

    // End of Saturate: '<S1>/Saturation'

    // Update for Memory: '<S1>/P_memory' incorporates:
    //   Merge: '<S1>/Merge1'

    MEA0_DW.P_memory_PreviousInput[0] = MEA0_B.x_est_c[0];
    MEA0_DW.P_memory_PreviousInput[1] = MEA0_B.x_est_c[1];
    MEA0_DW.P_memory_PreviousInput[2] = MEA0_B.x_est_c[2];
    MEA0_DW.P_memory_PreviousInput[3] = MEA0_B.x_est_c[3];

    // Update for Memory: '<S1>/Memory' incorporates:
    //   Inport: '<Root>/Combustion Chamber Bus'

    MEA0_DW.Memory_PreviousInput = MEA0_U.CombustionChamberBus.CCTimestamp;

    // Update for Memory: '<S1>/mass_memory'
    MEA0_DW.mass_memory_PreviousInput = MEA0_B.Saturation;

    // End of Outputs for SubSystem: '<Root>/MEA - Biliquid'

    // Outport: '<Root>/Mass'
    MEA0_Y.Mass = MEA0_B.Saturation;

    // Outport: '<Root>/Sates' incorporates:
    //   Merge: '<S1>/Merge'

    MEA0_Y.Sates[0] = MEA0_B.x_est[0];
    MEA0_Y.Sates[1] = MEA0_B.x_est[1];

    // Outport: '<Root>/Pressure'
    MEA0_Y.Pressure = MEA0_B.MatrixMultiply;
  }

  rate_scheduler((&MEA0_M));
}

// Model initialize function
void MEA0::initialize()
{
  // SystemInitialize for Atomic SubSystem: '<Root>/MEA - Biliquid'
  // InitializeConditions for Memory: '<S1>/x_memory'
  MEA0_DW.x_memory_PreviousInput[0] = MEA0_P.x_memory_InitialCondition[0];
  MEA0_DW.x_memory_PreviousInput[1] = MEA0_P.x_memory_InitialCondition[1];

  // InitializeConditions for Memory: '<S1>/Memory'
  MEA0_DW.Memory_PreviousInput = MEA0_P.Memory_InitialCondition;

  // InitializeConditions for Memory: '<S1>/mass_memory'
  MEA0_DW.mass_memory_PreviousInput = MEA0_P.mass_memory_InitialCondition;

  // InitializeConditions for Memory: '<S1>/P_memory'
  MEA0_DW.P_memory_PreviousInput[0] = MEA0_P.P_memory_InitialCondition[0];

  // SystemInitialize for Enabled SubSystem: '<S1>/cc baro correction'
  // SystemInitialize for Enabled SubSystem: '<S3>/Subsystem'
  // Start for IdentityMatrix: '<S6>/IdentityMatrix'
  MEA0_B.I[0] = MEA0_P.IdentityMatrix_IDMatrixData[0];

  // End of SystemInitialize for SubSystem: '<S3>/Subsystem'
  // End of SystemInitialize for SubSystem: '<S1>/cc baro correction'

  // InitializeConditions for Memory: '<S1>/P_memory'
  MEA0_DW.P_memory_PreviousInput[1] = MEA0_P.P_memory_InitialCondition[1];

  // SystemInitialize for Enabled SubSystem: '<S1>/cc baro correction'
  // SystemInitialize for Enabled SubSystem: '<S3>/Subsystem'
  // Start for IdentityMatrix: '<S6>/IdentityMatrix'
  MEA0_B.I[1] = MEA0_P.IdentityMatrix_IDMatrixData[1];

  // End of SystemInitialize for SubSystem: '<S3>/Subsystem'
  // End of SystemInitialize for SubSystem: '<S1>/cc baro correction'

  // InitializeConditions for Memory: '<S1>/P_memory'
  MEA0_DW.P_memory_PreviousInput[2] = MEA0_P.P_memory_InitialCondition[2];

  // SystemInitialize for Enabled SubSystem: '<S1>/cc baro correction'
  // SystemInitialize for Enabled SubSystem: '<S3>/Subsystem'
  // Start for IdentityMatrix: '<S6>/IdentityMatrix'
  MEA0_B.I[2] = MEA0_P.IdentityMatrix_IDMatrixData[2];

  // End of SystemInitialize for SubSystem: '<S3>/Subsystem'
  // End of SystemInitialize for SubSystem: '<S1>/cc baro correction'

  // InitializeConditions for Memory: '<S1>/P_memory'
  MEA0_DW.P_memory_PreviousInput[3] = MEA0_P.P_memory_InitialCondition[3];

  // SystemInitialize for Enabled SubSystem: '<S1>/cc baro correction'
  // SystemInitialize for Enabled SubSystem: '<S3>/Subsystem'
  // Start for IdentityMatrix: '<S6>/IdentityMatrix'
  MEA0_B.I[3] = MEA0_P.IdentityMatrix_IDMatrixData[3];

  // End of SystemInitialize for SubSystem: '<S3>/Subsystem'
  // End of SystemInitialize for SubSystem: '<S1>/cc baro correction'
  // End of SystemInitialize for SubSystem: '<Root>/MEA - Biliquid'
}

// Model terminate function
void MEA0::terminate()
{
  // (no terminate code required)
}

MEAState MEA0::getState()
{
  MEAState outputMeaState{};
  outputMeaState.estimatedMass = MEA0_Y.Mass;
  outputMeaState.estimatedPressure = MEA0_Y.Pressure;
  outputMeaState.estimatedForce = computeForce(lastVerticalSpeed, lastMslAltitude);
  outputMeaState.estimatedApogee = computeApogee(lastVerticalSpeed, lastMslAltitude);
  outputMeaState.x0 = MEA0_Y.Sates[0];
  outputMeaState.x1 = MEA0_Y.Sates[1];
  outputMeaState.x2 = 0;
  outputMeaState.timestamp = Boardcore::TimestampTimer::getTimestamp();
  return outputMeaState;
}

float MEA0::computeForce(float verticalSpeed, float mslAltitude)
{
    // Temporary config
    float d = 0.15f;
    float cdCorrectionFactor = 1;
    float crossSection = Boardcore::Constants::PI * (d / 2) * (d / 2);
    float ae = 0.00285943f;
    float p0 = 100093.7492f;
    constexpr Boardcore::Aeroutils::AerodynamicCoeff AERO_COEFF = {
    .n000 = 0.394970609413534f,
    .n100 = 0.0552077159951773f,
    .n200 = -4.24112361410578f,
    .n300 = 17.48579079769f,
    .n400 = -30.2920765271961f,
    .n500 = 24.2503163396895f,
    .n600 = -7.35809370785798f};
    Boardcore::Aeroutils::AerodynamicCoeff coeffs = AERO_COEFF;


    // NOTE: Here we assume that verticalSpeed roughly equals the total speed,
    // so that we don't depend on N/E speed components, which can be quite
    // unreliable in case of no GPS fix or during powered ascent
    float mach = Boardcore::Aeroutils::computeMach(-mslAltitude, verticalSpeed,
                                  Boardcore::Constants::MSL_TEMPERATURE);

    float cd  = Boardcore::Aeroutils::computeCd(coeffs, mach);
    float rho = Boardcore::Aeroutils::computeRho(mslAltitude, Boardcore::Constants::MSL_TEMPERATURE);

    // Dynamic pressure
    float q = 0.5f * rho * (verticalSpeed * verticalSpeed);

    // Aerodynamic force component
    float force = q * crossSection * (cd * cdCorrectionFactor);

    if (mslAltitude > 800)
    {
        // At high altitudes we need to compensate for low external pressure

        // External pressure
        float p = Boardcore::Aeroutils::relPressure(mslAltitude);
        force -= (p0 - p) * ae;
    }
    return force;
}

float MEA0::computeApogee(float verticalSpeed, float mslAltitude)
{
    // Temp config
      constexpr Boardcore::Aeroutils::AerodynamicCoeff AERO_COEFF = {
    .n000 = 0.394970609413534f,
    .n100 = 0.0552077159951773f,
    .n200 = -4.24112361410578f,
    .n300 = 17.48579079769f,
    .n400 = -30.2920765271961f,
    .n500 = 24.2503163396895f,
    .n600 = -7.35809370785798f};
    Boardcore::Aeroutils::AerodynamicCoeff coeffs = AERO_COEFF;
    float mach = Boardcore::Aeroutils::computeMach(-mslAltitude, verticalSpeed,
                                  Boardcore::Constants::MSL_TEMPERATURE);
    float d = 0.15f;
    float cd  = Boardcore::Aeroutils::computeCd(coeffs, mach);
    float rho = Boardcore::Aeroutils::computeRho(mslAltitude, Boardcore::Constants::MSL_TEMPERATURE);
    float crossSection = Boardcore::Constants::PI * (d / 2) * (d / 2);
    // Simplified massive formula for apogee estimation.
    // Warning: log1p(x) = log(1 + x)
    float mass = MEA0_Y.Mass;
    float temp = ((rho * cd * crossSection) / mass);
    float apogee = mslAltitude +
          1 / temp *
              log1p(0.5 * (verticalSpeed * verticalSpeed * temp) /
                    Boardcore::Constants::g);
    return apogee;
}

// Constructor
MEA0::MEA0() :
  MEA0_U(),
  MEA0_Y(),
  MEA0_B(),
  MEA0_DW(),
  MEA0_M()
{
  // Currently there is no constructor body generated.
}

// Destructor
// Currently there is no destructor body generated.
MEA0::~MEA0() = default;

// Real-Time Model get method
MEA0::RT_MODEL_MEA0_T * MEA0::getRTM()
{
  return (&MEA0_M);
}

//
// File trailer for generated code.
//
// [EOF]
//