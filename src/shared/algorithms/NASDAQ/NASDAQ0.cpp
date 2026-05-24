//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// File: NASDAQ0.cpp
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
#include "NASDAQ0.h"

#include <stdbool.h>
#include <stdint.h>

#include <cmath>
#include <cstring>

#include "NASDAQ0_private.h"
#include "NASDAQ0_types.h"


static void rate_scheduler(NASDAQ0::RT_MODEL_NASDAQ0_T* const NASDAQ0_M);

//
//         This function updates active task flag for each subrate.
//         The function is called at model base rate, hence the
//         generated code self-manages all its subrates.
//
static void rate_scheduler(NASDAQ0::RT_MODEL_NASDAQ0_T* const NASDAQ0_M)
{
    // Compute which subrates run during the next base time step.  Subrates
    //  are an integer multiple of the base rate counter.  Therefore, the
    //  subtask counter is reset when it reaches its limit (zero means run).

    (NASDAQ0_M->Timing.TaskCounters.TID[1])++;
    if ((NASDAQ0_M->Timing.TaskCounters.TID[1]) > 1)
    {  // Sample time: [0.02s, 0.0s]
        NASDAQ0_M->Timing.TaskCounters.TID[1] = 0;
    }

    (NASDAQ0_M->Timing.TaskCounters.TID[2])++;
    if ((NASDAQ0_M->Timing.TaskCounters.TID[2]) > 9)
    {  // Sample time: [0.1s, 0.0s]
        NASDAQ0_M->Timing.TaskCounters.TID[2] = 0;
    }
}

//
// Output and update for action system:
//    '<S11>/Correction'
//    '<S24>/Correction'
//
void NASDAQ0::NASDAQ0_Correction(float rtu_Sprev, const float rtu_PH_prev[6],
                                 float rty_K[6], float* rty_S)
{
    // Product: '<S14>/Matrix Divide' incorporates:
    //   SignalConversion generated from: '<S14>/Sprev'

    for (int16_t i{0}; i < 6; i++)
        rty_K[i] = rtu_PH_prev[i] / rtu_Sprev;

    // End of Product: '<S14>/Matrix Divide'

    // SignalConversion generated from: '<S14>/S'
    *rty_S = rtu_Sprev;
}

//
// Output and update for action system:
//    '<S11>/No correction'
//    '<S24>/No correction'
//
void NASDAQ0::NASDAQ0_Nocorrection(float rtu_R, const float rtu_PH_prev[6],
                                   float rty_K[6], float* rty_S,
                                   P_Nocorrection_NASDAQ0_T* localP)
{
    // Gain: '<S15>/Gain'
    for (int16_t i{0}; i < 6; i++)
        rty_K[i] = localP->Gain_Gain * rtu_PH_prev[i];

    // End of Gain: '<S15>/Gain'

    // SignalConversion generated from: '<S15>/R'
    *rty_S = rtu_R;
}

//
// Output and update for enable system:
//    '<S4>/No Correction Step'
//    '<S5>/No Correction Step'
//    '<S6>/No Correction Step'
//
void NASDAQ0::NASDAQ0_NoCorrectionStep(bool rtu_Enable,
                                       const float rtu_nextLinearState[6],
                                       const float rtu_nextLinearCov[36],
                                       float rty_State[6],
                                       float rty_Covariance[36])
{
    float rtu_nextLinearCov_0[36];
    float rtu_nextLinearState_0[6];

    // Outputs for Enabled SubSystem: '<S4>/No Correction Step' incorporates:
    //   EnablePort: '<S9>/Enable'

    if (rtu_Enable)
    {
        // SignalConversion generated from: '<S9>/nextLinearState'
        for (int16_t i{0}; i < 6; i++)
            rtu_nextLinearState_0[i] = rtu_nextLinearState[i];

        for (int16_t i{0}; i < 6; i++)
            rty_State[i] = rtu_nextLinearState_0[i];

        // SignalConversion generated from: '<S9>/nextLinearCov'
        for (int16_t i{0}; i < 36; i++)
            rtu_nextLinearCov_0[i] = rtu_nextLinearCov[i];

        for (int16_t i{0}; i < 36; i++)
            rty_Covariance[i] = rtu_nextLinearCov_0[i];
    }

    // End of Outputs for SubSystem: '<S4>/No Correction Step'
}

void LUf_boolfloatint32_t(float outU[], float outP[], int32_t N, bool outS[])
{
    // S-Function (sdsplu2): '<S18>/LU Factorization'
    // initialize status output
    outS[0UL] = false;

    // initialize row-pivot indices
    for (int32_t k{0L}; k < N; k++)
        outP[k] = static_cast<float>(k + 1L);

    for (int32_t k{0L}; k < N; k++)
    {
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
        idx1_tmp  = k * N;
        mTmp1_tmp = idx1_tmp + k;
        mTmp1     = outU[mTmp1_tmp];
        if (mTmp1 < 0.0F)
            mTmp1 = -mTmp1;

        for (r = k + 1L; r < N; r++)
        {
            float mTmp2;
            mTmp2 = outU[idx1_tmp + r];
            if (mTmp2 < 0.0F)
                mTmp2 = -mTmp2;

            if (mTmp2 > mTmp1)
            {
                p     = r;
                mTmp1 = mTmp2;
            }
        }

        // swap rows if required
        if (p != k)
        {
            for (int32_t c{0L}; c < N; c++)
            {
                idx1      = c * N;
                r         = idx1 + p;
                mTmp1     = outU[r];
                tmp       = idx1 + k;
                outU[r]   = outU[tmp];
                outU[tmp] = mTmp1;
            }

            // swap pivot row indices
            mTmp1   = outP[p];
            outP[p] = outP[k];
            outP[k] = mTmp1;
        }

        if (outU[mTmp1_tmp] == 0.0F)
        {
            outS[0UL] = true;
        }
        else
        {
            for (r = k + 1L; r < N; r++)
            {
                tmp = idx1_tmp + r;
                outU[tmp] /= outU[mTmp1_tmp];
            }

            for (int32_t c{k + 1L}; c < N; c++)
            {
                idx1 = c * N;
                for (r = k + 1L; r < N; r++)
                {
                    tmp = idx1 + r;
                    outU[tmp] -= outU[idx1_tmp + r] * outU[idx1 + k];
                }
            }
        }
    }

    // End of S-Function (sdsplu2): '<S18>/LU Factorization'
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
    for (int16_t j{0}; j < 3; j++)
    {
        int16_t iy;
        c     = j * 5 + 2;
        jj    = j * 5;
        jBcol = 4 - j;
        iy    = 1;
        smax  = std::abs(x[jj]);
        for (jA = 2; jA <= jBcol; jA++)
        {
            float s;
            s = std::abs(x[(c + jA) - 3]);
            if (s > smax)
            {
                iy   = jA;
                smax = s;
            }
        }

        if (x[(c + iy) - 3] != 0.0F)
        {
            if (iy - 1 != 0)
            {
                jA         = j + iy;
                ipiv[j]    = static_cast<int8_t>(jA);
                smax       = x[j];
                x[j]       = x[jA - 1];
                x[jA - 1]  = smax;
                smax       = x[j + 4];
                x[j + 4]   = x[jA + 3];
                x[jA + 3]  = smax;
                smax       = x[j + 8];
                x[j + 8]   = x[jA + 7];
                x[jA + 7]  = smax;
                smax       = x[j + 12];
                x[j + 12]  = x[jA + 11];
                x[jA + 11] = smax;
            }

            iy = c - j;
            for (int16_t ix{c}; ix <= iy + 2; ix++)
                x[ix - 1] /= x[jj];
        }

        jBcol = 2 - j;
        jA    = jj;
        jj += 4;
        for (int16_t ix{0}; ix <= jBcol; ix++)
        {
            smax = x[(ix << 2) + jj];
            if (smax != 0.0F)
            {
                iy    = jA + 6;
                kBcol = (jA - j) + 8;
                for (int16_t ijA{iy}; ijA <= kBcol; ijA++)
                    x[ijA - 1] += x[((c + ijA) - jA) - 7] * -smax;
            }

            jA += 4;
        }
    }

    for (int16_t j{0}; j < 4; j++)
    {
        jBcol = 6 * j - 1;
        jj    = (j << 2) - 1;
        for (jA = 0; jA < j; jA++)
        {
            kBcol = 6 * jA - 1;
            smax  = x[(jA + jj) + 1];
            if (smax != 0.0F)
            {
                for (int16_t ix{0}; ix < 6; ix++)
                {
                    c = (ix + jBcol) + 1;
                    u0[c] -= u0[(ix + kBcol) + 1] * smax;
                }
            }
        }

        smax = 1.0F / x[(j + jj) + 1];
        for (int16_t ix{0}; ix < 6; ix++)
        {
            c = (ix + jBcol) + 1;
            u0[c] *= smax;
        }
    }

    for (int16_t j{3}; j >= 0; j--)
    {
        jBcol = 6 * j - 1;
        jj    = (j << 2) - 1;
        for (jA = j + 2; jA < 5; jA++)
        {
            kBcol = (jA - 1) * 6 - 1;
            smax  = x[jA + jj];
            if (smax != 0.0F)
            {
                for (int16_t ix{0}; ix < 6; ix++)
                {
                    c = (ix + jBcol) + 1;
                    u0[c] -= u0[(ix + kBcol) + 1] * smax;
                }
            }
        }
    }

    for (int16_t j{2}; j >= 0; j--)
    {
        jj = ipiv[j];
        if (j + 1 != jj)
        {
            for (int16_t ix{0}; ix < 6; ix++)
            {
                jA     = 6 * j + ix;
                smax   = u0[jA];
                c      = (jj - 1) * 6 + ix;
                u0[jA] = u0[c];
                u0[c]  = smax;
            }
        }
    }
}

// Model step function
void NASDAQ0::step()
{
    ISAReference rtb_Switch;
    float rtb_Bias1[36];
    float rtb_MatrixMultiply[36];
    float rtb_MatrixMultiply_0[36];
    float rtb_Merge1[36];
    float rtb_Merge_l[36];
    float rtb_Merge_n[36];
    float rtb_Merge_b[24];
    float rtb_S_tmp[24];
    float rtb_Transpose1[24];
    float rtb_BackwardSubstitution[16];
    float rtb_LUFactorization_o1[16];
    float rtb_S[16];
    float rtb_Square_m[16];
    float rtb_MatrixConcatenate1[12];
    float rtb_Merge[6];
    float rtb_Merge_e[6];
    float rtb_Merge_f[6];
    float rtb_VectorConcatenate[6];
    float rtb_LUFactorization_o2[4];
    float rtb_Gain1_b;
    float rtb_LUFactorization_o1_p;
    float rtb_LUFactorization_o2_1;
    float rtb_LUFactorization_o2_2;
    float rtb_LUFactorization_o2_f;
    float rtb_LUFactorization_o2_p;
    float rtb_Matrix1Norm1;
    float rtb_Matrix1Norm2;
    float rtb_Matrix1Norm2_e;
    float rtb_Matrix1Norm2_p;
    int32_t idxLU;
    int32_t idxout;
    int16_t i;
    int16_t i_0;
    int16_t k;
    int16_t rtb_BackwardSubstitution_tmp;
    int16_t rtb_Bias1_tmp;
    int16_t rtb_S_tmp_0;
    uint8_t u0;
    bool rtb_AND;
    bool rtb_AND_c;
    bool rtb_AND_e;
    bool rtb_GreaterThanOrEqual;

    // Outputs for Atomic SubSystem: '<Root>/NASDAQ - Autocoding'
    if ((&NASDAQ0_M)->Timing.TaskCounters.TID[1] == 0)
    {
        // SignalConversion generated from: '<S1>/Vector Concatenate'
        // incorporates:
        //   Concatenate: '<S1>/Vector Concatenate'
        //   Inport: '<Root>/NASDAQ In ANAS'

        NASDAQ0_DW.VectorConcatenate[0] = NASDAQ0_U.NASDAQInANAS.Position[0];

        // SignalConversion generated from: '<S1>/Vector Concatenate'
        // incorporates:
        //   Concatenate: '<S1>/Vector Concatenate'
        //   Inport: '<Root>/NASDAQ In ANAS'

        NASDAQ0_DW.VectorConcatenate[3] = NASDAQ0_U.NASDAQInANAS.Velocity[0];

        // SignalConversion generated from: '<S1>/Vector Concatenate'
        // incorporates:
        //   Concatenate: '<S1>/Vector Concatenate'
        //   Inport: '<Root>/NASDAQ In ANAS'

        NASDAQ0_DW.VectorConcatenate[1] = NASDAQ0_U.NASDAQInANAS.Position[1];

        // SignalConversion generated from: '<S1>/Vector Concatenate'
        // incorporates:
        //   Concatenate: '<S1>/Vector Concatenate'
        //   Inport: '<Root>/NASDAQ In ANAS'

        NASDAQ0_DW.VectorConcatenate[4] = NASDAQ0_U.NASDAQInANAS.Velocity[1];

        // SignalConversion generated from: '<S1>/Vector Concatenate'
        // incorporates:
        //   Concatenate: '<S1>/Vector Concatenate'
        //   Inport: '<Root>/NASDAQ In ANAS'

        NASDAQ0_DW.VectorConcatenate[2] = NASDAQ0_U.NASDAQInANAS.Position[2];

        // SignalConversion generated from: '<S1>/Vector Concatenate'
        // incorporates:
        //   Concatenate: '<S1>/Vector Concatenate'
        //   Inport: '<Root>/NASDAQ In ANAS'

        NASDAQ0_DW.VectorConcatenate[5] = NASDAQ0_U.NASDAQInANAS.Velocity[2];
    }

    // Switch: '<S1>/Switch1' incorporates:
    //   UnitDelay: '<S1>/Unit Delay1'
    //   UnitDelay: '<S1>/Unit Delay2'

    for (i = 0; i < 6; i++)
        if (NASDAQ0_DW.UnitDelay2_DSTATE != 0)
            rtb_Merge_f[i] = NASDAQ0_DW.UnitDelay1_DSTATE_h[i];
        else
            rtb_Merge_f[i] = NASDAQ0_DW.VectorConcatenate[i];

    // End of Switch: '<S1>/Switch1'

    // Switch: '<S1>/Switch' incorporates:
    //   Inport: '<Root>/NASDAQ In ANAS'
    //   UnitDelay: '<S1>/Unit Delay'
    //   UnitDelay: '<S1>/Unit Delay3'

    rtb_GreaterThanOrEqual = (NASDAQ0_DW.UnitDelay3_DSTATE != 0);
    if (rtb_GreaterThanOrEqual)
    {
        std::memcpy(&rtb_Bias1[0], &NASDAQ0_DW.UnitDelay_DSTATE[0],
                    36U * sizeof(float));
    }
    else
    {
        std::memcpy(&rtb_Bias1[0], &NASDAQ0_U.NASDAQInANAS.LinearCovariance[0],
                    36U * sizeof(float));
    }

    // End of Switch: '<S1>/Switch'

    // Outputs for Atomic SubSystem: '<S1>/Prediction Step'
    for (i = 0; i < 36; i++)
    {
        // Bias: '<S3>/Bias' incorporates:
        //   Constant: '<S3>/Constant1'
        //   Gain: '<S3>/Gain1'
        //   Product: '<S10>/Matrix Multiply'

        rtb_MatrixMultiply[i] =
            NASDAQ0_P.Gain1_Gain_h * NASDAQ0_P.Constant1_Value_j[i] +
            NASDAQ0_P.Bias_Bias_d[i];
    }

    // Product: '<S3>/Matrix Multiply' incorporates:
    //   Math: '<S3>/Transpose'
    //   Product: '<S10>/Matrix Multiply'

    for (k = 0; k < 6; k++)
    {
        rtb_BackwardSubstitution_tmp = 0;
        for (i = 0; i < 6; i++)
        {
            rtb_Gain1_b   = 0.0F;
            rtb_Bias1_tmp = 0;
            for (rtb_S_tmp_0 = 0; rtb_S_tmp_0 < 6; rtb_S_tmp_0++)
            {
                rtb_Gain1_b += rtb_Bias1[rtb_Bias1_tmp + k] *
                               rtb_MatrixMultiply[rtb_Bias1_tmp + i];
                rtb_Bias1_tmp += 6;
            }

            rtb_MatrixMultiply_0[rtb_BackwardSubstitution_tmp + k] =
                rtb_Gain1_b;
            rtb_BackwardSubstitution_tmp += 6;
        }
    }

    // Bias: '<S3>/Bias1' incorporates:
    //   Product: '<S10>/Matrix Multiply'
    //   Product: '<S3>/Matrix Multiply'

    for (k = 0; k < 6; k++)
    {
        rtb_BackwardSubstitution_tmp = 0;
        for (i = 0; i < 6; i++)
        {
            rtb_Gain1_b   = 0.0F;
            rtb_Bias1_tmp = 0;
            for (rtb_S_tmp_0 = 0; rtb_S_tmp_0 < 6; rtb_S_tmp_0++)
            {
                rtb_Gain1_b +=
                    rtb_MatrixMultiply[rtb_Bias1_tmp + k] *
                    rtb_MatrixMultiply_0[rtb_S_tmp_0 +
                                         rtb_BackwardSubstitution_tmp];
                rtb_Bias1_tmp += 6;
            }

            rtb_Bias1_tmp = rtb_BackwardSubstitution_tmp + k;
            rtb_Bias1[rtb_Bias1_tmp] =
                NASDAQ0_P.Bias1_Bias_h[rtb_Bias1_tmp] + rtb_Gain1_b;
            rtb_BackwardSubstitution_tmp += 6;
        }
    }

    // End of Bias: '<S3>/Bias1'

    // SignalConversion generated from: '<S3>/Vector Concatenate'
    rtb_VectorConcatenate[3] = rtb_Merge_f[3];

    // Sum: '<S3>/Add1' incorporates:
    //   Gain: '<S3>/Gain'
    //   SignalConversion generated from: '<S3>/Previous State'

    rtb_VectorConcatenate[0] =
        NASDAQ0_P.Gain_Gain_i * rtb_Merge_f[3] + rtb_Merge_f[0];

    // SignalConversion generated from: '<S3>/Vector Concatenate'
    rtb_VectorConcatenate[4] = rtb_Merge_f[4];

    // Sum: '<S3>/Add1' incorporates:
    //   Gain: '<S3>/Gain'
    //   SignalConversion generated from: '<S3>/Previous State'

    rtb_VectorConcatenate[1] =
        NASDAQ0_P.Gain_Gain_i * rtb_Merge_f[4] + rtb_Merge_f[1];

    // SignalConversion generated from: '<S3>/Vector Concatenate'
    rtb_VectorConcatenate[5] = rtb_Merge_f[5];

    // Sum: '<S3>/Add1' incorporates:
    //   Gain: '<S3>/Gain'
    //   SignalConversion generated from: '<S3>/Previous State'

    rtb_VectorConcatenate[2] =
        NASDAQ0_P.Gain_Gain_i * rtb_Merge_f[5] + rtb_Merge_f[2];

    // End of Outputs for SubSystem: '<S1>/Prediction Step'

    // Outputs for Atomic SubSystem: '<S1>/Correction Step'
    // Outputs for Atomic SubSystem: '<S2>/GPS Correction'
    // RelationalOperator: '<S34>/Relational Operator' incorporates:
    //   Inport: '<Root>/NASDAQ In Sensors'
    //   Memory: '<S34>/Memory'

    rtb_GreaterThanOrEqual = (NASDAQ0_U.NASDAQInSensors_i.GPSTimestamp >
                              NASDAQ0_DW.Memory_PreviousInput);

    // Logic: '<S34>/AND' incorporates:
    //   Constant: '<S34>/Constant'

    rtb_AND = ((NASDAQ0_P.Constant_Value_a != 0.0) && rtb_GreaterThanOrEqual);

    // Outputs for Enabled SubSystem: '<S6>/Active Correction Step'
    // incorporates:
    //   EnablePort: '<S33>/Enable'

    if (rtb_AND)
    {
        // Gain: '<S41>/Gain1' incorporates:
        //   Bias: '<S37>/Bias'
        //   Gain: '<S37>/Gain'

        rtb_Gain1_b = (NASDAQ0_P.Gain_Gain_e * rtb_VectorConcatenate[0] +
                       NASDAQ0_P.Bias_Bias) *
                      NASDAQ0_P.Gain1_Gain;

        // Trigonometry: '<S37>/Cos' incorporates:
        //   Trigonometry: '<S37>/Cos1'

        rtb_LUFactorization_o1_p = std::cos(rtb_Gain1_b);

        // Gain: '<S37>/Gain3' incorporates:
        //   Constant: '<S37>/Constant'
        //   Constant: '<S37>/Constant1'
        //   Constant: '<S37>/Constant3'
        //   Constant: '<S37>/Constant4'
        //   Gain: '<S37>/Gain1'
        //   Math: '<S37>/Square'
        //   Product: '<S37>/Divide'
        //   Product: '<S37>/Divide1'
        //   Product: '<S37>/Product'
        //   Product: '<S37>/Product1'
        //   Trigonometry: '<S37>/Cos'
        //   Trigonometry: '<S37>/Sin'

        rtb_MatrixConcatenate1[0] =
            NASDAQ0_P.Gain3_Gain_k * NASDAQ0_P.Constant_Value_fg;
        rtb_MatrixConcatenate1[1] =
            rtb_VectorConcatenate[1] * std::sin(rtb_Gain1_b) /
            (rtb_LUFactorization_o1_p * rtb_LUFactorization_o1_p *
             NASDAQ0_P.Constant3_Value) *
            NASDAQ0_P.Gain3_Gain_k;
        rtb_MatrixConcatenate1[2] =
            NASDAQ0_P.Gain3_Gain_k * NASDAQ0_P.Constant1_Value_f;
        rtb_MatrixConcatenate1[3] =
            NASDAQ0_P.Constant4_Value /
            (NASDAQ0_P.Gain1_Gain_n * rtb_LUFactorization_o1_p) *
            NASDAQ0_P.Gain3_Gain_k;

        // Constant: '<S37>/Constant5'
        for (i = 0; i < 8; i++)
        {
            // Constant: '<S37>/Constant5'
            rtb_MatrixConcatenate1[i + 4] = NASDAQ0_P.Constant5_Value[i];
        }

        // End of Constant: '<S37>/Constant5'

        // Concatenate: '<S37>/Matrix Concatenate2' incorporates:
        //   Concatenate: '<S37>/Matrix Concatenate1'
        //   Constant: '<S37>/Constant6'
        //   Math: '<S36>/Transpose1'

        k                            = 0;
        rtb_BackwardSubstitution_tmp = 0;
        for (i = 0; i < 6; i++)
        {
            rtb_Transpose1[k] =
                rtb_MatrixConcatenate1[rtb_BackwardSubstitution_tmp];
            rtb_Transpose1[k + 2] =
                NASDAQ0_P.Constant6_Value[rtb_BackwardSubstitution_tmp];
            rtb_Transpose1[k + 1] =
                rtb_MatrixConcatenate1[rtb_BackwardSubstitution_tmp + 1];
            rtb_Transpose1[k + 3] =
                NASDAQ0_P.Constant6_Value[rtb_BackwardSubstitution_tmp + 1];
            k += 4;
            rtb_BackwardSubstitution_tmp += 2;
        }

        // End of Concatenate: '<S37>/Matrix Concatenate2'

        // Math: '<S33>/Square' incorporates:
        //   Constant: '<S33>/Constant'

        for (i = 0; i < 16; i++)
        {
            rtb_Gain1_b     = NASDAQ0_P.Constant_Value_o[i];
            rtb_Square_m[i] = rtb_Gain1_b * rtb_Gain1_b;
        }

        // End of Math: '<S33>/Square'
        for (k = 0; k < 6; k++)
        {
            // Product: '<S38>/Matrix Multiply' incorporates:
            //   Bias: '<S3>/Bias1'
            //   Math: '<S36>/Transpose1'
            //   Math: '<S38>/Transpose'
            //   Product: '<S38>/Matrix Multiply1'

            rtb_BackwardSubstitution_tmp = 0;
            for (i = 0; i < 4; i++)
            {
                rtb_Gain1_b   = 0.0F;
                rtb_Bias1_tmp = 0;
                rtb_S_tmp_0   = 0;
                for (i_0 = 0; i_0 < 6; i_0++)
                {
                    rtb_Gain1_b += rtb_Bias1[rtb_Bias1_tmp + k] *
                                   rtb_Transpose1[rtb_S_tmp_0 + i];
                    rtb_Bias1_tmp += 6;
                    rtb_S_tmp_0 += 4;
                }

                rtb_S_tmp[rtb_BackwardSubstitution_tmp + k] = rtb_Gain1_b;
                rtb_BackwardSubstitution_tmp += 6;
            }
        }

        // Sum: '<S38>/Add' incorporates:
        //   Math: '<S33>/Square'
        //   Math: '<S36>/Transpose1'
        //   Product: '<S38>/Matrix Multiply'

        for (k = 0; k < 4; k++)
        {
            rtb_BackwardSubstitution_tmp = 0;
            i                            = 0;
            for (rtb_Bias1_tmp = 0; rtb_Bias1_tmp < 4; rtb_Bias1_tmp++)
            {
                rtb_Gain1_b = 0.0F;
                rtb_S_tmp_0 = 0;
                for (i_0 = 0; i_0 < 6; i_0++)
                {
                    rtb_Gain1_b +=
                        rtb_Transpose1[rtb_S_tmp_0 + k] * rtb_S_tmp[i_0 + i];
                    rtb_S_tmp_0 += 4;
                }

                rtb_S_tmp_0        = rtb_BackwardSubstitution_tmp + k;
                rtb_S[rtb_S_tmp_0] = rtb_Square_m[rtb_S_tmp_0] + rtb_Gain1_b;
                rtb_BackwardSubstitution_tmp += 4;
                i += 6;
            }
        }

        // End of Sum: '<S38>/Add'

        // S-Function (sdsplu2): '<S46>/LU Factorization' incorporates:
        //   Sum: '<S38>/Add'

        std::memcpy(&rtb_LUFactorization_o1[0], &rtb_S[0], sizeof(float) << 4U);
        LUf_boolfloatint32_t(&rtb_LUFactorization_o1[0],
                             &rtb_LUFactorization_o2[0], 4L,
                             &rtb_GreaterThanOrEqual);

        // Switch: '<S44>/Switch' incorporates:
        //   Constant: '<S44>/Constant'
        //   Math: '<S44>/Math Function'
        //   Product: '<S44>/Product'
        //
        //  About '<S44>/Math Function':
        //   Operator: reciprocal

        if (rtb_GreaterThanOrEqual)
        {
            rtb_Gain1_b = NASDAQ0_P.Constant_Value_l;
        }
        else
        {
            // S-Function (sdspm1norm2): '<S44>/Matrix  1-Norm2'

            // DSP System Toolbox Matrix 1-Norm (sdspm1norm2) - '<S44>/Matrix
            // 1-Norm2'
            {
                const float* uPtr{&rtb_S[0]};

                float m1norm{0.0};

                int jdx;
                for (jdx = 4; jdx-- > 0;)
                {
                    float sumabsAj{0.0};

                    int idxFlt;
                    for (idxFlt = 4; idxFlt-- > 0;)
                    {
                        float temp{*uPtr++};

                        sumabsAj += fabsf(temp);
                    }

                    m1norm = MAX(m1norm, sumabsAj);
                }

                rtb_Matrix1Norm2 = m1norm;
            }

            // S-Function (sdspperm2): '<S46>/Permute Matrix' incorporates:
            //   IdentityMatrix: '<S46>/Identity Matrix'
            //   S-Function (sdspfbsub2): '<S46>/Backward Substitution'

            rtb_Gain1_b              = rtb_LUFactorization_o2[0L];
            rtb_LUFactorization_o1_p = rtb_LUFactorization_o2[1L];
            rtb_LUFactorization_o2_1 = rtb_LUFactorization_o2[2L];
            rtb_LUFactorization_o2_2 = rtb_LUFactorization_o2[3L];
            for (k = 0; k < 4; k++)
            {
                idxout = static_cast<int32_t>(std::floor(rtb_Gain1_b)) - 1L;
                if (idxout < 0L)
                    idxout = 0L;
                else if (idxout >= 4L)
                    idxout = 3L;

                rtb_BackwardSubstitution_tmp = k << 2;
                rtb_BackwardSubstitution[static_cast<int32_t>(
                    rtb_BackwardSubstitution_tmp)] =
                    NASDAQ0_DW.IdentityMatrix[static_cast<int32_t>(
                        rtb_BackwardSubstitution_tmp +
                        static_cast<int16_t>(idxout))];
                idxout =
                    static_cast<int32_t>(std::floor(rtb_LUFactorization_o1_p)) -
                    1L;
                if (idxout < 0L)
                    idxout = 0L;
                else if (idxout >= 4L)
                    idxout = 3L;

                rtb_BackwardSubstitution[static_cast<int32_t>(
                    rtb_BackwardSubstitution_tmp + 1)] =
                    NASDAQ0_DW.IdentityMatrix[static_cast<int32_t>(
                        rtb_BackwardSubstitution_tmp +
                        static_cast<int16_t>(idxout))];
                idxout =
                    static_cast<int32_t>(std::floor(rtb_LUFactorization_o2_1)) -
                    1L;
                if (idxout < 0L)
                    idxout = 0L;
                else if (idxout >= 4L)
                    idxout = 3L;

                rtb_BackwardSubstitution[static_cast<int32_t>(
                    rtb_BackwardSubstitution_tmp + 2)] =
                    NASDAQ0_DW.IdentityMatrix[static_cast<int32_t>(
                        rtb_BackwardSubstitution_tmp +
                        static_cast<int16_t>(idxout))];
                idxout =
                    static_cast<int32_t>(std::floor(rtb_LUFactorization_o2_2)) -
                    1L;
                if (idxout < 0L)
                    idxout = 0L;
                else if (idxout >= 4L)
                    idxout = 3L;

                rtb_BackwardSubstitution[static_cast<int32_t>(
                    rtb_BackwardSubstitution_tmp + 3)] =
                    NASDAQ0_DW.IdentityMatrix[static_cast<int32_t>(
                        rtb_BackwardSubstitution_tmp +
                        static_cast<int16_t>(idxout))];
            }

            // End of S-Function (sdspperm2): '<S46>/Permute Matrix'

            // S-Function (sdspfbsub2): '<S46>/Forward Substitution'
            // incorporates:
            //   S-Function (sdspfbsub2): '<S46>/Backward Substitution'
            //   S-Function (sdsplu2): '<S46>/LU Factorization'

            for (rtb_BackwardSubstitution_tmp = 0;
                 rtb_BackwardSubstitution_tmp < 4;
                 rtb_BackwardSubstitution_tmp++)
            {
                idxout = rtb_BackwardSubstitution_tmp << 2;
                rtb_BackwardSubstitution[static_cast<int32_t>(
                    static_cast<int16_t>(idxout) + 1)] -=
                    rtb_LUFactorization_o1[1L] *
                    rtb_BackwardSubstitution[idxout];
                idxLU       = 2L;
                rtb_Gain1_b = rtb_BackwardSubstitution[static_cast<int32_t>(
                    static_cast<int16_t>(idxout) + 2)];
                for (k = 0; k < 2; k++)
                {
                    rtb_Gain1_b -=
                        rtb_BackwardSubstitution[static_cast<int32_t>(
                            static_cast<int16_t>(idxout) + k)] *
                        rtb_LUFactorization_o1[idxLU];
                    idxLU += 4L;
                }

                rtb_BackwardSubstitution[static_cast<int32_t>(
                    static_cast<int16_t>(idxout) + 2)] = rtb_Gain1_b;
                idxLU                                  = 3L;
                rtb_Gain1_b = rtb_BackwardSubstitution[static_cast<int32_t>(
                    static_cast<int16_t>(idxout) + 3)];
                for (k = 0; k < 3; k++)
                {
                    rtb_Gain1_b -=
                        rtb_BackwardSubstitution[static_cast<int32_t>(
                            static_cast<int16_t>(idxout) + k)] *
                        rtb_LUFactorization_o1[idxLU];
                    idxLU += 4L;
                }

                rtb_BackwardSubstitution[static_cast<int32_t>(
                    static_cast<int16_t>(idxout) + 3)] = rtb_Gain1_b;
            }

            // End of S-Function (sdspfbsub2): '<S46>/Forward Substitution'

            // S-Function (sdspfbsub2): '<S46>/Backward Substitution'
            // incorporates:
            //   S-Function (sdsplu2): '<S46>/LU Factorization'

            for (rtb_BackwardSubstitution_tmp = 0;
                 rtb_BackwardSubstitution_tmp < 4;
                 rtb_BackwardSubstitution_tmp++)
            {
                idxout = rtb_BackwardSubstitution_tmp << 2;
                rtb_BackwardSubstitution[static_cast<int32_t>(
                    static_cast<int16_t>(idxout) + 3)] /=
                    rtb_LUFactorization_o1[15];
                rtb_BackwardSubstitution[static_cast<int32_t>(
                    static_cast<int16_t>(idxout) + 2)] =
                    (rtb_BackwardSubstitution[static_cast<int32_t>(
                         static_cast<int16_t>(idxout) + 2)] -
                     rtb_BackwardSubstitution[static_cast<int32_t>(
                         static_cast<int16_t>(idxout) + 3)] *
                         rtb_LUFactorization_o1[14L]) /
                    rtb_LUFactorization_o1[10L];
                idxLU       = 13L;
                rtb_Gain1_b = rtb_BackwardSubstitution[static_cast<int32_t>(
                    static_cast<int16_t>(idxout) + 1)];
                for (k = 3; k > 1; k--)
                {
                    rtb_Gain1_b -=
                        rtb_BackwardSubstitution[static_cast<int32_t>(
                            static_cast<int16_t>(idxout) + k)] *
                        rtb_LUFactorization_o1[idxLU];
                    idxLU -= 4L;
                }

                rtb_BackwardSubstitution[static_cast<int32_t>(
                    static_cast<int16_t>(idxout) + 1)] =
                    rtb_Gain1_b / rtb_LUFactorization_o1[idxLU];
                idxLU       = 12L;
                rtb_Gain1_b = rtb_BackwardSubstitution[idxout];
                for (k = 3; k > 0; k--)
                {
                    rtb_Gain1_b -=
                        rtb_BackwardSubstitution[static_cast<int32_t>(
                            static_cast<int16_t>(idxout) + k)] *
                        rtb_LUFactorization_o1[idxLU];
                    idxLU -= 4L;
                }

                rtb_BackwardSubstitution[idxout] =
                    rtb_Gain1_b / rtb_LUFactorization_o1[idxLU];
            }

            // End of S-Function (sdspfbsub2): '<S46>/Backward Substitution'

            // S-Function (sdspm1norm2): '<S44>/Matrix  1-Norm1'

            // DSP System Toolbox Matrix 1-Norm (sdspm1norm2) - '<S44>/Matrix
            // 1-Norm1'
            {
                const float* uPtr{&rtb_BackwardSubstitution[0]};

                float m1norm{0.0};

                int jdx;
                for (jdx = 4; jdx-- > 0;)
                {
                    float sumabsAj{0.0};

                    int idxFlt;
                    for (idxFlt = 4; idxFlt-- > 0;)
                    {
                        float temp{*uPtr++};

                        sumabsAj += fabsf(temp);
                    }

                    m1norm = MAX(m1norm, sumabsAj);
                }

                rtb_Matrix1Norm1 = m1norm;
            }

            rtb_Gain1_b = 1.0F / (rtb_Matrix1Norm1 * rtb_Matrix1Norm2);
        }

        // End of Switch: '<S44>/Switch'

        // If: '<S38>/If' incorporates:
        //   Constant: '<S38>/NASCondLim'
        //   Merge: '<S38>/Merge'
        //   Product: '<S38>/Matrix Multiply1'
        //   Product: '<S42>/Matrix Divide'
        //   RelationalOperator: '<S38>/GreaterThanOrEqual'

        if (rtb_Gain1_b >= NASDAQ0_P.NASCondLim_Value_o)
        {
            // Outputs for IfAction SubSystem: '<S38>/Correction' incorporates:
            //   ActionPort: '<S42>/Action Port'

            std::memcpy(&rtb_Merge_b[0], &rtb_S_tmp[0], 24U * sizeof(float));

            // Product: '<S42>/Matrix Divide' incorporates:
            //   Merge: '<S38>/Merge'
            //   Product: '<S38>/Matrix Multiply1'
            //   SignalConversion generated from: '<S42>/Sprev'
            //   Sum: '<S38>/Add'

            rt_mrdivide_U1f6x4_U2f4x4_Yf6x4(rtb_Merge_b, rtb_S);

            // End of Outputs for SubSystem: '<S38>/Correction'
        }
        else
        {
            // Outputs for IfAction SubSystem: '<S38>/No correction'
            // incorporates:
            //   ActionPort: '<S43>/Action Port'

            // Gain: '<S43>/Gain' incorporates:
            //   Merge: '<S38>/Merge'
            //   Product: '<S38>/Matrix Multiply1'

            for (k = 0; k < 24; k++)
                rtb_Merge_b[k] = NASDAQ0_P.Gain_Gain_dp * rtb_S_tmp[k];

            // End of Gain: '<S43>/Gain'
            // End of Outputs for SubSystem: '<S38>/No correction'
        }

        // End of If: '<S38>/If'

        // Sum: '<S40>/Subtract' incorporates:
        //   Constant: '<S40>/Constant'
        //   Math: '<S36>/Transpose1'
        //   Product: '<S10>/Matrix Multiply2'
        //   Product: '<S40>/Matrix Multiply'

        for (k = 0; k < 6; k++)
        {
            // Product: '<S40>/Matrix Multiply' incorporates:
            //   Merge: '<S38>/Merge'

            rtb_Matrix1Norm2             = rtb_Merge_b[k + 6];
            rtb_Matrix1Norm1             = rtb_Merge_b[k];
            rtb_Gain1_b                  = rtb_Merge_b[k + 12];
            rtb_LUFactorization_o1_p     = rtb_Merge_b[k + 18];
            rtb_BackwardSubstitution_tmp = 0;
            i                            = 0;
            for (rtb_Bias1_tmp = 0; rtb_Bias1_tmp < 6; rtb_Bias1_tmp++)
            {
                rtb_S_tmp_0 = rtb_BackwardSubstitution_tmp + k;
                rtb_MatrixMultiply[rtb_S_tmp_0] =
                    NASDAQ0_P.Constant_Value_p1[rtb_S_tmp_0] -
                    (((rtb_Transpose1[i + 1] * rtb_Matrix1Norm2 +
                       rtb_Transpose1[i] * rtb_Matrix1Norm1) +
                      rtb_Transpose1[i + 2] * rtb_Gain1_b) +
                     rtb_Transpose1[i + 3] * rtb_LUFactorization_o1_p);
                rtb_BackwardSubstitution_tmp += 6;
                i += 4;
            }
        }

        // End of Sum: '<S40>/Subtract'

        // Product: '<S36>/Matrix Multiply' incorporates:
        //   Bias: '<S3>/Bias1'
        //   Math: '<S36>/Transpose'
        //   Product: '<S10>/Matrix Multiply2'

        for (k = 0; k < 6; k++)
        {
            rtb_BackwardSubstitution_tmp = 0;
            for (i = 0; i < 6; i++)
            {
                rtb_Matrix1Norm2 = 0.0F;
                rtb_Bias1_tmp    = 0;
                for (rtb_S_tmp_0 = 0; rtb_S_tmp_0 < 6; rtb_S_tmp_0++)
                {
                    rtb_Matrix1Norm2 += rtb_Bias1[rtb_Bias1_tmp + k] *
                                        rtb_MatrixMultiply[rtb_Bias1_tmp + i];
                    rtb_Bias1_tmp += 6;
                }

                rtb_Merge1[rtb_BackwardSubstitution_tmp + k] = rtb_Matrix1Norm2;
                rtb_BackwardSubstitution_tmp += 6;
            }
        }

        // Product: '<S36>/Matrix Multiply2' incorporates:
        //   Math: '<S33>/Square'
        //   Math: '<S36>/Transpose1'
        //   Merge: '<S38>/Merge'

        for (k = 0; k < 4; k++)
        {
            rtb_Matrix1Norm2             = rtb_Square_m[k + 4];
            rtb_Matrix1Norm1             = rtb_Square_m[k];
            rtb_Gain1_b                  = rtb_Square_m[k + 8];
            rtb_LUFactorization_o1_p     = rtb_Square_m[k + 12];
            rtb_BackwardSubstitution_tmp = 0;
            for (i = 0; i < 6; i++)
            {
                rtb_Transpose1[rtb_BackwardSubstitution_tmp + k] =
                    ((rtb_Merge_b[i + 6] * rtb_Matrix1Norm2 +
                      rtb_Matrix1Norm1 * rtb_Merge_b[i]) +
                     rtb_Merge_b[i + 12] * rtb_Gain1_b) +
                    rtb_Merge_b[i + 18] * rtb_LUFactorization_o1_p;
                rtb_BackwardSubstitution_tmp += 4;
            }
        }

        // Product: '<S36>/Matrix Multiply' incorporates:
        //   Merge: '<S38>/Merge'
        //   Product: '<S10>/Matrix Multiply2'
        //   Product: '<S36>/Matrix Multiply2'

        k                            = 0;
        rtb_BackwardSubstitution_tmp = 0;
        for (i = 0; i < 6; i++)
        {
            for (rtb_Bias1_tmp = 0; rtb_Bias1_tmp < 6; rtb_Bias1_tmp++)
            {
                rtb_Matrix1Norm1 = 0.0F;
                rtb_S_tmp_0      = 0;
                for (i_0 = 0; i_0 < 6; i_0++)
                {
                    rtb_Matrix1Norm1 +=
                        rtb_MatrixMultiply[rtb_S_tmp_0 + rtb_Bias1_tmp] *
                        rtb_Merge1[i_0 + k];
                    rtb_S_tmp_0 += 6;
                }

                rtb_S_tmp_0                       = rtb_Bias1_tmp + k;
                rtb_MatrixMultiply_0[rtb_S_tmp_0] = rtb_Matrix1Norm1;
                rtb_Merge_n[rtb_S_tmp_0] =
                    ((rtb_Transpose1[rtb_BackwardSubstitution_tmp + 1] *
                          rtb_Merge_b[rtb_Bias1_tmp + 6] +
                      rtb_Transpose1[rtb_BackwardSubstitution_tmp] *
                          rtb_Merge_b[rtb_Bias1_tmp]) +
                     rtb_Transpose1[rtb_BackwardSubstitution_tmp + 2] *
                         rtb_Merge_b[rtb_Bias1_tmp + 12]) +
                    rtb_Transpose1[rtb_BackwardSubstitution_tmp + 3] *
                        rtb_Merge_b[rtb_Bias1_tmp + 18];
            }

            k += 6;
            rtb_BackwardSubstitution_tmp += 4;
        }

        // Sum: '<S36>/Sum1' incorporates:
        //   Merge: '<S6>/Merge1'

        for (k = 0; k < 36; k++)
            rtb_Merge1[k] = rtb_MatrixMultiply_0[k] + rtb_Merge_n[k];

        // End of Sum: '<S36>/Sum1'

        // Bias: '<S39>/Bias' incorporates:
        //   Gain: '<S39>/Gain3'

        rtb_Gain1_b = NASDAQ0_P.Gain3_Gain_h * rtb_VectorConcatenate[0] +
                      NASDAQ0_P.Bias_Bias_l;

        // Gain: '<S39>/Gain' incorporates:
        //   Bias: '<S39>/Bias1'
        //   Gain: '<S39>/Gain4'
        //   Gain: '<S47>/Gain1'
        //   Inport: '<Root>/NASDAQ In Sensors'
        //   Product: '<S39>/Divide1'
        //   SignalConversion generated from: '<S39>/Vector Concatenate'
        //   Sum: '<S39>/Add1'
        //   Trigonometry: '<S39>/Cos1'

        rtb_Matrix1Norm2 =
            (NASDAQ0_U.NASDAQInSensors_i.GPSMeasure[0] - rtb_Gain1_b) *
            NASDAQ0_P.Gain_Gain_l[0];
        rtb_Gain1_b = (NASDAQ0_U.NASDAQInSensors_i.GPSMeasure[1] -
                       (1.0F /
                            (std::cos(NASDAQ0_P.Gain1_Gain_d * rtb_Gain1_b) *
                             NASDAQ0_P.Gain4_Gain_l) *
                            rtb_VectorConcatenate[1] +
                        NASDAQ0_P.Bias1_Bias)) *
                      NASDAQ0_P.Gain_Gain_l[1];

        // Outputs for Atomic SubSystem: '<S1>/Prediction Step'
        rtb_LUFactorization_o1_p =
            (NASDAQ0_U.NASDAQInSensors_i.GPSMeasure[2] - rtb_Merge_f[3]) *
            NASDAQ0_P.Gain_Gain_l[2];
        rtb_LUFactorization_o2_1 =
            (NASDAQ0_U.NASDAQInSensors_i.GPSMeasure[3] - rtb_Merge_f[4]) *
            NASDAQ0_P.Gain_Gain_l[3];

        // End of Outputs for SubSystem: '<S1>/Prediction Step'
        for (i = 0; i < 6; i++)
        {
            // Product: '<S33>/Matrix Multiply2' incorporates:
            //   Merge: '<S38>/Merge'

            rtb_Matrix1Norm1 =
                ((rtb_Merge_b[i + 6] * rtb_Gain1_b +
                  rtb_Merge_b[i] * rtb_Matrix1Norm2) +
                 rtb_Merge_b[i + 12] * rtb_LUFactorization_o1_p) +
                rtb_Merge_b[i + 18] * rtb_LUFactorization_o2_1;
            rtb_Merge_f[i] = rtb_Matrix1Norm1;

            // Sum: '<S33>/Add' incorporates:
            //   Product: '<S33>/Matrix Multiply2'

            rtb_Merge[i] = rtb_Matrix1Norm1 + rtb_VectorConcatenate[i];
        }
    }

    // End of Outputs for SubSystem: '<S6>/Active Correction Step'

    // Outputs for Enabled SubSystem: '<S6>/No Correction Step'
    // Logic: '<S6>/NOT'
    NASDAQ0_NoCorrectionStep(!rtb_AND, rtb_VectorConcatenate, rtb_Bias1,
                             rtb_Merge, rtb_Merge1);

    // End of Outputs for SubSystem: '<S6>/No Correction Step'

    // Update for Memory: '<S34>/Memory' incorporates:
    //   Inport: '<Root>/NASDAQ In Sensors'

    NASDAQ0_DW.Memory_PreviousInput = NASDAQ0_U.NASDAQInSensors_i.GPSTimestamp;

    // End of Outputs for SubSystem: '<S2>/GPS Correction'

    // Outputs for Atomic SubSystem: '<S2>/Baro Correction'
    // RelationalOperator: '<S20>/Relational Operator' incorporates:
    //   Inport: '<Root>/NASDAQ In Sensors'
    //   Memory: '<S20>/Memory'

    rtb_GreaterThanOrEqual = (NASDAQ0_U.NASDAQInSensors_i.BaroTimestamp >
                              NASDAQ0_DW.Memory_PreviousInput_d);

    // Logic: '<S20>/AND' incorporates:
    //   Constant: '<S20>/Constant'

    rtb_AND_e = (NASDAQ0_P.Constant_Value_o1 && rtb_GreaterThanOrEqual);

    // Outputs for Enabled SubSystem: '<S5>/Active Correction Step'
    // incorporates:
    //   EnablePort: '<S19>/Enable'

    if (rtb_AND_e)
    {
        // Constant: '<S23>/Constant'
        rtb_Merge_f[0] = NASDAQ0_P.Constant_Value_n[0];
        rtb_Merge_f[1] = NASDAQ0_P.Constant_Value_n[1];

        // Switch: '<S19>/Switch' incorporates:
        //   Inport: '<Root>/NASDAQ In Reference'
        //   UnitDelay: '<S19>/Unit Delay'
        //   UnitDelay: '<S19>/Unit Delay1'

        if (NASDAQ0_DW.UnitDelay_DSTATE_j > NASDAQ0_P.Switch_Threshold)
            rtb_Switch = NASDAQ0_DW.UnitDelay1_DSTATE;
        else
            rtb_Switch = NASDAQ0_U.NASDAQInReference;

        // End of Switch: '<S19>/Switch'

        // Sum: '<S23>/Sum1' incorporates:
        //   Gain: '<S23>/Gain4'

        rtb_Gain1_b =
            NASDAQ0_P.Gain4_Gain * rtb_Merge[2] + rtb_Switch.GroundTemperature;

        // Product: '<S23>/Product1' incorporates:
        //   Constant: '<S23>/g R'
        //   Gain: '<S23>/Gain3'
        //   Math: '<S23>/Exp'
        //   Math: '<S23>/Square'
        //   Product: '<S23>/Divide2'
        //   Product: '<S23>/Divide3'
        //   Product: '<S23>/Product'
        //   Product: '<S23>/Product4'
        //
        //  About '<S23>/Exp':
        //   Operator: exp

        rtb_Merge_f[2] =
            rtb_Switch.GroundPressure * rtb_Switch.GroundTemperature *
            NASDAQ0_P.Gain3_Gain / (rtb_Gain1_b * rtb_Gain1_b) *
            std::exp(NASDAQ0_P.gR_Value / rtb_Gain1_b * rtb_Merge[2]);

        // Constant: '<S23>/Constant1'
        rtb_Merge_f[3] = NASDAQ0_P.Constant1_Value[0];
        rtb_Merge_f[4] = NASDAQ0_P.Constant1_Value[1];
        rtb_Merge_f[5] = NASDAQ0_P.Constant1_Value[2];

        // Math: '<S19>/Square' incorporates:
        //   Constant: '<S19>/Constant'

        rtb_Matrix1Norm2 =
            NASDAQ0_P.Constant_Value_p * NASDAQ0_P.Constant_Value_p;

        // Product: '<S24>/Matrix Multiply'
        rtb_Matrix1Norm1 = 0.0F;
        for (i = 0; i < 6; i++)
        {
            // Math: '<S24>/Transpose' incorporates:
            //   Merge: '<S24>/Merge'
            //   S-Function (sdspdiag2): '<S1>/Extract Diagonal'

            rtb_Merge_e[i] = rtb_Merge_f[i];

            // Product: '<S24>/Matrix Multiply' incorporates:
            //   Math: '<S24>/Transpose'
            //   Merge: '<S6>/Merge1'
            //   Product: '<S24>/Matrix Multiply1'
            //   S-Function (sdspdiag2): '<S1>/Extract Diagonal'

            rtb_Gain1_b = 0.0F;
            k           = 0;
            for (rtb_BackwardSubstitution_tmp = 0;
                 rtb_BackwardSubstitution_tmp < 6;
                 rtb_BackwardSubstitution_tmp++)
            {
                rtb_Gain1_b += rtb_Merge1[k + i] *
                               rtb_Merge_f[rtb_BackwardSubstitution_tmp];
                k += 6;
            }

            // Math: '<S24>/Transpose'
            rtb_VectorConcatenate[i] = rtb_Gain1_b;

            // Product: '<S24>/Matrix Multiply' incorporates:
            //   Math: '<S24>/Transpose'
            //   S-Function (sdspdiag2): '<S1>/Extract Diagonal'

            rtb_Matrix1Norm1 += rtb_Merge_f[i] * rtb_Gain1_b;
        }

        // Sum: '<S24>/Add' incorporates:
        //   Product: '<S24>/Matrix Multiply'

        rtb_Matrix1Norm1 += rtb_Matrix1Norm2;

        // S-Function (sdsplu2): '<S31>/LU Factorization'
        rtb_LUFactorization_o1_p = rtb_Matrix1Norm1;
        LUf_boolfloatint32_t(&rtb_LUFactorization_o1_p,
                             &rtb_LUFactorization_o2_p, 1L,
                             &rtb_GreaterThanOrEqual);

        // Switch: '<S29>/Switch' incorporates:
        //   Constant: '<S29>/Constant'
        //   Math: '<S29>/Math Function'
        //   Product: '<S29>/Product'
        //
        //  About '<S29>/Math Function':
        //   Operator: reciprocal

        if (rtb_GreaterThanOrEqual)
        {
            rtb_Gain1_b = NASDAQ0_P.Constant_Value_h;
        }
        else
        {
            // S-Function (sdspm1norm2): '<S29>/Matrix  1-Norm2'

            // DSP System Toolbox Matrix 1-Norm (sdspm1norm2) - '<S29>/Matrix
            // 1-Norm2'
            {
                const float* uPtr{&rtb_Matrix1Norm1};

                float sumabsAj{0.0};

                float temp{*uPtr++};

                sumabsAj += fabsf(temp);
                rtb_Matrix1Norm2_p = sumabsAj;
            }

            // S-Function (sdspperm2): '<S31>/Permute Matrix'
            rtb_LUFactorization_o2_p = NASDAQ0_DW.IdentityMatrix_h;

            // S-Function (sdspfbsub2): '<S31>/Backward Substitution'
            rtb_LUFactorization_o2_p /= rtb_LUFactorization_o1_p;

            // S-Function (sdspm1norm2): '<S29>/Matrix  1-Norm1'

            // DSP System Toolbox Matrix 1-Norm (sdspm1norm2) - '<S29>/Matrix
            // 1-Norm1'
            {
                const float* uPtr{&rtb_LUFactorization_o2_p};

                float sumabsAj{0.0};

                float temp{*uPtr++};

                sumabsAj += fabsf(temp);
                rtb_LUFactorization_o2_p = sumabsAj;
            }

            rtb_Gain1_b =
                1.0F / (rtb_LUFactorization_o2_p * rtb_Matrix1Norm2_p);
        }

        // End of Switch: '<S29>/Switch'

        // If: '<S24>/If' incorporates:
        //   Constant: '<S24>/NASCondLim'
        //   RelationalOperator: '<S24>/GreaterThanOrEqual'

        if (rtb_Gain1_b >= NASDAQ0_P.NASCondLim_Value_l)
        {
            // Outputs for IfAction SubSystem: '<S24>/Correction' incorporates:
            //   ActionPort: '<S27>/Action Port'

            NASDAQ0_Correction(rtb_Matrix1Norm1, rtb_VectorConcatenate,
                               rtb_Merge_e, &rtb_LUFactorization_o1_p);

            // End of Outputs for SubSystem: '<S24>/Correction'
        }
        else
        {
            // Outputs for IfAction SubSystem: '<S24>/No correction'
            // incorporates:
            //   ActionPort: '<S28>/Action Port'

            NASDAQ0_Nocorrection(rtb_Matrix1Norm1, rtb_VectorConcatenate,
                                 rtb_Merge_e, &rtb_LUFactorization_o1_p,
                                 &NASDAQ0_P.Nocorrection_h);

            // End of Outputs for SubSystem: '<S24>/No correction'
        }

        // End of If: '<S24>/If'

        // Sum: '<S26>/Subtract' incorporates:
        //   Constant: '<S26>/Constant'
        //   Merge: '<S24>/Merge'
        //   Product: '<S10>/Matrix Multiply2'
        //   Product: '<S26>/Matrix Multiply'
        //   S-Function (sdspdiag2): '<S1>/Extract Diagonal'

        k = 0;
        for (rtb_BackwardSubstitution_tmp = 0; rtb_BackwardSubstitution_tmp < 6;
             rtb_BackwardSubstitution_tmp++)
        {
            for (i = 0; i < 6; i++)
            {
                rtb_S_tmp_0 = i + k;
                rtb_MatrixMultiply[rtb_S_tmp_0] =
                    NASDAQ0_P.Constant_Value_c[rtb_S_tmp_0] -
                    rtb_Merge_e[i] * rtb_Merge_f[rtb_BackwardSubstitution_tmp];
            }

            k += 6;
        }

        // End of Sum: '<S26>/Subtract'

        // Product: '<S22>/Matrix Multiply' incorporates:
        //   Math: '<S22>/Transpose'
        //   Math: '<S22>/Transpose1'
        //   Merge: '<S24>/Merge'
        //   Merge: '<S6>/Merge1'
        //   Product: '<S10>/Matrix Multiply2'
        //   Product: '<S22>/Matrix Multiply2'

        for (k = 0; k < 6; k++)
        {
            rtb_BackwardSubstitution_tmp = 0;
            for (i = 0; i < 6; i++)
            {
                rtb_Matrix1Norm2_p = 0.0F;
                rtb_Bias1_tmp      = 0;
                for (rtb_S_tmp_0 = 0; rtb_S_tmp_0 < 6; rtb_S_tmp_0++)
                {
                    rtb_Matrix1Norm2_p += rtb_Merge1[rtb_Bias1_tmp + k] *
                                          rtb_MatrixMultiply[rtb_Bias1_tmp + i];
                    rtb_Bias1_tmp += 6;
                }

                rtb_Merge_l[rtb_BackwardSubstitution_tmp + k] =
                    rtb_Matrix1Norm2_p;
                rtb_BackwardSubstitution_tmp += 6;
            }
        }

        k = 0;
        for (rtb_BackwardSubstitution_tmp = 0; rtb_BackwardSubstitution_tmp < 6;
             rtb_BackwardSubstitution_tmp++)
        {
            i = 0;
            for (rtb_Bias1_tmp = 0; rtb_Bias1_tmp < 6; rtb_Bias1_tmp++)
            {
                rtb_Matrix1Norm1 = 0.0F;
                rtb_S_tmp_0      = 0;
                for (i_0 = 0; i_0 < 6; i_0++)
                {
                    rtb_Matrix1Norm1 +=
                        rtb_MatrixMultiply[rtb_S_tmp_0 +
                                           rtb_BackwardSubstitution_tmp] *
                        rtb_Merge_l[i_0 + i];
                    rtb_S_tmp_0 += 6;
                }

                rtb_MatrixMultiply_0[i + rtb_BackwardSubstitution_tmp] =
                    rtb_Matrix1Norm1;
                rtb_Bias1[rtb_Bias1_tmp + k] =
                    rtb_Matrix1Norm2 *
                    rtb_Merge_e[rtb_BackwardSubstitution_tmp] *
                    rtb_Merge_e[rtb_Bias1_tmp];
                i += 6;
            }

            k += 6;
        }

        // End of Product: '<S22>/Matrix Multiply'

        // Sum: '<S22>/Sum1' incorporates:
        //   Merge: '<S5>/Merge'

        for (k = 0; k < 36; k++)
            rtb_Merge_l[k] = rtb_MatrixMultiply_0[k] + rtb_Bias1[k];

        // End of Sum: '<S22>/Sum1'

        // Sum: '<S25>/Subtract' incorporates:
        //   Constant: '<S32>/HeightTemperatureGradient'
        //   Constant: '<S32>/R air'
        //   Constant: '<S32>/gravity'
        //   Gain: '<S25>/Gain'
        //   Inport: '<Root>/NASDAQ In Sensors'
        //   Math: '<S32>/Power'
        //   Product: '<S32>/Divide'
        //   Product: '<S32>/Divide1'
        //   Product: '<S32>/Product'
        //   Product: '<S32>/Product2'
        //   Product: '<S32>/Product3'
        //   Sum: '<S32>/Subtract'

        rtb_Gain1_b = NASDAQ0_U.NASDAQInSensors_i.BaroMeasure -
                      std::pow((rtb_Switch.GroundTemperature -
                                NASDAQ0_P.Gain_Gain_d * rtb_Merge[2] *
                                    NASDAQ0_P.HeightTemperatureGradient_Value) /
                                   rtb_Switch.GroundTemperature,
                               NASDAQ0_P.gravity_Value /
                                   (NASDAQ0_P.HeightTemperatureGradient_Value *
                                    NASDAQ0_P.Rair_Value)) *
                          rtb_Switch.GroundPressure;
        for (i = 0; i < 6; i++)
        {
            // Product: '<S19>/Matrix Multiply2' incorporates:
            //   Merge: '<S24>/Merge'

            rtb_Matrix1Norm1 = rtb_Merge_e[i] * rtb_Gain1_b;
            rtb_Merge_f[i]   = rtb_Matrix1Norm1;

            // Sum: '<S19>/Add' incorporates:
            //   Product: '<S19>/Matrix Multiply2'

            rtb_Merge_e[i] = rtb_Merge[i] + rtb_Matrix1Norm1;
        }

        // Update for UnitDelay: '<S19>/Unit Delay1' incorporates:
        //   Switch: '<S19>/Switch'

        NASDAQ0_DW.UnitDelay1_DSTATE = rtb_Switch;

        // Bias: '<S19>/Bias' incorporates:
        //   UnitDelay: '<S19>/Unit Delay'

        u0 = static_cast<uint8_t>(NASDAQ0_DW.UnitDelay_DSTATE_j +
                                  NASDAQ0_P.Bias_Bias_m);

        // Saturate: '<S19>/Saturation'
        if (u0 > NASDAQ0_P.Saturation_UpperSat)
        {
            // Update for UnitDelay: '<S19>/Unit Delay'
            NASDAQ0_DW.UnitDelay_DSTATE_j = NASDAQ0_P.Saturation_UpperSat;
        }
        else if (u0 < NASDAQ0_P.Saturation_LowerSat)
        {
            // Update for UnitDelay: '<S19>/Unit Delay'
            NASDAQ0_DW.UnitDelay_DSTATE_j = NASDAQ0_P.Saturation_LowerSat;
        }
        else
        {
            // Update for UnitDelay: '<S19>/Unit Delay'
            NASDAQ0_DW.UnitDelay_DSTATE_j = u0;
        }

        // End of Saturate: '<S19>/Saturation'
    }

    // End of Outputs for SubSystem: '<S5>/Active Correction Step'

    // Outputs for Enabled SubSystem: '<S5>/No Correction Step'
    // Logic: '<S5>/NOT'
    NASDAQ0_NoCorrectionStep(!rtb_AND_e, rtb_Merge, rtb_Merge1, rtb_Merge_e,
                             rtb_Merge_l);

    // End of Outputs for SubSystem: '<S5>/No Correction Step'

    // Update for Memory: '<S20>/Memory' incorporates:
    //   Inport: '<Root>/NASDAQ In Sensors'

    NASDAQ0_DW.Memory_PreviousInput_d =
        NASDAQ0_U.NASDAQInSensors_i.BaroTimestamp;

    // End of Outputs for SubSystem: '<S2>/Baro Correction'

    // Outputs for Atomic SubSystem: '<S2>/ADA Correction'
    if ((&NASDAQ0_M)->Timing.TaskCounters.TID[1] == 0)
    {
        // Gain: '<S4>/Gain' incorporates:
        //   Inport: '<Root>/NASDAQ In ADA'

        NASDAQ0_DW.Gain =
            NASDAQ0_P.Gain_Gain * NASDAQ0_U.NASDAQInADA_i.VerticalSpeed;
    }

    // RelationalOperator: '<S8>/Relational Operator' incorporates:
    //   Inport: '<Root>/NASDAQ In ADA'
    //   Memory: '<S8>/Memory'

    rtb_GreaterThanOrEqual =
        (NASDAQ0_U.NASDAQInADA_i.Timestamp > NASDAQ0_DW.Memory_PreviousInput_a);

    // Logic: '<S8>/AND' incorporates:
    //   Constant: '<S8>/Constant'

    rtb_AND_c = ((NASDAQ0_P.Constant_Value != 0.0) && rtb_GreaterThanOrEqual);

    // Outputs for Enabled SubSystem: '<S4>/Active Correction Step'
    // incorporates:
    //   EnablePort: '<S7>/Enable'

    if (rtb_AND_c)
    {
        if ((&NASDAQ0_M)->Timing.TaskCounters.TID[1] == 0)
        {
            // Switch: '<S7>/Switch' incorporates:
            //   Constant: '<S7>/Constant1'

            if (NASDAQ0_P.Constant1_Value_f5)
            {
                // Switch: '<S7>/Switch' incorporates:
                //   Inport: '<Root>/NASDAQ In ADA'

                NASDAQ0_DW.Switch =
                    NASDAQ0_U.NASDAQInADA_i.VerticalSpeedCovariance;
            }
            else
            {
                // Switch: '<S7>/Switch' incorporates:
                //   Constant: '<S7>/Constant'
                //   Math: '<S7>/Square'

                NASDAQ0_DW.Switch =
                    NASDAQ0_P.Constant_Value_an * NASDAQ0_P.Constant_Value_an;
            }

            // End of Switch: '<S7>/Switch'
        }

        // Product: '<S11>/Matrix Multiply' incorporates:
        //   Constant: '<S7>/H Matrix'
        //   Math: '<S11>/Transpose'
        //   Merge: '<S5>/Merge'

        rtb_Gain1_b = 0.0F;
        k           = 0;
        for (rtb_BackwardSubstitution_tmp = 0; rtb_BackwardSubstitution_tmp < 6;
             rtb_BackwardSubstitution_tmp++)
        {
            rtb_Matrix1Norm2_p = 0.0F;
            for (i = 0; i < 6; i++)
            {
                rtb_Matrix1Norm2_p +=
                    rtb_Merge_l[i + k] *
                    static_cast<float>(NASDAQ0_P.HMatrix_Value[i]);
            }

            rtb_Gain1_b +=
                rtb_Matrix1Norm2_p *
                static_cast<float>(
                    NASDAQ0_P.HMatrix_Value[rtb_BackwardSubstitution_tmp]);
            k += 6;
        }

        // Sum: '<S11>/Add' incorporates:
        //   Product: '<S11>/Matrix Multiply'
        //   Switch: '<S7>/Switch'

        rtb_LUFactorization_o1_p = rtb_Gain1_b + NASDAQ0_DW.Switch;

        // S-Function (sdsplu2): '<S18>/LU Factorization'
        rtb_Matrix1Norm2_p = rtb_LUFactorization_o1_p;
        LUf_boolfloatint32_t(&rtb_Matrix1Norm2_p, &rtb_LUFactorization_o2_f, 1L,
                             &rtb_GreaterThanOrEqual);

        // Switch: '<S16>/Switch' incorporates:
        //   Constant: '<S16>/Constant'
        //   Math: '<S16>/Math Function'
        //   Product: '<S16>/Product'
        //
        //  About '<S16>/Math Function':
        //   Operator: reciprocal

        if (rtb_GreaterThanOrEqual)
        {
            rtb_Gain1_b = NASDAQ0_P.Constant_Value_g;
        }
        else
        {
            // S-Function (sdspm1norm2): '<S16>/Matrix  1-Norm2'

            // DSP System Toolbox Matrix 1-Norm (sdspm1norm2) - '<S16>/Matrix
            // 1-Norm2'
            {
                const float* uPtr{&rtb_LUFactorization_o1_p};

                float sumabsAj{0.0};

                float temp{*uPtr++};

                sumabsAj += fabsf(temp);
                rtb_Matrix1Norm2_e = sumabsAj;
            }

            // S-Function (sdspperm2): '<S18>/Permute Matrix'
            rtb_LUFactorization_o2_f = NASDAQ0_DW.IdentityMatrix_b;

            // S-Function (sdspfbsub2): '<S18>/Backward Substitution'
            rtb_LUFactorization_o2_f /= rtb_Matrix1Norm2_p;

            // S-Function (sdspm1norm2): '<S16>/Matrix  1-Norm1'

            // DSP System Toolbox Matrix 1-Norm (sdspm1norm2) - '<S16>/Matrix
            // 1-Norm1'
            {
                const float* uPtr{&rtb_LUFactorization_o2_f};

                float sumabsAj{0.0};

                float temp{*uPtr++};

                sumabsAj += fabsf(temp);
                rtb_LUFactorization_o2_f = sumabsAj;
            }

            rtb_Gain1_b =
                1.0F / (rtb_LUFactorization_o2_f * rtb_Matrix1Norm2_e);
        }

        // End of Switch: '<S16>/Switch'

        // Product: '<S11>/Matrix Multiply1' incorporates:
        //   Constant: '<S7>/H Matrix'
        //   Math: '<S11>/Transpose'
        //   Merge: '<S5>/Merge'

        for (k = 0; k < 6; k++)
        {
            // Product: '<S11>/Matrix Multiply1'
            rtb_Matrix1Norm2_e           = 0.0F;
            rtb_BackwardSubstitution_tmp = 0;
            for (i = 0; i < 6; i++)
            {
                rtb_Matrix1Norm2_e +=
                    rtb_Merge_l[rtb_BackwardSubstitution_tmp + k] *
                    static_cast<float>(NASDAQ0_P.HMatrix_Value[i]);
                rtb_BackwardSubstitution_tmp += 6;
            }

            // Product: '<S11>/Matrix Multiply1' incorporates:
            //   Constant: '<S7>/H Matrix'
            //   Math: '<S11>/Transpose'
            //   Merge: '<S5>/Merge'

            rtb_Merge[k] = rtb_Matrix1Norm2_e;
        }

        // End of Product: '<S11>/Matrix Multiply1'

        // If: '<S11>/If' incorporates:
        //   Constant: '<S11>/NASCondLim'
        //   RelationalOperator: '<S11>/GreaterThanOrEqual'

        if (rtb_Gain1_b >= NASDAQ0_P.NASCondLim_Value)
        {
            // Outputs for IfAction SubSystem: '<S11>/Correction' incorporates:
            //   ActionPort: '<S14>/Action Port'

            NASDAQ0_Correction(rtb_LUFactorization_o1_p, rtb_Merge, rtb_Merge_f,
                               &rtb_Matrix1Norm2_p);

            // End of Outputs for SubSystem: '<S11>/Correction'
        }
        else
        {
            // Outputs for IfAction SubSystem: '<S11>/No correction'
            // incorporates:
            //   ActionPort: '<S15>/Action Port'

            NASDAQ0_Nocorrection(rtb_LUFactorization_o1_p, rtb_Merge,
                                 rtb_Merge_f, &rtb_Matrix1Norm2_p,
                                 &NASDAQ0_P.Nocorrection);

            // End of Outputs for SubSystem: '<S11>/No correction'
        }

        // End of If: '<S11>/If'

        // Sum: '<S13>/Subtract' incorporates:
        //   Constant: '<S13>/Constant'
        //   Constant: '<S7>/H Matrix'
        //   Merge: '<S11>/Merge'
        //   Product: '<S10>/Matrix Multiply2'
        //   Product: '<S13>/Matrix Multiply'

        k = 0;
        for (rtb_BackwardSubstitution_tmp = 0; rtb_BackwardSubstitution_tmp < 6;
             rtb_BackwardSubstitution_tmp++)
        {
            for (i = 0; i < 6; i++)
            {
                rtb_S_tmp_0 = i + k;
                rtb_MatrixMultiply[rtb_S_tmp_0] =
                    NASDAQ0_P.Constant_Value_f[rtb_S_tmp_0] -
                    rtb_Merge_f[i] *
                        static_cast<float>(
                            NASDAQ0_P
                                .HMatrix_Value[rtb_BackwardSubstitution_tmp]);
            }

            k += 6;
        }

        // End of Sum: '<S13>/Subtract'

        // Product: '<S10>/Matrix Multiply' incorporates:
        //   Math: '<S10>/Transpose'
        //   Math: '<S10>/Transpose1'
        //   Merge: '<S11>/Merge'
        //   Merge: '<S5>/Merge'
        //   Product: '<S10>/Matrix Multiply2'
        //   Switch: '<S7>/Switch'

        for (k = 0; k < 6; k++)
        {
            rtb_BackwardSubstitution_tmp = 0;
            for (i = 0; i < 6; i++)
            {
                rtb_Matrix1Norm2_e = 0.0F;
                rtb_Bias1_tmp      = 0;
                for (rtb_S_tmp_0 = 0; rtb_S_tmp_0 < 6; rtb_S_tmp_0++)
                {
                    rtb_Matrix1Norm2_e += rtb_Merge_l[rtb_Bias1_tmp + k] *
                                          rtb_MatrixMultiply[rtb_Bias1_tmp + i];
                    rtb_Bias1_tmp += 6;
                }

                rtb_Merge1[rtb_BackwardSubstitution_tmp + k] =
                    rtb_Matrix1Norm2_e;
                rtb_BackwardSubstitution_tmp += 6;
            }
        }

        k = 0;
        for (rtb_BackwardSubstitution_tmp = 0; rtb_BackwardSubstitution_tmp < 6;
             rtb_BackwardSubstitution_tmp++)
        {
            i = 0;
            for (rtb_Bias1_tmp = 0; rtb_Bias1_tmp < 6; rtb_Bias1_tmp++)
            {
                rtb_Matrix1Norm1 = 0.0F;
                rtb_S_tmp_0      = 0;
                for (i_0 = 0; i_0 < 6; i_0++)
                {
                    rtb_Matrix1Norm1 +=
                        rtb_MatrixMultiply[rtb_S_tmp_0 +
                                           rtb_BackwardSubstitution_tmp] *
                        rtb_Merge1[i_0 + i];
                    rtb_S_tmp_0 += 6;
                }

                rtb_MatrixMultiply_0[i + rtb_BackwardSubstitution_tmp] =
                    rtb_Matrix1Norm1;
                rtb_Bias1[rtb_Bias1_tmp + k] =
                    NASDAQ0_DW.Switch *
                    rtb_Merge_f[rtb_BackwardSubstitution_tmp] *
                    rtb_Merge_f[rtb_Bias1_tmp];
                i += 6;
            }

            k += 6;
        }

        // End of Product: '<S10>/Matrix Multiply'
        for (k = 0; k < 36; k++)
        {
            // Merge: '<S4>/Merge' incorporates:
            //   Sum: '<S10>/Sum1'

            NASDAQ0_DW.Merge[k] = rtb_MatrixMultiply_0[k] + rtb_Bias1[k];
        }

        // Sum: '<S12>/Subtract'
        rtb_Gain1_b = NASDAQ0_DW.Gain - rtb_Merge_e[5];
        for (i = 0; i < 6; i++)
        {
            // Product: '<S7>/Matrix Multiply2' incorporates:
            //   Merge: '<S11>/Merge'
            //   S-Function (sdspdiag2): '<S1>/Extract Diagonal'

            rtb_Matrix1Norm1 = rtb_Merge_f[i] * rtb_Gain1_b;
            rtb_Merge_f[i]   = rtb_Matrix1Norm1;

            // Merge: '<S4>/Merge1' incorporates:
            //   Merge: '<S11>/Merge'
            //   Product: '<S7>/Matrix Multiply2'
            //   S-Function (sdspdiag2): '<S1>/Extract Diagonal'
            //   Sum: '<S7>/Add'

            NASDAQ0_DW.Merge1[i] = rtb_Merge_e[i] + rtb_Matrix1Norm1;
        }
    }

    // End of Outputs for SubSystem: '<S4>/Active Correction Step'

    // Outputs for Enabled SubSystem: '<S4>/No Correction Step'
    // Logic: '<S4>/NOT'
    NASDAQ0_NoCorrectionStep(!rtb_AND_c, rtb_Merge_e, rtb_Merge_l,
                             NASDAQ0_DW.Merge1, NASDAQ0_DW.Merge);

    // End of Outputs for SubSystem: '<S4>/No Correction Step'

    // Update for Memory: '<S8>/Memory' incorporates:
    //   Inport: '<Root>/NASDAQ In ADA'

    NASDAQ0_DW.Memory_PreviousInput_a = NASDAQ0_U.NASDAQInADA_i.Timestamp;

    // End of Outputs for SubSystem: '<S2>/ADA Correction'
    // End of Outputs for SubSystem: '<S1>/Correction Step'

    // BusCreator generated from: '<S1>/NASDAQ Logs OBSW_BusCreator'
    // incorporates:
    //   Outport: '<Root>/NASDAQ Logs OBSW'

    NASDAQ0_Y.NASDAQLogsOBSW.Timestamp   = 0ULL;
    NASDAQ0_Y.NASDAQLogsOBSW.Position[0] = NASDAQ0_DW.Merge1[0];
    NASDAQ0_Y.NASDAQLogsOBSW.Velocity[0] = NASDAQ0_DW.Merge1[3];
    NASDAQ0_Y.NASDAQLogsOBSW.Position[1] = NASDAQ0_DW.Merge1[1];
    NASDAQ0_Y.NASDAQLogsOBSW.Velocity[1] = NASDAQ0_DW.Merge1[4];
    NASDAQ0_Y.NASDAQLogsOBSW.Position[2] = NASDAQ0_DW.Merge1[2];
    NASDAQ0_Y.NASDAQLogsOBSW.Velocity[2] = NASDAQ0_DW.Merge1[5];
    for (i = 0; i < 6; i++)
    {
        // BusCreator generated from: '<S1>/NASDAQ Logs OBSW_BusCreator'
        // incorporates:
        //   Merge: '<S4>/Merge'
        //   Outport: '<Root>/NASDAQ Logs OBSW'
        //   S-Function (sdspdiag2): '<S1>/Extract Diagonal'

        NASDAQ0_Y.NASDAQLogsOBSW.CovarianceMatrixDiagonal[i] =
            NASDAQ0_DW.Merge[static_cast<int32_t>(i * 7LL)];
    }

    // BusCreator generated from: '<S1>/NASDAQ Logs OBSW_BusCreator'
    // incorporates:
    //   Outport: '<Root>/NASDAQ Logs OBSW'

    NASDAQ0_Y.NASDAQLogsOBSW.BaroActivation = rtb_AND_e;
    NASDAQ0_Y.NASDAQLogsOBSW.GPSActivation  = rtb_AND;
    NASDAQ0_Y.NASDAQLogsOBSW.ADAActovation  = rtb_AND_c;

    // BusCreator generated from: '<S1>/NASDAQ Out_BusCreator' incorporates:
    //   Outport: '<Root>/NASDAQ Out'

    NASDAQ0_Y.NASDAQOut_l.Timestamp   = 0ULL;
    NASDAQ0_Y.NASDAQOut_l.Position[0] = NASDAQ0_DW.Merge1[0];
    NASDAQ0_Y.NASDAQOut_l.Velocity[0] = NASDAQ0_DW.Merge1[3];
    NASDAQ0_Y.NASDAQOut_l.Position[1] = NASDAQ0_DW.Merge1[1];
    NASDAQ0_Y.NASDAQOut_l.Velocity[1] = NASDAQ0_DW.Merge1[4];
    NASDAQ0_Y.NASDAQOut_l.Position[2] = NASDAQ0_DW.Merge1[2];
    NASDAQ0_Y.NASDAQOut_l.Velocity[2] = NASDAQ0_DW.Merge1[5];

    // Update for UnitDelay: '<S1>/Unit Delay' incorporates:
    //   Merge: '<S4>/Merge'

    std::memcpy(&NASDAQ0_DW.UnitDelay_DSTATE[0], &NASDAQ0_DW.Merge[0],
                36U * sizeof(float));

    // Bias: '<S1>/Bias2' incorporates:
    //   UnitDelay: '<S1>/Unit Delay3'

    u0 = static_cast<uint8_t>(NASDAQ0_DW.UnitDelay3_DSTATE +
                              NASDAQ0_P.Bias2_Bias);

    // Saturate: '<S1>/Saturation2'
    if (u0 > NASDAQ0_P.Saturation2_UpperSat)
    {
        // Update for UnitDelay: '<S1>/Unit Delay3'
        NASDAQ0_DW.UnitDelay3_DSTATE = NASDAQ0_P.Saturation2_UpperSat;
    }
    else if (u0 < NASDAQ0_P.Saturation2_LowerSat)
    {
        // Update for UnitDelay: '<S1>/Unit Delay3'
        NASDAQ0_DW.UnitDelay3_DSTATE = NASDAQ0_P.Saturation2_LowerSat;
    }
    else
    {
        // Update for UnitDelay: '<S1>/Unit Delay3'
        NASDAQ0_DW.UnitDelay3_DSTATE = u0;
    }

    // End of Saturate: '<S1>/Saturation2'

    // Bias: '<S1>/Bias1' incorporates:
    //   UnitDelay: '<S1>/Unit Delay2'

    u0 = static_cast<uint8_t>(NASDAQ0_DW.UnitDelay2_DSTATE +
                              NASDAQ0_P.Bias1_Bias_l);

    // Saturate: '<S1>/Saturation1'
    if (u0 > NASDAQ0_P.Saturation1_UpperSat)
    {
        // Update for UnitDelay: '<S1>/Unit Delay2'
        NASDAQ0_DW.UnitDelay2_DSTATE = NASDAQ0_P.Saturation1_UpperSat;
    }
    else if (u0 < NASDAQ0_P.Saturation1_LowerSat)
    {
        // Update for UnitDelay: '<S1>/Unit Delay2'
        NASDAQ0_DW.UnitDelay2_DSTATE = NASDAQ0_P.Saturation1_LowerSat;
    }
    else
    {
        // Update for UnitDelay: '<S1>/Unit Delay2'
        NASDAQ0_DW.UnitDelay2_DSTATE = u0;
    }

    // End of Saturate: '<S1>/Saturation1'

    // Update for UnitDelay: '<S1>/Unit Delay1'
    for (i = 0; i < 6; i++)
        NASDAQ0_DW.UnitDelay1_DSTATE_h[i] = NASDAQ0_DW.Merge1[i];

    // End of Update for UnitDelay: '<S1>/Unit Delay1'
    // End of Outputs for SubSystem: '<Root>/NASDAQ - Autocoding'
    rate_scheduler((&NASDAQ0_M));
}

// Model initialize function
void NASDAQ0::initialize()
{
    {
        int16_t i;

        // SystemInitialize for Atomic SubSystem: '<Root>/NASDAQ - Autocoding'
        // InitializeConditions for UnitDelay: '<S1>/Unit Delay'
        for (i = 0; i < 36; i++)
        {
            NASDAQ0_DW.UnitDelay_DSTATE[i] =
                NASDAQ0_P.UnitDelay_InitialCondition;
        }

        // End of InitializeConditions for UnitDelay: '<S1>/Unit Delay'

        // InitializeConditions for UnitDelay: '<S1>/Unit Delay3'
        NASDAQ0_DW.UnitDelay3_DSTATE = NASDAQ0_P.UnitDelay3_InitialCondition;

        // InitializeConditions for UnitDelay: '<S1>/Unit Delay2'
        NASDAQ0_DW.UnitDelay2_DSTATE = NASDAQ0_P.UnitDelay2_InitialCondition;

        // InitializeConditions for UnitDelay: '<S1>/Unit Delay1'
        for (i = 0; i < 6; i++)
        {
            NASDAQ0_DW.UnitDelay1_DSTATE_h[i] =
                NASDAQ0_P.UnitDelay1_InitialCondition_o;
        }

        // End of InitializeConditions for UnitDelay: '<S1>/Unit Delay1'

        // SystemInitialize for Atomic SubSystem: '<S1>/Correction Step'
        // SystemInitialize for Atomic SubSystem: '<S2>/GPS Correction'
        // InitializeConditions for Memory: '<S34>/Memory'
        NASDAQ0_DW.Memory_PreviousInput = NASDAQ0_P.Memory_InitialCondition_o;

        // SystemInitialize for Enabled SubSystem: '<S6>/Active Correction Step'
        // Start for IdentityMatrix: '<S46>/Identity Matrix'
        std::memcpy(&NASDAQ0_DW.IdentityMatrix[0],
                    &NASDAQ0_P.IdentityMatrix_IDMatrixData_a[0],
                    sizeof(float) << 4U);

        // End of SystemInitialize for SubSystem: '<S6>/Active Correction Step'
        // End of SystemInitialize for SubSystem: '<S2>/GPS Correction'

        // SystemInitialize for Atomic SubSystem: '<S2>/Baro Correction'
        // InitializeConditions for Memory: '<S20>/Memory'
        NASDAQ0_DW.Memory_PreviousInput_d = NASDAQ0_P.Memory_InitialCondition_m;

        // SystemInitialize for Enabled SubSystem: '<S5>/Active Correction Step'
        // InitializeConditions for UnitDelay: '<S19>/Unit Delay1'
        NASDAQ0_DW.UnitDelay1_DSTATE = NASDAQ0_P.UnitDelay1_InitialCondition;

        // InitializeConditions for UnitDelay: '<S19>/Unit Delay'
        NASDAQ0_DW.UnitDelay_DSTATE_j = NASDAQ0_P.UnitDelay_InitialCondition_n;

        // Start for IdentityMatrix: '<S31>/Identity Matrix'
        NASDAQ0_DW.IdentityMatrix_h = NASDAQ0_P.IdentityMatrix_IDMatrixData_b;

        // End of SystemInitialize for SubSystem: '<S5>/Active Correction Step'
        // End of SystemInitialize for SubSystem: '<S2>/Baro Correction'

        // SystemInitialize for Atomic SubSystem: '<S2>/ADA Correction'
        // InitializeConditions for Memory: '<S8>/Memory'
        NASDAQ0_DW.Memory_PreviousInput_a = NASDAQ0_P.Memory_InitialCondition;

        // SystemInitialize for Enabled SubSystem: '<S4>/Active Correction Step'
        // Start for IdentityMatrix: '<S18>/Identity Matrix'
        NASDAQ0_DW.IdentityMatrix_b = NASDAQ0_P.IdentityMatrix_IDMatrixData;

        // End of SystemInitialize for SubSystem: '<S4>/Active Correction Step'
        // End of SystemInitialize for SubSystem: '<S2>/ADA Correction'
        // End of SystemInitialize for SubSystem: '<S1>/Correction Step'
        // End of SystemInitialize for SubSystem: '<Root>/NASDAQ - Autocoding'
    }
}

// Model terminate function
void NASDAQ0::terminate()
{
    // (no terminate code required)
}

// Constructor
NASDAQ0::NASDAQ0() : NASDAQ0_U(), NASDAQ0_Y(), NASDAQ0_DW(), NASDAQ0_M()
{
    // Currently there is no constructor body generated.
}

// Destructor
// Currently there is no destructor body generated.
NASDAQ0::~NASDAQ0() = default;

// Real-Time Model get method
NASDAQ0::RT_MODEL_NASDAQ0_T* NASDAQ0::getRTM() { return (&NASDAQ0_M); }

//
// File trailer for generated code.
//
// [EOF]
//
