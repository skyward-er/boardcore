//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// File: ABK_private.h
//
// Code generated for Simulink model 'ABK'.
//
// Model version                  : 11.280
// Simulink Coder version         : 24.2 (R2024b) 21-Jun-2024
// C/C++ source code generated on : Mon Jun  1 12:33:18 2026
//
// Target selection: ert.tlc
// Embedded hardware selection: ARM Compatible->ARM Cortex-M
// Code generation objectives:
//    1. Execution efficiency
//    2. Debugging
//    3. RAM efficiency
// Validation result: Not run
//
#ifndef ABK_private_h_
#define ABK_private_h_
#include <stdbool.h>
#include <stdint.h>
#include "zero_crossing_types.h"
#include "ABK_types.h"
#include "ABK.h"

extern uint32_t plook_u32ff_evenx(float u, float bp0, float bpSpace, uint32_t
  maxIndex, float *fraction);
extern uint8_t plook_u8f_evencka(float u, float bp0, float bpSpace, uint32_t
  maxIndex);
extern uint16_t plook_u16ff_binc(float u, const float bp[], uint32_t maxIndex,
  float *fraction);
extern uint16_t plook_u16fdf_evenx(float u, double bp0, double bpSpace, uint32_t
  maxIndex, float *fraction);
extern uint16_t binsearch_u16f(float u, const float bp[], uint32_t startIndex,
  uint32_t maxIndex);

#endif                                 // ABK_private_h_

//
// File trailer for generated code.
//
// [EOF]
//
