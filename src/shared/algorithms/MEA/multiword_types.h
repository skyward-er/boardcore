//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// File: multiword_types.h
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

#ifndef MULTIWORD_TYPES_H
#define MULTIWORD_TYPES_H
#include <stdint.h>

//
//  MultiWord types
typedef struct {
  uint32_t chunks[2];
} int64m_T;

typedef struct {
  uint32_t chunks[2];
} uint64m_T;

typedef struct {
  uint32_t chunks[3];
} int96m_T;

typedef struct {
  uint32_t chunks[3];
} uint96m_T;

typedef struct {
  uint32_t chunks[4];
} int128m_T;

typedef struct {
  uint32_t chunks[4];
} uint128m_T;

typedef struct {
  uint32_t chunks[5];
} int160m_T;

typedef struct {
  uint32_t chunks[5];
} uint160m_T;

typedef struct {
  uint32_t chunks[6];
} int192m_T;

typedef struct {
  uint32_t chunks[6];
} uint192m_T;

typedef struct {
  uint32_t chunks[7];
} int224m_T;

typedef struct {
  uint32_t chunks[7];
} uint224m_T;

typedef struct {
  uint32_t chunks[8];
} int256m_T;

typedef struct {
  uint32_t chunks[8];
} uint256m_T;

#endif                                 // MULTIWORD_TYPES_H

//
// File trailer for generated code.
//
// [EOF]
//
