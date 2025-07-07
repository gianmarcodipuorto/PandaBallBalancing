/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * unaryElementwise.h
 *
 * Code generation for function 'unaryElementwise'
 *
 */

#ifndef UNARYELEMENTWISE_H
#define UNARYELEMENTWISE_H

/* Include files */
#include "rtwtypes.h"
#include <stddef.h>
#include <stdlib.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Function Declarations */
void b_unaryElementwise(const float X[16], float Z[16]);

void c_unaryElementwise(const float X[8], float Z[8]);

void d_unaryElementwise(const float X[2], float Z[2]);

void unaryElementwise(const float X[32], float Z[32]);

#ifdef __cplusplus
}
#endif

#endif
/* End of code generation (unaryElementwise.h) */
