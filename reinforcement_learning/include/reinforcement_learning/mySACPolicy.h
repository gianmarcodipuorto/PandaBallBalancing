/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * mySACPolicy.h
 *
 * Code generation for function 'mySACPolicy'
 *
 */

#ifndef MYSACPOLICY_H
#define MYSACPOLICY_H

/* Include files */
#include "rtwtypes.h"
#include <stddef.h>
#include <stdlib.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Function Declarations */
extern void mySACPolicy(const double observation1[10], double action1[2]);

void mySACPolicy_delete(void);

void mySACPolicy_init(void);

void mySACPolicy_new(void);

#ifdef __cplusplus
}
#endif

#endif
/* End of code generation (mySACPolicy.h) */
