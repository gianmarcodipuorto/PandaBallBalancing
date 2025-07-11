/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * eml_rand_mt19937ar.h
 *
 * Code generation for function 'eml_rand_mt19937ar'
 *
 */

#ifndef EML_RAND_MT19937AR_H
#define EML_RAND_MT19937AR_H

/* Include files */
#include "rtwtypes.h"
#include <stddef.h>
#include <stdlib.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Function Declarations */
void eml_rand_mt19937ar(unsigned int b_state[625]);

void genrand_uint32_vector(unsigned int mt[625], unsigned int u[2]);

#ifdef __cplusplus
}
#endif

#endif
/* End of code generation (eml_rand_mt19937ar.h) */
