/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * mySACPolicy_initialize.c
 *
 * Code generation for function 'mySACPolicy_initialize'
 *
 */

/* Include files */
#include "mySACPolicy_initialize.h"
#include "eml_rand_mt19937ar_stateful.h"
#include "mySACPolicy.h"
#include "mySACPolicy_data.h"
#include "omp.h"

/* Function Definitions */
void mySACPolicy_initialize(void)
{
  omp_init_nest_lock(&mySACPolicy_nestLockGlobal);
  mySACPolicy_new();
  mySACPolicy_init();
  c_eml_rand_mt19937ar_stateful_i();
  isInitialized_mySACPolicy = true;
}

/* End of code generation (mySACPolicy_initialize.c) */
