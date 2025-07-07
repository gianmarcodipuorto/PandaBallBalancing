/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * mySACPolicy_terminate.c
 *
 * Code generation for function 'mySACPolicy_terminate'
 *
 */

/* Include files */
#include "mySACPolicy_terminate.h"
#include "mySACPolicy.h"
#include "mySACPolicy_data.h"
#include "omp.h"

/* Function Definitions */
void mySACPolicy_terminate(void)
{
  mySACPolicy_delete();
  omp_destroy_nest_lock(&mySACPolicy_nestLockGlobal);
  isInitialized_mySACPolicy = false;
}

/* End of code generation (mySACPolicy_terminate.c) */
