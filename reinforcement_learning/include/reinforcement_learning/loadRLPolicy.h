/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * loadRLPolicy.h
 *
 * Code generation for function 'loadRLPolicy'
 *
 */

#ifndef LOADRLPOLICY_H
#define LOADRLPOLICY_H

/* Include files */
#include "mySACPolicy_internal_types.h"
#include "rtwtypes.h"
#include <stddef.h>
#include <stdlib.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Function Declarations */
c_rl_codegen_policy_rlContinuou *
loadRLPolicy(rl_codegen_model_DLNetworkModel *iobj_0,
             c_rl_codegen_policy_rlContinuou *iobj_1);

#ifdef __cplusplus
}
#endif

#endif
/* End of code generation (loadRLPolicy.h) */
