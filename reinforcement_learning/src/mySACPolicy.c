/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * mySACPolicy.c
 *
 * Code generation for function 'mySACPolicy'
 *
 */

/* Include files */
#include "mySACPolicy.h"
#include "PolicySystem.h"
#include "handle.h"
#include "loadRLPolicy.h"
#include "mySACPolicy_data.h"
#include "mySACPolicy_initialize.h"
#include "mySACPolicy_internal_types.h"

/* Variable Definitions */
static rl_codegen_model_DLNetworkModel gobj_1;

static c_rl_codegen_policy_rlContinuou policy;

static boolean_T policy_not_empty;

/* Function Definitions */
void mySACPolicy(const double observation1[10], double action1[2])
{
  if (!isInitialized_mySACPolicy) {
    mySACPolicy_initialize();
  }
  /*  Reinforcement Learning Toolbox */
  /*  Generated on: 03-Jun-2025 15:13:19 */
  if (!policy_not_empty) {
    loadRLPolicy(&gobj_1, &policy);
    policy_not_empty = true;
  }
  /*  evaluate the policy */
  PolicySystem_getAction(&policy, observation1, action1);
}

void mySACPolicy_delete(void)
{
  b_handle_matlabCodegenDestructo(&policy);
  handle_matlabCodegenDestructor(&gobj_1);
}

void mySACPolicy_init(void)
{
  policy_not_empty = false;
}

void mySACPolicy_new(void)
{
  gobj_1.matlabCodegenIsDeleted = true;
  policy.matlabCodegenIsDeleted = true;
}

/* End of code generation (mySACPolicy.c) */
