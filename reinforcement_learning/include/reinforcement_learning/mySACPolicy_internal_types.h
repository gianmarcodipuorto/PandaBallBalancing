/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * mySACPolicy_internal_types.h
 *
 * Code generation for function 'mySACPolicy'
 *
 */

#ifndef MYSACPOLICY_INTERNAL_TYPES_H
#define MYSACPOLICY_INTERNAL_TYPES_H

/* Include files */
#include "mySACPolicy_types.h"
#include "rtwtypes.h"

/* Type Definitions */
#ifndef typedef_cell_wrap_8
#define typedef_cell_wrap_8
typedef struct {
  double f1[2];
} cell_wrap_8;
#endif /* typedef_cell_wrap_8 */

#ifndef typedef_cell_wrap_9
#define typedef_cell_wrap_9
typedef struct {
  boolean_T f1[2];
} cell_wrap_9;
#endif /* typedef_cell_wrap_9 */

#ifndef c_typedef_rl_codegen_model_DLNe
#define c_typedef_rl_codegen_model_DLNe
typedef struct {
  boolean_T matlabCodegenIsDeleted;
  int isInitialized;
  boolean_T isSetupComplete;
} rl_codegen_model_DLNetworkModel;
#endif /* c_typedef_rl_codegen_model_DLNe */

#ifndef c_typedef_c_rl_codegen_policy_i
#define c_typedef_c_rl_codegen_policy_i
typedef struct {
  cell_wrap_8 Scale_[1];
  cell_wrap_8 Bias_[1];
  cell_wrap_9 UpperBoundedIdx_[1];
  cell_wrap_9 LowerBoundedIdx_[1];
} c_rl_codegen_policy_internal_Ta;
#endif /* c_typedef_c_rl_codegen_policy_i */

#ifndef c_typedef_c_rl_codegen_policy_r
#define c_typedef_c_rl_codegen_policy_r
typedef struct {
  boolean_T matlabCodegenIsDeleted;
  int isInitialized;
  boolean_T isSetupComplete;
  rl_codegen_model_DLNetworkModel *Model_;
  boolean_T UseMaxLikelihoodAction_;
  c_rl_codegen_policy_internal_Ta ActionBounder_;
} c_rl_codegen_policy_rlContinuou;
#endif /* c_typedef_c_rl_codegen_policy_r */

#endif
/* End of code generation (mySACPolicy_internal_types.h) */
