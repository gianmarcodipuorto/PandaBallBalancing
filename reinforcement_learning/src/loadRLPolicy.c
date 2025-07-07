/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * loadRLPolicy.c
 *
 * Code generation for function 'loadRLPolicy'
 *
 */

/* Include files */
#include "loadRLPolicy.h"
#include "mySACPolicy_internal_types.h"

/* Function Definitions */
c_rl_codegen_policy_rlContinuou *
loadRLPolicy(rl_codegen_model_DLNetworkModel *iobj_0,
             c_rl_codegen_policy_rlContinuou *iobj_1)
{
  c_rl_codegen_policy_rlContinuou *varargout_1;
  cell_wrap_8 this_Bias_;
  cell_wrap_8 this_Scale_;
  cell_wrap_9 this_LowerBoundedIdx_;
  cell_wrap_9 this_UpperBoundedIdx_;
  iobj_0->isInitialized = 0;
  iobj_0->matlabCodegenIsDeleted = false;
  iobj_0->isSetupComplete = false;
  iobj_0->isInitialized = 1;
  iobj_0->isSetupComplete = true;
  iobj_1->isInitialized = 0;
  varargout_1 = iobj_1;
  iobj_1->UseMaxLikelihoodAction_ = true;
  iobj_1->Model_ = iobj_0;
  this_Scale_.f1[0] = 0.3;
  this_Bias_.f1[0] = 0.0;
  this_UpperBoundedIdx_.f1[0] = true;
  this_LowerBoundedIdx_.f1[0] = true;
  this_Scale_.f1[1] = 0.3;
  this_Bias_.f1[1] = 0.0;
  this_UpperBoundedIdx_.f1[1] = true;
  this_LowerBoundedIdx_.f1[1] = true;
  iobj_1->ActionBounder_.Scale_[0] = this_Scale_;
  iobj_1->ActionBounder_.Bias_[0] = this_Bias_;
  iobj_1->ActionBounder_.UpperBoundedIdx_[0] = this_UpperBoundedIdx_;
  iobj_1->ActionBounder_.LowerBoundedIdx_[0] = this_LowerBoundedIdx_;
  iobj_1->matlabCodegenIsDeleted = false;
  iobj_1->isSetupComplete = false;
  iobj_1->isInitialized = 1;
  iobj_1->isSetupComplete = true;
  return varargout_1;
}

/* End of code generation (loadRLPolicy.c) */
