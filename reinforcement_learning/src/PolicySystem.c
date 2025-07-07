/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * PolicySystem.c
 *
 * Code generation for function 'PolicySystem'
 *
 */

/* Include files */
#include "PolicySystem.h"
#include "callPredict.h"
#include "mySACPolicy_internal_types.h"
#include "randn.h"
#include <emmintrin.h>
#include <math.h>

/* Function Definitions */
void PolicySystem_getAction(const c_rl_codegen_policy_rlContinuou *b_this,
                            const double varargin_1[10], double varargout_1[2])
{
  __m128d r;
  __m128d r1;
  __m128d r2;
  double tmp_data[2];
  float b_varargin_1[10];
  float action[2];
  float t5_Data[2];
  float t6_Data[2];
  int k;
  int partialTrueCount;
  boolean_T b;
  boolean_T b1;
  for (partialTrueCount = 0; partialTrueCount < 10; partialTrueCount++) {
    b_varargin_1[partialTrueCount] = (float)varargin_1[partialTrueCount];
  }
  predict(b_varargin_1, t6_Data, t5_Data);
  action[0] = t6_Data[0];
  action[1] = t6_Data[1];
  if (!b_this->UseMaxLikelihoodAction_) {
    randn(action);
    action[0] = t6_Data[0] + t5_Data[0] * action[0];
    action[1] = t6_Data[1] + t5_Data[1] * action[1];
  }
  varargout_1[0] = action[0];
  varargout_1[1] = action[1];
  partialTrueCount = 0;
  b = b_this->ActionBounder_.UpperBoundedIdx_[0].f1[0];
  b1 = b_this->ActionBounder_.LowerBoundedIdx_[0].f1[0];
  if ((b && (action[0] > 0.0F)) || (b1 && (action[0] < 0.0F)) || (b && b1)) {
    tmp_data[0] = action[0];
    partialTrueCount = 1;
  }
  b = b_this->ActionBounder_.UpperBoundedIdx_[0].f1[1];
  b1 = b_this->ActionBounder_.LowerBoundedIdx_[0].f1[1];
  if ((b && (action[1] > 0.0F)) || (b1 && (action[1] < 0.0F)) || (b && b1)) {
    tmp_data[partialTrueCount] = action[1];
  }
  partialTrueCount = 0;
  b = b_this->ActionBounder_.UpperBoundedIdx_[0].f1[0];
  b1 = b_this->ActionBounder_.LowerBoundedIdx_[0].f1[0];
  if ((b && (action[0] > 0.0F)) || (b1 && (action[0] < 0.0F)) || (b && b1)) {
    partialTrueCount = 1;
  }
  b = b_this->ActionBounder_.UpperBoundedIdx_[0].f1[1];
  b1 = b_this->ActionBounder_.LowerBoundedIdx_[0].f1[1];
  if ((b && (action[1] > 0.0F)) || (b1 && (action[1] < 0.0F)) || (b && b1)) {
    partialTrueCount++;
  }
  for (k = 0; k < partialTrueCount; k++) {
    tmp_data[k] = tanh(tmp_data[k]);
  }
  partialTrueCount = 0;
  b = b_this->ActionBounder_.UpperBoundedIdx_[0].f1[0];
  b1 = b_this->ActionBounder_.LowerBoundedIdx_[0].f1[0];
  if ((b && (action[0] > 0.0F)) || (b1 && (action[0] < 0.0F)) || (b && b1)) {
    varargout_1[0] = tmp_data[0];
    partialTrueCount = 1;
  }
  b = b_this->ActionBounder_.UpperBoundedIdx_[0].f1[1];
  b1 = b_this->ActionBounder_.LowerBoundedIdx_[0].f1[1];
  if ((b && (action[1] > 0.0F)) || (b1 && (action[1] < 0.0F)) || (b && b1)) {
    varargout_1[1] = tmp_data[partialTrueCount];
  }
  r = _mm_loadu_pd(&varargout_1[0]);
  r1 = _mm_loadu_pd(&b_this->ActionBounder_.Scale_[0].f1[0]);
  r2 = _mm_loadu_pd(&b_this->ActionBounder_.Bias_[0].f1[0]);
  _mm_storeu_pd(&varargout_1[0], _mm_add_pd(_mm_mul_pd(r, r1), r2));
}

/* End of code generation (PolicySystem.c) */
