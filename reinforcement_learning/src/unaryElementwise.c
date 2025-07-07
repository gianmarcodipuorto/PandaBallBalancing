/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * unaryElementwise.c
 *
 * Code generation for function 'unaryElementwise'
 *
 */

/* Include files */
#include "unaryElementwise.h"
#include "omp.h"
#include <math.h>
#include <xmmintrin.h>

/* Function Declarations */
static void elementwise_relu(const float *inputTensor, float *outputTensor,
                             int numElements);

/* Function Definitions */
static void elementwise_relu(const float *inputTensor, float *outputTensor,
                             int numElements)
{
  __m128 reg_0;
  __m128 reluZero;
  int baseIdx;
  int numSimdBlockTiles;
  int simdBlockIdx;
  reluZero = _mm_set1_ps(0.0F);
  numSimdBlockTiles = numElements >> 2;
#pragma omp parallel for num_threads(omp_get_max_threads()) private(baseIdx,   \
                                                                        reg_0)

  for (simdBlockIdx = 0; simdBlockIdx < numSimdBlockTiles; simdBlockIdx++) {
    baseIdx = simdBlockIdx << 2;
    reg_0 = _mm_loadu_ps(&inputTensor[baseIdx]);
    reg_0 = _mm_max_ps(reg_0, reluZero);
    _mm_storeu_ps(&outputTensor[baseIdx], reg_0);
  }
  baseIdx = numSimdBlockTiles << 2;
  for (numSimdBlockTiles = 0; numSimdBlockTiles < numElements % 4;
       numSimdBlockTiles++) {
    outputTensor[baseIdx] = fmaxf(inputTensor[baseIdx], 0.0F);
    baseIdx++;
  }
}

void b_unaryElementwise(const float X[16], float Z[16])
{
  elementwise_relu(&X[0], &Z[0], 16);
}

void c_unaryElementwise(const float X[8], float Z[8])
{
  elementwise_relu(&X[0], &Z[0], 8);
}

void d_unaryElementwise(const float X[2], float Z[2])
{
  elementwise_relu(&X[0], &Z[0], 2);
}

void unaryElementwise(const float X[32], float Z[32])
{
  elementwise_relu(&X[0], &Z[0], 32);
}

/* End of code generation (unaryElementwise.c) */
