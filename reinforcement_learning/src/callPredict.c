/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * callPredict.c
 *
 * Code generation for function 'callPredict'
 *
 */

/* Include files */
#include "callPredict.h"
#include "unaryElementwise.h"
#include <math.h>
#include <string.h>
#include <xmmintrin.h>

/* Function Declarations */
static int div_nde_s32_floor(int numerator);

static int div_s32_floor(int numerator, int denominator);

static void macroKernel1(int M, int K, int N, const float *A, int LDA,
                         const float *B, int LDB, float *C, int LDC);

static void macroKernel2(int M, int K, int N, const float *A, int LDA,
                         const float *B, int LDB, float *C, int LDC);

static void macroKernel3(int M, int K, int N, const float *A, int LDA,
                         const float *B, int LDB, float *C, int LDC);

static void macroKernel4(int M, int K, int N, const float *A, int LDA,
                         const float *B, int LDB, float *C, int LDC);

static void matrixMultiply1(int M, int K, int N, int blockSizeM, int blockSizeK,
                            int blockSizeN, const float *A, const float *B,
                            float *C);

static void matrixMultiply2(int M, int K, int N, int blockSizeM, int blockSizeK,
                            int blockSizeN, const float *A, const float *B,
                            float *C);

static void matrixMultiply3(int M, int K, int N, int blockSizeM, int blockSizeK,
                            int blockSizeN, const float *A, const float *B,
                            float *C);

static void matrixMultiply4(int M, int K, int N, int blockSizeM, int blockSizeK,
                            int blockSizeN, const float *A, const float *B,
                            float *C);

static void microKernel11(int K, const float *A, int LDA, const float *B,
                          float *C);

static void microKernel12(int K, const float *A, int LDA, const float *B,
                          float *C);

static void microKernel13(int K, const float *A, int LDA, const float *B,
                          float *C);

static void microKernel21(int K, const float *A, int LDA, const float *B,
                          float *C);

static void microKernel31(int K, const float *A, int LDA, const float *B,
                          float *C);

/* Function Definitions */
static int div_nde_s32_floor(int numerator)
{
  int quotient;
  if ((numerator < 0) && (numerator % 28 != 0)) {
    quotient = -1;
  } else {
    quotient = 0;
  }
  quotient += numerator / 28;
  return quotient;
}

static int div_s32_floor(int numerator, int denominator)
{
  int quotient;
  if (denominator == 0) {
    if (numerator >= 0) {
      quotient = MAX_int32_T;
    } else {
      quotient = MIN_int32_T;
    }
  } else {
    unsigned int absDenominator;
    unsigned int absNumerator;
    unsigned int tempAbsQuotient;
    boolean_T quotientNeedsNegation;
    if (numerator < 0) {
      absNumerator = ~(unsigned int)numerator + 1U;
    } else {
      absNumerator = (unsigned int)numerator;
    }
    if (denominator < 0) {
      absDenominator = ~(unsigned int)denominator + 1U;
    } else {
      absDenominator = (unsigned int)denominator;
    }
    quotientNeedsNegation = ((numerator < 0) != (denominator < 0));
    tempAbsQuotient = absNumerator / absDenominator;
    if (quotientNeedsNegation) {
      absNumerator %= absDenominator;
      if (absNumerator > 0U) {
        tempAbsQuotient++;
      }
      quotient = -(int)tempAbsQuotient;
    } else {
      quotient = (int)tempAbsQuotient;
    }
  }
  return quotient;
}

static void macroKernel1(int M, int K, int N, const float *A, int LDA,
                         const float *B, int LDB, float *C, int LDC)
{
  int idxB;
  int j;
  j = 0;
  idxB = 0;
  while (j <= N - 1) {
    int i;
    int idxA;
    int idxC;
    idxC = LDC * j;
    i = 0;
    idxA = 0;
    while (i <= M - 28) {
      microKernel11(K, &A[idxA], LDA, &B[idxB], &C[idxC]);
      idxA += 28;
      idxC += 28;
      i += 28;
    }
    while (i <= M - 4) {
      microKernel12(K, &A[idxA], LDA, &B[idxB], &C[idxC]);
      idxA += 4;
      idxC += 4;
      i += 4;
    }
    while (i <= M - 1) {
      microKernel13(K, &A[idxA], LDA, &B[idxB], &C[idxC]);
      idxA++;
      idxC++;
      i++;
    }
    idxB += LDB;
    j++;
  }
}

static void macroKernel2(int M, int K, int N, const float *A, int LDA,
                         const float *B, int LDB, float *C, int LDC)
{
  int idxB;
  int j;
  j = 0;
  idxB = 0;
  while (j <= N - 1) {
    int i;
    int idxA;
    int idxC;
    idxC = LDC * j;
    i = 0;
    idxA = 0;
    while (i <= M - 16) {
      microKernel21(K, &A[idxA], LDA, &B[idxB], &C[idxC]);
      idxA += 16;
      idxC += 16;
      i += 16;
    }
    while (i <= M - 4) {
      microKernel12(K, &A[idxA], LDA, &B[idxB], &C[idxC]);
      idxA += 4;
      idxC += 4;
      i += 4;
    }
    while (i <= M - 1) {
      microKernel13(K, &A[idxA], LDA, &B[idxB], &C[idxC]);
      idxA++;
      idxC++;
      i++;
    }
    idxB += LDB;
    j++;
  }
}

static void macroKernel3(int M, int K, int N, const float *A, int LDA,
                         const float *B, int LDB, float *C, int LDC)
{
  int idxB;
  int j;
  j = 0;
  idxB = 0;
  while (j <= N - 1) {
    int i;
    int idxA;
    int idxC;
    idxC = LDC * j;
    i = 0;
    idxA = 0;
    while (i <= M - 8) {
      microKernel31(K, &A[idxA], LDA, &B[idxB], &C[idxC]);
      idxA += 8;
      idxC += 8;
      i += 8;
    }
    while (i <= M - 4) {
      microKernel12(K, &A[idxA], LDA, &B[idxB], &C[idxC]);
      idxA += 4;
      idxC += 4;
      i += 4;
    }
    while (i <= M - 1) {
      microKernel13(K, &A[idxA], LDA, &B[idxB], &C[idxC]);
      idxA++;
      idxC++;
      i++;
    }
    idxB += LDB;
    j++;
  }
}

static void macroKernel4(int M, int K, int N, const float *A, int LDA,
                         const float *B, int LDB, float *C, int LDC)
{
  int idxB;
  int j;
  j = 0;
  idxB = 0;
  while (j <= N - 1) {
    int i;
    int idxA;
    int idxC;
    idxC = LDC * j;
    i = 0;
    idxA = 0;
    while (i <= M - 4) {
      microKernel12(K, &A[idxA], LDA, &B[idxB], &C[idxC]);
      idxA += 4;
      idxC += 4;
      i += 4;
    }
    while (i <= M - 1) {
      microKernel13(K, &A[idxA], LDA, &B[idxB], &C[idxC]);
      idxA++;
      idxC++;
      i++;
    }
    idxB += LDB;
    j++;
  }
}

static void matrixMultiply1(int M, int K, int N, int blockSizeM, int blockSizeK,
                            int blockSizeN, const float *A, const float *B,
                            float *C)
{
  int b_j1;
  int i0;
  int i0_ub;
  int k0;
  int k0_ub;
  memset(C, 0, (unsigned int)((M * N) << 2));
  if (blockSizeM >= M) {
    blockSizeM = M;
  } else {
    blockSizeM = div_nde_s32_floor(blockSizeM) * 28;
    if (blockSizeM <= 0) {
      blockSizeM = 1;
    }
  }
  if (blockSizeN >= N) {
    blockSizeN = N;
  } else if (blockSizeN <= 0) {
    blockSizeN = 1;
  }
  i0_ub = div_s32_floor(M - 1, blockSizeM) + 1;
  k0_ub = div_s32_floor(K - 1, blockSizeK) + 1;
  for (b_j1 = 0; b_j1 < N; b_j1 += blockSizeN) {
    int N2;
    if (b_j1 > N - blockSizeN) {
      N2 = N - b_j1;
    } else {
      N2 = blockSizeN;
    }
    for (k0 = 1; k0 <= k0_ub; k0++) {
      int K2;
      int k;
      k = (k0 - 1) * blockSizeK;
      if (k > K - blockSizeK) {
        K2 = K - k;
      } else {
        K2 = blockSizeK;
      }
      for (i0 = 1; i0 <= i0_ub; i0++) {
        int b_i;
        int i;
        i = (i0 - 1) * blockSizeM;
        if (i > M - blockSizeM) {
          b_i = M - i;
        } else {
          b_i = blockSizeM;
        }
        macroKernel1(b_i, K2, N2, &A[i + M * k], M, &B[k + K * b_j1], K,
                     &C[i + M * b_j1], M);
      }
    }
  }
}

static void matrixMultiply2(int M, int K, int N, int blockSizeM, int blockSizeK,
                            int blockSizeN, const float *A, const float *B,
                            float *C)
{
  int b_j1;
  int i0;
  int i0_ub;
  int k0;
  int k0_ub;
  memset(C, 0, (unsigned int)((M * N) << 2));
  if (blockSizeM >= M) {
    blockSizeM = M;
  } else {
    blockSizeM = (blockSizeM >> 4) << 4;
    if (blockSizeM <= 0) {
      blockSizeM = 1;
    }
  }
  if (blockSizeN >= N) {
    blockSizeN = N;
  } else if (blockSizeN <= 0) {
    blockSizeN = 1;
  }
  i0_ub = div_s32_floor(M - 1, blockSizeM) + 1;
  k0_ub = div_s32_floor(K - 1, blockSizeK) + 1;
  for (b_j1 = 0; b_j1 < N; b_j1 += blockSizeN) {
    int N2;
    if (b_j1 > N - blockSizeN) {
      N2 = N - b_j1;
    } else {
      N2 = blockSizeN;
    }
    for (k0 = 1; k0 <= k0_ub; k0++) {
      int K2;
      int k;
      k = (k0 - 1) * blockSizeK;
      if (k > K - blockSizeK) {
        K2 = K - k;
      } else {
        K2 = blockSizeK;
      }
      for (i0 = 1; i0 <= i0_ub; i0++) {
        int b_i;
        int i;
        i = (i0 - 1) * blockSizeM;
        if (i > M - blockSizeM) {
          b_i = M - i;
        } else {
          b_i = blockSizeM;
        }
        macroKernel2(b_i, K2, N2, &A[i + M * k], M, &B[k + K * b_j1], K,
                     &C[i + M * b_j1], M);
      }
    }
  }
}

static void matrixMultiply3(int M, int K, int N, int blockSizeM, int blockSizeK,
                            int blockSizeN, const float *A, const float *B,
                            float *C)
{
  int b_j1;
  int i0;
  int i0_ub;
  int k0;
  int k0_ub;
  memset(C, 0, (unsigned int)((M * N) << 2));
  if (blockSizeM >= M) {
    blockSizeM = M;
  } else {
    blockSizeM = (blockSizeM >> 3) << 3;
    if (blockSizeM <= 0) {
      blockSizeM = 1;
    }
  }
  if (blockSizeN >= N) {
    blockSizeN = N;
  } else if (blockSizeN <= 0) {
    blockSizeN = 1;
  }
  i0_ub = div_s32_floor(M - 1, blockSizeM) + 1;
  k0_ub = div_s32_floor(K - 1, blockSizeK) + 1;
  for (b_j1 = 0; b_j1 < N; b_j1 += blockSizeN) {
    int N2;
    if (b_j1 > N - blockSizeN) {
      N2 = N - b_j1;
    } else {
      N2 = blockSizeN;
    }
    for (k0 = 1; k0 <= k0_ub; k0++) {
      int K2;
      int k;
      k = (k0 - 1) * blockSizeK;
      if (k > K - blockSizeK) {
        K2 = K - k;
      } else {
        K2 = blockSizeK;
      }
      for (i0 = 1; i0 <= i0_ub; i0++) {
        int b_i;
        int i;
        i = (i0 - 1) * blockSizeM;
        if (i > M - blockSizeM) {
          b_i = M - i;
        } else {
          b_i = blockSizeM;
        }
        macroKernel3(b_i, K2, N2, &A[i + M * k], M, &B[k + K * b_j1], K,
                     &C[i + M * b_j1], M);
      }
    }
  }
}

static void matrixMultiply4(int M, int K, int N, int blockSizeM, int blockSizeK,
                            int blockSizeN, const float *A, const float *B,
                            float *C)
{
  int b_j1;
  int i0;
  int i0_ub;
  int k0;
  int k0_ub;
  memset(C, 0, (unsigned int)((M * N) << 2));
  if (blockSizeM >= M) {
    blockSizeM = M;
  } else {
    blockSizeM = (blockSizeM >> 2) << 2;
    if (blockSizeM <= 0) {
      blockSizeM = 1;
    }
  }
  if (blockSizeN >= N) {
    blockSizeN = N;
  } else if (blockSizeN <= 0) {
    blockSizeN = 1;
  }
  i0_ub = div_s32_floor(M - 1, blockSizeM) + 1;
  k0_ub = div_s32_floor(K - 1, blockSizeK) + 1;
  for (b_j1 = 0; b_j1 < N; b_j1 += blockSizeN) {
    int N2;
    if (b_j1 > N - blockSizeN) {
      N2 = N - b_j1;
    } else {
      N2 = blockSizeN;
    }
    for (k0 = 1; k0 <= k0_ub; k0++) {
      int K2;
      int k;
      k = (k0 - 1) * blockSizeK;
      if (k > K - blockSizeK) {
        K2 = K - k;
      } else {
        K2 = blockSizeK;
      }
      for (i0 = 1; i0 <= i0_ub; i0++) {
        int b_i;
        int i;
        i = (i0 - 1) * blockSizeM;
        if (i > M - blockSizeM) {
          b_i = M - i;
        } else {
          b_i = blockSizeM;
        }
        macroKernel4(b_i, K2, N2, &A[i + M * k], M, &B[k + K * b_j1], K,
                     &C[i + M * b_j1], M);
      }
    }
  }
}

static void microKernel11(int K, const float *A, int LDA, const float *B,
                          float *C)
{
  __m128 b_c;
  __m128 c;
  __m128 c_c;
  __m128 d_c;
  __m128 e_c;
  __m128 f_c;
  __m128 g_c;
  int idxA;
  int idxB;
  int k;
  idxA = 0;
  idxB = 0;
  c = _mm_loadu_ps(&C[0]);
  b_c = _mm_loadu_ps(&C[4]);
  c_c = _mm_loadu_ps(&C[8]);
  d_c = _mm_loadu_ps(&C[12]);
  e_c = _mm_loadu_ps(&C[16]);
  f_c = _mm_loadu_ps(&C[20]);
  g_c = _mm_loadu_ps(&C[24]);
  for (k = 0; k < K; k++) {
    __m128 aFloat;
    __m128 b;
    __m128 b_aFloat;
    __m128 c_aFloat;
    __m128 d_aFloat;
    __m128 e_aFloat;
    __m128 f_aFloat;
    __m128 g_aFloat;
    aFloat = _mm_loadu_ps(&A[idxA]);
    b_aFloat = _mm_loadu_ps(&A[idxA + 4]);
    c_aFloat = _mm_loadu_ps(&A[idxA + 8]);
    d_aFloat = _mm_loadu_ps(&A[idxA + 12]);
    e_aFloat = _mm_loadu_ps(&A[idxA + 16]);
    f_aFloat = _mm_loadu_ps(&A[idxA + 20]);
    g_aFloat = _mm_loadu_ps(&A[idxA + 24]);
    b = _mm_set1_ps(B[idxB]);
    c = _mm_add_ps(c, _mm_mul_ps(aFloat, b));
    b_c = _mm_add_ps(b_c, _mm_mul_ps(b_aFloat, b));
    c_c = _mm_add_ps(c_c, _mm_mul_ps(c_aFloat, b));
    d_c = _mm_add_ps(d_c, _mm_mul_ps(d_aFloat, b));
    e_c = _mm_add_ps(e_c, _mm_mul_ps(e_aFloat, b));
    f_c = _mm_add_ps(f_c, _mm_mul_ps(f_aFloat, b));
    g_c = _mm_add_ps(g_c, _mm_mul_ps(g_aFloat, b));
    idxA += LDA;
    idxB++;
  }
  _mm_storeu_ps(&C[0], c);
  _mm_storeu_ps(&C[4], b_c);
  _mm_storeu_ps(&C[8], c_c);
  _mm_storeu_ps(&C[12], d_c);
  _mm_storeu_ps(&C[16], e_c);
  _mm_storeu_ps(&C[20], f_c);
  _mm_storeu_ps(&C[24], g_c);
}

static void microKernel12(int K, const float *A, int LDA, const float *B,
                          float *C)
{
  __m128 c;
  int idxA;
  int idxB;
  int k;
  idxA = 0;
  idxB = 0;
  c = _mm_loadu_ps(&C[0]);
  for (k = 0; k < K; k++) {
    __m128 aFloat;
    aFloat = _mm_loadu_ps(&A[idxA]);
    c = _mm_add_ps(c, _mm_mul_ps(aFloat, _mm_set1_ps(B[idxB])));
    idxA += LDA;
    idxB++;
  }
  _mm_storeu_ps(&C[0], c);
}

static void microKernel13(int K, const float *A, int LDA, const float *B,
                          float *C)
{
  float c;
  int idxA;
  int idxB;
  int k;
  idxA = 0;
  idxB = 0;
  c = C[0];
  for (k = 0; k < K; k++) {
    c += A[idxA] * B[idxB];
    idxA += LDA;
    idxB++;
  }
  C[0] = c;
}

static void microKernel21(int K, const float *A, int LDA, const float *B,
                          float *C)
{
  __m128 b_c;
  __m128 c;
  __m128 c_c;
  __m128 d_c;
  int idxA;
  int idxB;
  int k;
  idxA = 0;
  idxB = 0;
  c = _mm_loadu_ps(&C[0]);
  b_c = _mm_loadu_ps(&C[4]);
  c_c = _mm_loadu_ps(&C[8]);
  d_c = _mm_loadu_ps(&C[12]);
  for (k = 0; k < K; k++) {
    __m128 aFloat;
    __m128 b;
    __m128 b_aFloat;
    __m128 c_aFloat;
    __m128 d_aFloat;
    aFloat = _mm_loadu_ps(&A[idxA]);
    b_aFloat = _mm_loadu_ps(&A[idxA + 4]);
    c_aFloat = _mm_loadu_ps(&A[idxA + 8]);
    d_aFloat = _mm_loadu_ps(&A[idxA + 12]);
    b = _mm_set1_ps(B[idxB]);
    c = _mm_add_ps(c, _mm_mul_ps(aFloat, b));
    b_c = _mm_add_ps(b_c, _mm_mul_ps(b_aFloat, b));
    c_c = _mm_add_ps(c_c, _mm_mul_ps(c_aFloat, b));
    d_c = _mm_add_ps(d_c, _mm_mul_ps(d_aFloat, b));
    idxA += LDA;
    idxB++;
  }
  _mm_storeu_ps(&C[0], c);
  _mm_storeu_ps(&C[4], b_c);
  _mm_storeu_ps(&C[8], c_c);
  _mm_storeu_ps(&C[12], d_c);
}

static void microKernel31(int K, const float *A, int LDA, const float *B,
                          float *C)
{
  __m128 b_c;
  __m128 c;
  int idxA;
  int idxB;
  int k;
  idxA = 0;
  idxB = 0;
  c = _mm_loadu_ps(&C[0]);
  b_c = _mm_loadu_ps(&C[4]);
  for (k = 0; k < K; k++) {
    __m128 aFloat;
    __m128 b;
    __m128 b_aFloat;
    aFloat = _mm_loadu_ps(&A[idxA]);
    b_aFloat = _mm_loadu_ps(&A[idxA + 4]);
    b = _mm_set1_ps(B[idxB]);
    c = _mm_add_ps(c, _mm_mul_ps(aFloat, b));
    b_c = _mm_add_ps(b_c, _mm_mul_ps(b_aFloat, b));
    idxA += LDA;
    idxB++;
  }
  _mm_storeu_ps(&C[0], c);
  _mm_storeu_ps(&C[4], b_c);
}

void predict(const float inputsT_0_f1[10], float outputs_0_f1[2],
             float outputs_1_f1[2])
{
  static const float t1_Weights[512] = {
      0.273887664F,    -0.286094785F,    0.0947484821F,   0.00581273902F,
      -0.346033096F,   -0.470569104F,    -0.678508699F,   -0.373185605F,
      -0.295627236F,   -0.000174845918F, -0.214842051F,   -0.482367843F,
      -0.339461178F,   -0.113512479F,    -1.39027F,       -0.377665818F,
      0.0976703614F,   0.00995043F,      0.168123603F,    -0.00215100637F,
      0.0822000653F,   0.102169275F,     -0.0250721108F,  0.247948304F,
      0.0138744721F,   0.0768092F,       0.00450785598F,  -0.0758304894F,
      0.158063844F,    0.272457182F,     -0.126990393F,   0.00639511505F,
      0.0550178438F,   -0.0297288056F,   -0.569118321F,   -0.527129531F,
      -0.800863624F,   -0.88834542F,     0.142978191F,    -1.29870057F,
      -0.147015944F,   -0.0738465041F,   -0.107529171F,   -0.0963657647F,
      -1.23403227F,    -0.223674938F,    -0.22297591F,    0.067865856F,
      -0.01259501F,    0.0537479445F,    0.0387209393F,   0.373141F,
      -0.336087227F,   -0.446708143F,    0.119960606F,    -0.189188153F,
      -0.0193327591F,  -0.0859500095F,   -0.0053394367F,  -0.0347892866F,
      -0.192854717F,   0.144838661F,     0.00774644315F,  -0.154120266F,
      0.248903781F,    0.108766556F,     0.261193305F,    0.0989876539F,
      0.0260767F,      0.0264934842F,    0.026936179F,    0.341106713F,
      0.167471185F,    0.194492415F,     0.114465408F,    -0.237386331F,
      0.147738755F,    0.282392323F,     -0.150067538F,   0.0602082349F,
      -0.114668839F,   0.0123110153F,    0.0572023913F,   0.0137611572F,
      0.114292257F,    0.139414594F,     0.0978172943F,   0.0158263948F,
      -0.00105960702F, 0.0703113154F,    0.0117755365F,   0.425112456F,
      0.107227497F,    -0.0630325228F,   0.184217751F,    0.129559949F,
      -0.0729232F,     0.0118476413F,    0.0854852796F,   0.0289129801F,
      0.123158164F,    0.152326763F,     0.071428977F,    0.0893702F,
      -0.00220294786F, 0.0769016296F,    0.00174822379F,  0.332268834F,
      0.132966056F,    0.0286764838F,    0.0994524956F,   0.0930140465F,
      -0.177426249F,   -0.0891123563F,   -0.953170896F,   -0.391268253F,
      -0.333794773F,   -0.221806601F,    -0.0847573057F,  -1.17854846F,
      -0.134840757F,   -0.409576505F,    -0.0527878292F,  -0.235287592F,
      -0.922373772F,   0.112023182F,     -0.096454829F,   -0.451016963F,
      0.135672644F,    0.0186680611F,    0.184310213F,    0.0797276199F,
      0.0603038259F,   0.0732474849F,    -0.0132492781F,  0.282852888F,
      0.0265593044F,   0.0893914625F,    0.00378944259F,  -0.164117768F,
      0.162445411F,    0.25600335F,      -0.140649036F,   -0.00233362825F,
      0.272118956F,    -0.426534623F,    -0.0339614339F,  -0.163510144F,
      -0.309064627F,   -0.382395864F,    -0.674369097F,   -0.332990974F,
      -0.380415767F,   -0.0485932902F,   -0.435039699F,   -0.466116816F,
      -0.336696953F,   -0.209423125F,    -1.12932789F,    -0.327423841F,
      -0.167306691F,   -0.195160076F,    -0.920037568F,   -0.417863458F,
      -0.567997932F,   -0.418199241F,    -0.198343545F,   -1.31501949F,
      -0.202351406F,   -0.353262067F,    -0.125674173F,   -0.44085592F,
      -1.16444647F,    0.197013408F,     -0.231210753F,   -0.54885608F,
      0.0453414805F,   -0.399332166F,    -0.368987024F,   -0.195938721F,
      -0.54124105F,    -0.753075F,       -0.303378493F,   -0.721817553F,
      -0.518249273F,   -0.0433438122F,   -0.410416365F,   -0.251191258F,
      -0.653840899F,   -0.225839227F,    -0.599314272F,   -0.32437852F,
      0.0761870891F,   0.183742434F,     0.0452783108F,   0.0184876826F,
      -0.0256458409F,  -0.0302446373F,   0.160863534F,    -0.0673976F,
      0.198222041F,    0.131105855F,     0.18973881F,     0.0639313459F,
      0.00341327046F,  -0.0206494145F,   0.241864F,       0.198703453F,
      0.138501436F,    -0.969848275F,    -0.504279F,      -1.00499129F,
      -0.0775170848F,  -0.035982009F,    -0.518143654F,   -0.464317918F,
      -1.1851263F,     -1.00652695F,     -1.146106F,      -0.375247359F,
      -0.317799F,      -0.0914904699F,   -0.442366958F,   -0.582846582F,
      0.129767403F,    0.182048097F,     -0.0331160091F,  0.0803414807F,
      -0.0687706172F,  -0.0510011353F,   0.145661786F,    -0.0721214488F,
      0.211129829F,    0.120907299F,     0.181499884F,    -0.0334811062F,
      -0.0656690598F,  -0.0573110096F,   0.140660688F,    0.106369995F,
      0.166948274F,    0.139763013F,     0.161097467F,    0.0973744243F,
      0.0044660056F,   -0.0145544885F,   0.112618171F,    0.171635881F,
      0.161440894F,    0.176316544F,     0.174245194F,    -0.104971834F,
      0.0572025552F,   0.161886185F,     0.0561284609F,   0.132971168F,
      -0.0722352341F,  0.019742582F,     0.0976661369F,   0.0431002975F,
      0.107645422F,    0.140564263F,     0.0611169599F,   0.0849831849F,
      0.0214804951F,   0.11271821F,      0.0198163521F,   0.314699471F,
      0.110776782F,    0.0148436828F,    0.0904617682F,   0.0786573142F,
      -0.0741076097F,  -0.223277912F,    -0.835185707F,   -0.336749792F,
      -0.67025727F,    -0.550511301F,    -0.271799833F,   -1.23398852F,
      -0.209245831F,   -0.357335478F,    -0.161541104F,   -0.574897051F,
      -1.16520703F,    0.289696842F,     -0.343964964F,   -0.610313475F,
      0.23386319F,     -1.24501908F,     -0.327474058F,   -0.90936321F,
      -0.1674463F,     -0.160190746F,    -0.642659843F,   -0.297773778F,
      -1.39757025F,    -0.894674F,       -1.42211092F,    -0.421089828F,
      -0.248503968F,   -0.11272122F,     -0.410380691F,   -0.40171206F,
      0.00226142327F,  -0.000881835062F, 0.127297655F,    0.054393921F,
      0.143152595F,    0.180684924F,     0.00223781308F,  0.197350264F,
      0.0282434188F,   0.0741420165F,    -0.0207706019F,  0.137924805F,
      0.172172368F,    0.166816637F,     -0.0519861616F,  0.00994803384F,
      0.045073092F,    -0.565551698F,    -0.565676749F,   -0.881523669F,
      -0.0870738104F,  -0.0208772F,      -0.231988877F,   -0.527822554F,
      -0.757406414F,   -0.842014134F,    -0.662606776F,   -0.234786272F,
      -0.374761224F,   -0.0106583349F,   -0.194818974F,   -0.33775875F,
      0.0972014442F,   0.14089717F,      0.0764963925F,   0.0161496438F,
      -0.0161847975F,  -0.0136228511F,   0.13942945F,     0.0607231148F,
      0.157526523F,    0.120912291F,     0.162693277F,    0.00449129846F,
      0.0125351353F,   0.0105704768F,    0.178197801F,    0.171562493F,
      0.16493243F,     0.139265493F,     0.119676806F,    0.0647885278F,
      -0.0359190218F,  -0.0497568734F,   0.104047738F,    0.114171751F,
      0.186901048F,    0.156122103F,     0.177121401F,    -0.157044291F,
      0.0391664542F,   0.127056688F,     0.0720027F,      0.126803011F,
      0.613537133F,    -0.714466214F,    -0.144585118F,   0.0550968871F,
      -0.393155962F,   -0.534825563F,    -0.89330703F,    -0.234481916F,
      -0.555438161F,   -0.122229137F,    -0.755614102F,   -0.738938212F,
      -0.294599F,      -0.287977159F,    -1.05849159F,    -0.187554747F,
      0.188870609F,    0.128564492F,     0.15253976F,     0.0703855455F,
      -0.0232733022F,  -0.0389273502F,   0.085233435F,    0.124922387F,
      0.183542699F,    0.157081276F,     0.14126201F,     -0.20251517F,
      0.0512948297F,   0.181790471F,     0.0250498354F,   0.096179612F,
      0.110448092F,    -0.00708768191F,  0.119547933F,    0.0057303817F,
      0.027847115F,    0.0367875509F,    -0.00921408553F, 0.207002982F,
      0.00332271052F,  0.063436F,        -0.0313768908F,  -0.145104811F,
      0.108299278F,    0.202674478F,     -0.120158903F,   0.00770814763F,
      0.0164474603F,   -0.207816F,       -0.0664920136F,  -0.205462754F,
      -0.986317217F,   -1.13328564F,     -0.261931926F,   -0.504989088F,
      -0.21965614F,    0.0564655326F,    -0.14139396F,    -0.413285196F,
      -0.605582833F,   -0.0160287488F,   -0.423715711F,   -0.513552964F,
      0.489314586F,    -1.03978312F,     -0.0742938F,     -0.355343848F,
      -0.286416233F,   -0.407789022F,    -0.865009964F,   -0.092550233F,
      -1.00827348F,    -0.417836666F,    -1.10837531F,    -0.664256036F,
      -0.149563178F,   -0.110295266F,    -0.799522936F,   -0.420298845F,
      0.343085F,       0.379103869F,     0.165052816F,    0.00554181822F,
      -0.354031861F,   -0.393898636F,    0.303136081F,    0.0563499F,
      0.36783278F,     0.327806175F,     0.362608135F,    -0.373706043F,
      -0.0913526267F,  -0.025574401F,    0.0202059727F,   -0.0142781464F,
      0.151859462F,    0.131501913F,     0.120723337F,    0.0489913747F,
      -0.0385070518F,  -0.0461246036F,   0.100490011F,    0.0811402053F,
      0.194757789F,    0.146651909F,     0.170301631F,    -0.143076092F,
      0.015466338F,    0.0880275071F,    0.0766836107F,   0.130941123F,
      0.0933811218F,   -0.368778557F,    -0.11364726F,    -0.186140612F,
      -1.03961492F,    -1.08937109F,     -0.501422F,      -0.411010832F,
      -0.313163906F,   -0.0901401F,      -0.262256503F,   -0.648852289F,
      -0.609548092F,   0.270618498F,     -0.612056434F,   -0.676163375F,
      -0.145786867F,   -0.192515329F,    -0.895343721F,   -0.374990553F,
      -0.588000119F,   -0.467674822F,    -0.169906065F,   -1.28581452F,
      -0.202814713F,   -0.33687678F,     -0.140117839F,   -0.429606557F,
      -1.12703609F,    0.192166626F,     -0.184286803F,   -0.558013797F};
  static const float t0_Weights[320] = {
      -1.09122097F,    0.152000815F,     -0.344743669F,   -0.0733888224F,
      0.15249975F,     -0.020740306F,    0.0375382788F,   0.664924502F,
      0.181290075F,    -1.00168109F,     0.652033925F,    0.0501724035F,
      -0.0968099684F,  0.328381687F,     -0.0577893406F,  0.0910317823F,
      0.0361885205F,   0.399421036F,     0.0146489367F,   0.123611577F,
      0.384132773F,    -0.0736102313F,   0.0402796715F,   -0.351794273F,
      0.0894220173F,   0.199962527F,     -0.228475332F,   -0.395171076F,
      -0.0515770093F,  0.00589899532F,   0.272248417F,    0.693630934F,
      -0.0654846F,     -0.0769041479F,   -0.0282801725F,  0.00291827251F,
      0.0642254725F,   -0.0779190138F,   -0.0406543799F,  0.0271324944F,
      0.0196915F,      -0.0203037616F,   0.024319848F,    0.0313379541F,
      -0.0252208672F,  0.0108519988F,    0.0047246865F,   0.00084358186F,
      -0.0227813572F,  0.0224301685F,    -0.00496810116F, 0.0559850931F,
      0.0185740385F,   -0.0184150375F,   -0.00629023416F, -0.0234560184F,
      0.00994697772F,  -0.10192129F,     -0.0166365467F,  -0.0153852776F,
      -0.0108895684F,  -0.00838007592F,  0.0169481598F,   0.0203350671F,
      -0.241470128F,   -0.0017731213F,   -0.100752205F,   0.0700849742F,
      0.0944628641F,   -0.143898562F,    -0.10667666F,    0.227503732F,
      0.0319431797F,   -0.0259869602F,   -0.00875409693F, -0.71642673F,
      0.0556837618F,   0.848353624F,     -0.231645495F,   0.191287652F,
      -0.10294991F,    -0.0462918431F,   0.722022712F,    -0.0375194289F,
      0.443366259F,    0.0569936484F,    0.131689861F,    0.382437944F,
      0.141407877F,    0.0655127913F,    -0.718268096F,   0.581284761F,
      -0.181145191F,   0.109550722F,     -0.465425521F,   -0.109849051F,
      -0.0246878397F,  0.0403209031F,    -0.0110482192F,  -0.0483274162F,
      0.00382774253F,  -0.043373324F,    -0.0430905931F,  0.011583793F,
      -0.012351336F,   -0.0181345157F,   -0.00608414039F, -0.0451400131F,
      0.00529775769F,  0.0347917192F,    -0.0117067238F,  -0.0153993517F,
      -0.0533919223F,  -0.0155415041F,   0.0248814914F,   -0.0100954203F,
      0.0337773189F,   0.00303407852F,   0.00399738643F,  0.0264293868F,
      0.00971351378F,  -0.0662648752F,   -0.0510911942F,  0.0735611692F,
      0.19972676F,     0.000425200647F,  -0.00316763041F, -0.0136855179F,
      0.679293871F,    -0.262770176F,    0.352740049F,    -0.406053036F,
      -0.216083258F,   0.249809831F,     0.111732379F,    -0.447165936F,
      -0.344073921F,   0.587298155F,     -0.463475049F,   0.0996511579F,
      0.254123479F,    -0.162609324F,    0.0619035847F,   0.0488788262F,
      0.185540587F,    -0.564103305F,    0.0669166297F,   -0.12744911F,
      -0.209099606F,   0.191473171F,     0.0554253943F,   0.823880255F,
      -0.0232645441F,  -0.128718108F,    0.00761945499F,  0.0360182524F,
      0.0642051F,      0.0810091197F,    -0.294653773F,   -0.477807254F,
      -0.318244249F,   -0.149848029F,    -0.10454502F,    -0.778670073F,
      0.108961008F,    -0.29376775F,     -0.288960576F,   0.19103156F,
      -0.0483792499F,  -0.070304893F,    -0.0365899242F,  -0.726753175F,
      0.16749683F,     0.757471204F,     0.513444424F,    0.138538748F,
      -0.261495233F,   -0.043721132F,    0.653809845F,    -0.272824109F,
      0.443518102F,    0.143955171F,     0.219677076F,    0.697609842F,
      0.218416303F,    -0.0484864898F,   -0.639166117F,   0.635600328F,
      0.181350648F,    0.197638899F,     -0.56429565F,    -0.116056226F,
      1.68950605F,     0.0606480464F,    2.16143131F,     0.131644592F,
      -0.0573439077F,  -0.0299826022F,   0.0173349325F,   -2.45560884F,
      0.0231415927F,   2.0593586F,       -2.82635546F,    1.26771832F,
      0.000524809177F, -1.64302051F,     0.172417775F,    -0.0421055444F,
      0.0433891565F,   -2.77038813F,     -0.989975154F,   0.0441485941F,
      -1.64380395F,    -0.00550078228F,  -0.0311874077F,  1.40042794F,
      -0.0500103235F,  0.128342241F,     0.21594429F,     0.536989927F,
      -0.156045973F,   -0.0164584946F,   -1.01928723F,    -2.71396804F,
      0.15190357F,     0.0974589959F,    0.162595034F,    -0.0140226437F,
      0.250066131F,    0.0265759397F,    0.0391760878F,   -0.163234621F,
      0.147160664F,    0.660960495F,     -0.876608729F,   -1.37057066F,
      0.00630061934F,  3.21817112F,      -0.12272495F,    0.117000937F,
      0.0256581847F,   -0.836527824F,    3.71981478F,     0.0513037071F,
      1.75638235F,     0.00757942488F,   0.111660853F,    1.61489522F,
      0.13646251F,     0.0402000695F,    -2.01401281F,    2.87549257F,
      0.156216219F,    0.0834764764F,    -2.0248363F,     -1.01666212F,
      0.0122069009F,   -0.00222094869F,  -0.00204143603F, -0.0114194509F,
      -0.00883816928F, 0.0117019666F,    0.00239213649F,  -0.000389044406F,
      -0.0121068023F,  -0.012184036F,    0.00397488661F,  -0.00249548582F,
      -0.0143755497F,  -0.0113563929F,   0.00412975671F,  0.0164096206F,
      0.00171578478F,  0.00493431045F,   -0.00797142647F, 0.00515500735F,
      -0.0124941571F,  -0.0195550881F,   -0.00227032625F, -0.00688735722F,
      -0.0045266524F,  -0.000311419193F, 0.00293828128F,  0.00146311219F,
      -0.00462505361F, -0.00570309255F,  0.0103849387F,   0.00332687725F,
      0.0174991954F,   -0.00925093517F,  -0.0031049035F,  -0.0146113979F,
      -0.00770980678F, 0.009638655F,     0.010297331F,    -0.00891210139F,
      0.0140264276F,   0.00337224081F,   0.00440014899F,  -0.0142504815F,
      -0.00114136795F, 0.00360735366F,   -0.00149342208F, 0.0211057626F,
      0.0118112536F,   0.00281257159F,   0.00438406877F,  0.0191095695F,
      0.00372551242F,  0.00195721374F,   0.00618156325F,  -0.0118396087F,
      -0.00768896379F, -0.0173250102F,   0.00369392731F,  0.00265331892F,
      -0.0409906F,     0.000593055F,     0.0116590336F,   0.00556002511F};
  static const float t2_Weights[128] = {
      0.0596417859F, -0.123500764F,  0.378955185F,   -0.205023646F,
      0.320218772F,  0.376565278F,   0.26222837F,    -0.07969F,
      -0.898790598F, -0.899918318F,  0.621907234F,   -0.926222861F,
      1.0854404F,    0.634832382F,   -0.0313955508F, -0.948962927F,
      0.512336254F,  -0.230574563F,  0.840927184F,   -0.387955874F,
      -0.592579305F, 0.821600318F,   1.01554024F,    -0.0482485443F,
      -0.441320568F, -0.715521216F,  0.682351947F,   -0.819867074F,
      0.405900061F,  0.678001344F,   0.390627474F,   -0.684366703F,
      0.728458703F,  0.456343889F,   -0.0611420535F, 0.500010073F,
      -1.2696557F,   -0.0810031071F, 0.500472963F,   0.491593063F,
      0.756184936F,  0.584852159F,   -0.189779148F,  0.65807569F,
      -1.26085067F,  -0.209568113F,  0.410492271F,   0.584174335F,
      -1.09986317F,  -0.250675976F,  -0.171490774F,  -0.177755684F,
      1.06241786F,   -0.15119487F,   -0.865476966F,  -0.45330897F,
      0.944373786F,  -0.205868304F,  1.01081502F,    -0.350158185F,
      -1.23646843F,  0.974013805F,   1.48220754F,    0.0287706386F,
      -0.695937276F, -1.01282477F,   0.849527538F,   -1.08014071F,
      1.07123387F,   0.858015835F,   0.231547743F,   -0.995658576F,
      -0.130254969F, -0.63669461F,   0.789966226F,   -0.764141202F,
      0.441667914F,  0.788602054F,   0.489148229F,   -0.515380085F,
      -0.81663847F,  -1.0644635F,    0.834915F,      -1.11856699F,
      0.842319965F,  0.840292811F,   0.222101182F,   -1.05153191F,
      0.296925813F,  0.628411174F,   -0.495353669F,  0.832856059F,
      0.32599619F,   -0.486065775F,  -0.300248444F,  0.4731929F,
      0.937562704F,  -0.0592757575F, 0.695724F,      -0.148293316F,
      -1.55253375F,  0.666025698F,   1.2400862F,     0.154257566F,
      0.515246868F,  0.137007639F,   0.276940227F,   0.0697219223F,
      0.261790037F,  0.279476106F,   0.46023947F,    0.207648605F,
      -0.724533081F, 0.352742434F,   -0.72829324F,   0.494225591F,
      1.22421491F,   -0.703749061F,  -1.26215875F,   0.0943252668F,
      0.109444104F,  0.263402015F,   -0.103346057F,  0.238133594F,
      0.884361565F,  -0.0915281326F, -0.148767039F,  0.236938775F};
  static const float fv[32] = {
      -0.00439859321F, 0.0463450253F, -0.0102102533F, -0.00282798149F,
      0.094898425F,    0.13322103F,   0.102872595F,   0.027217431F,
      0.0454820395F,   0.0216631722F, 0.0182489213F,  -0.0115978438F,
      0.0650967062F,   0.0196056925F, -0.002209862F,  0.0803567246F,
      0.0899127573F,   0.0203612167F, 0.0112698618F,  0.0835319832F,
      0.0229683537F,   0.0554791093F, 0.052016288F,   -0.0101809902F,
      0.0522097424F,   0.031854149F,  0.0352811106F,  -0.013503355F,
      -0.0641784742F,  0.0492846332F, 0.0124766072F,  0.0224263892F};
  static const float t4_Weights[32] = {
      0.13207151F,    0.125142157F,   -0.0806581452F, -0.0827458426F,
      0.140421271F,   0.123028718F,   -0.0113819223F, 0.00277956342F,
      -0.134694949F,  -0.0928916559F, -0.254153699F,  -0.160378218F,
      -0.139016286F,  -0.249927953F,  -0.135300606F,  -0.163159668F,
      -0.0824133083F, -0.076577723F,  0.19914344F,    0.232104644F,
      -0.0788133219F, -0.0673931614F, -0.368039608F,  -0.41027F,
      -0.125928253F,  -0.153939083F,  -0.207236409F,  -0.156967118F,
      -0.245212153F,  -0.388459384F,  -0.462218523F,  -0.564632297F};
  static const float fv1[16] = {
      0.0640454665F, 0.105010577F,  0.098603636F, 0.0401897654F,
      0.14509359F,   0.160701558F,  0.118734308F, 0.130532056F,
      0.0976723805F, 0.0776979104F, 0.0776142F,   0.303412467F,
      0.149239406F,  0.151838198F,  0.196920291F, 0.324962139F};
  static const float t3_Weights[16] = {
      1.35021925F,   1.21446788F,  1.4886061F,   -0.607532501F,
      -1.02044737F,  1.0710727F,   1.53360665F,  -0.761050582F,
      -1.53356123F,  -1.78629303F, -1.03060901F, 1.02730656F,
      -0.101905845F, 1.93756855F,  1.56858182F,  -0.229232758F};
  static const float fv2[8] = {0.13237679F,  0.183892727F, 0.101198055F,
                               0.284716398F, 0.231239185F, 0.0951209F,
                               0.0924315F,   0.112148903F};
  __m128 r;
  float layerOutput[32];
  float outT_f3_0_f1[32];
  float b_layerOutput[16];
  float outT_f5_0_f1[16];
  float c_layerOutput[8];
  float outT_f7_0_f1[8];
  float d_layerOutput[2];
  int i;
  matrixMultiply1(32, 10, 1, 128, 128, 128, &t0_Weights[0], &inputsT_0_f1[0],
                  &layerOutput[0]);
  for (i = 0; i <= 28; i += 4) {
    r = _mm_loadu_ps(&layerOutput[i]);
    _mm_storeu_ps(&layerOutput[i], _mm_add_ps(r, _mm_loadu_ps(&fv[i])));
  }
  float outT_f11_0_f1[2];
  unaryElementwise(layerOutput, outT_f3_0_f1);
  matrixMultiply2(16, 32, 1, 128, 128, 128, &t1_Weights[0], &outT_f3_0_f1[0],
                  &b_layerOutput[0]);
  r = _mm_loadu_ps(&b_layerOutput[0]);
  _mm_storeu_ps(&b_layerOutput[0], _mm_add_ps(r, _mm_loadu_ps(&fv1[0])));
  r = _mm_loadu_ps(&b_layerOutput[4]);
  _mm_storeu_ps(&b_layerOutput[4], _mm_add_ps(r, _mm_loadu_ps(&fv1[4])));
  r = _mm_loadu_ps(&b_layerOutput[8]);
  _mm_storeu_ps(&b_layerOutput[8], _mm_add_ps(r, _mm_loadu_ps(&fv1[8])));
  r = _mm_loadu_ps(&b_layerOutput[12]);
  _mm_storeu_ps(&b_layerOutput[12], _mm_add_ps(r, _mm_loadu_ps(&fv1[12])));
  b_unaryElementwise(b_layerOutput, outT_f5_0_f1);
  matrixMultiply3(8, 16, 1, 128, 128, 128, &t2_Weights[0], &outT_f5_0_f1[0],
                  &c_layerOutput[0]);
  r = _mm_loadu_ps(&c_layerOutput[0]);
  _mm_storeu_ps(&c_layerOutput[0], _mm_add_ps(r, _mm_loadu_ps(&fv2[0])));
  r = _mm_loadu_ps(&c_layerOutput[4]);
  _mm_storeu_ps(&c_layerOutput[4], _mm_add_ps(r, _mm_loadu_ps(&fv2[4])));
  c_unaryElementwise(c_layerOutput, outT_f7_0_f1);
  matrixMultiply4(2, 8, 1, 128, 128, 128, &t3_Weights[0], &outT_f7_0_f1[0],
                  &outputs_0_f1[0]);
  outputs_0_f1[0] += 0.0984569043F;
  outputs_0_f1[1] -= 0.371990472F;
  matrixMultiply4(2, 16, 1, 128, 128, 128, &t4_Weights[0], &outT_f5_0_f1[0],
                  &d_layerOutput[0]);
  d_layerOutput[0] += 0.240744367F;
  d_layerOutput[1] += 0.267782658F;
  d_unaryElementwise(d_layerOutput, outT_f11_0_f1);
  outputs_1_f1[0] = fmaxf(outT_f11_0_f1[0], 0.0F) +
                    logf(expf(-fabsf(outT_f11_0_f1[0])) + 1.0F);
  outputs_1_f1[1] = fmaxf(outT_f11_0_f1[1], 0.0F) +
                    logf(expf(-fabsf(outT_f11_0_f1[1])) + 1.0F);
}

/* End of code generation (callPredict.c) */
