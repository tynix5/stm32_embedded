/******************************************************************************
 * @file     transform_functions_f16.h
 * @brief    Public header file for CMSIS DSP Library
 * @version  V1.10.0
 * @date     08 July 2021
 * Target Processor: Cortex-M and Cortex-A cores
 ******************************************************************************/
/*
 * Copyright (c) 2010-2020 Arm Limited or its affiliates. All rights reserved.
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Licensed under the Apache License, Version 2.0 (the License); you may
 * not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an AS IS BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

 
#ifndef TRANSFORM_FUNCTIONS_F16_H_
#define TRANSFORM_FUNCTIONS_F16_H_

#include "arm_math_types_f16.h"
#include "arm_math_memory.h"

#include "dsp/none.h"
#include "dsp/utils.h"

#if !defined(ARM_MIXED_RADIX_FFT)
#define ARM_MIXED_RADIX_FFT 1
#endif

#if !defined(ARM_MATH_NEON)
#if defined(ARM_MFCC_CFFT_BASED)
#if !defined(ARM_MFCC_USE_CFFT)
#define ARM_MFCC_USE_CFFT
#endif
#endif
#endif

#ifdef   __cplusplus
extern "C"
{
#endif



#if defined(ARM_FLOAT16_SUPPORTED)


  /**
   * @brief Instance structure for the floating-point CFFT/CIFFT function.
   */
  typedef struct
  {
          uint16_t fftLen;                   /**< length of the FFT. */
          uint8_t ifftFlag;                  /**< flag that selects forward (ifftFlag=0) or inverse (ifftFlag=1) transform. */
          uint8_t bitReverseFlag;            /**< flag that enables (bitReverseFlag=1) or disables (bitReverseFlag=0) bit reversal of output. */
    const float16_t *pTwiddle;               /**< points to the Twiddle factor table. */
    const uint16_t *pBitRevTable;            /**< points to the bit reversal table. */
          uint16_t twidCoefModifier;         /**< twiddle coefficient modifier that supports different size FFTs with the same twiddle factor table. */
          uint16_t bitRevFactor;             /**< bit reversal modifier that supports different size FFTs with the same bit reversal table. */
          float16_t onebyfftLen;             /**< value of 1/fftLen. */
  } arm_cfft_radix2_instance_f16;

  /**
   * @brief Instance structure for the floating-point CFFT/CIFFT function.
   */
  typedef struct
  {
          uint16_t fftLen;                   /**< length of the FFT. */
          uint8_t ifftFlag;                  /**< flag that selects forward (ifftFlag=0) or inverse (ifftFlag=1) transform. */
          uint8_t bitReverseFlag;            /**< flag that enables (bitReverseFlag=1) or disables (bitReverseFlag=0) bit reversal of output. */
    const float16_t *pTwiddle;               /**< points to the Twiddle factor table. */
    const uint16_t *pBitRevTable;            /**< points to the bit reversal table. */
          uint16_t twidCoefModifier;         /**< twiddle coefficient modifier that supports different size FFTs with the same twiddle factor table. */
          uint16_t bitRevFactor;             /**< bit reversal modifier that supports different size FFTs with the same bit reversal table. */
          float16_t onebyfftLen;             /**< value of 1/fftLen. */
  } arm_cfft_radix4_instance_f16;

  /**
   * @brief Instance structure for the floating-point CFFT/CIFFT function.
   */
#if defined(ARM_MATH_NEON_FLOAT16)
typedef struct
{
          uint32_t fftLen;                   /**< length of the FFT. */
    const float16_t *pTwiddle;         /**< points to the Twiddle factor table. */
    const float16_t *last_twiddles; /**< last stage twiddle used for mixed radix */
    const uint32_t *factors;
    int32_t algorithm_flag;
} arm_cfft_instance_f16;
#else
  typedef struct
  {
          uint16_t fftLen;                   /**< length of the FFT. */
    const float16_t *pTwiddle;         /**< points to the Twiddle factor table. */
    const uint16_t *pBitRevTable;      /**< points to the bit reversal table. */
          uint16_t bitRevLength;             /**< bit reversal table length. */
#if defined(ARM_MATH_MVEF) && !defined(ARM_MATH_AUTOVECTORIZE)
   const uint32_t *rearranged_twiddle_tab_stride1_arr;        /**< Per stage reordered twiddle pointer (offset 1) */                                                       \
   const uint32_t *rearranged_twiddle_tab_stride2_arr;        /**< Per stage reordered twiddle pointer (offset 2) */                                                       \
   const uint32_t *rearranged_twiddle_tab_stride3_arr;        /**< Per stage reordered twiddle pointer (offset 3) */                                                       \
   const float16_t *rearranged_twiddle_stride1; /**< reordered twiddle offset 1 storage */                                                                   \
   const float16_t *rearranged_twiddle_stride2; /**< reordered twiddle offset 2 storage */                                                                   \
   const float16_t *rearranged_twiddle_stride3;
#endif
  } arm_cfft_instance_f16;
#endif

arm_status arm_cfft_init_4096_f16(arm_cfft_instance_f16 * S);
arm_status arm_cfft_init_2048_f16(arm_cfft_instance_f16 * S);
arm_status arm_cfft_init_1024_f16(arm_cfft_instance_f16 * S);
arm_status arm_cfft_init_512_f16(arm_cfft_instance_f16 * S);
arm_status arm_cfft_init_256_f16(arm_cfft_instance_f16 * S);
arm_status arm_cfft_init_128_f16(arm_cfft_instance_f16 * S);
arm_status arm_cfft_init_64_f16(arm_cfft_instance_f16 * S);
arm_status arm_cfft_init_32_f16(arm_cfft_instance_f16 * S);
arm_status arm_cfft_init_16_f16(arm_cfft_instance_f16 * S);


  arm_status arm_cfft_init_f16(
  arm_cfft_instance_f16 * S,
  uint16_t fftLen);

#if defined(ARM_MATH_NEON_FLOAT16)

extern arm_cfft_instance_f16 *arm_cfft_init_dynamic_f16(uint32_t fftLen);

void arm_cfft_f16(
  const arm_cfft_instance_f16 * S,
        const float16_t * pIn,
        float16_t * pOut,
        float16_t * pBuffer, /* When used, `in` is not modified */
        uint8_t ifftFlag);
#else
  void arm_cfft_f16(
  const arm_cfft_instance_f16 * S,
        float16_t * p1,
        uint8_t ifftFlag,
        uint8_t bitReverseFlag);
#endif
  /**
   * @brief Instance structure for the floating-point RFFT/RIFFT function.
   */
#if defined(ARM_MATH_NEON_FLOAT16)
  typedef struct
  {
    uint32_t nfft;
    const float16_t *r_twiddles;
    const uint32_t *r_factors;
    const float16_t *r_twiddles_backward;
    const float16_t *r_twiddles_neon;
    const float16_t *r_twiddles_neon_backward;
    const uint32_t *r_factors_neon;
    const float16_t *r_super_twiddles_neon;
  } arm_rfft_fast_instance_f16 ;
#else
typedef struct
  {
          arm_cfft_instance_f16 Sint;      /**< Internal CFFT structure. */
          uint16_t fftLenRFFT;             /**< length of the real sequence */
    const float16_t * pTwiddleRFFT;        /**< Twiddle factors real stage  */
  } arm_rfft_fast_instance_f16 ;
#endif 

arm_status arm_rfft_fast_init_32_f16( arm_rfft_fast_instance_f16 * S );
arm_status arm_rfft_fast_init_64_f16( arm_rfft_fast_instance_f16 * S );
arm_status arm_rfft_fast_init_128_f16( arm_rfft_fast_instance_f16 * S );
arm_status arm_rfft_fast_init_256_f16( arm_rfft_fast_instance_f16 * S );
arm_status arm_rfft_fast_init_512_f16( arm_rfft_fast_instance_f16 * S );
arm_status arm_rfft_fast_init_1024_f16( arm_rfft_fast_instance_f16 * S );
arm_status arm_rfft_fast_init_2048_f16( arm_rfft_fast_instance_f16 * S );
arm_status arm_rfft_fast_init_4096_f16( arm_rfft_fast_instance_f16 * S );

arm_status arm_rfft_fast_init_f16 (
         arm_rfft_fast_instance_f16 * S,
         uint16_t fftLen);

#if defined(ARM_MATH_NEON_FLOAT16)

extern arm_rfft_fast_instance_f16 *arm_rfft_fast_init_dynamic_f16 (uint32_t fftLen);

void arm_rfft_fast_f16(
        const arm_rfft_fast_instance_f16 * S,
        const float16_t * p, 
        float16_t * pOut,
        float16_t *tmpbuf,
        uint8_t ifftFlag);
#else
  void arm_rfft_fast_f16(
        const arm_rfft_fast_instance_f16 * S,
        float16_t * p, float16_t * pOut,
        uint8_t ifftFlag);
#endif 

/* Deprecated */
  arm_status arm_cfft_radix4_init_f16(
        arm_cfft_radix4_instance_f16 * S,
        uint16_t fftLen,
        uint8_t ifftFlag,
        uint8_t bitReverseFlag);

/* Deprecated */
  void arm_cfft_radix4_f16(
  const arm_cfft_radix4_instance_f16 * S,
        float16_t * pSrc);


/* Deprecated */
  arm_status arm_cfft_radix2_init_f16(
        arm_cfft_radix2_instance_f16 * S,
        uint16_t fftLen,
        uint8_t ifftFlag,
        uint8_t bitReverseFlag);

/* Deprecated */
  void arm_cfft_radix2_f16(
  const arm_cfft_radix2_instance_f16 * S,
        float16_t * pSrc);

  /**
   * @brief Instance structure for the Floating-point MFCC function.
   */
typedef struct
  {
     const float16_t *dctCoefs; /**< Internal DCT coefficients */
     const float16_t *filterCoefs; /**< Internal Mel filter coefficients */ 
     const float16_t *windowCoefs; /**< Windowing coefficients */ 
     const uint32_t *filterPos; /**< Internal Mel filter positions in spectrum */ 
     const uint32_t *filterLengths; /**< Internal Mel filter  lengths */ 
     uint32_t fftLen; /**< FFT length */
     uint32_t nbMelFilters; /**< Number of Mel filters */
     uint32_t nbDctOutputs; /**< Number of DCT outputs */
#if defined(ARM_MFCC_USE_CFFT)
     /* Implementation of the MFCC is using a CFFT */
     arm_cfft_instance_f16 cfft; /**< Internal CFFT instance */
#else
     /* Implementation of the MFCC is using a RFFT (default) */
     arm_rfft_fast_instance_f16 rfft;
#endif
  } arm_mfcc_instance_f16 ;

arm_status arm_mfcc_init_32_f16(
  arm_mfcc_instance_f16 * S,
  uint32_t nbMelFilters,
  uint32_t nbDctOutputs,
  const float16_t *dctCoefs,
  const uint32_t *filterPos,
  const uint32_t *filterLengths,
  const float16_t *filterCoefs,
  const float16_t *windowCoefs
  );

arm_status arm_mfcc_init_64_f16(
  arm_mfcc_instance_f16 * S,
  uint32_t nbMelFilters,
  uint32_t nbDctOutputs,
  const float16_t *dctCoefs,
  const uint32_t *filterPos,
  const uint32_t *filterLengths,
  const float16_t *filterCoefs,
  const float16_t *windowCoefs
  );

arm_status arm_mfcc_init_128_f16(
  arm_mfcc_instance_f16 * S,
  uint32_t nbMelFilters,
  uint32_t nbDctOutputs,
  const float16_t *dctCoefs,
  const uint32_t *filterPos,
  const uint32_t *filterLengths,
  const float16_t *filterCoefs,
  const float16_t *windowCoefs
  );

arm_status arm_mfcc_init_256_f16(
  arm_mfcc_instance_f16 * S,
  uint32_t nbMelFilters,
  uint32_t nbDctOutputs,
  const float16_t *dctCoefs,
  const uint32_t *filterPos,
  const uint32_t *filterLengths,
  const float16_t *filterCoefs,
  const float16_t *windowCoefs
  );

arm_status arm_mfcc_init_512_f16(
  arm_mfcc_instance_f16 * S,
  uint32_t nbMelFilters,
  uint32_t nbDctOutputs,
  const float16_t *dctCoefs,
  const uint32_t *filterPos,
  const uint32_t *filterLengths,
  const float16_t *filterCoefs,
  const float16_t *windowCoefs
  );

arm_status arm_mfcc_init_1024_f16(
  arm_mfcc_instance_f16 * S,
  uint32_t nbMelFilters,
  uint32_t nbDctOutputs,
  const float16_t *dctCoefs,
  const uint32_t *filterPos,
  const uint32_t *filterLengths,
  const float16_t *filterCoefs,
  const float16_t *windowCoefs
  );

arm_status arm_mfcc_init_2048_f16(
  arm_mfcc_instance_f16 * S,
  uint32_t nbMelFilters,
  uint32_t nbDctOutputs,
  const float16_t *dctCoefs,
  const uint32_t *filterPos,
  const uint32_t *filterLengths,
  const float16_t *filterCoefs,
  const float16_t *windowCoefs
  );

arm_status arm_mfcc_init_4096_f16(
  arm_mfcc_instance_f16 * S,
  uint32_t nbMelFilters,
  uint32_t nbDctOutputs,
  const float16_t *dctCoefs,
  const uint32_t *filterPos,
  const uint32_t *filterLengths,
  const float16_t *filterCoefs,
  const float16_t *windowCoefs
  );

arm_status arm_mfcc_init_f16(
  arm_mfcc_instance_f16 * S,
  uint32_t fftLen,
  uint32_t nbMelFilters,
  uint32_t nbDctOutputs,
  const float16_t *dctCoefs,
  const uint32_t *filterPos,
  const uint32_t *filterLengths,
  const float16_t *filterCoefs,
  const float16_t *windowCoefs
  );



/**
  @brief         MFCC F16
  @param[in]    S       points to the mfcc instance structure
  @param[in]     pSrc points to the input samples
  @param[out]     pDst  points to the output MFCC values
  @param[inout]     pTmp  points to a temporary buffer of complex
 */
#if defined(ARM_MATH_NEON_FLOAT16)
void arm_mfcc_f16(
  const arm_mfcc_instance_f16 * S,
  float16_t *pSrc,
  float16_t *pDst,
  float16_t *pTmp,
  float16_t *pTmp2
  );
#else
  void arm_mfcc_f16(
  const arm_mfcc_instance_f16 * S,
  float16_t *pSrc,
  float16_t *pDst,
  float16_t *pTmp
  );
#endif
  
#endif /* defined(ARM_FLOAT16_SUPPORTED)*/

#ifdef   __cplusplus
}
#endif

#endif /* ifndef _TRANSFORM_FUNCTIONS_F16_H_ */
