/*
 * Copyright (c) 2016, Freescale Semiconductor, Inc.
 * Copyright 2016-2017 NXP
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * o Redistributions of source code must retain the above copyright notice, this list
 *   of conditions and the following disclaimer.
 *
 * o Redistributions in binary form must reproduce the above copyright notice, this
 *   list of conditions and the following disclaimer in the documentation and/or
 *   other materials provided with the distribution.
 *
 * o Neither the name of the copyright holder nor the names of its
 *   contributors may be used to endorse or promote products derived from this
 *   software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
 * ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

/**
 * @addtogroup FSP
 * @{
 */
#include "fsl_fsp.h"

/*******************************************************************************
* Definitions
******************************************************************************/
#define kFSP
/*******************************************************************************
 * Prototypes
 ******************************************************************************/
/**
* @details  Check FSP work mode(Interrupt or Polling)
* @param base FSP peripheral base address.
* @param mask interrupt source.
* @return true or false.
*/
static inline bool FSP_CheckWorkMode(FSP_Type *base, uint32_t mask);

/**
 * @details  Processing function for the Matrix operation unit.
 * @brief    Check if the Matrix operation unit has finished calculation.
 * @param base FSP peripheral base address.
 * @return none.
 */
static inline void FSP_WaitMatOpDone(FSP_Type *base);

/**
 * @details  Processing function for the transfer engine.
 * @brief    Check if the transfer engine has finished calculation.
 * @param base FSP peripheral base address.
 * @return none.
 */
static inline void FSP_WaitTeOpDone(FSP_Type *base);

/**
 * @details  Processing function for the statistic engine.
 * @brief    Check if the statistic engine has finished calculation.
 * @param base FSP peripheral base address.
 * @return none.
 */
static inline void FSP_WaitSeOpDone(FSP_Type *base);

/**
 * @brief    Check if the correlation has finished or not.
 * @param base FSP peripheral base address.
 * @return none.
 */
static inline void FSP_WaitCorrOpDone(FSP_Type *base);

/*******************************************************************************
 * Code
 ******************************************************************************/
static inline bool FSP_CheckWorkMode(FSP_Type *base, uint32_t mask)
{
    return (base->INTEN & mask) ? false : true;
}

static inline void FSP_WaitMatOpDone(FSP_Type *base)
{
    if (FSP_CheckWorkMode(base, FSP_INTEN_MOU_DONE_INTEN_MASK))
    {
        while (!(FSP_GetStatusFlags(base) & FSP_INT_MOU_DONE_INT_MASK))
            ;
        FSP_ClearStatusFlags(base, FSP_INT_MOU_DONE_INT_MASK);
    }
}

static inline void FSP_WaitTeOpDone(FSP_Type *base)
{
    if (FSP_CheckWorkMode(base, FSP_INTEN_TE_DONE_INTEN_MASK))
    {
        while (!(FSP_GetStatusFlags(base) & FSP_INT_TE_DONE_INT_MASK))
            ;
        FSP_ClearStatusFlags(base, FSP_INT_TE_DONE_INT_MASK);
    }
}

static inline void FSP_WaitSeOpDone(FSP_Type *base)
{
    while (!(FSP_GetStatusFlags(base) & FSP_INT_SE_DONE_INT_MASK))
        ;
    FSP_ClearStatusFlags(base, FSP_INT_SE_DONE_INT_MASK);
}

static inline void FSP_WaitCorrOpDone(FSP_Type *base)
{
    if (FSP_CheckWorkMode(base, FSP_INTEN_COR_DONE_INTEN_MASK))
    {
        while (!(FSP_GetStatusFlags(base) & FSP_INT_COR_DONE_INT_MASK))
            ;
        FSP_ClearStatusFlags(base, FSP_INT_COR_DONE_INT_MASK);
    }
}

void FSP_Init(FSP_Type *base)
{
    CLOCK_EnableClock(kCLOCK_Fsp);
}

void FSP_Deinit(FSP_Type *base)
{
    CLOCK_DisableClock(kCLOCK_Fsp);
}

void FSP_MatInit(fsp_matrix_instance_t *S, uint16_t n_rows, uint16_t n_columns, void *p_data)
{
    /* Assign Number of Rows */
    S->num_rows = n_rows;

    /* Assign Number of Columns */
    S->num_cols = n_columns;

    /* Assign Data pointer */
    S->p_data = p_data;
}

void FSP_MatInverseF32(FSP_Type *base, const fsp_matrix_instance_t *p_src, const fsp_matrix_instance_t *p_dst)
{
    fsp_mat_op_instance_t S;
    S.mat_op_cfg = MAT_OP_CONFIG(0, 0, 0, p_src->num_cols, p_src->num_rows, kFSP_MouDoutFpSelFloat,
                                 kFSP_MouDinFpSelFloat, kFSP_MouOpModeInv);
    S.scale_a_f32 = 1.0;
    S.scale_b_f32 = 1.0;

    FSP_MatOperateStart(base, &S, p_src->p_data, NULL, p_dst->p_data);

    FSP_WaitMatOpDone(base);
}

void FSP_MatInverseQ31(FSP_Type *base, const fsp_matrix_instance_t *p_src, const fsp_matrix_instance_t *p_dst)
{
    fsp_mat_op_instance_t S;
    S.mat_op_cfg = MAT_OP_CONFIG(0, 0, 0, p_src->num_cols, p_src->num_rows, kFSP_MouDoutFpSelFix, kFSP_MouDinFpSelFix,
                                 kFSP_MouOpModeInv);
    S.scale_a_f32 = 1.0;
    S.scale_b_f32 = 1.0;

    FSP_MatOperateStart(base, &S, p_src->p_data, NULL, p_dst->p_data);

    FSP_WaitMatOpDone(base);
}

void FSP_MatMultF32(FSP_Type *base,
                    const fsp_matrix_instance_t *p_src_a,
                    const fsp_matrix_instance_t *p_src_b,
                    const fsp_matrix_instance_t *p_dst)
{
    fsp_mat_op_instance_t S;
    S.mat_op_cfg = MAT_OP_CONFIG(0, 0, p_src_b->num_cols, p_src_a->num_cols, p_src_a->num_rows, kFSP_MouDoutFpSelFloat,
                                 kFSP_MouDinFpSelFloat, kFSP_MouOpModeMult);
    S.scale_a_f32 = 1.0;
    S.scale_b_f32 = 1.0;

    FSP_MatOperateStart(base, &S, p_src_a->p_data, p_src_b->p_data, p_dst->p_data);

    FSP_WaitMatOpDone(base);
}

void FSP_MatMultQ31(FSP_Type *base,
                    const fsp_matrix_instance_t *p_src_a,
                    const fsp_matrix_instance_t *p_src_b,
                    const fsp_matrix_instance_t *p_dst)
{
    fsp_mat_op_instance_t S;
    S.mat_op_cfg = MAT_OP_CONFIG(0, 0, p_src_b->num_cols, p_src_a->num_cols, p_src_a->num_rows, kFSP_MouDoutFpSelFix,
                                 kFSP_MouDinFpSelFix, kFSP_MouOpModeMult);
    S.scale_a_f32 = 1.0;
    S.scale_b_f32 = 1.0;

    FSP_MatOperateStart(base, &S, p_src_a->p_data, p_src_b->p_data, p_dst->p_data);

    FSP_WaitMatOpDone(base);
}

void FSP_MatDotMultF32(FSP_Type *base,
                       const fsp_matrix_instance_t *p_src_a,
                       const fsp_matrix_instance_t *p_src_b,
                       const fsp_matrix_instance_t *p_dst)

{
    fsp_mat_op_instance_t S;
    S.mat_op_cfg = MAT_OP_CONFIG(0, 0, p_src_b->num_cols, p_src_a->num_cols, p_src_a->num_rows, kFSP_MouDoutFpSelFloat,
                                 kFSP_MouDinFpSelFloat, kFSP_MouOpModeDotMult);
    S.scale_a_f32 = 1.0;
    S.scale_b_f32 = 1.0;

    FSP_MatOperateStart(base, &S, p_src_a->p_data, p_src_b->p_data, p_dst->p_data);

    FSP_WaitMatOpDone(base);
}

void FSP_MatDotMultQ31(FSP_Type *base,
                       const fsp_matrix_instance_t *p_src_a,
                       const fsp_matrix_instance_t *p_src_b,
                       const fsp_matrix_instance_t *p_dst)
{
    fsp_mat_op_instance_t S;
    S.mat_op_cfg = MAT_OP_CONFIG(0, 0, p_src_b->num_cols, p_src_a->num_cols, p_src_a->num_rows, kFSP_MouDoutFpSelFix,
                                 kFSP_MouDinFpSelFix, kFSP_MouOpModeDotMult);
    S.scale_a_f32 = 1.0;
    S.scale_b_f32 = 1.0;

    FSP_MatOperateStart(base, &S, p_src_a->p_data, p_src_b->p_data, p_dst->p_data);

    FSP_WaitMatOpDone(base);
}

void FSP_MatTransF32(FSP_Type *base, const fsp_matrix_instance_t *p_src, const fsp_matrix_instance_t *p_dst)

{
    fsp_mat_op_instance_t S;
    S.mat_op_cfg = MAT_OP_CONFIG(0, 0, 0, p_src->num_cols, p_src->num_rows, kFSP_MouDoutFpSelFloat,
                                 kFSP_MouDinFpSelFloat, kFSP_MouOpModeTrans);
    S.scale_a_f32 = 1.0;
    S.scale_b_f32 = 1.0;

    FSP_MatOperateStart(base, &S, p_src->p_data, NULL, p_dst->p_data);

    FSP_WaitMatOpDone(base);
}

void FSP_MatTransQ31(FSP_Type *base, const fsp_matrix_instance_t *p_src, const fsp_matrix_instance_t *p_dst)
{
    fsp_mat_op_instance_t S;
    S.mat_op_cfg = MAT_OP_CONFIG(0, 0, 0, p_src->num_cols, p_src->num_rows, kFSP_MouDoutFpSelFix, kFSP_MouDinFpSelFix,
                                 kFSP_MouOpModeTrans);
    S.scale_a_f32 = 1.0;
    S.scale_b_f32 = 1.0;

    FSP_MatOperateStart(base, &S, p_src->p_data, NULL, p_dst->p_data);

    FSP_WaitMatOpDone(base);
}

void FSP_MatScaleF32(FSP_Type *base,
                     const fsp_matrix_instance_t *p_src_a,
                     float32_t scale_a,
                     const fsp_matrix_instance_t *p_src_b,
                     float32_t scale_b,
                     const fsp_matrix_instance_t *p_dst)

{
    fsp_mat_op_instance_t S;
    S.mat_op_cfg = MAT_OP_CONFIG(0, 0, 0, p_src_a->num_cols, p_src_a->num_rows, kFSP_MouDoutFpSelFloat,
                                 kFSP_MouDinFpSelFloat, kFSP_MouOpModeScale);
    S.scale_a_f32 = scale_a;
    S.scale_b_f32 = scale_b;

    FSP_MatOperateStart(base, &S, p_src_a->p_data, p_src_b->p_data, p_dst->p_data);

    FSP_WaitMatOpDone(base);
}

void FSP_MatScaleQ31(FSP_Type *base,
                     const fsp_matrix_instance_t *p_src_a,
                     float32_t scale_a,
                     const fsp_matrix_instance_t *p_src_b,
                     float32_t scale_b,
                     const fsp_matrix_instance_t *p_dst)

{
    fsp_mat_op_instance_t S;
    S.mat_op_cfg = MAT_OP_CONFIG(0, 0, 0, p_src_a->num_cols, p_src_a->num_rows, kFSP_MouDoutFpSelFix,
                                 kFSP_MouDinFpSelFix, kFSP_MouOpModeScale);
    S.scale_a_f32 = scale_a;
    S.scale_b_f32 = scale_b;

    FSP_MatOperateStart(base, &S, p_src_a->p_data, p_src_b->p_data, p_dst->p_data);

    FSP_WaitMatOpDone(base);
}

void FSP_MatAddF32(FSP_Type *base,
                   const fsp_matrix_instance_t *p_src_a,
                   const fsp_matrix_instance_t *p_src_b,
                   const fsp_matrix_instance_t *p_dst)

{
    fsp_mat_op_instance_t S;
    S.mat_op_cfg = MAT_OP_CONFIG(0, 0, 0, p_src_a->num_cols, p_src_a->num_rows, kFSP_MouDoutFpSelFloat,
                                 kFSP_MouDinFpSelFloat, kFSP_MouOpModeScale);
    S.scale_a_f32 = 1.0;
    S.scale_b_f32 = 1.0;

    FSP_MatOperateStart(base, &S, p_src_a->p_data, p_src_b->p_data, p_dst->p_data);

    FSP_WaitMatOpDone(base);
}

void FSP_MatAddQ31(FSP_Type *base,
                   const fsp_matrix_instance_t *p_src_a,
                   const fsp_matrix_instance_t *p_src_b,
                   const fsp_matrix_instance_t *p_dst)

{
    fsp_mat_op_instance_t S;
    S.mat_op_cfg = MAT_OP_CONFIG(0, 0, 0, p_src_a->num_cols, p_src_a->num_rows, kFSP_MouDoutFpSelFix,
                                 kFSP_MouDinFpSelFix, kFSP_MouOpModeScale);
    S.scale_a_f32 = 1.0;
    S.scale_b_f32 = 1.0;

    FSP_MatOperateStart(base, &S, p_src_a->p_data, p_src_b->p_data, p_dst->p_data);

    FSP_WaitMatOpDone(base);
}

void FSP_MatSubF32(FSP_Type *base,
                   const fsp_matrix_instance_t *p_src_a,
                   const fsp_matrix_instance_t *p_src_b,
                   const fsp_matrix_instance_t *p_dst)
{
    fsp_mat_op_instance_t S;
    S.mat_op_cfg = MAT_OP_CONFIG(0, 0, 0, p_src_a->num_cols, p_src_a->num_rows, kFSP_MouDoutFpSelFloat,
                                 kFSP_MouDinFpSelFloat, kFSP_MouOpModeScale);
    S.scale_a_f32 = 1.0;
    S.scale_b_f32 = -1.0;

    FSP_MatOperateStart(base, &S, p_src_a->p_data, p_src_b->p_data, p_dst->p_data);

    FSP_WaitMatOpDone(base);
}

void FSP_MatSubQ31(FSP_Type *base,
                   const fsp_matrix_instance_t *p_src_a,
                   const fsp_matrix_instance_t *p_src_b,
                   const fsp_matrix_instance_t *p_dst)

{
    fsp_mat_op_instance_t S;
    S.mat_op_cfg = MAT_OP_CONFIG(0, 0, 0, p_src_a->num_cols, p_src_a->num_rows, kFSP_MouDoutFpSelFix,
                                 kFSP_MouDinFpSelFix, kFSP_MouOpModeScale);
    S.scale_a_f32 = 1.0;
    S.scale_b_f32 = -1.0;

    FSP_MatOperateStart(base, &S, p_src_a->p_data, p_src_b->p_data, p_dst->p_data);

    FSP_WaitMatOpDone(base);
}
void FSP_TeIDCTPreProcess(FSP_Type *base, float32_t *p_data)
{
    (*p_data) *= FSP_SqrtF32(base, (float32_t)1 / 2);
}
void FSP_TePostProcess(
    FSP_Type *base, const fsp_te_instance_t *S, fsp_te_io_mode_t mode, fsp_te_mode_t te_mode, float32_t *p_data)
{
    uint32_t i = 0;
    uint32_t num = 1;
    float32_t post_mult1 = 1.0f;
    float32_t post_mult2 = 1.0f;
    float32_t *p = p_data;

    if (S->te_point == kFSP_TePts64Points)
    {
        num = 64;
    }
    else if (S->te_point == kFSP_TePts128Points)
    {
        num = 128;
    }
    else if (S->te_point == kFSP_TePts256Points)
    {
        num = 256;
    }
    switch (te_mode)
    {
        case kFSP_TeModeFft:
            post_mult1 = 256.0f;
            break;
        case kFSP_TeModeIfft:
            post_mult1 = 256.0f / num;
            break;
        case kFSP_TeModeDct:
            post_mult2 = FSP_SqrtF32(base, (float32_t)1 / num) * 256.0f;
            post_mult1 = FSP_SqrtF32(base, (float32_t)2 / num) * 256.0f;
            break;
        case kFSP_TeModeIdct:
            post_mult1 = FSP_SqrtF32(base, (float32_t)2 / num) * 256.0f;
            break;
        default:
            break;
    }
    if ((mode == kFSP_TeIoModeRealInputComplexOutput) || (mode == kFSP_TeIoModeComplexInputComplexOutput))
    {
        num *= 2;
    }
    for (i = 0; i < num; i++)
    {
        if ((te_mode == kFSP_TeModeDct) && (i == 0))
        {
            (*p++) *= post_mult2;
        }
        else
        {
            (*p++) *= post_mult1;
        }
    }
    if ((te_mode == kFSP_TeModeFft) && (mode == kFSP_TeIoModeRealInputComplexOutput))
    {
        *(p_data + 1) = *(p_data + num / 2);
    }
}
void FSP_RfftF32(FSP_Type *base, const fsp_te_instance_t *S, float32_t *p_src, float32_t *p_dst)

{
    FSP_TeStart(base, TE_CONFIG(S->te_point, S->te_scale, kFSP_TeDoutFpSelFloat, kFSP_TeDinFpSelFloat,
                                kFSP_TeIoModeRealInputComplexOutput, kFSP_TeModeFft),
                p_src, p_dst);

    FSP_WaitTeOpDone(base);
}

void FSP_RifftF32(FSP_Type *base, const fsp_te_instance_t *S, float32_t *p_src, float32_t *p_dst)

{
    FSP_TeStart(base, TE_CONFIG(S->te_point, S->te_scale, kFSP_TeDoutFpSelFloat, kFSP_TeDinFpSelFloat,
                                kFSP_TeIoModeComplexInputRealOutput, kFSP_TeModeIfft),
                p_src, p_dst);

    FSP_WaitTeOpDone(base);
}

void FSP_RfftQ31(FSP_Type *base, const fsp_te_instance_t *S, q31_t *p_src, q31_t *p_dst)

{
    FSP_TeStart(base, TE_CONFIG(S->te_point, S->te_scale, kFSP_TeDoutFpSelFix, kFSP_TeDinFpSelFix,
                                kFSP_TeIoModeRealInputComplexOutput, kFSP_TeModeFft),
                p_src, p_dst);

    FSP_WaitTeOpDone(base);
}

void FSP_RifftQ31(FSP_Type *base, const fsp_te_instance_t *S, q31_t *p_src, q31_t *p_dst)

{
    FSP_TeStart(base, TE_CONFIG(S->te_point, S->te_scale, kFSP_TeDoutFpSelFix, kFSP_TeDinFpSelFix,
                                kFSP_TeIoModeComplexInputRealOutput, kFSP_TeModeIfft),
                p_src, p_dst);

    FSP_WaitTeOpDone(base);
}

void FSP_CfftF32(FSP_Type *base, const fsp_te_instance_t *S, float32_t *p_src, float32_t *p_dst, uint8_t ifft_flag)

{
    FSP_TeStart(base, TE_CONFIG(S->te_point, S->te_scale, kFSP_TeDoutFpSelFloat, kFSP_TeDinFpSelFloat,
                                kFSP_TeIoModeComplexInputComplexOutput, kFSP_TeModeFft | ifft_flag),
                p_src, p_dst);

    FSP_WaitTeOpDone(base);
}

void FSP_CfftQ31(FSP_Type *base, const fsp_te_instance_t *S, q31_t *p_src, q31_t *p_dst, uint8_t ifft_flag)

{
    FSP_TeStart(base, TE_CONFIG(S->te_point, S->te_scale, kFSP_TeDoutFpSelFix, kFSP_TeDinFpSelFix,
                                kFSP_TeIoModeComplexInputComplexOutput, kFSP_TeModeFft | ifft_flag),
                p_src, p_dst);

    FSP_WaitTeOpDone(base);
}

void FSP_DctF32(FSP_Type *base, const fsp_te_instance_t *S, float32_t *p_src, float32_t *p_dst, uint8_t idct_flag)

{
    FSP_TeStart(base, TE_CONFIG(S->te_point, S->te_scale, kFSP_TeDoutFpSelFloat, kFSP_TeDinFpSelFloat,
                                kFSP_TeIoModeRealInputRealOutput, kFSP_TeModeDct | idct_flag),
                p_src, p_dst);

    FSP_WaitTeOpDone(base);
}

void FSP_DctQ31(FSP_Type *base, const fsp_te_instance_t *S, q31_t *p_src, q31_t *p_dst, uint8_t idct_flag)

{
    FSP_TeStart(base, TE_CONFIG(S->te_point, S->te_scale, kFSP_TeDoutFpSelFix, kFSP_TeDinFpSelFix,
                                kFSP_TeIoModeRealInputRealOutput, kFSP_TeModeDct | idct_flag),
                p_src, p_dst);

    FSP_WaitTeOpDone(base);
}

void FSP_MaxMinF32(FSP_Type *base, float32_t *p_src, uint32_t block_size, float32_t *p_max, float32_t *p_min)
{
    uint32_t p_index = 0;

    FSP_SeStart(base, SE_CONFIG(block_size, kFSP_SeDoutFpSelFloat, kFSP_SeDinFpSelFloat, 0, 0, 1, 1, 0, 0), p_src);

    FSP_WaitSeOpDone(base);
    p_index = base->SE_IDX >> FSP_SE_IDX_SE_MAX_IDX_SHIFT;
    *p_max = p_src[p_index];
    p_index = base->SE_IDX & FSP_SE_IDX_SE_MIN_IDX_MASK;
    *p_min = p_src[p_index];
}

void FSP_GetMaxMinIntResultF32(FSP_Type *base, float32_t *p_src, float32_t *p_max, float32_t *p_min)
{
    uint32_t p_index = 0;
    p_index = base->SE_IDX >> FSP_SE_IDX_SE_MAX_IDX_SHIFT;
    *p_max = p_src[p_index];
    p_index = base->SE_IDX & FSP_SE_IDX_SE_MIN_IDX_MASK;
    *p_min = p_src[p_index];
}

void FSP_MaxMinQ31(FSP_Type *base, q31_t *p_src, uint32_t block_size, q31_t *p_max, q31_t *p_min)
{
    uint32_t p_index = 0;
    FSP_SeStart(base, SE_CONFIG(block_size, kFSP_SeDoutFpSelFix, kFSP_SeDinFpSelFix, 0, 0, 1, 1, 0, 0), p_src);

    FSP_WaitSeOpDone(base);
    p_index = base->SE_IDX >> FSP_SE_IDX_SE_MAX_IDX_SHIFT;
    *p_max = p_src[p_index];
    p_index = base->SE_IDX & FSP_SE_IDX_SE_MIN_IDX_MASK;
    *p_min = p_src[p_index];
}

void FSP_GetMaxMinIntResultQ31(FSP_Type *base, q31_t *p_src, q31_t *p_max, q31_t *p_min)
{
    uint32_t p_index = 0;
    p_index = base->SE_IDX >> FSP_SE_IDX_SE_MAX_IDX_SHIFT;
    *p_max = p_src[p_index];
    p_index = base->SE_IDX & FSP_SE_IDX_SE_MIN_IDX_MASK;
    *p_min = p_src[p_index];
}

void FSP_MaxF32(FSP_Type *base, float32_t *p_src, uint32_t block_size, float32_t *p_result, uint32_t *p_index)

{
    FSP_SeStart(base, SE_CONFIG(block_size, kFSP_SeDoutFpSelFloat, kFSP_SeDinFpSelFloat, 0, 0, 1, 0, 0, 0), p_src);

    FSP_WaitSeOpDone(base);
    *p_index = base->SE_IDX >> FSP_SE_IDX_SE_MAX_IDX_SHIFT;
    *p_result = p_src[*p_index];
}

void FSP_GetMaxIntResultF32(FSP_Type *base, float32_t *p_src, float32_t *p_result, uint32_t *p_index)
{
    *p_index = base->SE_IDX >> FSP_SE_IDX_SE_MAX_IDX_SHIFT;
    *p_result = p_src[*p_index];
}

void FSP_MaxQ31(FSP_Type *base, q31_t *p_src, uint32_t block_size, q31_t *p_result, uint32_t *p_index)
{
    FSP_SeStart(base, SE_CONFIG(block_size, kFSP_SeDoutFpSelFix, kFSP_SeDinFpSelFix, 0, 0, 1, 0, 0, 0), p_src);

    FSP_WaitSeOpDone(base);
    *p_index = base->SE_IDX >> FSP_SE_IDX_SE_MAX_IDX_SHIFT;
    *p_result = p_src[*p_index];
}

void FSP_GetMaxIntResultQ31(FSP_Type *base, q31_t *p_src, q31_t *p_result, uint32_t *p_index)
{
    *p_index = base->SE_IDX >> FSP_SE_IDX_SE_MAX_IDX_SHIFT;
    *p_result = p_src[*p_index];
}

void FSP_MinF32(FSP_Type *base, float32_t *p_src, uint32_t block_size, float32_t *p_result, uint32_t *p_index)
{
    FSP_SeStart(base, SE_CONFIG(block_size, kFSP_SeDoutFpSelFloat, kFSP_SeDinFpSelFloat, 0, 0, 0, 1, 0, 0), p_src);

    FSP_WaitSeOpDone(base);
    *p_index = base->SE_IDX & FSP_SE_IDX_SE_MIN_IDX_MASK;
    *p_result = p_src[*p_index];
}

void FSP_GetMinIntResultF32(FSP_Type *base, float32_t *p_src, float32_t *p_result, uint32_t *p_index)
{
    *p_index = base->SE_IDX & FSP_SE_IDX_SE_MIN_IDX_MASK;
    *p_result = p_src[*p_index];
}

void FSP_MinQ31(FSP_Type *base, q31_t *p_src, uint32_t block_size, q31_t *p_result, uint32_t *p_index)
{
    FSP_SeStart(base, SE_CONFIG(block_size, kFSP_SeDoutFpSelFix, kFSP_SeDinFpSelFix, 0, 0, 0, 1, 0, 0), p_src);

    FSP_WaitSeOpDone(base);
    *p_index = base->SE_IDX & FSP_SE_IDX_SE_MIN_IDX_MASK;
    *p_result = p_src[*p_index];
}

void FSP_GetMinIntResultQ31(FSP_Type *base, q31_t *p_src, q31_t *p_result, uint32_t *p_index)
{
    *p_index = base->SE_IDX & FSP_SE_IDX_SE_MIN_IDX_MASK;
    *p_result = p_src[*p_index];
}

void FSP_SumF32(FSP_Type *base, float32_t *p_src, uint32_t block_size, float32_t *p_result)
{
    FSP_SeStart(base, SE_CONFIG(block_size, kFSP_SeDoutFpSelFloat, kFSP_SeDinFpSelFloat, 0, 1, 0, 0, 0, 0), p_src);

    FSP_WaitSeOpDone(base);
    *(uint32_t *)p_result = base->SE_SUM;
}

void FSP_SumQ31(FSP_Type *base, q31_t *p_src, uint32_t block_size, q31_t *p_result)
{
    FSP_SeStart(base, SE_CONFIG(block_size, kFSP_SeDoutFpSelFix, kFSP_SeDinFpSelFix, 0, 1, 0, 0, 0, 0), p_src);

    FSP_WaitSeOpDone(base);
    *p_result = base->SE_SUM;
}

void FSP_PowerF32(FSP_Type *base, float32_t *p_src, uint32_t block_size, float32_t *p_result)
{
    FSP_SeStart(base, SE_CONFIG(block_size, kFSP_SeDoutFpSelFloat, kFSP_SeDinFpSelFloat, 1, 0, 0, 0, 0, 0), p_src);

    FSP_WaitSeOpDone(base);
    *(uint32_t *)p_result = base->SE_PWR;
}

void FSP_PowerQ31(FSP_Type *base, q31_t *p_src, uint32_t block_size, q31_t *p_result)
{
    FSP_SeStart(base, SE_CONFIG(block_size, kFSP_SeDoutFpSelFix, kFSP_SeDinFpSelFix, 1, 0, 0, 0, 0, 0), p_src);

    FSP_WaitSeOpDone(base);
    *p_result = base->SE_PWR;
}

void FSP_CorrelateF32(
    FSP_Type *base, float32_t *p_src_a, uint32_t srcALen, float32_t *p_src_b, uint32_t srcBLen, float32_t *p_dst)
{
    FSP_CorrelationStart(base, CORR_CONFIG(srcBLen, srcALen, kFSP_CorDoutFpSelFloat, kFSP_CorDinFpSelFloat), 0, 0,
                         p_src_a, p_src_b, p_dst);

    FSP_WaitCorrOpDone(base);
}

void FSP_CorrelateQ31(FSP_Type *base, q31_t *p_src_a, uint32_t srcALen, q31_t *p_src_b, uint32_t srcBLen, q31_t *p_dst)
{
    FSP_CorrelationStart(base, CORR_CONFIG(srcBLen, srcALen, kFSP_CorDoutFpSelFix, kFSP_CorDinFpSelFix), 0, 0, p_src_a,
                         p_src_b, p_dst);

    FSP_WaitCorrOpDone(base);
}

void FSP_FirF32(FSP_Type *base, const fsp_fir_instance_t *S, float32_t *p_src, float32_t *p_dst, uint32_t block_size)
{
    uint32_t addr = (uint32_t)base + (S->ch_idx << 2);
    *((volatile uint32_t *)(addr + kFSP_FirCfgCh0Offset)) = S->fir_cfg;
    while (base->STATUS & FSP_STATUS_FPU0_BUSY_MASK)
        ;

    addr += kFSP_FirDat0FlOffset;

    *((volatile uint32_t *)(addr)) = *(uint32_t *)p_src++;
    for (int i = 0; i < block_size - 1; i++)
    {
        *((volatile uint32_t *)(addr)) = *(uint32_t *)p_src++;
        while (!(FSP_GetBusyStatusFlags(base) & FSP_STATUS_FIR_READY_MASK))
            ;
        *(uint32_t *)p_dst++ = (*((volatile uint32_t *)(addr)));
    }
    while (!(FSP_GetBusyStatusFlags(base) & FSP_STATUS_FIR_READY_MASK))
        ;
    *(uint32_t *)p_dst++ = (*((volatile uint32_t *)(addr)));
}

void FSP_FirQ31(FSP_Type *base, const fsp_fir_instance_t *S, q31_t *p_src, q31_t *p_dst, uint32_t block_size)
{
    uint32_t addr = (uint32_t)base + (S->ch_idx << 2);
    *((volatile uint32_t *)(addr + kFSP_FirCfgCh0Offset)) = S->fir_cfg;
    while (FSP_GetBusyStatusFlags(base) & FSP_STATUS_FPU0_BUSY_MASK)
        ;

    addr += kFSP_FirDat0FxOffset;
    *((volatile uint32_t *)(addr)) = *(uint32_t *)p_src++;
    for (int i = 0; i < block_size - 1; i++)
    {
        *((volatile uint32_t *)(addr)) = *(uint32_t *)p_src++;
        while (!(FSP_GetBusyStatusFlags(base) & FSP_STATUS_FIR_READY_MASK))
            ;
        *p_dst++ = (*((volatile uint32_t *)(addr)));
    }
    while (!(FSP_GetBusyStatusFlags(base) & FSP_STATUS_FIR_READY_MASK))
        ;
    *p_dst++ = (*((volatile uint32_t *)(addr)));
}
