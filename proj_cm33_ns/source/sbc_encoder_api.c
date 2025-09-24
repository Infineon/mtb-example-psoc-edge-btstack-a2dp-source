/******************************************************************************
 * File Name:   sbc_encoder_api.c
 *
 * Description: Source file for SBC encoder.
 *
 *******************************************************************************
 * Copyright 2024-2025, Cypress Semiconductor Corporation (an Infineon company) or
 * an affiliate of Cypress Semiconductor Corporation.  All rights reserved.
 *
 * This software, including source code, documentation and related
 * materials ("Software") is owned by Cypress Semiconductor Corporation
 * or one of its affiliates ("Cypress") and is protected by and subject to
 * worldwide patent protection (United States and foreign),
 * United States copyright laws and international treaty provisions.
 * Therefore, you may use this Software only as provided in the license
 * agreement accompanying the software package from which you
 * obtained this Software ("EULA").
 * If no EULA applies, Cypress hereby grants you a personal, non-exclusive,
 * non-transferable license to copy, modify, and compile the Software
 * source code solely for use in connection with Cypress's
 * integrated circuit products.  Any reproduction, modification, translation,
 * compilation, or representation of this Software except as specified
 * above is prohibited without the express written permission of Cypress.
 *
 * Disclaimer: THIS SOFTWARE IS PROVIDED AS-IS, WITH NO WARRANTY OF ANY KIND,
 * EXPRESS OR IMPLIED, INCLUDING, BUT NOT LIMITED TO, NONINFRINGEMENT, IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE. Cypress
 * reserves the right to make changes to the Software without notice. Cypress
 * does not assume any liability arising out of the application or use of the
 * Software or any product or circuit described in the Software. Cypress does
 * not authorize its products for use in any products where a malfunction or
 * failure of the Cypress product may reasonably be expected to result in
 * significant property damage, injury or death ("High Risk Product"). By
 * including Cypress's product in a High Risk Product, the manufacturer
 * of such system or application assumes all risk of such use and in doing
 * so agrees to indemnify Cypress against all liability.
 *******************************************************************************/
#include "wiced_bt_trace.h"
#include "sbc_encoder_api.h"
/******************************************************************************
 * Function definitions
 ******************************************************************************/

/******************************************************************************
 * Function Name: sbc_encoder_app_init()
 *******************************************************************************
 * Summary:
 *          Initialize SBC encoder MW.
 *
 * Parameters:
 *          enc_handle : Encoder Handle
 *          sbc_encoder_params : SBC encoder parameters
 *
 *
 * Return:
 *         Result of initialization.
 *
 ******************************************************************************/

int sbc_encoder_app_init(cy_audio_sw_codec_t *enc_handle, sbc_encoder_params_t *sbc_encoder_params)
{

    cy_rslt_t                           result;
    cy_audio_sw_codec_encode_config_t   enc_config;
    cy_asc_sbc_encode_config_t          sbc_config;

    enc_config.bitwidth         = cy_audio_sw_codec_get_bitwidth_enum(sbc_encoder_params->bit_width);
    enc_config.channel          = cy_audio_sw_codec_get_channel_enum(sbc_encoder_params->channels);
    enc_config.sampling_rate    = cy_audio_sw_codec_get_sampling_rate_enum(sbc_encoder_params->sample_rate);
    enc_config.bitrate          = CY_AUDIO_CODEC_BITRATE_UNKNOWN;
    enc_config.frame_ms         = CY_AUDIO_CODEC_FRAME_MS_UNKNOWN;
    enc_config.codec_type       = CY_AUDIO_CODEC_TYPE_SBC;


    sbc_config.allocation_method = sbc_encoder_params->sbc_config.allocation_method;
    sbc_config.sbc_mode          = sbc_encoder_params->sbc_config.sbc_mode;
    sbc_config.sub_blocks        = sbc_encoder_params->sbc_config.sub_blocks;
    sbc_config.sub_bands         = sbc_encoder_params->sbc_config.sub_bands;
    sbc_config.bit_pool          = sbc_encoder_params->sbc_config.bit_pool;
    sbc_config.channel_mode      = sbc_encoder_params->sbc_config.channel_mode;
    sbc_config.uui_id            = SBC_WB_SUPPORTED_UUID;

    WICED_BT_TRACE("***SBC config parameters*** \r\n");
    WICED_BT_TRACE("Allocation method %d \r\n",sbc_config.allocation_method);
    WICED_BT_TRACE("SBC mode %d \r\n",sbc_config.sbc_mode);
    WICED_BT_TRACE("Sub blocks %d \r\n",sbc_config.sub_blocks);
    WICED_BT_TRACE("Sub bands %d \r\n",sbc_config.sub_bands);
    WICED_BT_TRACE("Bit Pool %d \r\n",sbc_config.bit_pool);
    WICED_BT_TRACE("Channel Mode %d \r\n",sbc_config.channel_mode);

    enc_config.codec_specific_config.sbc = sbc_config;

    result = cy_audio_sw_codec_init(CY_AUDIO_CODEC_OPERATION_ENCODE, (void*)&enc_config, enc_handle);

    if (result != CY_RSLT_SUCCESS)
    {
        cy_asc_log_err(result, "sw encoder init failed");
    } else
    {
        cy_asc_log_dbg("sw encoder init success");
    }

    return result;
}

/******************************************************************************
 * Function Name: sbc_encoder_app_encode()
 *******************************************************************************
 * Summary: Encodes a buffer of PCM data to SBC.
 *
 *
 * Parameters:
 *          enc_handle : Encoder Handle
 *          input_buff : Buffers for input.
 *          in_size    : Input buffer size
 *          output_buff: Buffers for output
 *          out_size   : Output buffer size
 *
 * Return:
 *         Result of encoding.
 *
 ******************************************************************************/
int sbc_encoder_app_encode(cy_audio_sw_codec_t enc_handle, uint8_t* input_buff, uint32_t* in_size, uint8_t * output_buff, uint32_t* out_size)
{
    cy_rslt_t   result;

    result = cy_audio_sw_codec_encode(enc_handle, (uint8_t *)input_buff, (uint32_t*)in_size, (uint8_t*)output_buff, (uint32_t*)out_size);

    if(CY_RSLT_SUCCESS != result)
    {
        cy_asc_log_err(result, "Encoding failed");
    }

    return result;
}
/* [] END OF FILE */