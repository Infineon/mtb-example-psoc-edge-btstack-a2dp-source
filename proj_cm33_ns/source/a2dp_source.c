/******************************************************************************
* File Name:   a2dp_source.c
*
* Description: Source file for A2DP source task.
*
 ******************************************************************************
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
 ******************************************************************************/

/******************************************************************************
 * Includes
 *****************************************************************************/

#ifdef  WICED_BT_TRACE_ENABLE
#include "wiced_bt_trace.h"
#endif
#include "a2dp_source.h"
#include "cybsp.h"
#include "wiced_bt_avrc_tg.h"
#include "app_bt_bonding.h"
#include <stdio.h>
#include "wav_1_test.h"
#include "board.h"
/******************************************************************************
 * Macros
 *****************************************************************************/

#define A2DP_SOURCE_NVRAM_ID                (0U)
#define WICED_HS_EIR_BUF_MAX_SIZE           (264U)
#define MAX_KEY_SIZE                        (16U)
#define AUDIO_SAMPLE_RATE_48kHz             (48000U)
#define AUDIO_SAMPLE_RATE_44_1kHz           (44100U)
#define AUDIO_SAMPLE_RATE_32kHz             (32000U)
#define AUDIO_SAMPLE_RATE_16kHz             (16000U)
#define AUDIO_CHANNEL_MONO                  (1U)
#define AUDIO_CHANNEL_STEREO                (2U)
#define AUDIO_CHANNEL_DUAL_CHANNEL          (2U)
#define AUDIO_CHANNEL_JOINT_STEREO          (2U)
#define AUDIO_BIT_WIDTH                     (16U)
#define DEFAULT_SBC_BITPOOL                 (51U)
#define SBC_A2DP_MODE                       (0U)
#define SBC_WIDE_BAND_MODE                  (1U)
#define SEC1000_MICROSEC                    (1000000000U)
#define FRAME_TIME_STAMP                    (90000U)
#define MARKER_PAYLOAD_BYTE                 (0x60U)
#define INQUIRY_DURATION                    (5U)
#define MILLI_TO_MICRO                      (1000U)
#define US_TO_MS_CONVERSION                 (1000UL)
#define A2DP_SINK_DELAY_CAL_CONST           (108U)
#define DELAY_MULTIPLIER                    (100U)
/* 500 Miliseconds debounce delay */
#define DEBOUNCE_TICKS                      (pdMS_TO_TICKS(500))
/* ***************************************************************************
 * Static variables
 * **************************************************************************/

static uint16_t handle = 0x0000;
static uint32_t  SampleRate = AUDIO_SAMPLE_RATE_48kHz;
static uint64_t  usTimeout;
static uint32_t dwPCMBytesPerFrame;
static uint32_t Timestamp;
static uint32_t TimestampInc;
static uint8_t  stream_avdt_handle;
static uint32_t frame_per_packet;
static unsigned char *pBitstream = NULL;
static unsigned int asc_stream_read_max = 0;
static unsigned int asc_stream_read_counter = 0;
static wiced_bt_device_address_t a2dp_sink_bd_addr;
static uint8_t *a2dp_prefill_buffer;
/* ***************************************************************************
 * Global variables
 * **************************************************************************/

volatile bool button_status_flag = false;
volatile bool button_pressed1 = false;
volatile wiced_bool_t bStreamingActive = WICED_FALSE;
unsigned int    frame_size;
SemaphoreHandle_t stream_buff_sem;
uint16_t delay_reported_from_sink_micro_sec = 0;
tAV_APP_CB      av_app_cb;
wiced_bt_heap_t *p_default_heap = NULL;
uint8_t pincode[WICED_PIN_CODE_LEN] = { 0x30, 0x30, 0x30, 0x30 };
cy_audio_sw_codec_encode_config_t enc_config;
cy_audio_sw_codec_t enc_handle;
uint32_t recommended_in_size;
uint32_t min_out_size;
uint32_t n_channels = 0;
uint32_t i_channel_mask;
uint32_t sample_rate;
uint32_t pcm_sz;
int32_t  length;
wiced_bt_device_link_keys_t nv_key_ram;
int16_t wav_header=0;
uint8_t *a2dp_frame1;
uint8_t *a2dp_frame2;
uint8_t *a2dp_frame_tx_ptr;
uint8_t *a2dp_frame_enc_ptr;
uint8_t encoder_output[114] = {0};
uint8_t  status_flag = 0;
uint8_t  bondindex = 0;
uint8_t app_hello_sensor_notify_client_char_config[] = {0x00, 0x00};
bool initiate_bt_device_connection = false;
int8_t connection_status;
TaskHandle_t a2dp_task_handle;
/* Array to store connection information */
device_info_t device_info_list[MAX_DEVICES];
int device_count = 0;
uint32_t a2dp_packet_delay_ms = 0;
/******************************************************************************
 * Extern variables and functions
 *****************************************************************************/
extern const uint8_t                        a2dp_source_sdp_db[A2DP_SOURCE_SDP_DB_SIZE];
extern const wiced_bt_cfg_settings_t        a2dp_source_cfg_settings;
extern wiced_bt_a2dp_source_config_data_t   bt_audio_config;
/*******************************************************************************
* Function prototypes
*******************************************************************************/
void a2dp_source_audio_thread(void* arg);
static uint8_t a2dp_source_encode_sendSBC(void);
static void a2dp_source_parse_sbc_params(wiced_bt_a2d_sbc_cie_t * pSbc,
        uint32_t *pSf,uint32_t *pBlocks, uint32_t *pSubbands, uint32_t *pMode);
uint16_t a2dp_source_calculate_delay(void);
static void app_avrc_event_cback(uint8_t event_id, wiced_bt_rc_event_t *p_event);
static void rc_target_passthrough_cmd_event(
        wiced_bt_avrc_tg_passthrough_cmd_t *p_passthrough);

/******************************************************************************
 * Function definitions
 ******************************************************************************/

/* ****************************************************************************
 * Function Name: get_audio_freq_name
 *******************************************************************************
 * Summary:
 *          Gives the string equivalent for the Event type
 *
 * Parameters:
 *          freq - Sample Frequency type
 *
 * Return:
 *          Pointer to String containing the Frequency Name
 *
 * ***************************************************************************/
static const char *get_audio_freq_name(uint8_t freq)
{
    switch((int)freq)
    {
        CASE_RETURN_STR(A2D_SBC_IE_SAMP_FREQ_16)
        CASE_RETURN_STR(A2D_SBC_IE_SAMP_FREQ_32)
        CASE_RETURN_STR(A2D_SBC_IE_SAMP_FREQ_44)
        CASE_RETURN_STR(A2D_SBC_IE_SAMP_FREQ_48)
    }

    return NULL;
}

/* ****************************************************************************
 * Function Name: get_audio_chcfg_name
 *******************************************************************************
 * Summary:
 *          Gives the string equivalent for the Event type
 *
 * Parameters:
 *          chcfg - Channel Configuration type
 *
 * Return:
 *          Pointer to String containing the Channel Configuration Name
 *
 * ***************************************************************************/
static const char *get_audio_chcfg_name(uint8_t chcfg)
{
    switch((int)chcfg)
    {
        CASE_RETURN_STR(A2D_SBC_IE_CH_MD_MONO)
        CASE_RETURN_STR(A2D_SBC_IE_CH_MD_DUAL)
        CASE_RETURN_STR(A2D_SBC_IE_CH_MD_STEREO)
        CASE_RETURN_STR(A2D_SBC_IE_CH_MD_JOINT)
    }

    return NULL;
}

/* ****************************************************************************
 * Function Name: a2dp_source_deinit_streaming
 ******************************************************************************
 * Summary:
 *        De-initialize the buffers and variables used for Audio Streaming state
 *
 * Parameters:
 *          None
 *
 * Return:
 *          None
 *
 * ***************************************************************************/
void a2dp_source_deinit_streaming(void)
{
    WICED_BT_TRACE ("De-iniatializing the stream buffer \n");
    bStreamingActive = FALSE;

    if (NULL != a2dp_frame1)
    {
        free(a2dp_frame1);
        a2dp_frame1 = NULL;
    }

    if (NULL != a2dp_frame2)
    {
        free(a2dp_frame2);
        a2dp_frame2 = NULL;
    }

    if (NULL != a2dp_prefill_buffer)
    {
        free(a2dp_prefill_buffer);
    }

    a2dp_frame_tx_ptr  = NULL;
    a2dp_frame_enc_ptr  = NULL;
    a2dp_prefill_buffer = NULL;
}

/* ****************************************************************************
 * Function Name: a2dp_source_update_sink_rep_delay
 ******************************************************************************
 * Summary:
 *          Updates the delay report value to the latest as per the Sink
 *
 * Parameters:
 *          delay_ms: Delay in Milliseconds
 *
 * Return:
 *          None
 *
 * ***************************************************************************/
void a2dp_source_update_sink_rep_delay(uint16_t delay_ms)
{
    WICED_BT_TRACE("Updated Sink reported delay %u ms", delay_ms);
    delay_reported_from_sink_micro_sec += delay_ms * MILLI_TO_MICRO;
}

/******************************************************************************
 * Function Name: asc_reset_params()
 *******************************************************************************
 * Summary: Decodes a wav file and points to the data by skipping the wav header.
 *
 *
 * Parameters:
 *          None
 *
 *
 * Return:
 *          None
 *
 ******************************************************************************/


static void asc_reset_params(void)
{
#if !ENABLE_AM_FS
    pBitstream = (unsigned char *)&music_array_48kHz;
    asc_stream_read_max = music_array_48kHz_size;

    asc_stream_read_counter = 0;
#endif
    if( false != cy_wav_header_decode( &n_channels,
                                &i_channel_mask, &sample_rate,
                                &pcm_sz, &length,
                                pBitstream,&asc_stream_read_counter,asc_stream_read_max) )
    {
        cy_asc_log_err(CY_RSLT_ASC_GENERIC_ERROR, "Error wav header parsing failed");
    }

    cy_asc_log_err("Input Params:  sample_rate: %lu, pcm_sz: %lu, n_channels: %lu, length: %ld "
                            , sample_rate
                            , pcm_sz
                            , n_channels
                            , length );
}

/* ****************************************************************************
 * Function Name: a2dp_source_start_streaming
 ******************************************************************************
 * Summary:
 *          Loads the required .wav file to memory, initializes the Encoder,
 *          Starts a thread for encoding and streaming audio
 *
 * Parameters:
 *          bActivate: State of Streaming as requested by the user
 *          avdt_handle: AVDTP Connection handle as received in the event
 *                       WICED_BT_A2DP_SOURCE_START_IND_EVT
 *          p_stream_cfg: CODEC Configuration
 *          peer_mtu: Stream MTU of the Peer side
 * Return:
 *          None
 *
 * ***************************************************************************/
void a2dp_source_start_streaming (wiced_bool_t bActivate, uint8_t avdt_handle,
                                  wiced_bt_a2d_sbc_cie_t *p_stream_cfg, uint16_t peer_mtu)
{
    uint32_t  NumBlocks       = 0;
    uint32_t  NumSubBands     = 0;
    uint32_t  NumChannels     = 0;
    uint32_t  Mode            = 0;

    sbc_encoder_params_t sbc_params;

    unsigned int fr_per_1000sec, pkts_per_1000sec;
    int join;
    cy_rslt_t            result;

    /* Safety check in case we are called more than once to do the same thing */
    if (bActivate == bStreamingActive)
    {
        WICED_BT_TRACE ("Streaming State: Already in %s",
                      bActivate ? "ACTIVE" : "SUSPEND");
        return;
    }

    WICED_BT_TRACE ("Streaming State: setting to %s \n",
                  bActivate ? "ACTIVE" : "SUSPEND");

    if (!bActivate)
    {
        bStreamingActive = FALSE;
        return;
    }

    stream_avdt_handle = avdt_handle;

    WICED_BT_TRACE("\r\nPeer device MTU %u \nsamp_freq %x \nch_mode %d \nblock_len %d \nnum_subbands %d \nalloc_mthd %d \nmax_bitpool %d \nmin_bitpool %d \r\n",\
            peer_mtu, p_stream_cfg->samp_freq, p_stream_cfg->ch_mode, p_stream_cfg->block_len,
            p_stream_cfg->num_subbands, p_stream_cfg->alloc_mthd, p_stream_cfg->max_bitpool,
            p_stream_cfg->min_bitpool);

    NumBlocks       = p_stream_cfg->block_len;
    NumSubBands     = p_stream_cfg->num_subbands;
    Mode            = p_stream_cfg->ch_mode;

    /* Extract the codec parameters */
    a2dp_source_parse_sbc_params (p_stream_cfg, &SampleRate, &NumBlocks, &NumSubBands, &Mode);

    WICED_BT_TRACE ("SampleRate: %u  NumBlocks: %u  NumSubBands: %u  Mode: %u",
                    SampleRate, NumBlocks, NumSubBands, Mode);


    NumChannels = Mode >= SBC_STEREO ? SBC_STEREO : SBC_DUAL;
    join = Mode == SBC_JOINT_STEREO ? SBC_DUAL : SBC_MONO;

    sbc_params.sbc_config.allocation_method = (p_stream_cfg->alloc_mthd == A2D_SBC_IE_ALLOC_MD_S ? SBC_SNR : SBC_LOUDNESS);
    sbc_params.sbc_config.channel_mode      = Mode;
    sbc_params.sbc_config.sbc_mode          = SBC_A2DP_MODE;
    sbc_params.sbc_config.sub_blocks        = NumBlocks;
    sbc_params.sbc_config.sub_bands         = NumSubBands;
    sbc_params.sbc_config.bit_pool          = DEFAULT_SBC_BITPOOL;
    sbc_params.sample_rate                  = SampleRate;
    sbc_params.channels                     = AUDIO_CHANNEL_STEREO;
    sbc_params.bit_width                    = AUDIO_BIT_WIDTH;

    asc_reset_params();
    wav_header = asc_stream_read_counter;

    WICED_BT_TRACE("Wav header is %d \r\n",wav_header);

    result = sbc_encoder_app_init(&enc_handle, &sbc_params);
    if (CY_RSLT_SUCCESS != result)
    {
        cy_asc_log_err(result, "sw encoder init failed");
        return;
    }

    result = cy_audio_sw_codec_get_encoder_recommended_inbuf_size(enc_handle,&recommended_in_size);
    if(CY_RSLT_SUCCESS != result)
    {
        cy_asc_log_err(result, "failed to get recommended input buf sz.");
        return;
    }

    result = cy_audio_sw_codec_get_encoder_recommended_outbuf_size(enc_handle, recommended_in_size, &min_out_size);
    if(CY_RSLT_SUCCESS != result)
    {
        cy_asc_log_err(result, "failed to get recommended output buf sz.");
        return;
    }

    WICED_BT_TRACE("SBC encoder recommended_in_size: %lu, min_out_size:%lu \r\n",recommended_in_size, min_out_size);

    /*FORMULA as in A2DP Spec:
     *
     * frame_length = 4 + (4 * nrof_subbands * nrof_channels ) / 8
     * nrof_blocks * nrof_channels * bitpool / 8 .
     * for the MONO and DUAL_CHANNEL channel modes, and
     * frame_length = 4 + (4 * nrof_subbands * nrof_channels ) / 8
     * (join * nrof_subbands + nrof_blocks * bitpool ) / 8
     */

    /* Calculate the periodicity to compress and send data in microseconds */
    frame_size = 4 + (4 * NumSubBands * sbc_params.channels) / 8
    + (join * NumSubBands + NumBlocks * sbc_params.sbc_config.bit_pool) / 8;

    frame_per_packet = (peer_mtu - AVDT_MEDIA_OFFSET) / frame_size;

    if (frame_per_packet > A2D_SBC_HDR_NUM_MSK)
      frame_per_packet = A2D_SBC_HDR_NUM_MSK;

    if (frame_per_packet == 0)
      frame_per_packet = 1;

    fr_per_1000sec = (SampleRate * 1000) / (NumSubBands * NumBlocks);

    pkts_per_1000sec = fr_per_1000sec / frame_per_packet;
    usTimeout = SEC1000_MICROSEC / pkts_per_1000sec;

    usTimeout = usTimeout * DELAY_MULTIPLIER / A2DP_SINK_DELAY_CAL_CONST;

    a2dp_packet_delay_ms  = (uint32_t) usTimeout/US_TO_MS_CONVERSION;

    /*NumSubBands * NumBlocks * NumChannels* (bits_per_sample/2 )*/
    dwPCMBytesPerFrame = 2 * NumSubBands * NumBlocks * NumChannels;

    WICED_BT_TRACE ("Stream Starting:  pcm_bytes_per_frame %u \n", dwPCMBytesPerFrame);

    WICED_BT_TRACE("Frame size is %u Frame per packet %lu \r\n",frame_size,frame_per_packet);
    /* Allocate memory */

    /* Allocate memory */
    a2dp_frame1 = (uint8_t*) malloc(frame_size*frame_per_packet + 1);
    a2dp_frame2 = (uint8_t*) malloc(frame_size*frame_per_packet + 1);
    a2dp_prefill_buffer = (uint8_t*) malloc(A2DP_PREFILL_PACKET_COUNT * (frame_size * frame_per_packet + 1) + A2DP_PREFILL_PACKET_COUNT_BUFFER);
    a2dp_frame_tx_ptr = a2dp_frame1;
    a2dp_frame_enc_ptr = a2dp_frame2;

    Timestamp = 0;
    TimestampInc = ((NumBlocks * NumSubBands * FRAME_TIME_STAMP) / SampleRate) * frame_per_packet;
    WICED_BT_TRACE("Timestamp increment is %ld \r\n",TimestampInc);

    WICED_BT_TRACE("fr_per_1000sec %d \r\n",fr_per_1000sec);
    WICED_BT_TRACE("pkts_per_1000sec %d \r\n",pkts_per_1000sec);
    WICED_BT_TRACE("usTimeout %lld \r\n",usTimeout);

    WICED_BT_TRACE ("Stream Parameters: Sample rate %d  Blk: %d   SB: %d  BitP: %d  uS: %d  Frame Size: %d  Frame Per Packet: %d  fp1000s: %u \n",\
                    SampleRate, NumBlocks, NumSubBands, sbc_params.sbc_config.bit_pool, usTimeout, frame_size, frame_per_packet, fr_per_1000sec);

    /* Unblock the A2DP source thread */
    xTaskNotify(a2dp_task_handle, 0, eNoAction);

}

#if A2DP_ENABLE_PREFILL
/* ****************************************************************************
 * Function Name: a2dp_source_encode_sendSBC_prefill
 ******************************************************************************
 * Summary:
 *          Calls Encoder to encode the PCM data and sends to Stack
 *
 * Parameters:
 *          None
 * Return:
 *          A2DP_APP_SUCCESS if the data is sent successfully
 *          A2DP_APP_FAILED if there was an error
 *
 * ***************************************************************************/
static cy_rslt_t a2dp_source_encode_sendSBC_prefill(void)
{
    cy_rslt_t    result      = A2DP_APP_SUCCESS;
    uint8_t     *output_buff = a2dp_prefill_buffer;
    uint8_t     *output_buff1;

    /* Prefill the defined number of packets in buffer */
    for (uint8_t i = 0; i < A2DP_PREFILL_PACKET_COUNT; i++)
    {
        uint8_t     *pSBC;
        uint32_t    i;
        uint16_t    pkt_len     = 0;
        uint32_t    in_size     = 0;
        uint32_t    out_size    = 0;

        pkt_len = frame_size * frame_per_packet;

        asc_stream_read_max = music_array_48kHz_size;

        if (asc_stream_read_counter + recommended_in_size > asc_stream_read_max)
        {
            asc_stream_read_counter = wav_header;
        }

        in_size = recommended_in_size;
        out_size = min_out_size;

        output_buff1 = output_buff;

        memset(output_buff, 0x00, pkt_len + 1);
        memset(encoder_output, 0x00, out_size);

        for (i = 0; i < frame_per_packet; i++)
        {

            pSBC = (uint8_t *)music_array_48kHz;

            pSBC += asc_stream_read_counter;

            result = sbc_encoder_app_encode(enc_handle, pSBC, &in_size, encoder_output, &out_size);
            if (result != CY_RSLT_SUCCESS)
            {
                printf("Encoding error \r\n");
            }

            if (i == 0)
            {
                *output_buff1 = frame_per_packet;
                output_buff1 += 1;
            }

            memcpy(output_buff1, encoder_output, out_size);
            output_buff1 += out_size;
            asc_stream_read_counter += recommended_in_size;
        }

        result = wiced_bt_avdt_write_req(stream_avdt_handle, output_buff, pkt_len + 1,Timestamp, MARKER_PAYLOAD_BYTE, 0);

        if (result != AVDT_SUCCESS)
        {
            WICED_BT_TRACE("AVDT failed with result 0x%lX for timestamp %lu for outsize %u \r\n",result, Timestamp, pkt_len);
            result = A2DP_APP_FAILED;
            return result;
        }
        Timestamp += TimestampInc;

        /* if the read counter reaches end of wav file
         * restart reading from wav_header offset */
        if(asc_stream_read_counter >= asc_stream_read_max)
        {
            asc_stream_read_counter = wav_header;
        }

        output_buff += (pkt_len + 1);
    }
    return result;
}
#endif /* A2DP_ENABLE_PREFILL */

/* ****************************************************************************
 * Function Name: a2dp_source_encode_sendSBC
 ******************************************************************************
 * Summary:
 *          Calls Encoder to encode the PCM data and sends to Stack
 *
 * Parameters:
 *          None
 * Return:
 *          A2DP_APP_SUCCESS if the data is sent successfully
 *          A2DP_APP_FAILED if there was an error
 *
 * ***************************************************************************/
uint8_t a2dp_source_encode_sendSBC (void)
{
    cy_rslt_t    result      = A2DP_APP_SUCCESS;
        uint8_t     *pSBC;
        uint32_t    i;
        uint16_t    pkt_len     = 0;
        uint8_t     *output_buff;
        uint32_t    in_size     = 0;
        uint32_t    out_size    = 0;



        pkt_len = frame_size * frame_per_packet;


        asc_stream_read_max = music_array_48kHz_size;

        in_size = recommended_in_size;
        out_size = min_out_size;
        output_buff = a2dp_frame_enc_ptr;

        memset(encoder_output, 0x00, out_size);
        memset(a2dp_frame_enc_ptr, 0x00, pkt_len + 1);

        for (i = 0; i < frame_per_packet; i++)
        {
            pSBC = (uint8_t *)&music_array_48kHz;

            pSBC += asc_stream_read_counter;

            result = sbc_encoder_app_encode(enc_handle, pSBC, &in_size, encoder_output, &out_size);
            if (result != CY_RSLT_SUCCESS)
            {
                printf("Encoding error \r\n");
            }

            if (i == 0)
            {
                *output_buff = frame_per_packet;
                output_buff += 1;
            }
            memcpy(output_buff,encoder_output,out_size);
            output_buff += out_size;
            asc_stream_read_counter += recommended_in_size;
        }
        xSemaphoreTake(stream_buff_sem, portMAX_DELAY);

        uint8_t *temp_ptr = a2dp_frame_tx_ptr;
        a2dp_frame_tx_ptr = a2dp_frame_enc_ptr;
        a2dp_frame_enc_ptr = temp_ptr;

        if(asc_stream_read_counter > music_array_48kHz_size){
            result = wiced_bt_avdt_write_req (stream_avdt_handle, a2dp_frame_tx_ptr, frame_size,Timestamp, MARKER_PAYLOAD_BYTE, 0);
        }
        else
            result = wiced_bt_avdt_write_req (stream_avdt_handle, a2dp_frame_tx_ptr, pkt_len+1,Timestamp, MARKER_PAYLOAD_BYTE, 0);

        if (AVDT_SUCCESS != result)
        {
            WICED_BT_TRACE("AVDT failed with result 0x%lX for timestamp %lu for outsize %u \r\n", result, Timestamp, pkt_len);
            result = A2DP_APP_FAILED;
        }
        Timestamp += TimestampInc;

        if (asc_stream_read_counter  > asc_stream_read_max)
        {
            asc_stream_read_counter = wav_header;
        }

        return result;

}

/* ****************************************************************************
 * Function Name: a2dp_source_calculate_delay
 ******************************************************************************
 * Summary:
 *          Calculate the required delay to wait for next audio write
 *
 * Parameters:
 *          None
 * Return:
 *          delay_in_micro_sec: Delay in microseconds
 *
 * ***************************************************************************/
uint16_t a2dp_source_calculate_delay(void)
{
    float bytes_per_pkt = 0;
    float delay_in_micro_sec = 0;

    bytes_per_pkt = frame_size * frame_per_packet;
    delay_in_micro_sec = ((bytes_per_pkt * MILLI_TO_MICRO) / (SampleRate *2)) * MILLI_TO_MICRO;
    WICED_BT_TRACE (" Required delay is %d between pkts of size %d to achieve SampleRate %d",
                    (uint16_t)delay_in_micro_sec, (uint16_t)bytes_per_pkt, SampleRate);
    return (uint16_t)delay_in_micro_sec;
}

/* ****************************************************************************
 * Function Name: a2dp_source_audio_thread
 ******************************************************************************
 * Summary:
 *          Thread to handle the audio encoding and streaming
 *
 * Parameters:
 *          void* arg
 * Return:
 *          None
 *
 * ***************************************************************************/
void a2dp_source_audio_thread(void* arg)
{
    TickType_t lastDebounceTimeTicks = 0;

    stream_buff_sem = xSemaphoreCreateBinary();
    if (NULL ==  stream_buff_sem)
    {
        printf("Error in semaphore \r\n");
        return;
    }

    WICED_BT_TRACE("a2dp_source_thread usTimeout: %llu \r\n", (long long unsigned)usTimeout);

    /* Block the task till a notification is received. */
    xTaskNotifyWait(0, 0, NULL, portMAX_DELAY);

    xSemaphoreGive(stream_buff_sem);

    printf("\nPress Play/Pause button on audio sink device to play/pause the stream.\r\n");

    while (true)
    {
        if (button_status_flag)
        {
            /* Get the current tick count */
            TickType_t currentTicks = xTaskGetTickCount();

            /* Check if the current tick count is at least DEBOUNCE_TICKS
             * greater than the last debounce tick count */
            if ((currentTicks - lastDebounceTimeTicks) > DEBOUNCE_TICKS)
            {
                button_pressed1 = true;
                bStreamingActive = !bStreamingActive;
                /* Clear the button flag */
                button_status_flag = false;
            }
        }
        else
        {
            lastDebounceTimeTicks = xTaskGetTickCount();
        }

        if (bStreamingActive && button_pressed1)
        {
            printf("Audio Streaming...\r\n");
            button_pressed1 = false;

#if A2DP_ENABLE_PREFILL
            if (A2DP_APP_FAILED == a2dp_source_encode_sendSBC_prefill())
            {
                WICED_BT_TRACE("Encode and Send failed. \n");
            }
#endif /* A2DP_ENABLE_PREFILL */
        }

        if (!bStreamingActive && button_pressed1)
        {
            printf("Audio Streaming Paused!\r\n");
            button_pressed1 = false;
        }

        if (!bStreamingActive)
        {
            vTaskDelay(a2dp_packet_delay_ms);
            continue;
        }

        if (A2DP_APP_FAILED == a2dp_source_encode_sendSBC())
        {
            WICED_BT_TRACE("Encode and Send failed. \n");
        }
        vTaskDelay(pdMS_TO_TICKS(A2DP_SOURCE_TASK_DELAY));

    }
}

/* ****************************************************************************
 * Function Name: a2dp_source_parse_sbc_params
 ******************************************************************************
 * Summary:
 *          Parse SBC Parameters and set teh configuration for Encoder
 *
 * Parameters:
 *          pSbc :  SBC Codec Information
 *          pSf  :  Audio Sample frequency
 *          pBlocks : SBC Block
 *          pSubbands : SBC SUB BAND
 *          pMode : SBC channel MODE
 * Return:
 *          None
 *
 * ***************************************************************************/
static void a2dp_source_parse_sbc_params (wiced_bt_a2d_sbc_cie_t * pSbc, uint32_t *pSf,
                                          uint32_t *pBlocks, uint32_t *pSubbands, uint32_t *pMode)
{
    switch (pSbc->samp_freq)
    {
    case A2D_SBC_IE_SAMP_FREQ_16:
        *pSf = AUDIO_SAMPLE_RATE_16kHz;
        break;
    case A2D_SBC_IE_SAMP_FREQ_32:
        *pSf = AUDIO_SAMPLE_RATE_32kHz;
        break;
    case A2D_SBC_IE_SAMP_FREQ_44:
        *pSf = AUDIO_SAMPLE_RATE_44_1kHz;
        break;
    case A2D_SBC_IE_SAMP_FREQ_48:
        *pSf = AUDIO_SAMPLE_RATE_48kHz;
        break;
    }

    switch (pSbc->block_len)
    {
    case A2D_SBC_IE_BLOCKS_4:
        *pBlocks = SBC_BLOCK_0;
        break;
    case A2D_SBC_IE_BLOCKS_8:
        *pBlocks = SBC_BLOCK_1;
        break;
    case A2D_SBC_IE_BLOCKS_12:
        *pBlocks = SBC_BLOCK_2;
        break;
    case A2D_SBC_IE_BLOCKS_16:
        *pBlocks = SBC_BLOCK_3;
        break;
    }

    switch (pSbc->num_subbands)
    {
    case A2D_SBC_IE_SUBBAND_4:
        *pSubbands = SUB_BANDS_4;
        break;
    case A2D_SBC_IE_SUBBAND_8:
        *pSubbands = SUB_BANDS_8;
        break;
    }

    switch (pSbc->ch_mode)
    {
    case A2D_SBC_IE_CH_MD_MONO:
        *pMode = SBC_MONO;
        break;
    case A2D_SBC_IE_CH_MD_DUAL:
        *pMode = SBC_DUAL;
        break;
    case A2D_SBC_IE_CH_MD_STEREO:
        *pMode = SBC_STEREO;
        break;
    case A2D_SBC_IE_CH_MD_JOINT:
        *pMode = SBC_JOINT_STEREO;
        break;
    }
}

/* ****************************************************************************
 * Function Name: a2dp_source_set_audio_streaming
 ******************************************************************************
 * Summary:
 *          Set Audio Streaming state and initiate the action accordingly.
 *
 * Parameters:
 *          handle : avdt_handle Handle
 *          start_audio : Start/Stop audio
 *
 * Return:
 *          None
 *
 * ***************************************************************************/
static void a2dp_source_set_audio_streaming(uint8_t handle,
                                            wiced_bool_t start_audio)
{
    WICED_BT_TRACE("Setting Audio Stream State to %d \n", start_audio);

    if (start_audio)
    {
        wiced_bt_dev_cancel_sniff_mode(av_app_cb.peer_bda);
        a2dp_source_start_streaming(WICED_TRUE, handle,
                &av_app_cb.codec_config.cie.sbc, av_app_cb.stream_mtu);
    }
    else
    {
        a2dp_source_start_streaming(WICED_FALSE, handle, NULL, 0);
    }
}

/* ****************************************************************************
 * Function Name: a2dp_source_control_cback
 ******************************************************************************
 * Summary:
 *          Control callback supplied by the a2dp source profile code.
 *
 * Parameters:
 *          event - control event called back
 *          p_data - event data
 *
 * Return:
 *          None
 *
 * ***************************************************************************/
void  a2dp_source_control_cback (wiced_bt_a2dp_source_event_t event,
                                     wiced_bt_a2dp_source_event_data_t *p_data)
{
    switch (event)
    {
    case WICED_BT_A2DP_SOURCE_CONNECT_EVT:
        if (NULL == p_data)
        {
            WICED_BT_TRACE ("A2DP Source Control Callback with event data pointer NULL \n");
            return;
        }
        /**< Connected event, received on establishing connection to a peer device. Ready to stream. */
        if (p_data->connect.result == WICED_SUCCESS)
        {
            /* Save the address of the remote device on remote connection */
            memcpy (av_app_cb.peer_bda, p_data->connect.bd_addr,
                sizeof(wiced_bt_device_address_t));
            av_app_cb.lcid = p_data->connect.lcid;

            /* Maintain State */
            av_app_cb.state = AV_STATE_OPEN;

            WICED_BT_TRACE ("Connected to addr: <%x> Handle %d \n\r",p_data->connect.bd_addr,
                            p_data->connect.handle);

            handle = p_data->connect.handle;

            connection_status = A2DP_PEER_CONNECTED;

            printf("The PSOC Edge kit is now connected to BT audio sink device.\r\n\r\n");
            WICED_BT_TRACE("Configuring A2DP stream to 48KHz, Stereo \r\n");
            a2dp_source_command_stream_config ((uint8_t)AUDIO_SF_48K, AUDIO_CHCFG_STEREO);

            wiced_bt_dev_setAclPacketTypes (
            av_app_cb.peer_bda,
            HCI_PKT_TYPES_MASK_DM5 | HCI_PKT_TYPES_MASK_DH5 | /* Use 1 mbps 5 slot packets */
            HCI_PKT_TYPES_MASK_DH3 | HCI_PKT_TYPES_MASK_DM3 | /* Use 1 mbps 3 slot packets */
            HCI_PKT_TYPES_MASK_DH1 | HCI_PKT_TYPES_MASK_DM1 | /* Use 1 mbps 1 slot packets */
            HCI_PKT_TYPES_MASK_NO_3_DH1 | /* Don't use 3 mbps 1 slot packets */
            HCI_PKT_TYPES_MASK_NO_3_DH3 | /* Don't use 3 mbps 3 slot packets */
            HCI_PKT_TYPES_MASK_NO_3_DH5); /* Don't use 3 mbps 5 slot packets */

            a2dp_source_command_stream_start ();

        }
        else
        {
            WICED_BT_TRACE (" a2dp source connection to <%x> failed %d \n", p_data->connect.bd_addr,
                  p_data->connect.result);
            printf("Connection Failed to %x:%x:%x:%x:%x:%x. Reset the board. \r\n",p_data->connect.bd_addr[0],
                    p_data->connect.bd_addr[1], p_data->connect.bd_addr[2],p_data->connect.bd_addr[3],
                    p_data->connect.bd_addr[4],p_data->connect.bd_addr[5]);
        }
        break;

    case WICED_BT_A2DP_SOURCE_DISCONNECT_EVT:
        /**< Disconnected event, received on disconnection from a peer device */
        /* Maintain State */
        av_app_cb.state = AV_STATE_IDLE;
        a2dp_source_deinit_streaming();
        handle = 0; /* reset connection handle */
        printf ("A2DP peer disconnected. Reset the board. \r\n");

        connection_status = A2DP_PEER_DISCONNECTED;

        break;

    case WICED_BT_A2DP_SOURCE_CONFIGURE_EVT:
        if (NULL == p_data)
        {
            WICED_BT_TRACE ("A2DP Source Control Callback with event data pointer NULL \n");
            return;
        }
        /* Maintain State */
        av_app_cb.state = AV_STATE_CONFIGURE;
        memcpy (&av_app_cb.codec_config, p_data->set_config.codec_config,
          sizeof(wiced_bt_a2dp_codec_info_t));
        av_app_cb.stream_mtu = p_data->set_config.stream_mtu;
        break;

    case WICED_BT_A2DP_SOURCE_START_IND_EVT:
        printf("WICED_BT_A2DP_SOURCE_START_IND_EVT \r\n");
        if (NULL == p_data)
        {
            WICED_BT_TRACE ("A2DP Source Control Callback with event data pointer NULL \n");
            return;
        }
        if (!wiced_bt_a2dp_source_send_start_response (p_data->start_ind.handle,
                             p_data->start_ind.label,
                             A2D_SUCCESS))
        {
            printf("wiced_bt_a2dp_source_send_start_response no error !!!! \r\n");
            if (av_app_cb.state != AV_STATE_STARTED)
            {
                WICED_BT_TRACE (" a2dp source streaming started handle:%d lcid %x \n",
                  p_data->start_cfm.handle, av_app_cb.lcid);
                printf("WICED_BT_A2DP_SOURCE_START_IND_EVT start streaming \r\n");

                a2dp_source_set_audio_streaming (p_data->start_ind.handle, WICED_TRUE);
                av_app_cb.state = AV_STATE_STARTED;
            }
            else
            {
                WICED_BT_TRACE (" a2dp source streaming already started \n");
            }
        }
        break;

    case WICED_BT_A2DP_SOURCE_START_CFM_EVT:
        if (NULL == p_data)
        {
            WICED_BT_TRACE ("A2DP Source Control Callback with event data pointer NULL \n");
            return;
        }
        /*Start stream event, received when audio streaming is about to start*/
        if (p_data->start_cfm.result != WICED_SUCCESS)
        {
            WICED_BT_TRACE ("Stream start Error \n");
            break;
        }
        if (av_app_cb.state != AV_STATE_STARTED)
        {
            WICED_BT_TRACE (" a2dp source streaming started handle:%d lcid %x \n",
              p_data->start_cfm.handle, av_app_cb.lcid);
            printf("A2DP source is ready to stream...\r\n");
            a2dp_source_set_audio_streaming (p_data->start_cfm.handle, WICED_TRUE);
            av_app_cb.state = AV_STATE_STARTED;
        }
        else
        {
            WICED_BT_TRACE (" a2dp source streaming already started \n");
        }
        break;

    case WICED_BT_A2DP_SOURCE_SUSPEND_EVT:
        /**< Suspend stream event, received when audio streaming is suspended */
        /* Maintain State */
        av_app_cb.state = AV_STATE_OPEN;
        WICED_BT_TRACE (" a2dp source streaming suspended \n");
        a2dp_source_set_audio_streaming (p_data->suspend.handle, WICED_FALSE);
        break;

    case WICED_BT_A2DP_SOURCE_WRITE_CFM_EVT:

        xSemaphoreGive(stream_buff_sem);

        break;

    case WICED_BT_A2DP_SOURCE_DELAY_RPT_EVT:
        if (NULL == p_data)
        {
            WICED_BT_TRACE ("A2DP Source Control Callback with event data pointer NULL \n");
            return;
        }
        WICED_BT_TRACE ("a2dp source WICED_BT_A2DP_SOURCE_DELAY_RPT_EVT delay is %d\n",
              p_data->delay_ms);
        a2dp_source_update_sink_rep_delay (p_data->delay_ms);
        break;

    default:
        WICED_BT_TRACE ("a2dp source Unhandled Event %d\n", event);
        break;
    }
}

/* ****************************************************************************
 * Function Name: a2dp_source_bt_inquiry_result_cback
 ******************************************************************************
 * Summary:
 *         Function to store BT device information ( BT device Address, name ,
 *          RSSI value
 *
 * Parameters:
 *          p_inquiry_result - Inquiry result consisting BD address,
 *                             RSSI and Device class
 *          p_eir_data - EIR data
 *
 * Return:
 *          None
 *
 * ***************************************************************************/
void store_device_info(wiced_bt_dev_inquiry_scan_result_t *p_inquiry_result,
                                uint8_t *p_eir_data)
{
    if (device_count >= MAX_DEVICES)
    {
        printf("Device info storage is full. Cannot store more devices.\n");
        return;
    }

    /* Store the Bluetooth Device Address */
    memcpy(device_info_list[device_count].remote_bd_addr,
            p_inquiry_result->remote_bd_addr, BD_ADDR_LEN);

    /* Store the EIR data if available */
    if (p_eir_data != NULL)
    {
        uint8_t len = *p_eir_data; // First byte is the length of the EIR data
        if (len > 0 && len < MAX_EIR_LEN)
        {
            memcpy(device_info_list[device_count].eir_data, p_eir_data, len);
        }
    }

    /* Store the RSSI value */
    device_info_list[device_count].rssi = p_inquiry_result->rssi;

    /* Increment the device count */
    device_count++;
}

/* ****************************************************************************
 * Function Name: a2dp_source_bt_inquiry_result_cback
 ******************************************************************************
 * Summary:
 *          Handle Inquiry result callback from the stack, format and
 *          send event over UART
 *
 * Parameters:
 *          p_inquiry_result - Inquiry result consisting BD address,
 *                             RSSI and Device class
 *          p_eir_data - EIR data
 *
 * Return:
 *          None
 *
 * ***************************************************************************/
void a2dp_source_bt_inquiry_result_cback(
    wiced_bt_dev_inquiry_scan_result_t *p_inquiry_result, uint8_t *p_eir_data )
{
    if (p_inquiry_result == NULL)
    {
        WICED_BT_TRACE( "Scanning complete.\r\n");
        a2dp_source_bt_inquiry (0);
        initiate_bt_device_connection = true;
    }
    else
    {
        store_device_info(p_inquiry_result, p_eir_data);
    }
}

/* ****************************************************************************
 * Function Name: a2dp_source_bt_inquiry
 ******************************************************************************
 * Summary:
 *        Handle Inquiry command from user
 *
 * Parameters:
 *        enable - Enable Inquiry if 1, Cancel if 0
 *
 * Return:
 *        wiced_result_t: result of start or cancel inquiry operation initiation
 *
 * ***************************************************************************/
wiced_result_t a2dp_source_bt_inquiry( uint8_t enable )
{
    wiced_result_t           result;
    wiced_bt_dev_inq_parms_t params;

    if ( enable )
    {
        memset( &params, 0, sizeof( params ) );

        params.mode             = BTM_GENERAL_INQUIRY;
        params.duration         = INQUIRY_DURATION;
        params.filter_cond_type = BTM_CLR_INQUIRY_FILTER;

        result = wiced_bt_start_inquiry( &params, &a2dp_source_bt_inquiry_result_cback );
        if (result == WICED_BT_PENDING)
        {
            result = WICED_BT_SUCCESS;
        }
        WICED_BT_TRACE( "Inquiry started:%d\n", result );
    }
    else
    {
        result = wiced_bt_cancel_inquiry( );
        WICED_BT_TRACE( "Cancel inquiry:%d\n", result );
    }
    return result;
}

/* ****************************************************************************
 * Function Name: a2dp_source_bt_set_visibility
 ******************************************************************************
 * Summary:
 *          Handle Set Visibility command
 *
 * Parameters:
 *          discoverability: Discoverable if 1, Non-discoverable if 0
 *          connectability: Connectable if 1, Non-connectable if 0
 *
 * Return:
 *          wiced_result_t: result of set visibility operation
 *
 * ***************************************************************************/
wiced_result_t a2dp_source_bt_set_visibility( uint8_t discoverability, uint8_t connectability )
{
    wiced_result_t status = WICED_BT_SUCCESS;

    if ((discoverability > 1) || (connectability > 1))
    {
        WICED_BT_TRACE( "Invalid Input \n");
        status = WICED_BT_ERROR;
    }
    else if ((discoverability != 0) && (connectability == 0))
    {
        /* we cannot be discoverable and not connectable */
        WICED_BT_TRACE("we cannot be discoverable and not connectable \n");
        status = WICED_BT_ERROR;
    }
    else
    {
        wiced_bt_dev_set_discoverability((discoverability != 0) ? BTM_GENERAL_DISCOVERABLE : BTM_NON_DISCOVERABLE,
                                            BTM_DEFAULT_DISC_WINDOW,
                                            BTM_DEFAULT_DISC_INTERVAL);

        wiced_bt_dev_set_connectability((connectability != 0) ? WICED_TRUE : WICED_FALSE,
                                            BTM_DEFAULT_CONN_WINDOW,
                                            BTM_DEFAULT_CONN_INTERVAL);
    }
    return status;
}

/* ****************************************************************************
 * Function Name: a2dp_source_bt_set_pairability
 ******************************************************************************
 * Summary:
 *          Handle Set Pairability command
 *
 * Parameters:
 *          pairing_allowed: Pairing allowed if 1, not allowed if 0
 *
 * Return:
 *          None
 *
 * ***************************************************************************/
void a2dp_source_bt_set_pairability( uint8_t pairing_allowed )
{
    wiced_bt_set_pairable_mode(pairing_allowed, TRUE);
    WICED_BT_TRACE( "Set the pairing allowed to %d \n", pairing_allowed );
}

/* ****************************************************************************
 * Function Name: a2dp_source_bt_print_local_bda
 ******************************************************************************
 * Summary:
 *          Print Local BDA command
 *
 * Parameters:
 *          None
 *
 * Return:
 *          None
 *
 * ***************************************************************************/
void a2dp_source_bt_print_local_bda( void )
{
    wiced_bt_device_address_t bda = { 0 };

    wiced_bt_dev_read_local_addr(bda);
    WICED_BT_TRACE( "Local Bluetooth Device Address:");
    print_bd_address (bda);
}

/* ****************************************************************************
 * Function Name: a2dp_source_command_connect
 ******************************************************************************
 * Summary:
 *          Connects to the A2DP Sink with the given BD-Address
 *
 * Parameters:
 *          bd_addr: Remote BD Address
 *          len: Length of the BD-address.
 *
 * Return:
 *          status: result of a2dp connect API
 *
 * ***************************************************************************/
wiced_result_t a2dp_source_command_connect(wiced_bt_device_address_t bd_addr, uint32_t len)
{
    wiced_result_t status = WICED_BT_SUCCESS;

    if (handle)
    {
        WICED_BT_TRACE("Already Connected \n\r");
        status = WICED_BT_ERROR;
    }
    else
    {
        WICED_BT_TRACE("Connecting to BDA: ");
        if(ENABLE_BT_VERBOSE_LOGS)
        {
            print_bd_address(bd_addr);
        }
        status = wiced_bt_a2dp_source_connect(bd_addr);
    }
    return status;
}

/* ****************************************************************************
 * Function Name: a2dp_source_command_disconnect
 ******************************************************************************
 * Summary:
 *          Disconnects the A2DP Sink that is already connected
 *
 * Parameters:
 *          None
 *
 * Return:
 *          status: result of a2dp disconnect API
 *
 * ***************************************************************************/
wiced_result_t a2dp_source_command_disconnect(void)
{
    wiced_result_t status = WICED_BT_SUCCESS;

    WICED_BT_TRACE( "Disconnecting Connection with Handle %d\n\r", handle );
    if (!handle)
    {
        status = WICED_BT_ERROR;
    }
    else
    {
        status = wiced_bt_a2dp_source_disconnect(handle);
        av_app_cb.state = AV_STATE_DISCONNECTING;
    }

    return status;
}

/* ****************************************************************************
 * Function Name: a2dp_source_command_stream_config
 ******************************************************************************
 * Summary:
 *          Configures the A2DP Stream
 *
 * Parameters:
 *          sf: sample frequency
 *          chcfg: channel configuration.
 *
 * Return:
 *          None
 *
 * ***************************************************************************/
void a2dp_source_command_stream_config(uint8_t sf, uint8_t chcfg)
{
    switch (sf)
    {
    case AUDIO_SF_16K:
        sf = A2D_SBC_IE_SAMP_FREQ_16;
        break;
    case AUDIO_SF_32K:
        sf = A2D_SBC_IE_SAMP_FREQ_32;
        break;
    case AUDIO_SF_44_1K:
        sf = A2D_SBC_IE_SAMP_FREQ_44;
        break;
    case AUDIO_SF_48K:
        sf = A2D_SBC_IE_SAMP_FREQ_48;
        break;
    default:
        sf = A2D_SBC_IE_SAMP_FREQ_48;
        break;
    }

    switch (chcfg)
    {
    case AUDIO_CHCFG_MONO:
        chcfg = A2D_SBC_IE_CH_MD_MONO;
        break;
    case AUDIO_CHCFG_STEREO:
        chcfg = A2D_SBC_IE_CH_MD_STEREO;
        break;
    default:
        chcfg = A2D_SBC_IE_CH_MD_STEREO;
        break;
    }

    if (!(sf == bt_audio_config.default_codec_config.cie.sbc.samp_freq) &&
     (chcfg == bt_audio_config.default_codec_config.cie.sbc.ch_mode))
    {
        a2dp_source_deinit_streaming();
        bt_audio_config.default_codec_config.cie.sbc.samp_freq = sf;
        bt_audio_config.default_codec_config.cie.sbc.ch_mode = chcfg;
    }
    WICED_BT_TRACE( "Handle %d sf:%s chcfg:%s\n\r", handle,
                    get_audio_freq_name (sf), get_audio_chcfg_name (chcfg) );
}

/* ****************************************************************************
 * Function Name: a2dp_source_command_stream_start
 ******************************************************************************
 * Summary:
 *          Initiates Stream Start
 *
 * Parameters:
 *          None
 *
 * Return:
 *          status: result of Stream Start API
 *
 * ***************************************************************************/
wiced_result_t a2dp_source_command_stream_start(void)
{
    WICED_BT_TRACE( "Stream Start for Connection Handle %d\n\r", handle);

    if (!handle)
    {
       return WICED_BT_ERROR;
    }
    return wiced_bt_a2dp_source_start(handle, &bt_audio_config.default_codec_config);
}

/* ****************************************************************************
 * Function Name: a2dp_source_command_stream_stop
 ******************************************************************************
 * Summary:
 *          Initiates Stream Suspend
 *
 * Parameters:
 *          None
 *
 * Return:
 *          status: result of Stream Suspend API
 *
 * ***************************************************************************/
wiced_result_t a2dp_source_command_stream_stop(void)
{
    WICED_BT_TRACE( "Stream Stop for Connection Handle %d\n\r", handle);

    if (!handle)
    {
        return WICED_BT_ERROR;
    }

    return wiced_bt_a2dp_source_suspend(handle);
}

/******************************************************************************
 * Function Name: a2dp_source_write_eir
 *******************************************************************************
 * Summary:
 *          Prepare extended inquiry response data.  Current version publishes
 *          audio source services.
 *
 * Parameters:
 *          None
 *
 * Return:
 *          None
 *
 ******************************************************************************/
void a2dp_source_write_eir( void )
{
    uint8_t *pBuf;
    uint8_t *p;
    uint8_t *p_tmp;
    uint8_t nb_uuid = 0;
    uint8_t length;

    /* Allocating a buffer from the public pool */
    pBuf = (uint8_t*) wiced_bt_get_buffer ( WICED_HS_EIR_BUF_MAX_SIZE);

    if (!pBuf)
    {
        WICED_BT_TRACE ("Buffer allocation for EIR Data failed \n");
        return;
    }

    p = pBuf;

    length = (uint8_t) strlen ((char*) a2dp_source_cfg_settings.device_name);
    UINT8_TO_STREAM (p, length + 1);
    UINT8_TO_STREAM (p, BT_EIR_COMPLETE_LOCAL_NAME_TYPE);
    memcpy (p, a2dp_source_cfg_settings.device_name, length);
    p += length;

    /* Add other BR/EDR UUIDs */
    p_tmp = p;
    p++;
    UINT8_TO_STREAM (p, BT_EIR_COMPLETE_16BITS_UUID_TYPE);
    UINT16_TO_STREAM (p, UUID_SERVCLASS_AUDIO_SOURCE);
    nb_uuid++;

    /* Now, we can update the UUID Tag's length */
    UINT8_TO_STREAM (p_tmp, (nb_uuid * LEN_UUID_16) + 1);

    /* Last Tag */
    UINT8_TO_STREAM (p, 0x00);

    /* print EIR data */
    printf( "Current device's EIR (Extended Inquiry Response):");
    for(int i = 0; i < MIN( p-( uint8_t* )pBuf,100 ); i++)
    {
        printf("%c",(char)pBuf[i+1]);
    }
    wiced_bt_dev_write_eir( pBuf, (uint16_t)(p - pBuf) );

    /* Allocated buffer not anymore needed. Free it */
    wiced_bt_free_buffer (pBuf);
}


/******************************************************************************
 * Function Name: a2dp_source_write_nvram
 *******************************************************************************
 * Summary:
 *          Function to write information in NVRAM. but it will not be stored.
 *
 * Parameters:
 *          nvram_id: NVRAM Id
 *          data_len: Length of the data to be written
 *          p_data: Data to be written
 *
 * Return:
 *          Number of bytes written
 *
 *****************************************************************************/
uint16_t a2dp_source_write_nvram( int nvram_id, int data_len, void *p_data)
{
    uint16_t        bytes_written = 0;
    int i=0;

    if (NULL != p_data)
    {
        bytes_written=data_len;
        memcpy(&nv_key_ram,p_data,sizeof(wiced_bt_device_link_keys_t));
        WICED_BT_TRACE("Link Key is ");
        for (i=0; i<LINK_KEY_LEN; i++) {
            WICED_BT_TRACE("%02x ",nv_key_ram.key_data.br_edr_key[i]);
        }
        WICED_BT_TRACE("\r\n");

        WICED_BT_TRACE("NVRAM ID:%d written :%d bytes \n", nvram_id, bytes_written);
    }

    return (bytes_written);
}


/******************************************************************************
 * Function Name: a2dp_source_read_nvram
 *******************************************************************************
 * Summary:
 *          Read data from the NVRAM and return in the passed buffer
 *
 * Parameters:
 *          nvram_id: NVRAM Id
 *          data_len: Length of the data to be read
 *          p_data: Data buffer pointer to hold read data
 *
 * Return:
 *          Number of bytes read
 *
 *****************************************************************************/
uint16_t a2dp_source_read_nvram( int nvram_id, void *p_data, int data_len)
{
    uint16_t        read_bytes = 0;

    if ((NULL != p_data) && (data_len >= sizeof(wiced_bt_device_link_keys_t)))
    {
        memset(&nv_key_ram,0xFF,sizeof(wiced_bt_device_link_keys_t));
        WICED_BT_TRACE("NVRAM ID:%d read out of %d bytes:%d \n", nvram_id,
                       sizeof(wiced_bt_device_link_keys_t), read_bytes );
    }
    return (read_bytes);
}

/******************************************************************************
 * Function Name: a2dp_source_app_init()
 *******************************************************************************
 * Summary:
 *   This function handles application level initialization tasks and is
 *   called from the BT management callback once the Bluetooth Stack enabled
 *   event (BTM_ENABLED_EVT) is triggered
 *   This function is executed in the BTM_ENABLED_EVT management callback.
 *
 * Parameters:
 *   None
 *
 * Return:
 *   None
 *
 ******************************************************************************/
void a2dp_source_app_init( void )
{
    wiced_result_t result = WICED_BT_SUCCESS;
    av_app_cb.state = AV_STATE_IDLE;

    /* Register with the A2DP source profile code */
    result = wiced_bt_a2dp_source_init( &bt_audio_config,
                                        a2dp_source_control_cback, p_default_heap);
    if (WICED_BT_SUCCESS == result)
    {
        WICED_BT_TRACE( "A2DP Source initialized \n\r");

    }
    else
    {
        WICED_BT_TRACE( "A2DP Source initialization failed \n\r");
    }
}

/******************************************************************************
 * Function Name: a2dp_source_management_callback
 *******************************************************************************
 * Summary:
 *   This is a Bluetooth stack event handler function to receive management
 *   events from the Bluetooth stack and process as per the application.
 *
 * Parameters:
 *   wiced_bt_management_evt_t event : BLE event code of one byte length
 *   wiced_bt_management_evt_data_t *p_event_data: Pointer to BTStack management
 *   event structures
 *
 * Return:
 *  wiced_result_t: Error code from WICED_RESULT_LIST or BT_RESULT_LIST
 *
 ******************************************************************************/

wiced_result_t a2dp_source_management_callback( wiced_bt_management_evt_t event,
        wiced_bt_management_evt_data_t *p_event_data )
{
    wiced_bt_ble_advert_mode_t          *p_mode; /* Advertisement Mode */
    wiced_bt_device_address_t           bda = {0};
    wiced_result_t                      result = WICED_BT_SUCCESS;

    wiced_bt_power_mgmt_notification_t *p_power_mgmt_notification;
    wiced_bt_dev_encryption_status_t   *p_encryption_status;
    wiced_bt_dev_pairing_cplt_t        *p_pairing_cmpl;
    int                                 pairing_result;

    WICED_BT_TRACE("[A2DP Source management callback] %s\n", get_bt_event_name(event));

    switch( event )
    {
        /* Bluetooth  stack enabled */
        case BTM_ENABLED_EVT:

            if (NULL == p_event_data)
            {
                WICED_BT_TRACE("Callback data pointer p_event_data is NULL \n");
                break;
            }

            if (WICED_BT_SUCCESS == p_event_data->enabled.status)
            {
                /* Bluetooth is enabled */
                wiced_bt_dev_read_local_addr(bda);
                WICED_BT_TRACE("Local Bluetooth Address (PSOC kit): ");
                if(ENABLE_BT_VERBOSE_LOGS)
                {
                    print_bd_address(bda);
                }

                /* Set as connectable */
                result = a2dp_source_bt_set_visibility (WICED_FALSE, WICED_TRUE);

                /* Check for existing data in KV store for a bonded device */
                if(CY_RSLT_SUCCESS == app_bt_restore_bond_data())
                {
                    WICED_BT_TRACE("Keys found in NVM, Adding it to Address Resolution DB\n");
                    /* Load previous paired keys for address resolution */
                    app_bt_add_devices_to_address_resolution_db();
                }

                a2dp_source_write_eir();

                /* create SDP records */
                wiced_bt_sdp_db_init((uint8_t*) a2dp_source_sdp_db,
                        sizeof(a2dp_source_sdp_db));

                /* start the a2dp application */
                a2dp_source_app_init();

                wiced_bt_avrc_tg_init(app_avrc_event_cback);

                wiced_bt_avrc_tg_register();

                a2dp_source_bt_set_pairability(true);

                /* Enable the Inquiry for scanning the devices. */
                a2dp_source_bt_inquiry (true);

                printf("\nScanning for BT audio sink devices...\n");

            }
            else
            {
                WICED_BT_TRACE("Bluetooth Enable failed \n");
            }
            break;

        case BTM_DISABLED_EVT:
            WICED_BT_TRACE("Bluetooth Disabled \n");
            break;

        case BTM_SECURITY_FAILED_EVT:
            if (NULL == p_event_data)
            {
                WICED_BT_TRACE("Callback data pointer p_event_data is NULL \n");
                break;
            }
            WICED_BT_TRACE("Security failed: %d / %d\n",
                    p_event_data->security_failed.status,
                    p_event_data->security_failed.hci_status);
            break;

        case BTM_PIN_REQUEST_EVT:
            if (NULL == p_event_data)
            {
                WICED_BT_TRACE("Callback data pointer p_event_data is NULL \n");
                break;
            }

            wiced_bt_dev_pin_code_reply(*p_event_data->pin_request.bd_addr,
                    WICED_BT_SUCCESS, WICED_PIN_CODE_LEN, (uint8_t *)&pincode[0]);
            break;

        case BTM_USER_CONFIRMATION_REQUEST_EVT:
            if (NULL == p_event_data)
            {
                WICED_BT_TRACE("Callback data pointer p_event_data is NULL \n");
                break;
            }
            /* If this is just_works pairing, accept.
             * Otherwise send event to the MCU to confirm the same value.
             */

            WICED_BT_TRACE("User confirmation request for device address: ");

            if(ENABLE_BT_VERBOSE_LOGS)
            {
                print_bd_address(p_event_data->user_confirmation_request.bd_addr);
            }

            if (p_event_data->user_confirmation_request.just_works)
            {
                WICED_BT_TRACE("User confirmation request is just_works.\r\n");
                wiced_bt_dev_confirm_req_reply( WICED_BT_SUCCESS, p_event_data->user_confirmation_request.bd_addr);
            }
            else
            {
                WICED_BT_TRACE("User confirmation request is not supported.\r\n");
                wiced_bt_dev_confirm_req_reply(WICED_BT_UNSUPPORTED, p_event_data->user_confirmation_request.bd_addr);
            }
            break;

        case BTM_PASSKEY_NOTIFICATION_EVT:
            if (NULL == p_event_data)
            {
                WICED_BT_TRACE("Callback data pointer p_event_data is NULL \n");
                break;
            }
            WICED_BT_TRACE("Passkey: %lu, BDA: ", p_event_data->user_passkey_notification.passkey);

            if(ENABLE_BT_VERBOSE_LOGS)
            {
                print_bd_address(p_event_data->user_passkey_notification.bd_addr);
            }
            wiced_bt_dev_confirm_req_reply( WICED_BT_SUCCESS,
                    p_event_data->user_passkey_notification.bd_addr );
            break;

        case BTM_PAIRING_IO_CAPABILITIES_BR_EDR_REQUEST_EVT:
            if (NULL == p_event_data)
            {
                WICED_BT_TRACE("Callback data pointer p_event_data is NULL \n");
                break;
            }

            p_event_data->pairing_io_capabilities_br_edr_request.local_io_cap = BTM_IO_CAPABILITIES_NONE;
            p_event_data->pairing_io_capabilities_br_edr_request.oob_data = BTM_OOB_NONE;
            p_event_data->pairing_io_capabilities_br_edr_request.auth_req = BTM_AUTH_ALL_PROFILES_NO;
            break;

        case BTM_PAIRING_IO_CAPABILITIES_BLE_REQUEST_EVT:
            if (NULL == p_event_data)
            {
                WICED_BT_TRACE("Callback data pointer p_event_data is NULL \n");
                break;
            }

            p_event_data->pairing_io_capabilities_ble_request.local_io_cap  = BTM_IO_CAPABILITIES_NONE;
            p_event_data->pairing_io_capabilities_ble_request.oob_data = BTM_OOB_NONE;
            p_event_data->pairing_io_capabilities_ble_request.auth_req = BTM_LE_AUTH_REQ_SC_BOND;
            p_event_data->pairing_io_capabilities_ble_request.max_key_size = MAX_KEY_SIZE;
            p_event_data->pairing_io_capabilities_ble_request.init_keys = BTM_LE_KEY_PENC|BTM_LE_KEY_PID|BTM_LE_KEY_PCSRK|BTM_LE_KEY_LENC;
            p_event_data->pairing_io_capabilities_ble_request.resp_keys = BTM_LE_KEY_PENC|BTM_LE_KEY_PID|BTM_LE_KEY_PCSRK|BTM_LE_KEY_LENC;
            break;

        case BTM_PAIRING_COMPLETE_EVT:
            if (NULL == p_event_data)
            {
                WICED_BT_TRACE("Callback data pointer p_event_data is NULL \n");
                break;
            }

            p_pairing_cmpl = &p_event_data->pairing_complete;
            if(BT_TRANSPORT_BR_EDR == p_pairing_cmpl->transport)
            {
                pairing_result = p_pairing_cmpl->pairing_complete_info.br_edr.status;
            }
            else
            {
                pairing_result = p_pairing_cmpl->pairing_complete_info.ble.reason;
            }
            WICED_BT_TRACE("Pairing Result: %d\n", pairing_result);

            if (A2DP_APP_SUCCESS == pairing_result )
            {
                /* Update Num of bonded devices and next free slot in slot data*/
                app_bt_update_slot_data();

                /* Enable here if bonding request API is called so that connection happens after pairing. */
                a2dp_source_command_connect(a2dp_sink_bd_addr, BD_ADDR_LEN);
                WICED_BT_TRACE("Connection Request sent. Waiting for paired device to get connected...\r\n");
            }

            break;

        case BTM_ENCRYPTION_STATUS_EVT:
            if (NULL == p_event_data)
            {
                WICED_BT_TRACE("Callback data pointer p_event_data is NULL \n");
                break;
            }
            p_encryption_status = &p_event_data->encryption_status;

            WICED_BT_TRACE("Encryption status result:%u, BDA: ",
                    p_encryption_status->result );
            if(ENABLE_BT_VERBOSE_LOGS)
            {
                print_bd_address(p_encryption_status->bd_addr);
            }
            /* Check and retreive the index of the bond data of the device that
             * got connected */
            /* This call will return BOND_INDEX_MAX if the device is not found */
            bondindex = app_bt_find_device_in_nvm(p_event_data->encryption_status.bd_addr);
            if (BOND_INDEX_MAX > bondindex)
            {
                app_bt_restore_bond_data();
                app_bt_restore_cccd();
                /* Set CCCD value from the value that was previously saved in the NVRAM */
                app_hello_sensor_notify_client_char_config[0] = peer_cccd_data[bondindex]; // TODO

                WICED_BT_TRACE("Bond info present in NVM for device: ");
                if(ENABLE_BT_VERBOSE_LOGS)
                {
                    print_bd_address(p_event_data->encryption_status.bd_addr);
                }
            }
            else
            {
                WICED_BT_TRACE("No Bond info present in NVM for device: ");
                if(ENABLE_BT_VERBOSE_LOGS)
                {
                    print_bd_address(p_event_data->encryption_status.bd_addr);
                }
                bondindex = 0;
            }
            break;

        case BTM_SECURITY_REQUEST_EVT:
            if (NULL == p_event_data)
            {
                WICED_BT_TRACE("Callback data pointer p_event_data is NULL \n");
                break;
            }

            wiced_bt_ble_security_grant( p_event_data->security_request.bd_addr,
                    WICED_BT_SUCCESS );
            break;

        case BTM_PAIRED_DEVICE_LINK_KEYS_UPDATE_EVT:
            if (NULL == p_event_data)
            {
                WICED_BT_TRACE("Callback data pointer p_event_data is NULL \n");
                break;
            }
            WICED_BT_TRACE("Paired device link key size:%d, BDA: ",
                    sizeof(wiced_bt_device_link_keys_t));

            if(ENABLE_BT_VERBOSE_LOGS)
            {
                print_bd_address(p_event_data->paired_device_link_keys_update.bd_addr);
            }

            /* This application supports a single paired host, we can save keys
             * under the same NVRAM ID overwriting previous pairing if any */
            a2dp_source_write_nvram(A2DP_SOURCE_NVRAM_ID,
                    sizeof(wiced_bt_device_link_keys_t), &p_event_data->paired_device_link_keys_update);

            /* save device keys to NVRAM */
            if (CY_RSLT_SUCCESS == app_bt_save_device_link_keys(&(p_event_data->paired_device_link_keys_update)))
            {
                printf("Successfully Bonded to ");
                print_bd_address(p_event_data->paired_device_link_keys_update.bd_addr);
            }
            else
            {
                printf("Failed to bond! \n");
            }

            break;

        case BTM_PAIRED_DEVICE_LINK_KEYS_REQUEST_EVT:
            if (NULL == p_event_data)
            {
                WICED_BT_TRACE("Callback data pointer p_event_data is NULL \n");
                break;
            }
            WICED_BT_TRACE("Paired device link key size:%d, BDA: ",
                    sizeof(wiced_bt_device_link_keys_t));
            if(ENABLE_BT_VERBOSE_LOGS)
            {
                print_bd_address(p_event_data->paired_device_link_keys_request.bd_addr);
            }

            /* Need to search to see if the BD_ADDR we are
             * looking for is in NVRAM. If not, we return WICED_BT_ERROR
             * and the stack will generate keys and will then call
             * BTM_PAIRED_DEVICE_LINK_KEYS_UPDATE_EVT so that they
             * can be stored
             */

            /* Assume the device won't be found.
             * If it is, we will set this back to WICED_BT_SUCCESS */
            result = WICED_BT_ERROR;

            bondindex = app_bt_find_device_in_nvm(p_event_data->paired_device_link_keys_request.bd_addr);
            if(BOND_INDEX_MAX > bondindex)
            {
                /* Copy the keys to where the stack wants it */
                memcpy(&(p_event_data->paired_device_link_keys_request),
                        &bond_info.link_keys[bondindex],
                        sizeof(wiced_bt_device_link_keys_t));
                result = WICED_BT_SUCCESS;

                WICED_BT_TRACE("Paired device Link Keys found. BDA: ");
                if(ENABLE_BT_VERBOSE_LOGS)
                {
                    print_bd_address(p_event_data->paired_device_link_keys_request.bd_addr);
                }
            }
            else
            {
                WICED_BT_TRACE("Paired device Link Keys not found in the database! \n\n");
                bondindex = 0;
            }
            break;

        case BTM_LOCAL_IDENTITY_KEYS_UPDATE_EVT:
            if (NULL == p_event_data)
            {
                WICED_BT_TRACE("Callback data pointer p_event_data is NULL \n");
                break;
            }
            /* Update of local privacy keys - save to NVRAM */
            if (CY_RSLT_SUCCESS != app_bt_save_local_identity_key(p_event_data->local_identity_keys_update))
            {
                result = WICED_BT_ERROR;
            }else
            {
                result = WICED_BT_SUCCESS;
            }

            break;

        case BTM_LOCAL_IDENTITY_KEYS_REQUEST_EVT:
            if (NULL == p_event_data)
            {
                WICED_BT_TRACE("Callback data pointer p_event_data is NULL \n");
                break;
            }

            /* Read Local Identity Resolution Keys if present in NVRAM*/
            if(CY_RSLT_SUCCESS == app_bt_read_local_identity_keys())
            {
                memcpy(&(p_event_data->local_identity_keys_request),
                        &(identity_keys), sizeof(wiced_bt_local_identity_keys_t));
                WICED_BT_TRACE("Local Identity keys\n");
                if(ENABLE_BT_VERBOSE_LOGS)
                {
                    print_array(&identity_keys,
                            sizeof(wiced_bt_local_identity_keys_t));
                }
                result = WICED_BT_SUCCESS;
            }
            else
            {
                result = WICED_BT_ERROR;
            }
            break;

        case BTM_POWER_MANAGEMENT_STATUS_EVT:
            if (NULL == p_event_data)
            {
                WICED_BT_TRACE("Callback data pointer p_event_data is NULL \n");
                break;
            }
            p_power_mgmt_notification = &p_event_data->power_mgmt_notification;

            WICED_BT_TRACE( "Power management event status:%d, hci_status:%d, BDA: ",
                    p_power_mgmt_notification->status,
                    p_power_mgmt_notification->hci_status);
            if(ENABLE_BT_VERBOSE_LOGS)
            {
                print_bd_address(p_power_mgmt_notification->bd_addr);
            }
            break;

        case BTM_BLE_ADVERT_STATE_CHANGED_EVT:
            if (NULL == p_event_data)
            {
                WICED_BT_TRACE("Callback data pointer p_event_data is NULL \n");
                break;
            }
            p_mode = &p_event_data->ble_advert_state_changed;

            WICED_BT_TRACE("Advertisement state changed: %d\n", *p_mode);
            break;

        case BTM_BLE_CONNECTION_PARAM_UPDATE:
            if (NULL == p_event_data)
            {
                WICED_BT_TRACE("Callback data pointer p_event_data is NULL \n");
                break;
            }

            /* Connection parameters updated */
            if (WICED_SUCCESS == p_event_data->ble_connection_param_update.status)
            {
                WICED_BT_TRACE("Supervision Time Out = %d\n",
                        (p_event_data->ble_connection_param_update.supervision_timeout * 10));
            }
            break;

        default:
            WICED_BT_TRACE("Unhandled Bluetooth Management Event: 0x%x %s\n", event,
                    get_bt_event_name(event));
            break;
        }
        return result;
}

/*******************************************************************************
 * Function Name: reset_link_keys
 ********************************************************************************
 * Summary: Reset and erase all stored link keys for BLE
 *
 * Parameters:
 *  None
 *
 * Return:
 *  None
 *
 *******************************************************************************/
void reset_link_keys(void)
{
    /* Reset kv-store library, this will clear the NVM */
    if(CY_RSLT_SUCCESS == mtb_kvstore_reset(&kvstore_obj))
    {
        printf("Successfully reset kv-store library, \nPlease reset the device to generate new keys!\n");
    }
    else
    {
        printf("failed to reset kv-store library\n");
    }

    /* Clear peer link keys and identity keys structure */
    memset(&bond_info, 0, sizeof(bond_info));
    memset(&identity_keys, 0, sizeof(identity_keys));
}


/*******************************************************************************
 * Function Name: app_avrc_event_cback
 ********************************************************************************
 * Summary:
 *     AVRC event callback function
 *
 * Parameters:
 *     uint8_t event_id : event ID of AVRC
 *     wiced_bt_rc_event_t *p_event : pointer to AVRC event structure
 *
 * Return:
 *     void
 *
 *******************************************************************************/
static void app_avrc_event_cback(uint8_t event_id,  wiced_bt_rc_event_t *p_event)
{
    uint8_t event_data[sizeof(uint16_t) + sizeof(uint8_t)];
    uint8_t *p;

    WICED_BT_TRACE( "[%s]: id %d ", __FUNCTION__, event_id);

    switch(event_id)
    {
    case APP_AVRC_EVENT_DEVICE_CONNECTED:
        WICED_BT_TRACE("connected.");
        break;

    case APP_AVRC_EVENT_DEVICE_DISCONNECTED:
        WICED_BT_TRACE("disconnected.");
        break;

#ifdef APP_AVRC_SETTING_CHANGE_SUPPORTED
    case APP_AVRC_EVENT_REPEAT_SETTINGS_CHANGED:
        WICED_BT_TRACE("repeat settings.");
        break;
    case APP_AVRC_EVENT_SHUFFLE_SETTINGS_CHANGED:
        WICED_BT_TRACE("shuffle settings.");
        break;
#endif

    case APP_AVRC_EVENT_PASSTHROUGH_CMD:
        WICED_BT_TRACE("passthrough cmd.");
        rc_target_passthrough_cmd_event(&p_event->passthrough_command);
        break;

    case APP_AVRC_EVENT_ABS_VOL_CHANGED:
        p = event_data;
        UINT16_TO_STREAM(p,  p_event->absolute_volume.handle);
        UINT8_TO_STREAM(p,  p_event->absolute_volume.volume);
        printf("Volume Changed :%u\r\n", p_event->absolute_volume.volume );
        break;
    }
    WICED_BT_TRACE("\n");
}

/*******************************************************************************
 * Function Name: rc_target_passthrough_cmd_event
 ********************************************************************************
 * Summary:
 *     RC target Passthrough command event callback
 *
 * Parameters:
 *     wiced_bt_avrc_tg_passthrough_cmd_t *p_passthrough
 *
 * Return:
 *     void
 *
 *******************************************************************************/
static void rc_target_passthrough_cmd_event(wiced_bt_avrc_tg_passthrough_cmd_t *p_passthrough)
{
    uint8_t event_data[sizeof(uint16_t)];
    uint8_t *p = event_data;

    /* Every AVRC Wiced Event start with AVRC Handle */
    UINT16_TO_STREAM(p,  p_passthrough->handle);

    switch(p_passthrough->command)
    {
    case APP_AVRC_EVENT_PASSTHROUGH_CMD_PLAY:
        bStreamingActive = WICED_TRUE;
        button_pressed1 = true;
        WICED_BT_TRACE("[AVRCP] PLAY\r\n");
        break;
    case APP_AVRC_EVENT_PASSTHROUGH_CMD_PAUSE:
        bStreamingActive = WICED_FALSE;
        button_pressed1 = true;
        WICED_BT_TRACE("[AVRCP] PAUSE\r\n");
        break;
    case APP_AVRC_EVENT_PASSTHROUGH_CMD_STOP:
        WICED_BT_TRACE("[AVRCP] STOP\r\n");
        break;
    case APP_AVRC_EVENT_PASSTHROUGH_CMD_NEXT_TRACK:
        WICED_BT_TRACE("[AVRCP] NEXT_TRACK\r\n");
        break;
    case APP_AVRC_EVENT_PASSTHROUGH_CMD_PREVIOUS_TRACK:
        WICED_BT_TRACE("[AVRCP] PREVIOUS_TRACK\r\n");
        break;
    default:
        WICED_BT_TRACE( "[%s] Unknown cmd:%u\n\r", __FUNCTION__, p_passthrough->command );
        return;
    }
}

/* [] END OF FILE */
