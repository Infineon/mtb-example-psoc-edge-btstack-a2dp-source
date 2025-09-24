/******************************************************************************
* File Name:   a2dp_source.h
*
* Description: Header file for A2DP source task.
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
#ifndef A2DP_SOURCE_H
#define A2DP_SOURCE_H

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

/******************************************************************************
 * Includes
 *****************************************************************************/
#include "app_bt_utils/app_bt_utils.h"
#include <stdio.h>
#include <stdlib.h>
#include <signal.h>
#include <stdbool.h>
#include <FreeRTOS.h>
#include <semphr.h>
#include <ctype.h>
#include <string.h>
#include "string.h"
#include "wiced_bt_stack.h"
#include "wiced_memory.h"
#include "wiced_timer.h"
#include "wiced_bt_dev.h"
#include "wiced_bt_types.h"
#include "wiced_bt_l2c.h"
#include "wiced_bt_ble.h"
#include "wiced_bt_avdt.h"
#include "wiced_bt_a2d.h"
#include "wiced_bt_a2d_sbc.h"
#include "wiced_bt_a2dp_source.h"
#include "wiced_bt_a2dp_defs.h"
#include "wiced_bt_trace.h"
#include "hcidefs.h"
#include "wiced_bt_gatt.h"
#include "wiced_bt_cfg.h"
#include "wiced_bt_uuid.h"
#include "wiced_result.h"
#include "wiced_timer.h"
#include "sbc_encoder_api.h"
#include "asc_wav_header_parser.h"

typedef short SINT16;
typedef int SINT32;

/******************************************************************************
 * Macros
 *****************************************************************************/

#define A2DP_SOURCE_SDP_DB_SIZE         (63U)

#define AUDIO_SF_48K                    (1U)
#define AUDIO_SF_44_1K                  (2U)
#define AUDIO_SF_32K                    (3U)
#define AUDIO_SF_16K                    (4U)

#define AUDIO_CHCFG_MONO                (0U)
#define AUDIO_CHCFG_STEREO              (1U)

#define A2DP_APP_SUCCESS                (0U)
#define A2DP_APP_FAILED                 (1U)

#define WICED_PIN_CODE_LEN              (4U)

#define SBC_LOUDNESS                    (0U)
#define SBC_SNR                         (1U)

#define SUB_BANDS_8                     (8U)
#define SUB_BANDS_4                     (4U)

#define SBC_MONO                        (0U)
#define SBC_DUAL                        (1U)
#define SBC_STEREO                      (2U)
#define SBC_JOINT_STEREO                (3U)

#define SBC_BLOCK_0                     (4U)
#define SBC_BLOCK_1                     (8U)
#define SBC_BLOCK_2                     (12U)
#define SBC_BLOCK_3                     (16U)

#define A2DP_PEER_CONNECTED             (1U)
#define A2DP_PEER_DISCONNECTED          (0U)

#define BDA_ADDRESS_BYTE_0              0x2C
#define BDA_ADDRESS_BYTE_1              0xFD
#define BDA_ADDRESS_BYTE_2              0xB4
#define BDA_ADDRESS_BYTE_3              0xE6
#define BDA_ADDRESS_BYTE_4              0x6C
#define BDA_ADDRESS_BYTE_5              0x45


#define MAX_DEVICES 9   // Maximum number of devices to store
#define BD_ADDR_LEN 6     // Length of Bluetooth Device Address
#define MAX_EIR_LEN 240   // Maximum length of EIR data

/* Enable below block for verbose logging via printf
 * Note that enabling IPC VERBOSE might affect the application
 * performance delay which may result in malfunctioning */
#define ENABLE_BT_VERBOSE_LOGS          (0U)

#if ENABLE_BT_VERBOSE_LOGS
#ifdef WICED_BT_TRACE
#undef WICED_BT_TRACE
#endif
#define WICED_BT_TRACE                  printf
#endif

#ifndef A2DP_SOURCE_TASK_DELAY
    #define A2DP_SOURCE_TASK_DELAY     (10U)
#endif

#define TASK_DELAY_10MS                 (10U)

#define RESPONSE_YES_CAPITAL            ('Y')
#define RESPONSE_YES_SMALL              ('y')
#define RESPONSE_NO_CAPITAL             ('N')
#define RESPONSE_NO_SMALL               ('n')

#define A2DP_ENABLE_PREFILL                    (1U)
#define A2DP_PREFILL_PACKET_COUNT              (5U)
#define A2DP_PREFILL_PACKET_COUNT_BUFFER       (10U)

#define A2DP_TASK_STACK_SIZE                (512U)
#define A2DP_TASK_PRIORITY                  (configMAX_PRIORITIES - (3U))
/******************************************************************************
 * Enumerations
 *****************************************************************************/

typedef enum
{
    AV_STATE_IDLE,              /* Initial state (channel is unused) */
    AV_STATE_CONFIGURE,         /* Remote has sent configuration request */
    AV_STATE_OPEN,              /* Data channel connected but not streaming */
    AV_STATE_STARTED,           /* Data streaming */
    AV_STATE_RECONFIG,          /* Reconfiguring stream */
    AV_STATE_DISCONNECTING      /* Disconnecting */
} AV_STATE;

/******************************************************************************
 * Structures
 *****************************************************************************/

typedef struct
{
    wiced_bt_device_address_t peer_bda;      /* Peer bd address */
    AV_STATE state;                          /* AVDT State machine state */
    wiced_bt_a2dp_codec_info_t codec_config; /*Codec config */
    uint16_t stream_mtu;                     /* MTU of stream */
    uint16_t lcid;                           /* Local identifier */
} tAV_APP_CB;

typedef struct {
    uint8_t remote_bd_addr[BD_ADDR_LEN];  // Bluetooth Device Address
    uint8_t eir_data[MAX_EIR_LEN];       // EIR data
    int8_t rssi;                         // RSSI value
} device_info_t;

extern wiced_bt_heap_t *p_default_heap;
extern const wiced_bt_cfg_settings_t a2dp_source_cfg_settings;
extern int8_t connection_status;
extern bool initiate_bt_device_connection;
/*******************************************************************************
* Function prototypes
*******************************************************************************/
uint16_t a2dp_source_write_nvram (int nvram_id, int data_len, void *p_data);
uint16_t a2dp_source_read_nvram (int nvram_id, void *p_data, int data_len);
wiced_result_t a2dp_source_management_callback (
        wiced_bt_management_evt_t event,
        wiced_bt_management_evt_data_t *p_event_data);
void reset_link_keys(void);
void av_app_init (void);
void a2dp_source_control_cback (wiced_bt_a2dp_source_event_t event,
                wiced_bt_a2dp_source_event_data_t *p_data);
void a2dp_source_bt_set_pairability ( uint8_t pairing_allowed );
void a2dp_source_bt_print_local_bda( void );
void a2dp_source_bt_inquiry_result_cback( wiced_bt_dev_inquiry_scan_result_t *p_inquiry_result, uint8_t *p_eir_data );
wiced_result_t a2dp_source_bt_inquiry( uint8_t enable );
wiced_result_t a2dp_source_bt_set_visibility( uint8_t discoverability, uint8_t connectability );
wiced_result_t a2dp_source_command_connect(wiced_bt_device_address_t bd_addr, uint32_t len);
wiced_result_t a2dp_source_command_disconnect(void);
void a2dp_source_command_stream_config(uint8_t sf, uint8_t chcfg);
wiced_result_t a2dp_source_command_stream_start(void);
wiced_result_t a2dp_source_command_stream_stop(void);
void a2dp_source_audio_thread(void* arg);
#endif  /* A2DP_SOURCE_H */

